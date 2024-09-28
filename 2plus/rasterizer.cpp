// clang-format off
//
// Created by goksu on 4/6/19.
//

#include <algorithm>
#include <vector>
#include "rasterizer.hpp"
#include <opencv2/opencv.hpp>
#include <math.h>

//get min of 3 numbers
inline float min3(float x,float y,float z)
{
    x=x<=y?x:y;
    x=x<=z?x:z;
    return x;
}
//get max of 3 numbers
inline float max3(float x,float y,float z)
{
    x=x>=y?x:y;
    x=x>=z?x:z;
    return x;
}
rst::pos_buf_id rst::rasterizer::load_positions(const std::vector<Eigen::Vector3f> &positions)
{
    auto id = get_next_id();
    pos_buf.emplace(id, positions);

    return {id};
}

rst::ind_buf_id rst::rasterizer::load_indices(const std::vector<Eigen::Vector3i> &indices)
{
    auto id = get_next_id();
    ind_buf.emplace(id, indices);

    return {id};
}

rst::col_buf_id rst::rasterizer::load_colors(const std::vector<Eigen::Vector3f> &cols)
{
    auto id = get_next_id();
    col_buf.emplace(id, cols);

    return {id};
}

auto to_vec4(const Eigen::Vector3f& v3, float w = 1.0f)
{
    return Vector4f(v3.x(), v3.y(), v3.z(), w);
}


static float insideTriangle(float x, float y, const Vector3f* _v)
{   
    /*
    to judge if a pixel is inside a triangle
    the point is get the clockwise or couter-clockwise order of 3 vectors of the triangle
    and 3 vectors with starting point P and the end point,3 vertexes of the triangle
    then do the cross product of each pair of vectors
    judge if the z component of the results are of the same sign
    if true, the pixel in inside the triangle
    */
    Eigen::Vector3f pixel(x,y,1.0f);
    const Eigen::Vector3f A=_v[0];
    const Eigen::Vector3f B=_v[1];
    const Eigen::Vector3f C=_v[2];
    const Eigen::Vector3f AB=B-A;
    const Eigen::Vector3f BC=C-B;
    const Eigen::Vector3f CA=A-C;
    const Eigen::Vector3f PA=A-pixel;
    const Eigen::Vector3f PB=B-pixel;
    const Eigen::Vector3f PC=C-pixel;
    float c1,c2,c3;
    c1=AB.cross(PA).z();
    c2=BC.cross(PB).z();
    c3=CA.cross(PC).z();
    return c1<0&&c2<0&&c3<0||c1>0&&c2>0&&c3>0;
}

static std::tuple<float, float, float> computeBarycentric2D(float x, float y, const Vector3f* v)
{
    float c1 = (x*(v[1].y() - v[2].y()) + (v[2].x() - v[1].x())*y + v[1].x()*v[2].y() - v[2].x()*v[1].y()) / (v[0].x()*(v[1].y() - v[2].y()) + (v[2].x() - v[1].x())*v[0].y() + v[1].x()*v[2].y() - v[2].x()*v[1].y());
    float c2 = (x*(v[2].y() - v[0].y()) + (v[0].x() - v[2].x())*y + v[2].x()*v[0].y() - v[0].x()*v[2].y()) / (v[1].x()*(v[2].y() - v[0].y()) + (v[0].x() - v[2].x())*v[1].y() + v[2].x()*v[0].y() - v[0].x()*v[2].y());
    float c3 = (x*(v[0].y() - v[1].y()) + (v[1].x() - v[0].x())*y + v[0].x()*v[1].y() - v[1].x()*v[0].y()) / (v[2].x()*(v[0].y() - v[1].y()) + (v[1].x() - v[0].x())*v[2].y() + v[0].x()*v[1].y() - v[1].x()*v[0].y());
    return {c1,c2,c3};
}

void rst::rasterizer::draw(pos_buf_id pos_buffer, ind_buf_id ind_buffer, col_buf_id col_buffer, Primitive type)
{
    auto& buf = pos_buf[pos_buffer.pos_id];
    auto& ind = ind_buf[ind_buffer.ind_id];
    auto& col = col_buf[col_buffer.col_id];

    float f1 = (50 - 0.1) / 2.0;
    float f2 = (50 + 0.1) / 2.0;

    Eigen::Matrix4f mvp = projection * view * model;
    for (auto& i : ind)
    {
        Triangle t;
        Eigen::Vector4f v[] = {
                mvp * to_vec4(buf[i[0]], 1.0f),
                mvp * to_vec4(buf[i[1]], 1.0f),
                mvp * to_vec4(buf[i[2]], 1.0f)
        };
        //Homogeneous division
        for (auto& vec : v) {
            vec /= vec.w();
        }
        //Viewport transformation
        for (auto & vert : v)
        {
            vert.x() = 0.5*width*(vert.x()+1.0);
            vert.y() = 0.5*height*(vert.y()+1.0);
            vert.z() = vert.z() * f1 + f2;
        }

        for (int i = 0; i < 3; ++i)
        {
            t.setVertex(i, v[i].head<3>());
            t.setVertex(i, v[i].head<3>());
            t.setVertex(i, v[i].head<3>());
        }

        auto col_x = col[i[0]];
        auto col_y = col[i[1]];
        auto col_z = col[i[2]];

        t.setColor(0, col_x[0], col_x[1], col_x[2]);
        t.setColor(1, col_y[0], col_y[1], col_y[2]);
        t.setColor(2, col_z[0], col_z[1], col_z[2]);

        rasterize_triangle(t);
    }
}

//Screen space rasterization
void rst::rasterizer::rasterize_triangle(const Triangle& t) {
    auto v = t.toVector4();
    
    // TODO : Find out the bounding box of current triangle.
    // iterate through the pixel and find if the current pixel is inside the triangle
    /*
    first find out the bounding box of the triangle to avoid unneccesary emamination of whether a pixel is within the triangle
    */
    int x_min,x_max;
    float y_min,y_max;
    x_min=(int)min3(t.v[0].x(),t.v[1].x(),t.v[2].x());
    x_max=(int)max3(t.v[0].x(),t.v[1].x(),t.v[2].x());
    y_min=min3(t.v[0].y(),t.v[1].y(),t.v[2].y());
    y_max=max3(t.v[0].y(),t.v[1].y(),t.v[2].y());
    //iterate through each pixel in the bouding box
    //using super-sampling method for anti-aliasing
    //each pair of x,y determines a area containing 4 pixels
    for(int x=x_min;x<=x_max;++x)
    {
        for(int y=y_min;y<=y_max;++y)
        {
            bool inside=false;
            float z_nearest=0;//the smallest z component of the pixels in the triangle
            float color_index=0;//the index of color,create a dim-out effect on the edges to avoid apparent jaggies
            Vector3f pixels[4];//the 4 pixels in the area
            pixels[0]<<x+0.25,y+0.25,1.0f;
            pixels[1]<<x+0.75,y+0.25,1.0f;
            pixels[2]<<x+0.25,y+0.75,1.0f;
            pixels[3]<<x+0.75,y+0.75,1.0f;

            for(int i=0;i<4;++i)
            {
                if(insideTriangle(pixels[i].x(),pixels[i].y(),t.v))
            {
                //calculate the z component of each pixel by interpolation
                auto[alpha, beta, gamma] = computeBarycentric2D(pixels[i].x(), pixels[i].y(), t.v);
                float w_reciprocal = 1.0/(alpha / v[0].w() + beta / v[1].w() + gamma / v[2].w());
                float z_interpolated = alpha * v[0].z() / v[0].w() + beta * v[1].z() / v[1].w() + gamma * v[2].z() / v[2].w();
                z_interpolated *= w_reciprocal;
                inside=true;
                color_index+=0.25;
                if(z_nearest>z_interpolated||z_nearest==0)
                z_nearest=z_interpolated;
            }

            }
            //sets the color of the pixel
            if(inside)
            {
            int index=get_index(x,y);
            if(z_nearest>=depth_buf[index])
            continue;
            depth_buf[index]=z_nearest;
            set_pixel(Vector3f(x,y,1),color_index*t.getColor());

            }
            
        }
    }

    // If so, use the following code to get the interpolated z value.
    //auto[alpha, beta, gamma] = computeBarycentric2D(x, y, t.v);
    //float w_reciprocal = 1.0/(alpha / v[0].w() + beta / v[1].w() + gamma / v[2].w());
    //float z_interpolated = alpha * v[0].z() / v[0].w() + beta * v[1].z() / v[1].w() + gamma * v[2].z() / v[2].w();
    //z_interpolated *= w_reciprocal;

    // TODO : set the current pixel (use the set_pixel function) to the color of the triangle (use getColor function) if it should be painted.
}

void rst::rasterizer::set_model(const Eigen::Matrix4f& m)
{
    model = m;
}

void rst::rasterizer::set_view(const Eigen::Matrix4f& v)
{
    view = v;
}

void rst::rasterizer::set_projection(const Eigen::Matrix4f& p)
{
    projection = p;
}

void rst::rasterizer::clear(rst::Buffers buff)
{
    if ((buff & rst::Buffers::Color) == rst::Buffers::Color)
    {
        std::fill(frame_buf.begin(), frame_buf.end(), Eigen::Vector3f{0, 0, 0});
    }
    if ((buff & rst::Buffers::Depth) == rst::Buffers::Depth)
    {
        std::fill(depth_buf.begin(), depth_buf.end(), std::numeric_limits<float>::infinity());
    }
}

rst::rasterizer::rasterizer(int w, int h) : width(w), height(h)
{
    frame_buf.resize(w * h);
    depth_buf.resize(w * h);
}

int rst::rasterizer::get_index(int x, int y)
{
    return (height-1-y)*width + x;
}

void rst::rasterizer::set_pixel(const Eigen::Vector3f& point, const Eigen::Vector3f& color)
{
    //old index: auto ind = point.y() + point.x() * width;
    auto ind = (height-1-point.y())*width + point.x();
    frame_buf[ind] = color;

}

// clang-format on