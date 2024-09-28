#include <chrono>
#include <iostream>
#include <opencv2/opencv.hpp>

std::vector<cv::Point2f> control_points;

void mouse_handler(int event, int x, int y, int flags, void *userdata) 
{
    if (event == cv::EVENT_LBUTTONDOWN && control_points.size() < 4) 
    {
        std::cout << "Left button of the mouse is clicked - position (" << x << ", "
        << y << ")" << '\n';
        control_points.emplace_back(x, y);
    }     
}
/* Bilinear lerp in homework3 texture.hpp
    Eigen::Vector3f getColorBilinear(float u,float v)
    {
        auto u_img = u * width;
        auto v_img = (1 - v) * height;
        int x=int(u_img),y=int(v_img);
        x=(u_img-x)<0.5?std::floor(u_img):std::ceil(u_img);
        y=(v_img-y)<0.5?std::floor(v_img):std::ceil(v_img);
        float xl=x-0.5,xr=x+0.5,yl=y-0.5,yh=y+0.5;//x_left,x_right,y_high,y_low

        auto color1=image_data.at<cv::Vec3b>(yh, xl);//notice that the first argument is v not u!!!
        auto color2=image_data.at<cv::Vec3b>(yh, xr);
        auto color3=image_data.at<cv::Vec3b>(yl, xl);
        auto color4=image_data.at<cv::Vec3b>(yl, xr);
        Eigen::Vector3f _color1(color1[0],color1[1],color1[2]);//improve precision
        Eigen::Vector3f _color2(color2[0],color2[1],color2[2]);
        Eigen::Vector3f _color3(color3[0],color3[1],color3[2]);
        Eigen::Vector3f _color4(color4[0],color4[1],color4[2]);


        float wl=xr-u_img,wr=1-wl,wh=v_img-yl,w_low=1-wh;//weight left right high low

        auto color = (wl*_color1+wr*_color2)*wh+(wl*_color3+wr*_color4)*w_low;
        return Eigen::Vector3f(color[0], color[1], color[2]);

    }

};
*/
/*logic of lerp
        auto u_img = u * width;
        auto v_img = (1 - v) * height;
        int x=int(u_img),y=int(v_img);
        x=(u_img-x)<0.5?std::floor(u_img):std::ceil(u_img);
        y=(v_img-y)<0.5?std::floor(v_img):std::ceil(v_img);
        float xl=x-0.5,xr=x+0.5,yl=y-0.5,yh=y+0.5;//x_left,x_right,y_high,y_low
        float wl=xr-u_img,wr=1-wl,wh=v_img-yl,w_low=1-wh;//weight left right high low

*/
//a point has 4 nearest pixels,distribute its color to these 4 pixels by weights or distance
void naive_bezier(const std::vector<cv::Point2f> &points, cv::Mat &window) 
{
    auto &p_0 = points[0];
    auto &p_1 = points[1];
    auto &p_2 = points[2];
    auto &p_3 = points[3];

    for (double t = 0.0; t <= 1.0; t += 0.001) 
    {
        auto point = std::pow(1 - t, 3) * p_0 + 3 * t * std::pow(1 - t, 2) * p_1 +
                 3 * std::pow(t, 2) * (1 - t) * p_2 + std::pow(t, 3) * p_3;
        auto coord_x = point.x;
        auto coord_y = point.y;
        int x=int(coord_x),y=int(coord_y);
        x=(coord_x-x)<0.5?std::floor(coord_x):std::ceil(coord_x);
        y=(coord_y-y)<0.5?std::floor(coord_y):std::ceil(coord_y);
        float xl=x-0.5,xr=x+0.5,yl=y-0.5,yh=y+0.5;//x_left,x_right,y_high,y_low
        float wl=xr-coord_x,wr=1-wl,wh=coord_y-yl,w_low=1-wh;//weight left right high low

        window.at<cv::Vec3b>(yh, xl)[2]=std::min(255.0f,window.at<cv::Vec3b>(yh, xl)[2] + 255.0f*wl*wh);
        window.at<cv::Vec3b>(yh, xr)[2]=std::min(255.0f,window.at<cv::Vec3b>(yh, xr)[2] + 255.0f*wr*wh);
        window.at<cv::Vec3b>(yl, xl)[2]=std::min(255.0f,window.at<cv::Vec3b>(yl, xl)[2] + 255.0f*wl*w_low);
        window.at<cv::Vec3b>(yl, xr)[2]=std::min(255.0f,window.at<cv::Vec3b>(yl, xr)[2] + 255.0f*wr*w_low);
    }
}

cv::Point2f recursive_bezier(const std::vector<cv::Point2f> &control_points, float t) 
{
    // TODO: Implement de Casteljau's algorithm
    auto &p_0 = control_points[0];
    auto &p_1 = control_points[1];
    auto &p_2 = control_points[2];
    auto &p_3 = control_points[3];
            auto point = std::pow(1 - t, 3) * p_0 + 3 * t * std::pow(1 - t, 2) * p_1 +
                 3 * std::pow(t, 2) * (1 - t) * p_2 + std::pow(t, 3) * p_3;
    return point;

}

void bezier(const std::vector<cv::Point2f> &control_points, cv::Mat &window) 
{
    // TODO: Iterate through all t = 0 to t = 1 with small steps, and call de Casteljau's 
    // recursive Bezier algorithm.
    for(double t=0.0;t<=1.0;t+=0.001)
    {
        auto point=recursive_bezier(control_points,t);
                auto coord_x = point.x;
        auto coord_y = point.y;
        int x=int(coord_x),y=int(coord_y);
        x=(coord_x-x)<0.5?std::floor(coord_x):std::ceil(coord_x);
        y=(coord_y-y)<0.5?std::floor(coord_y):std::ceil(coord_y);
        float xl=x-0.5,xr=x+0.5,yl=y-0.5,yh=y+0.5;//x_left,x_right,y_high,y_low
        float wl=xr-coord_x,wr=1-wl,wh=coord_y-yl,w_low=1-wh;//weight left right high low

        
        window.at<cv::Vec3b>(yh, xl)[1]=std::min(255.0f,window.at<cv::Vec3b>(yh, xl)[1] + 255.0f*wl*wh);
        window.at<cv::Vec3b>(yh, xr)[1]=std::min(255.0f,window.at<cv::Vec3b>(yh, xr)[1] + 255.0f*wr*wh);
        window.at<cv::Vec3b>(yl, xl)[1]=std::min(255.0f,window.at<cv::Vec3b>(yl, xl)[1] + 255.0f*wl*w_low);
        window.at<cv::Vec3b>(yl, xr)[1]=std::min(255.0f,window.at<cv::Vec3b>(yl, xr)[1] + 255.0f*wr*w_low);
    }

}

int main() 
{
    cv::Mat window = cv::Mat(700, 700, CV_8UC3, cv::Scalar(0));
    cv::cvtColor(window, window, cv::COLOR_BGR2RGB);
    cv::namedWindow("Bezier Curve", cv::WINDOW_AUTOSIZE);

    cv::setMouseCallback("Bezier Curve", mouse_handler, nullptr);

    int key = -1;
    while (key != 27) 
    {
        for (auto &point : control_points) 
        {
            cv::circle(window, point, 3, {255, 255, 255}, 3);
        }

        if (control_points.size() == 4) 
        {
            naive_bezier(control_points, window);
            bezier(control_points, window);

            cv::imshow("Bezier Curve", window);
            cv::imwrite("my_bezier_curve.png", window);
            key = cv::waitKey(0);

            return 0;
        }

        cv::imshow("Bezier Curve", window);
        key = cv::waitKey(20);
    }

return 0;
}
