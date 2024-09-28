//
// Created by LEI XU on 4/27/19.
//

#ifndef RASTERIZER_TEXTURE_H
#define RASTERIZER_TEXTURE_H
#include "global.hpp"
#include <eigen3/Eigen/Eigen>
#include <opencv2/opencv.hpp>
class Texture{
private:
    cv::Mat image_data;

public:
    Texture(const std::string& name)
    {
        image_data = cv::imread(name);
        cv::cvtColor(image_data, image_data, cv::COLOR_RGB2BGR);
        width = image_data.cols;
        height = image_data.rows;
    }

    int width, height;

    Eigen::Vector3f getColor(float u, float v)
    {
        auto u_img = u * width;
        auto v_img = (1 - v) * height;
        auto color = image_data.at<cv::Vec3b>(v_img, u_img);
        return Eigen::Vector3f(color[0], color[1], color[2]);
    }
    Eigen::Vector3f getColorBilinear(float u,float v)
    {
        auto u_img = u * width;
        auto v_img = (1 - v) * height;
        int x=int(u_img),y=int(v_img);
        x=(u_img-x)<0.5?std::floor(u_img):std::ceil(u_img);
        y=(v_img-y)<0.5?std::floor(v_img):std::ceil(v_img);
//        float x1=x-0.5,x2=x1+0.5,x3=x-0.5,x4=x+0.5,y1=y+0.5,y2=y+0.5,y3=y-0.5,y4=y-0.5;
/*
points
1  2
3  4
*/
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
#endif //RASTERIZER_TEXTURE_H
