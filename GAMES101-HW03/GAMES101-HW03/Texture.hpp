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

    Eigen::Vector3f getColorBilinear(float u, float v)
    {
        auto u_img = u * width;
        auto v_img = (1 - v) * height;

        auto u_min = std::floor(u_img);
        auto u_max = std::min(static_cast<float>(width), std::ceil(u_img));
        auto v_min = std::floor(v_img);
        auto v_max = std::min(static_cast<float>(height), std::ceil(v_img));
        
        auto c00 = image_data.at<cv::Vec3b>(v_max, u_min);
        auto c01 = image_data.at<cv::Vec3b>(v_min, u_min);
        auto c10 = image_data.at<cv::Vec3b>(v_max, u_max);
        auto c11 = image_data.at<cv::Vec3b>(v_min, u_max);

        auto s = u_img - u_min;
        auto t = v_max - v_img;
        auto c0 = c00 + s * (c10 - c00);
        auto c1 = c01 + s * (c11 - c01);
        auto c = c1 + t * (c0 - c1);
        
        return Eigen::Vector3f(c[0], c[1], c[2]);
    }

};
#endif //RASTERIZER_TEXTURE_H
