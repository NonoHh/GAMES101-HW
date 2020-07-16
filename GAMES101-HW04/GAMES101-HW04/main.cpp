#include <chrono>
#include <iostream>
#include <opencv2/opencv.hpp>

std::vector<cv::Point2f> control_points;

const int control_point_num = 4;

void mouse_handler(int event, int x, int y, int flags, void *userdata) 
{
    if (event == cv::EVENT_LBUTTONDOWN && control_points.size() < control_point_num)
    {
        std::cout << "Left button of the mouse is clicked - position (" << x << ", "
        << y << ")" << '\n';
        control_points.emplace_back(x, y);
    }     

    // test the Anti-aliasing method
    // control_points.emplace_back(141, 432);
    // control_points.emplace_back(270, 193);
    // control_points.emplace_back(421, 200);
    // control_points.emplace_back(300, 508);
    // control_points.resize(4);
}

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

        window.at<cv::Vec3b>(point.y, point.x)[2] = 255;
    }
}

cv::Point2f recursive_bezier(const std::vector<cv::Point2f>& control_points, float t)
{
    if (control_points.size() == 1) {
        return control_points[0];
    }
    std::vector<cv::Point2f> new_control_points;
    for (int i = 0; i < control_points.size() - 1; i++) {
        new_control_points.push_back(control_points[i] + t * (control_points[i + 1] - control_points[i]));
    }
    return recursive_bezier(new_control_points, t);
}

std::vector<cv::Point2f> get_points(float x, float y)
{
    auto x_min = std::floor(x) + 0.5f;
    auto y_min = std::floor(y) + 0.5f;

    auto x_near = x - x_min < 0.5 ? -1 : 1;
    auto y_near = y - y_min < 0.5 ? -1 : 1;

    std::vector<cv::Point2f> res;
    res.emplace_back(cv::Point2f(x_min, y_min));
    res.emplace_back(cv::Point2f(x_min + x_near * 1, y_min + y_near * 1));
    res.emplace_back(cv::Point2f(x_min + x_near * 1, y_min + y_near * 0));
    res.emplace_back(cv::Point2f(x_min + x_near * 0, y_min + y_near * 1));

    return res;
}

void anti_aliasing(float x, float y, cv::Mat& window)
{
    cv::Point2f p(x, y);
    auto neighbor_point = get_points(x, y);

    double dis[4];
    double min_dis = std::numeric_limits<double>::max();
    int ind = 0;
    for (int i = 0; i < 4; i++) {
        dis[i] = cv::norm(p - neighbor_point[i]);
        if (dis[i] < min_dis) {
            min_dis = dis[i];
            ind = i;
        }
    }

    for (int i = 0; i < 4; i++) {
        window.at<cv::Vec3b>(neighbor_point[i].y, neighbor_point[i].x)[1] = std::max(static_cast<double>(window.at<cv::Vec3b>(neighbor_point[i].y, neighbor_point[i].x)[1]), 255 * min_dis / dis[i]);
    }
}

void bezier(const std::vector<cv::Point2f>& control_points, cv::Mat& window)
{
    float step = 0.0001;
    for (float t = 0; t <= 1; t += step) {
        auto point = recursive_bezier(control_points, t);
        window.at<cv::Vec3b>(point.y, point.x)[1] = 255;
        anti_aliasing(point.x, point.y, window);
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
        for (auto& point : control_points)
        {
            cv::circle(window, point, 3, { 255, 255, 255 }, 3);
        }

        if (control_points.size() == control_point_num)
        {
            // naive_bezier(control_points, window);
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
