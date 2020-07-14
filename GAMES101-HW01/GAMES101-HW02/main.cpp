#include "Triangle.hpp"
#include "rasterizer.hpp"
#include <eigen3/Eigen/Eigen>
#include <iostream>
#include <opencv2/opencv.hpp>

constexpr double MY_PI = 3.1415926;

Eigen::Matrix4f get_view_matrix(Eigen::Vector3f eye_pos)
{
    Eigen::Matrix4f view = Eigen::Matrix4f::Identity();

    Eigen::Matrix4f translate;
    translate << 1, 0, 0, -eye_pos[0], 0, 1, 0, -eye_pos[1], 0, 0, 1,
        -eye_pos[2], 0, 0, 0, 1;

    view = translate * view;

    return view;
}

Eigen::Matrix4f get_model_matrix(float rotation_angle)
{
    Eigen::Matrix4f model = Eigen::Matrix4f::Identity();

    double a = std::cos(rotation_angle / 180.0 * MY_PI);
    double b = -std::sin(rotation_angle / 180.0 * MY_PI);
    double c = -b;
    double d = a;

    Eigen::Matrix4f rotation;
    rotation << a, b, 0, 0,
    c, d, 0, 0,
    0, 0, 1, 0,
    0, 0, 0, 1;

    model = rotation * model;

    return model;
}

Eigen::Matrix4f get_projection_matrix(float eye_fov, float aspect_ratio,
                                      float zNear, float zFar)
{
    zNear = -zNear;
    zFar = -zFar; 
    
    Eigen::Matrix4f projection = Eigen::Matrix4f::Identity();

    Eigen::Matrix4f squish;
    squish << zNear, 0, 0, 0,
    0, zNear, 0, 0,
    0, 0, zNear + zFar, -zNear * zFar,
    0, 0, 1, 0;

    auto t = std::abs(zNear) * std::tan(0.5 * eye_fov / 180.0 * MY_PI);
    auto r = t * aspect_ratio;
    auto b = -t;
    auto l = -r;

    Eigen::Matrix4f ortho, ortho_scale, ortho_trans;
    ortho_scale << 2/(r-l), 0, 0, 0,
    0, 2/(t-b), 0, 0,
    0, 0, 2 / (zNear - zFar), 0,
    0, 0, 0, 1;
    ortho_trans << 1, 0, 0, -(r + l) / 2,
    0, 1, 0, -(b + t) / 2,
    0, 0, 1, -(zNear + zFar) / 2,
    0, 0, 0, 1;
    ortho = ortho_scale * ortho_trans;

    projection = ortho * squish;

    return projection;
}

Eigen::Matrix4f get_rotation(Vector3f axis, float angle)
{
    Eigen::Matrix3f rotation = Eigen::Matrix3f::Identity();

    Eigen::Matrix3f N;
    N << 0, -axis.z(), axis.y(),
    axis.z(), 0, -axis.x(),
    -axis.y(), axis.x(), 0;

    auto angle_radian = angle / 180.0 * MY_PI;
    rotation = std::cos(angle_radian) * rotation + (1 - std::cos(angle_radian)) * axis * axis.transpose() + std::sin(angle_radian) * N;

    Eigen::Matrix4f model;
    model << rotation.row(0), 0,
    rotation.row(1), 0,
    rotation.row(2),
    0, 0, 0, 0, 1;

    return model;
}

int main(int argc, const char** argv)
{
    float angle = 0;
    bool command_line = false;
    bool other_axis = false;
    Vector3f axis = { 1,1,1 };
    std::string filename = "output.png";

    if (argc >= 3) {
        command_line = true;
        angle = std::stof(argv[2]); // -r by default
        if (argc == 4) {
            filename = std::string(argv[3]);
        }
        else
            return 0;
    }

    rst::rasterizer r(700, 700);

    Eigen::Vector3f eye_pos = {0, 0, 5};

    std::vector<Eigen::Vector3f> pos{{2, 0, -2}, {0, 2, -2}, {-2, 0, -2}};

    std::vector<Eigen::Vector3i> ind{{0, 1, 2}};

    auto pos_id = r.load_positions(pos);
    auto ind_id = r.load_indices(ind);

    int key = 0;
    int frame_count = 0;

    if (command_line) {
        r.clear(rst::Buffers::Color | rst::Buffers::Depth);

        r.set_model(get_model_matrix(angle));
        r.set_view(get_view_matrix(eye_pos));
        r.set_projection(get_projection_matrix(45, 1, 0.1, 50));

        r.draw(pos_id, ind_id, rst::Primitive::Triangle);
        cv::Mat image(700, 700, CV_32FC3, r.frame_buffer().data());
        image.convertTo(image, CV_8UC3, 1.0f);

        cv::imwrite(filename, image);

        return 0;
    }

    while (key != 27) {
        r.clear(rst::Buffers::Color | rst::Buffers::Depth);

        if(other_axis) {
            r.set_model(get_rotation(axis, angle));
        }else {
            r.set_model(get_model_matrix(angle));
        }
        r.set_view(get_view_matrix(eye_pos));
        r.set_projection(get_projection_matrix(45, 1, 0.1, 50));

        r.draw(pos_id, ind_id, rst::Primitive::Triangle);

        cv::Mat image(700, 700, CV_32FC3, r.frame_buffer().data());
        image.convertTo(image, CV_8UC3, 1.0f);
        cv::imshow("image", image);
        key = cv::waitKey(10);

        std::cout << "frame count: " << frame_count++ << '\n';

        if (key == 'a') {
            angle += 10;
        }
        else if (key == 'd') {
            angle -= 10;
        }else if(key =='o') {
            other_axis = !other_axis;
            std::cout << "reverse any axis. \n";
        }
    }

    return 0;
}
