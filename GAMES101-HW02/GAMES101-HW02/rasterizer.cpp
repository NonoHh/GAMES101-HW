// clang-format off
//
// Created by goksu on 4/6/19.
//

#include <algorithm>
#include <vector>
#include "rasterizer.hpp"
#include <opencv2/opencv.hpp>
#include <math.h>


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

static bool insideTriangle(float x, float y, const Vector3f* _v)
{
    Eigen::Vector3f v_01, v_12, v_20;
    v_01 << _v[1].x() - _v[0].x(), _v[1].y() - _v[0].y(), 0;
    v_12 << _v[2].x() - _v[1].x(), _v[2].y() - _v[1].y(), 0;
    v_20 << _v[0].x() - _v[2].x(), _v[0].y() - _v[2].y(), 0;

    Eigen::Vector3f v_0p, v_1p, v_2p;
    v_0p << x - _v[0].x(), y - _v[0].y(), 0;
    v_1p << x - _v[1].x(), y - _v[1].y(), 0;
    v_2p << x - _v[2].x(), y - _v[2].y(), 0;

    Eigen::Vector3f cross_product_0 = v_01.cross(v_0p);
    Eigen::Vector3f cross_product_1 = v_12.cross(v_1p);
    Eigen::Vector3f cross_product_2 = v_20.cross(v_2p);

    return cross_product_0.dot(cross_product_1) > 0 && cross_product_0.dot(cross_product_2) > 0 && cross_product_1.dot(cross_product_2) > 0;
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


//Screen space rasterization without MSAA
// void rst::rasterizer::rasterize_triangle(const Triangle& t) {
//     auto v = t.toVector4();
//
//     float x_min = std::min(v[0].x(), std::min(v[1].x(), v[2].x()));
//     float x_max = std::max(v[0].x(), std::max(v[1].x(), v[2].x()));
//     float y_min = std::min(v[0].y(), std::min(v[1].y(), v[2].y()));
//     float y_max = std::max(v[0].y(), std::max(v[1].y(), v[2].y()));
//
//     Eigen::Vector3f _v[3];
//     for (int i = 0; i < 3; i++) {
//         _v[i] << v[i].x(), v[i].y(), v[i].z();
//     }
//     
//     // iterate through the pixel and find if the current pixel is inside the triangle
//     for (int x = std::floor(x_min); x <= std::ceil(x_max); x++) {
//         for (int y = std::floor(y_min); y <= std::ceil(y_max); y++) {
//             if (insideTriangle(x + 0.5, y + 0.5, _v)) {
//                 // If so, use the following code to get the interpolated z value.
//                 auto [alpha, beta, gamma] = computeBarycentric2D(x + 0.5, y + 0.5, t.v);
//                 float w_reciprocal = 1.0 / (alpha / v[0].w() + beta / v[1].w() + gamma / v[2].w());
//                 float z_interpolated = -(alpha * v[0].z() / v[0].w() + beta * v[1].z() / v[1].w() + gamma * v[2].z() / v[2].w());
//                 z_interpolated *= w_reciprocal;
//
//                 auto ind = get_index(x, y);
//                 if (z_interpolated < depth_buf.at(ind)) {
//                     set_pixel(Eigen::Vector3f(x, y, 0), t.getColor());
//                     auto it = depth_buf.begin() + ind;
//                     *it = z_interpolated;
//                 }
//             }
//         }
//     }
// }

//Screen space rasterization with MSAA 2x2
void rst::rasterizer::rasterize_triangle(const Triangle& t) {
    auto v = t.toVector4();

    // TODO : Find out the bounding box of current triangle.
    float x_min = std::min(v[0].x(), std::min(v[1].x(), v[2].x()));
    float x_max = std::max(v[0].x(), std::max(v[1].x(), v[2].x()));
    float y_min = std::min(v[0].y(), std::min(v[1].y(), v[2].y()));
    float y_max = std::max(v[0].y(), std::max(v[1].y(), v[2].y()));

    Eigen::Vector3f _v[3];
    for (int i = 0; i < 3; i++) {
        _v[i] << v[i].x(), v[i].y(), v[i].z();
    }

    int MS = 2 * 2;
    float step = 1.0 / MS;
    int pixel_ind;
    // iterate through the pixel and find if the current pixel is inside the triangle
    for (int x = std::floor(x_min); x <= std::ceil(x_max); x++) {
        for (int y = std::floor(y_min); y <= std::ceil(y_max); y++) {
            float _x = x, _y = y;
            int inside_sample_counter = 0;
            for (int i = 0; i < 2; i++) {
                _x += (i + 1) * step;
                for (int j = 0; j < 2; j++) {
                    _y += (j + 1) * step;
                    if (insideTriangle(_x, _y, _v)) {

                        // If so, use the following code to get the interpolated z value.
                        pixel_ind = get_index(x, y);
                        int sample_ind = i * 2 + j;
                        auto [alpha, beta, gamma] = computeBarycentric2D(_x, _y, t.v);
                        float w_reciprocal = 1.0 / (alpha / v[0].w() + beta / v[1].w() + gamma / v[2].w());
                        float z_interpolated = -(alpha * v[0].z() / v[0].w() + beta * v[1].z() / v[1].w() + gamma * v[2].z() / v[2].w());
                        z_interpolated *= w_reciprocal;

                        if (z_interpolated < depth_buff_MSAA_2x2.at(pixel_ind).at(sample_ind)) {
                            inside_sample_counter++;
                            auto it1 = depth_buff_MSAA_2x2.begin() + pixel_ind;
                            (*it1)[sample_ind] = z_interpolated;
                            auto it2 = color_buff_MSAA_2x2.begin() + pixel_ind;
                            (*it2)[sample_ind] = t.getColor();
                        }
                    }
                }
            }
            if (inside_sample_counter != 0) {
                Eigen::Vector3f color = Eigen::Vector3f::Zero();
                for (int i = 0; i < MS; i++) {
                    color += color_buff_MSAA_2x2.at(pixel_ind).at(i);
                }
                set_pixel(Eigen::Vector3f(x, y, 0), color / 4.0);
            }
        }
    }
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
        std::fill(color_buff_MSAA_2x2.begin(), color_buff_MSAA_2x2.end(), std::array<Eigen::Vector3f, 4> {Eigen::Vector3f::Zero(), Eigen::Vector3f::Zero(), Eigen::Vector3f::Zero(), Eigen::Vector3f::Zero()});
    }
    if ((buff & rst::Buffers::Depth) == rst::Buffers::Depth)
    {
        std::fill(depth_buf.begin(), depth_buf.end(), std::numeric_limits<float>::infinity());
        auto inf = std::numeric_limits<float>::infinity();
        std::fill(depth_buff_MSAA_2x2.begin(), depth_buff_MSAA_2x2.end(), std::array<float, 4> {inf, inf, inf, inf});
    }
}

rst::rasterizer::rasterizer(int w, int h) : width(w), height(h)
{
    frame_buf.resize(w * h);
    color_buff_MSAA_2x2.resize(w * h);
    depth_buf.resize(w * h);
    depth_buff_MSAA_2x2.resize(w * h);
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