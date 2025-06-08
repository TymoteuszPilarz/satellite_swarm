#pragma once

#include <Eigen/Dense>

#include <cmath>
#include <fstream>
#include <iostream>
#include <numbers>
#include <ranges>
#include <string>
#include <vector>

struct Point3D
{
    float x;
    float y;
    float z;
};

struct Rotation3D
{
    float pitch;
    float yaw;
    float roll;
};

inline std::vector<float> linspace(float start, float end, int num)
{
    std::vector<float> result;

    if (num <= 0)
    {
        return result;
    }
    if (num == 1)
    {
        result.push_back(start);
        return result;
    }

    float step = (end - start) / static_cast<float>(num - 1);
    for (int i = 0; i < num; ++i)
    {
        result.push_back(start + (step * static_cast<float>(i)));
    }

    return result;
}

inline float distance(const Point3D& first_point, const Point3D& second_point)
{
    return std::sqrt(((first_point.x - second_point.x) * (first_point.x - second_point.x))
                     + ((first_point.y - second_point.y) * (first_point.y - second_point.y))
                     + ((first_point.z - second_point.z) * (first_point.z - second_point.z)));
}

inline std::vector<Point3D>
  find_nearby_points(const std::vector<Point3D>& points, const Point3D& target_point, float max_distance)
{
    return points
           | std::views::filter([&](const auto& x) {
                 return distance(x, target_point) <= max_distance;
             })
           | std::ranges::to<std::vector<Point3D>>();
}

inline std::vector<Point3D> gen_circle_points(float radius, int num_of_points)
{
    std::vector<Point3D> points;

    for (int i = 0; i < num_of_points; ++i)
    {
        auto angle = 2.f * std::numbers::pi_v<float> * static_cast<float>(i) / static_cast<float>(num_of_points);
        auto x = radius * std::cos(angle);
        auto y = radius * std::sin(angle);
        auto z = 0.0f;

        points.push_back({x, y, z});
    }

    return points;
}

inline std::vector<Point3D> gen_sphere_points(float radius, int num_of_points)
{
    std::vector<Point3D> points;
    points.reserve(num_of_points);

    const auto offset = 2.f / static_cast<float>(num_of_points);
    const auto increment = 2 * std::numbers::pi_v<float> * (1 - 1 / std::numbers::phi_v<float>);

    for (int i = 0; i < num_of_points; ++i)
    {
        float y = ((static_cast<float>(i) * offset) - 1.f) + (offset / 2.f);
        float r = std::sqrt(1.f - (y * y));
        float phi = static_cast<float>(i) * increment;
        float x = std::cos(phi) * r;
        float z = std::sin(phi) * r;

        points.push_back({x * radius, y * radius, z * radius});
    }

    return points;
}

inline bool is_segment_intersecting_sphere(float radius,
                                           const Point3D& first_point,
                                           const Point3D& second_point,
                                           float epsilon = 0.15f
)
{
    Eigen::Vector3f p1(first_point.x, first_point.y, first_point.z);
    Eigen::Vector3f p2(second_point.x, second_point.y, second_point.z);
    Eigen::Vector3f center(0.0f, 0.0f, 0.0f);

    Eigen::Vector3f d = p2 - p1;
    Eigen::Vector3f m = p1 - center;

    float a = d.dot(d);
    float b = 2.0f * m.dot(d);
    float c = m.dot(m) - radius * radius;

    float discriminant = b * b - 4.0f * a * c;
    if (discriminant < 0.0f)
    {
        return false;
    }

    float sqrt_disc = std::sqrt(discriminant);
    float t1 = (-b - sqrt_disc) / (2.0f * a);
    float t2 = (-b + sqrt_disc) / (2.0f * a);

    auto is_central_hit = [epsilon](float t) {
        return t >= epsilon && t <= (1.0f - epsilon);
    };

    return is_central_hit(t1) || is_central_hit(t2);
}

inline std::vector<Point3D> rotate_plane_to_face_target(const std::vector<Point3D>& original_points,
                                                        const Point3D& new_center_point,
                                                        const Point3D& target_point)
{
    const auto new_center = Eigen::Vector3f{new_center_point.x, new_center_point.y, new_center_point.z};
    const auto target = Eigen::Vector3f{target_point.x, target_point.y, target_point.z};

    const auto target_normal = Eigen::Vector3f{target - new_center}.normalized();
    const auto original_normal = Eigen::Vector3f{0.0f, 0.0f, 1.0f};

    auto rotation = Eigen::Quaternionf{};
    if (original_normal.isApprox(target_normal))
    {
        rotation = Eigen::Quaternionf::Identity();
    }
    else if (original_normal.isApprox(-target_normal))
    {
        auto axis = Eigen::Vector3f{1.0f, 0.0f, 0.0f};
        if (original_normal.cross(axis).norm() < 1e-6f)
        {
            axis = Eigen::Vector3f(0.0f, 1.0f, 0.0f);
        }
        rotation = Eigen::AngleAxisf(std::numbers::pi_v<float>, axis);
    }
    else
    {
        const auto axis = Eigen::Vector3f{original_normal.cross(target_normal).normalized()};
        const auto angle = std::acosf(original_normal.dot(target_normal));
        rotation = Eigen::AngleAxisf(angle, axis);
    }

    return original_points
           | std::views::transform([&](const Point3D& point) {
                 const auto point_vec = Eigen::Vector3f{point.x, point.y, point.z};
                 const auto rotated_point_vec = Eigen::Vector3f{rotation * point_vec + new_center};
                 return Point3D{.x = rotated_point_vec.x(), .y = rotated_point_vec.y(), .z = rotated_point_vec.z()};
             })
           | std::ranges::to<std::vector>();
}

inline float wrap_phase(float phase)
{
    constexpr float two_pi = 2.f * std::numbers::pi_v<float>;
    return std::fmod(std::fmod(phase, two_pi) + two_pi, two_pi);
}

inline void save_points_to_csv(const std::string& filename, const std::vector<Point3D>& points)
{
    auto file = std::ofstream{filename};
    file << "x,y,z\n";
    for (const auto& p : points)
    {
        file << p.x << ',' << p.y << ',' << p.z << '\n';
    }
}

inline void save_results_to_csv(const std::string& filename,
                                const std::vector<std::pair<Point3D, std::vector<std::pair<Point3D, float>>>>& data)
{
    auto file = std::ofstream(filename);

    file << "satellite_x,satellite_y,satellite_z,point_x,point_y,point_z,power\n";

    for (const auto& [satellite_position, points] : data)
    {
        for (const auto& [pt, power] : points)
        {
            file
              << satellite_position.x
              << ","
              << satellite_position.y
              << ","
              << satellite_position.z
              << ","
              << pt.x
              << ","
              << pt.y
              << ","
              << pt.z
              << ","
              << power
              << "\n";
        }
    }
}
