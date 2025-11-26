//
// Created by pbustos on 11/11/25.
//

#include "door_detector.h"

#include <expected>
#include <cppitertools/sliding_window.hpp>
#include <cppitertools/combinations.hpp>
#include <QGraphicsItem>
#include <ranges>
#include <cppitertools/enumerate.hpp>


Doors DoorDetector::detect(const RoboCompLidar3D::TPoints &points, QGraphicsScene *scene)
{

    // Vector para almacenar los picos por diferencia de distancia
    Peaks peaks;

    //Comprobamos el primer punto con el ultimo
    if (!points.empty()) {
        const auto &last_point = points.back();
        const auto &first_point = points.front();

        //Comprobamos el salto entre el final y el principio
        if (std::abs(last_point.distance2d - first_point.distance2d) > 1000) {
            auto min_point = (last_point.distance2d < first_point.distance2d) ? last_point : first_point;
            peaks.emplace_back(Eigen::Vector2f(min_point.x,min_point.y), min_point.distance2d);
        }
    }

    for (const auto &pair: iter::sliding_window(points,2))
    {
        auto point1 = pair[0];
        auto point2 = pair[1];

        if (std::abs(point2.distance2d - point1.distance2d) > 1000)
        {
            // Guardamos el punto más cercano y el valor mínimo.
            auto min_point = (point1.distance2d < point2.distance2d) ? point1 : point2;
            peaks.emplace_back(Eigen::Vector2f(min_point.x,min_point.y), min_point.distance2d);
        }
    }

    // non-maximum suppression of peaks: remove peaks closer than 500mm
    Peaks nms_peaks;
    for (const auto &[p, a] : peaks)
    {
        const bool too_close = std::ranges::any_of(nms_peaks, [&p](const auto &p2) {
            return (p - std::get<0>(p2)).norm() < 500.f;
        });
        if (!too_close)
            nms_peaks.emplace_back(p, a);
    }
    peaks = nms_peaks;

    static std::vector<QGraphicsItem*> draw_points;
    if (scene != nullptr)
    {
        for (const auto &p : draw_points)
        {
            scene->removeItem(p);
            delete p;
        }
        draw_points.clear();

        const QColor color("black");

        for (const auto &[p, distance] : peaks)
        {
            const auto dp = scene->addRect(-25, -25, 50, 50, QPen(color), QBrush(color));
            dp->setPos(p.x(), p.y());
            draw_points.push_back(dp);   // add to the list of points to be deleted next time
        }
    }

    // Detectamos puertas (picos entre 800mm y 1200mm)
    Doors doors;

    for (const auto &combination : peaks | iter::combinations(2))
    {
        auto &[p1, a1] = combination[0];
        auto &[p2, a2] = combination[1];

        float distance = (p1-p2).norm();

        if (distance >= 800.f && distance <= 1200.f ) {
            auto p1_angle = std::atan2(p1.y(), p1.x());
            auto p2_angle = std::atan2(p2.y(), p2.x());
            doors.emplace_back(p1, p1_angle, p2, p2_angle);
        }
    }

    doors_cache = doors;

    return doors;
}

// Method to use the Doors vector to filter out the LiDAR points that como from a room outside the current one
RoboCompLidar3D::TPoints DoorDetector::filter_points(const RoboCompLidar3D::TPoints &points, QGraphicsScene *scene)
{
    const auto doors = detect(points, scene);
    if(doors.empty()) return points;

    auto ccw = [](Eigen::Vector2f a, Eigen::Vector2f b, Eigen::Vector2f c) {
        return (c.y() - a.y()) * (b.x() - a.x()) > (b.y() - a.y()) * (c.x() - a.x());
    };

    RoboCompLidar3D::TPoints filtered;
    for(const auto &point : points)
    {
        auto vector_point = Eigen::Vector2f(point.x,point.y);

        Eigen::Vector2f robot_pos(0.f, 110.f);

        bool is_occluded = std::ranges::any_of(doors, [&](const auto &door) {
            return (ccw(door.p1, door.p2, robot_pos) != ccw(door.p1, door.p2, vector_point)) &&
                   (ccw(door.p1, robot_pos, vector_point) != ccw(door.p2, robot_pos, vector_point));
        });

        if (!is_occluded) filtered.emplace_back(point);
    }
    return filtered;
}

std::expected<Door, std::string> DoorDetector::get_current_door() const
{
    if (doors_cache.empty())
        return std::unexpected<std::string>{"No doors detected"};
    return doors_cache[0];
}
