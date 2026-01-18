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


Doors DoorDetector::detect(const RoboCompLidar3D::TPoints &points,
                           const Eigen::Affine2d &robot_pose,
                           bool localised,
                           QGraphicsScene *robot_scene,
                           QGraphicsScene *scene_room)
{
    if(points.empty()) return {};

    // get the peaks
    Peaks peaks;
    for (const auto &p : iter::sliding_window(points, 2))
    {
        const auto &p1 = p[0]; const auto &p2 = p[1];
        const float d1 = p1.distance2d; const float d2 = p2.distance2d;
        if (const float dd_da1 = abs(d2 - d1); dd_da1 >min_peak_distance)
        {
            const auto m = std::ranges::min_element(p, [](auto &pa, auto &pb){return pa.distance2d < pb.distance2d;});
            peaks.emplace_back(Eigen::Vector2f(m->x, m->y), m->phi);
        }
    }

    // non-maximum suppression of peaks: remove peaks closer than 500mm
    Peaks nms_peaks;
    for (const auto &[p, a] : peaks)
        if (const bool too_close = std::ranges::any_of(nms_peaks, [&p](const auto &p2) { return (p - std::get<0>(p2)).norm() < 500.f; }); not too_close)
            nms_peaks.emplace_back(p, a);
    peaks = nms_peaks;

    // find doors as pairs of peaks separated by a gap < 1200mm and > 800mm
    Doors doors;
    for(const auto &p : peaks | iter::combinations(2))
    {
        const auto &[p0,a0] = p[0]; const auto &[p1, a1] = p[1];
        const float gap = (p1-p0).norm();
        //qInfo() << "Gap: " << gap;
        if(gap < max_door_width and gap > min_door_width)
            doors.emplace_back(p0, a0, p1, a1);
    }
    //qInfo() << __FUNCTION__ << "Peaks found: " << peaks.size() << "Doors found: " << doors.size();
    doors_cache = doors;
    return doors;
}

// Method to use the Doors vector to filter out the LiDAR points that como from a room outside the current one
RoboCompLidar3D::TPoints DoorDetector::filter_points(const RoboCompLidar3D::TPoints &points)
{
    const auto doors = detect(points);
    if(doors.empty()) return points;

    const Eigen::Vector2f robot_front(0.f, 185.f);
    const Eigen::Vector2f robot_back(0.f, -185.f);

    auto is_blocked = [&](const Eigen::Vector2f &target, const Eigen::Vector2f &origin) {
        return std::ranges::any_of(doors, [&](const Door &door) {
            return segment_intersects_door(door, origin, target);
        });
    };

    RoboCompLidar3D::TPoints filtered;
    for(const auto &point : points)
    {
        auto target = Eigen::Vector2f(point.x,point.y);

        if (is_blocked(target, robot_front)) continue;
        if (is_blocked(target, robot_back))  continue;

        filtered.emplace_back(point);
    }
    return filtered;
}

std::expected<Door, std::string> DoorDetector::get_current_door() const
{
    if (doors_cache.empty())
        return std::unexpected<std::string>{"No doors detected"};
    return doors_cache[0];
}

bool DoorDetector::is_ccw(const Eigen::Vector2f &a, const Eigen::Vector2f &b, const Eigen::Vector2f &c)
{
    return (c.y() - a.y()) * (b.x() - a.x()) > (b.y() - a.y()) * (c.x() - a.x());
}

bool DoorDetector::segment_intersects_door(const Door &door, const Eigen::Vector2f &start, const Eigen::Vector2f &end)
{
    // Un segmento (start->end) cruza la puerta si:
    // 1. Los puntos start y end están en lados opuestos de la línea de la puerta.
    // 2. Los puntos de la puerta (p1, p2) están en lados opuestos del segmento start->end.

    return (is_ccw(door.p1, door.p2, start) != is_ccw(door.p1, door.p2, end)) &&
           (is_ccw(door.p1, start, end)     != is_ccw(door.p2, start, end));
}