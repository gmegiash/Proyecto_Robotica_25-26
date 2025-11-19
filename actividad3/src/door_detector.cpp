//
// Created by pbustos on 11/11/25.
//

#include "door_detector.h"
#include <cppitertools/sliding_window.hpp>
#include <cppitertools/combinations.hpp>
#include <QGraphicsItem>
#include <cmath>

#include "specificworker.h"


Doors DoorDetector::detect(const RoboCompLidar3D::TPoints &points, QGraphicsScene *scene)
{

    // Vector para almacenar los picos
    Peaks peaks;

    auto pairs = points | iter::sliding_window(2);
    for (const auto &pair: pairs)
    {
        auto point1 = pair[0];
        auto point2 = pair[1];

        if ((point2.distance2d - point1.distance2d) < 1000)
        {
            continue;
        }

        auto min_distance = (point1 < point2) ? point1.distance2d : point2.distance2d;
        peaks.emplace_back(pair, min_distance);
    }

    // Aplicar Non-Maximun Supression (NMS)
    Peaks nms_peaks;
    for (const auto &[p, a] : peaks)
    {
        if (const bool to_close = std::ranges::any_of(nms_peaks, [&p](const auto &p2) {return (p - std::get<0>(p2)).norm() < 500.f;}); not to_close)
            nms_peaks.emplace_back(p, a);

        peaks = nms_peaks;
    }

    // Detectamos puertas (picos entre 800mm y 1200mm)
    Doors doors;

    for (const auto &[p1, _] :  iter::combinations(peaks, 2);


   return Doors();
}

// Method to use the Doors vector to filter out the LiDAR points that como from a room outside the current one
RoboCompLidar3D::TPoints DoorDetector::filter_points(const RoboCompLidar3D::TPoints &points, QGraphicsScene *scene)
{
    const auto doors = detect(points, scene);
    if(doors.empty()) return points;

    // for each door, check if the distance from the robot to each lidar point is smaller than the distance from the robot to the door
    RoboCompLidar3D::TPoints filtered;
    for(const auto &d : doors)
    {
        const float dist_to_door = d.center().norm();
        // Check if the angular range wraps around the -π/+π boundary
        const bool angle_wraps = d.p2_angle < d.p1_angle;
        for(const auto &p : points)
        {
            // Determine if point is within the door's angular range
            bool point_in_angular_range;
            if (angle_wraps)
            {
                // If the range wraps around, point is in range if it's > p1_angle OR < p2_angle
                point_in_angular_range = (p.phi > d.p1_angle) or (p.phi < d.p2_angle);
            }
            else
            {
                // Normal case: point is in range if it's between p1_angle and p2_angle
                point_in_angular_range = (p.phi > d.p1_angle) and (p.phi < d.p2_angle);
            }

            // Filter out points that are through the door (in angular range and farther than door)
            if(point_in_angular_range and p.distance2d >= dist_to_door)
                continue;

            //qInfo() << __FUNCTION__ << "Point angle: " << p.phi << " Door angles: " << d.p1_angle << ", " << d.p2_angle << " Point distance: " << p.distance2d << " Door distance: " << dist_to_door;
            filtered.emplace_back(p);
        }
    }
    return filtered;
}
