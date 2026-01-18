//
// Created by pbustos on 11/11/25.
//

#ifndef DOORDETECTOR_H
#define DOORDETECTOR_H

#include "common_types.h"
#include <Lidar3D.h>
#include <QGraphicsScene>
#include <expected>

class DoorDetector
{
public:
    DoorDetector() = default;
    ~DoorDetector() = default;

    Doors detect(const RoboCompLidar3D::TPoints &points,
                       const Eigen::Affine2d &robot_pose = Eigen::Affine2d::Identity(),
                       bool localised = false,
                       QGraphicsScene *robot_scene = nullptr,
                       QGraphicsScene *scene_room = nullptr);
    RoboCompLidar3D::TPoints filter_points(const RoboCompLidar3D::TPoints &points);
    [[nodiscard]] Doors doors() const { return doors_cache; };
    [[nodiscard]] std::expected<Door, std::string> get_current_door() const;

private:
    Doors doors_cache;
    const float min_door_width = 600.f;  // mm
    const float max_door_width = 1200.f;
    const float min_peak_distance = 500.f;

    static bool is_ccw(const Eigen::Vector2f &a, const Eigen::Vector2f &b, const Eigen::Vector2f &c);
    static bool segment_intersects_door(const Door &door, const Eigen::Vector2f &start, const Eigen::Vector2f &end);
};


#endif //DOORDETECTOR_H