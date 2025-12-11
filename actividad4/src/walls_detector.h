//
// Created by usuario on 11/12/25.
//

#ifndef LOCALISER_WALL_DETECTOR_H
#define LOCALISER_WALL_DETECTOR_H

#include "common_types.h"

class walls_detector {
private:
    std::vector<Wall> walls;
public:
    walls_detector() = default;
    ~walls_detector() = default;

    void compute_walls(Corners cs);

    Wall closest_wall_to_point(const Eigen::Vector2f &p);

    Eigen::Vector2f project_point(const Eigen::Vector2f &p, int wall_id);
};


#endif //LOCALISER_WALL_DETECTOR_H