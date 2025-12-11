//
// Created by usuario on 11/12/25.
//

#include "walls_detector.h"

void walls_detector::compute_walls(Corners cs)
{
    cs.push_back(cs[0]);
    for (const auto &[i,c]: cs | iter::sliding_window(2) | iter::enumerate)
    {
        Doors doors;
        Corners corners;
        const auto c1 = c[0];
        const auto c2 = c[1];
        corners.emplace_back(c1);
        corners.emplace_back(c2);

        const auto &c1_QPoint = get<0>(c[0]);
        const auto &c2_QPoint = get<0>(c[1]);

        const auto &c1_point = Eigen::Vector2f(c1_QPoint.x(), c1_QPoint.y());
        const auto &c2_point = Eigen::Vector2f(c2_QPoint.x(), c2_QPoint.y());

        const auto r = Eigen::ParametrizedLine<float, 2>::Through(c1_point, c2_point);

        walls.emplace_back(i, r, doors, corners);
    }
}

Wall walls_detector::closest_wall_to_point(const Eigen::Vector2f &p)
{
    auto m = std::ranges::min_element(walls, {}, [&p](const auto &wall) {
        auto wall_line = get<1>(wall);

        return wall_line.distance(p);
    });
    return *m;
}

Eigen::Vector2f walls_detector::project_point(const Eigen::Vector2f &p, int wall_id) {
    auto wall = this->walls.at(wall_id);
    return get<1>(wall).projection(p);
}
