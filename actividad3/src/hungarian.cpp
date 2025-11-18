//
// Created by robolab on 12/5/24.
//
#include "hungarian.h"
#include <ranges>

namespace rc
{
    Match Hungarian::match(const Corners &measurement_corners, const Corners &nominal_corners, double max_corner_diff)
    {
        // create cost matrix for Hungarian //
        std::vector<double> costs;
        for (const auto &[c, _, __]: measurement_corners)    // rows
            for (const auto &[rc, _, __]: nominal_corners)   // cols
                costs.emplace_back(euclidean_distance(c, rc) / (hypot(c.x(),c.y()) / 1000 ));
        const auto rows = measurement_corners.size();
        const auto cols = nominal_corners.size();

        // if costs is empty, return empty matches
        if (costs.empty())
            return {};

        // print costs for debugging
        for (int i = 0; i < rows; i++)
        {
            const auto &[c, _, __] = measurement_corners[i];
            std::cout << "[" << c.x() << "," << c.y() << "]: ";
            for (int j = 0; j < cols; j++)
            {
                std::cout << costs[i*rows + j] << ", ";
            }
            std::cout << std::endl;
        }
        std::cout << "---------------------------------" << std::endl;

        // lambda to access the costs matrix
        Match match;   //  measurement - nominal
        auto cost = [costs, cols](const unsigned r, const unsigned c) { return costs[r * cols + c]; };
        const auto matching = munkres_algorithm<double>(rows, cols, cost);
        for (const auto &[r, c]: matching)
            if (cost(r, c) < max_corner_diff)
                match.emplace_back(std::make_tuple(measurement_corners[r], nominal_corners[c], euclidean_distance(nominal_corners[c], measurement_corners[r])));
        return match;
    }


     double Hungarian::euclidean_distance(const QPointF &p1, const QPointF &p2)
     {
            return sqrt((p1.x()-p2.x())*(p1.x()-p2.x())+(p1.y()-p2.y())*(p1.y()-p2.y()));
     }
    double Hungarian::euclidean_distance(const Corner &c1, const Corner &c2)
    {
        auto &[p1, _, __] = c1;
        auto &[p2, ___, ____] = c2;
        return sqrt((p1.x()-p2.x())*(p1.x()-p2.x())+(p1.y()-p2.y())*(p1.y()-p2.y()));
    }
}

