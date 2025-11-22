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

    // Vector para almacenar los picos por diferencia de distancia
    Peaks peaks;

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

	static std::vector<QGraphicsItem*> draw_points;
    for (const auto &p : draw_points)
    {
        scene->removeItem(p);
        delete p;
    }
    draw_points.clear();

    const QColor color("black");
    const QPen pen(color, 10);

    for (const auto &[p, distance] : peaks)
    {
        const auto dp = scene->addRect(-25, -25, 50, 50, pen);
        dp->setPos(p.x(), p.y());
        draw_points.push_back(dp);   // add to the list of points to be deleted next time
    }

    // Aplicar Non-Maximun Supression (NMS)
    Peaks nms_peaks;
    for (const auto &[p, a] : peaks)
    {
        if (const bool to_close = std::ranges::any_of(nms_peaks, [&p](const auto &p2) {return (p - std::get<0>(p2)).norm() < 500.f;}); not to_close)
            nms_peaks.emplace_back(p, a);
    }
    peaks = nms_peaks;

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

   return doors;
}

RoboCompLidar3D::TPoints DoorDetector::filter_points(const RoboCompLidar3D::TPoints &points, QGraphicsScene *scene)
{
    // 1. Detectar puertas
    const auto doors = detect(points, scene);

    if(doors.empty())
    {
        // qInfo() << "No hay puertas"; // Descomentar si es necesario
        return points;
    }

    RoboCompLidar3D::TPoints filtered_points;
    filtered_points.reserve(points.size()); // Optimización de memoria

    // 2. Iterar punto por punto (bucle principal)
    for(const auto &p : points)
    {
        bool is_behind_any_door = false;

        // Comprobamos este punto contra TODAS las puertas
        for(const auto &d : doors)
        {
            // A. Definir rango angular ordenado
            // Aseguramos que min_a es el menor y max_a el mayor para simplificar comparaciones
            float min_a = std::min(d.p1_angle, d.p2_angle);
            float max_a = std::max(d.p1_angle, d.p2_angle);

            bool inside_sector = false;

            // B. Chequeo de ángulo (Manejo del paso por -PI/+PI)
            // Si la diferencia es mayor a PI, significa que la puerta cruza la línea de corte del lidar (atrás)
            if (max_a - min_a > M_PI)
            {
                // Caso "Wrap around": El sector va de max_a hasta PI y de -PI hasta min_a
                if (p.phi >= max_a || p.phi <= min_a)
                    inside_sector = true;
            }
            else
            {
                // Caso Normal
                if (p.phi >= min_a && p.phi <= max_a)
                    inside_sector = true;
            }

            // C. Chequeo de Distancia
            // Si está dentro del ángulo Y está más lejos que la puerta, lo marcamos para eliminar
            if (inside_sector)
            {
                float dist_to_door = d.center().norm();
                if (p.distance2d > dist_to_door)
                {
                    is_behind_any_door = true;
                    break; // No hace falta mirar más puertas, este punto se descarta
                }
            }
        }

        // 3. Si el punto sobrevive a todas las puertas, lo guardamos
        if (!is_behind_any_door)
        {
            filtered_points.push_back(p);
        }
    }

    return filtered_points;
}