/*
 *    Copyright (C) 2025 by YOUR NAME HERE
 *
 *    This file is part of RoboComp
 *
 *    RoboComp is free software: you can redistribute it and/or modify
 *    it under the terms of the GNU General Public License as published by
 *    the Free Software Foundation, either version 3 of the License, or
 *    (at your option) any later version.
 *
 *    RoboComp is distributed in the hope that it will be useful,
 *    but WITHOUT ANY WARRANTY; without even the implied warranty of
 *    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *    GNU General Public License for more details.
 *
 *    You should have received a copy of the GNU General Public License
 *    along with RoboComp.  If not, see <http://www.gnu.org/licenses/>.
 */

/**
	\brief
	@author authorname
*/



#ifndef SPECIFICWORKER_H
#define SPECIFICWORKER_H


// If you want to reduce the period automatically due to lack of use, you must uncomment the following line
//#define HIBERNATION_ENABLED

#include <genericworker.h>
#include <abstract_graphic_viewer/abstract_graphic_viewer.h>
#ifdef emit
#undef emit
#endif
#include <cppitertools/enumerate.hpp>
#include <execution>
#include "common_types.h"
#include "hungarian.h"
#include "ransac_line_detector.h"
#include "room_detector.h"

/**
 * \brief Class SpecificWorker implements the core functionality of the component.
 */

//	Constantes de distancias
#define width_distances 40
#define MAX_ADV 1000.0f      // velocidad avance
#define MAX_ROT 1.0f        // velocidad rotación
#define OBSTACLE_DIST 750.0f  // mm
#define WALL_DIST 700.0f      // mm
#define INIT_ROTATION 0.0f
#define INIT_VELOCITY  1000.0f

struct NominalRoom
{
	float width; //  mm
	float length;
	Corners corners;
	QRectF rect{-5000, -2500, 10000,5000};
	explicit NominalRoom(const float width_=10000.f, const float length_=5000.f, Corners  corners_ = {}) : width(width_), length(length_), corners(std::move(corners_)){};
	Corners transform_corners_to(const Eigen::Affine2d &transform) const  // for room to robot pass the inverse of robot_pose
	{
		Corners transformed_corners;
		for(const auto &[p, _, __] : corners)
		{
			auto ep = Eigen::Vector2d{p.x(), p.y()};
			Eigen::Vector2d tp = transform * ep;
			transformed_corners.emplace_back(QPointF{static_cast<float>(tp.x()), static_cast<float>(tp.y())}, 0.f, 0.f);
		}
		return transformed_corners;
	}
};

enum class State { FORWARD, TURN, FOLLOW_WALL, SPIRAL, OFF };
class SpecificWorker : public GenericWorker
{
Q_OBJECT
public:
    /**
     * \brief Constructor for SpecificWorker.
     * \param configLoader Configuration loader for the component.
     * \param tprx Tuple of proxies required for the component.
     * \param startup_check Indicates whether to perform startup checks.
     */
	SpecificWorker(const ConfigLoader& configLoader, TuplePrx tprx, bool startup_check);

	/**
     * \brief Destructor for SpecificWorker.
     */
	~SpecificWorker();

public slots:

	/**
	 * \brief Initializes the worker one time.
	 */
	void initialize();

	/**
	 * \brief Main compute loop of the worker.
	 */
	void compute();

	/**
	 * \brief Handles the emergency state loop.
	 */
	void emergency();

	/**
	 * \brief Restores the component from an emergency state.
	 */
	void restore();

    /**
     * \brief Performs startup checks for the component.
     * \return An integer representing the result of the checks.
     */
	int startup_check();

	void doStartStop();

	void new_target_slot(QPointF);


private:
	// Data
	NominalRoom nominal_room{
		10000.f, 5000.f,
		{
					{QPointF{-5000.f, -2500.f}, 0.f, 0.f},
					{QPointF{5000.f, -2500.f}, 0.f, 0.f},
					{QPointF{5000.f, 2500.f}, 0.f, 0.f},
					{QPointF{-5000.f, 2500.f}, 0.f, 0.f}
		}
	};
	Eigen::Affine2d robot_pose;
	rc::Room_Detector room_detector;
	rc::Hungarian hungarian;

	void read_data();


	// graphics
	QRectF dimensions;
	const int ROBOT_LENGTH = 400;
		// robot perspective
		AbstractGraphicViewer *viewer;
		QGraphicsPolygonItem *robot_polygon;
		// nominal perspective
		AbstractGraphicViewer *viewer_room;
		QGraphicsPolygonItem *room_draw_robot;


	void draw_lidar(const auto &points, QGraphicsScene* scene);
	void draw_collisions(QGraphicsScene* scene);
	void update_windows_values();
	void update_robot_position();

	// Distances
	float front_distance;
	float right_distance;
	float left_distance;

	double right_angle;
	double left_angle;

	void calculateDistances(const RoboCompLidar3D::TPoints &points);

	// States
	//enum class State { FORWARD, TURN, FOLLOW_WALL, SPIRAL };
	State state = State::OFF;

	double current_rotation = INIT_ROTATION;
	double current_velocity = INIT_VELOCITY;
	bool right_turn = true;

	void doStateMachine();

	void forwardState();
	void turnState();
	void follow_WallState();
	void spiralState();

	State stateRandomizer();



	std::optional<RoboCompLidar3D::TPoints> filter_min_distance_cppintertools(const RoboCompLidar3D::TPoints &points);
	RoboCompLidar3D::TPoints filter_isolated_points(const RoboCompLidar3D::TPoints &points, float d);

	/**
     * \brief Flag indicating whether startup checks are enabled.
     */
	bool startup_check_flag;

signals:
	//void customSignal();
};


#endif
