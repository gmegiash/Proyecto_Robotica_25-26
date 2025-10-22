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
#include <cppitertools/enumerate.hpp>

/**
 * \brief Class SpecificWorker implements the core functionality of the component.
 */

enum class State { FORWARD, TURN, FOLLOW_WALL };
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

	void new_target_slot(QPointF);


private:
	// graphics
	QRectF dimensions;
	AbstractGraphicViewer *viewer;
	const int ROBOT_LENGTH = 400;
	QGraphicsPolygonItem *robot_polygon;

	void draw_lidar(const auto &points, QGraphicsScene* scene);
	void draw_collisions(QGraphicsScene* scene);
	void update_windows_values(QGraphicsScene* scene);
	void update_robot_position();

	// Distances
	float front_distance;
	float right_distance;
	float left_distance;

	double right_angle;
	double left_angle;

	void calculateDistancesOLD(const RoboCompLidar3D::TPoints &points);
	void calculateDistances(const RoboCompLidar3D::TPoints &points);

	// States
	enum class State { FORWARD, TURN, FOLLOW_WALL };
	State state = State::FORWARD;

	void doStateMachine();

	void forwardState();
	void turnState();
	void follow_WallState();



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
