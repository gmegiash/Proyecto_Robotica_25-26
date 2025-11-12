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
#include <doublebuffer/DoubleBuffer.h>
#include "common_types.h"
#include "hungarian.h"
#include "ransac_line_detector.h"
#include "room_detector.h"
#include "time_series_plotter.h"
#include "door_detector.h"
#include "image_processor.h"
#include <expected>

/**
 * \brief Class SpecificWorker implements the core functionality of the component.
 */

//	Constantes de distancias
#define Params_ROBOT_LENGTH 400
#define ROBOT_SECTION (ROBOT_LENGTH/2 + 75)
#define WIDTH_DISTANCES 40
#define MAX_ADV 800.0f      // velocidad avance
#define MAX_ROT 0.6125f        // velocidad rotación
#define OBSTACLE_DIST 1000.0f  // mm

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
enum class STATE {TURN, IDLE, LOCALISE, GOTO_DOOR, ORIENT_TO_DOOR, GOTO_ROOM_CENTER, CROSS_DOOR, OFF};
enum class RotationDirection { RIGHT, NONE, LEFT };
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

	struct Params
	{
		float ROBOT_WIDTH = 460;  // mm
		float ROBOT_LENGTH = 480;  // mm
		float MAX_ADV_SPEED = 1000; // mm/s
		float MAX_ROT_SPEED = 1; // rad/s
		float MAX_SIDE_SPEED = 50; // mm/s
		float MAX_TRANSLATION = 500; // mm/s
		float MAX_ROTATION = 0.2;
		float STOP_THRESHOLD = 700; // mm
		float ADVANCE_THRESHOLD = ROBOT_WIDTH * 3; // mm
		float LIDAR_FRONT_SECTION = 0.2; // rads, aprox 12 degrees
		// wall
		float LIDAR_RIGHT_SIDE_SECTION = M_PI/3; // rads, 90 degrees
		float LIDAR_LEFT_SIDE_SECTION = -M_PI/3; // rads, 90 degrees
		float WALL_MIN_DISTANCE = ROBOT_WIDTH*1.2;
		// match error correction
		float MATCH_ERROR_SIGMA = 150.f; // mm
		float DOOR_REACHED_DIST = 300.f;
		std::string LIDAR_NAME_LOW = "bpearl";
		std::string LIDAR_NAME_HIGH = "helios";
		QRectF GRID_MAX_DIM{-5000, 2500, 10000, -5000};

		// relocalization
		float RELOCAL_CENTER_EPS = 300.f;    // mm: stop when |mean| < eps
		float RELOCAL_KP = 0.002f;           // gain to convert mean (mm) -> speed (magnitude)
		float RELOCAL_MAX_ADV = 300.f;       // mm/s cap while re-centering
		float RELOCAL_MAX_SIDE = 300.f;      // mm/s cap while re-centering
		float RELOCAL_ROT_SPEED = 0.3f;     // rad/s while aligning
		float RELOCAL_DELTA = 5.0f * M_PI/180.f; // small probe angle in radians
		float RELOCAL_MATCH_MAX_DIST = 2000.f;   // mm for Hungarian gating
		float RELOCAL_DONE_COST = 500.f;
		float RELOCAL_DONE_MATCH_MAX_ERROR = 1000.f;
	};
	Params params;

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

	RoboCompLidar3D::TPoints filter_isolated_points(const RoboCompLidar3D::TPoints &points, float d);
	void read_data();

	// graphics
	QRectF dimensions;
		// robot perspective
		AbstractGraphicViewer *viewer;
		QGraphicsPolygonItem *robot_polygon;
		// nominal perspective
		AbstractGraphicViewer *viewer_room;
		QGraphicsPolygonItem *room_draw_robot;


	void draw_lidar(const auto &points);
	void draw_collisions();
	void update_windows_values();
	void update_robot_position();

	// Distances
	float front_distance;
	float right_distance;
	float left_distance;

	double right_angle;
	double left_angle;

	void calculateDistances(const RoboCompLidar3D::TPoints &points);

	// Movements
	State state = State::OFF;
	STATE state2 = STATE::LOCALISE;
	using RetVal = std::tuple<STATE, float, float>;

	double rotation = 0;
	double advance = 0;
	RotationDirection rotation_direction = RotationDirection::NONE;

	void set_robot_speed(float advx, float advy, float rot);
	RotationDirection evaluate_rotation_direction(float rot);

	void doStateMachine();
	void doState2Machine();

	void forward_state();
	void turn_state();
	void followWall_state();
	void spiral_state();

	//aux
	std::expected<int, std::string> closest_lidar_index_to_given_angle(const auto &points, float angle);
	RoboCompLidar3D::TPoints filter_same_phi(const RoboCompLidar3D::TPoints &points);
	void print_match(const Match &match, const float error =1.f) const;

	RetVal goto_door(const RoboCompLidar3D::TPoints &points);
	RetVal orient_to_door(const RoboCompLidar3D::TPoints &points);
	RetVal cross_door(const RoboCompLidar3D::TPoints &points);
	RetVal localise(const Match &match);
	RetVal goto_room_center(const RoboCompLidar3D::TPoints &points);
	RetVal update_pose(const Corners &corners, const Match &match);
	RetVal turn(const Corners &corners);
	RetVal process_state(const RoboCompLidar3D::TPoints &data, const Corners &corners, const Match &match, AbstractGraphicViewer *viewer);

	// DoubleBuffer for velocity commands
	DoubleBuffer<std::tuple<float, float, float, long>, std::tuple<float, float, float, long>> commands_buffer;
	std::tuple<float, float, float, long> last_velocities{0.f, 0.f, 0.f, 0.f};

	// plotter
	std::unique_ptr<TimeSeriesPlotter> time_series_plotter;
	int match_error_graph;

	// doors
	DoorDetector door_detector;

	// image processor
	rc::ImageProcessor image_processor;

	// timing
	std::chrono::time_point<std::chrono::high_resolution_clock> last_time = std::chrono::high_resolution_clock::now();

	//relocalization
	bool relocal_centered = false;
	bool localised = false;

	bool update_robot_pose(const Corners &corners, const Match &match);
	void move_robot(float adv, float rot, float max_match_error);
	Eigen::Vector3d solve_pose(const Corners &corners, const Match &match);
	void predict_robot_pose();
	std::tuple<float, float> robot_controller(const Eigen::Vector2f &target);

	State stateRandomizer();

	/**
     * \brief Flag indicating whether startup checks are enabled.
     */
	bool startup_check_flag;

signals:
	//void customSignal();
};


#endif
