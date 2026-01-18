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
#include "nominal_room.h"
#include "door_detector.h"
#include "image_processor.h"
#include <expected>
#include "pointcloud_center_estimator.h"
#include "door_crossing_tracker.h"
/**
 * \brief Class SpecificWorker implements the core functionality of the component.
 */

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


private:

	struct Params
	{
		const float ROBOT_WIDTH = 460;  // mm
		const float ROBOT_LENGTH = 480;  // mm
		const float ROBOT_SECTION = ROBOT_LENGTH/2 + 75;
		const float WIDTH_DISTANCES = 40;
		const float MAX_ADV_SPEED = 1000; // mm/s
		const float MAX_ROT_SPEED = 1; // rad/s
		const float MAX_SIDE_SPEED = 50; // mm/s
		const float MAX_TRANSLATION = 500; // mm/s
		const float MAX_ROTATION = 0.2;
		const float STOP_THRESHOLD = 700; // mm
		const float ADVANCE_THRESHOLD = ROBOT_WIDTH * 3; // mm
		const float LIDAR_FRONT_SECTION = 0.2; // rads, aprox 12 degrees
		// wall
		const float LIDAR_RIGHT_SIDE_SECTION = M_PI/3; // rads, 90 degrees
		const float LIDAR_LEFT_SIDE_SECTION = -M_PI/3; // rads, 90 degrees
		const float WALL_MIN_DISTANCE = ROBOT_WIDTH*1.2;
		// match error correction
		const float MATCH_ERROR_SIGMA = 150.f; // mm
		const float DOOR_REACHED_DIST = 50.f;
		const std::string LIDAR_NAME_LOW = "bpearl";
		const std::string LIDAR_NAME_HIGH = "helios";
		const QRectF GRID_MAX_DIM{-5000, 2500, 10000, -5000};

		// relocalization
		const float RELOCAL_CENTER_EPS = 50.f;    // mm: stop when |mean| < eps
		const float RELOCAL_KP = 0.002f;           // gain to convert mean (mm) -> speed (magnitude)
		const float RELOCAL_MAX_ADV = 300.f;       // mm/s cap while re-centering
		const float RELOCAL_MAX_SIDE = 300.f;      // mm/s cap while re-centering
		const float RELOCAL_ROT_SPEED = 0.3f;     // rad/s while aligning
		const float RELOCAL_DELTA = 3.0f * M_PI/180.f; // small probe angle in radians
		const float RELOCAL_MATCH_MAX_DIST = 2000.f;   // mm for Hungarian gating
		const float RELOCAL_DONE_COST = 500.f;
		const float RELOCAL_DONE_MATCH_MAX_ERROR = 1000.f;
		const float RELOCAL_MAX_ORIENTED_ERROR = 0.03f; // ~5 grados, error permitido para encarar la puerta
		const float DIST_TARGET_BEFORE_DOOR = 1000.f;

		const int BLIND_CROSS_TIME = 3000; // ms
		const float BLIND_ADV_VELOCITY = 500; // mm/s
	};
	Params params;

	// viewer
	AbstractGraphicViewer *viewer, *viewer_room;
	QGraphicsPolygonItem *robot_draw, *robot_room_draw;

	// robot
	Eigen::Affine2f robot_pose;

	// rooms
	std::vector<NominalRoom> nominal_rooms{ NominalRoom{5500.f, 4000.f}, NominalRoom{8000.f, 4000.f}};
	rc::Room_Detector room_detector;
	rc::Hungarian hungarian;
	rc::PointcloudCenterEstimator center_estimator;
	std::optional<Eigen::Vector2d> estimated_center;
	int current_room = -1;

	// state machine
	enum class STATE {GOTO_DOOR, ORIENT_TO_DOOR, GOTO_ROOM_CENTER, TURN, IDLE, CROSS_DOOR, LOCALISE};
	inline const char* to_string(const STATE s) const
	{
		switch(s) {
			case STATE::IDLE:               return "IDLE";
			case STATE::GOTO_DOOR:          return "GOTO_DOOR";
			case STATE::TURN:               return "TURN";
			case STATE::ORIENT_TO_DOOR:     return "ORIENT_TO_DOOR";
			case STATE::GOTO_ROOM_CENTER:   return "GOTO_ROOM_CENTER";
			case STATE::CROSS_DOOR:         return "CROSS_DOOR";
			case STATE::LOCALISE:			return "LOCALISE";
			default:                        return "UNKNOWN";
		}
	}
	STATE state = STATE::IDLE;
	using RetVal = std::tuple<STATE, float, float>;

	RetVal goto_door(const RoboCompLidar3D::TPoints &points);
	RetVal turn(const RoboCompLidar3D::TPoints &points);
	RetVal orient_to_door(const RoboCompLidar3D::TPoints &points);
	RetVal goto_room_center();
	RetVal cross_door(const RoboCompLidar3D::TPoints &points);
	RetVal update_pose(const Corners &corners, const Match &match);
	RetVal localise(const RoboCompLidar3D::TPoints &points, QGraphicsScene *scene);
	RetVal process_state(const RoboCompLidar3D::TPoints &points);

	// draw
	std::vector<QGraphicsItemGroup*> to_draw;
	void draw_lidar(const auto &points, QGraphicsScene *scene);
	void draw_room_center(QGraphicsScene *scene);
	void update_robot_position();
	void draw_current_room(QGraphicsScene *scene);
	void draw_controller();

	// aux
	RoboCompLidar3D::TPoints read_data();

	// plotter
	std::unique_ptr<TimeSeriesPlotter> time_series_plotter;
	int match_error_graph;

	// doors
	DoorDetector door_detector;
	int current_door = -1;

	// image processor
	rc::ImageProcessor image_processor;

	// timing
	std::chrono::time_point<std::chrono::high_resolution_clock> last_time = std::chrono::high_resolution_clock::now();

	// relocalization
	bool relocal_centered = false;
	bool localised = false;

	//new door crossing detector
	DoorCrossing door_crossing; //used the file in beta-robotica-class

	[[nodiscard]]unsigned long choose_next_door() const;


	std::optional<std::pair<Eigen::Affine2f, float>> update_robot_pose(const RoboCompLidar3D::TPoints &data, bool transform_corners);
	void move_robot(float adv, float rot) const;

	static Eigen::Vector3d solve_pose(const Corners &corners, const Match &match);
	std::tuple<float, float> robot_controller(const Eigen::Vector2f &target) const;

	/**
     * \brief Flag indicating whether startup checks are enabled.
     */
	bool startup_check_flag;

signals:
	//void customSignal();
};


#endif