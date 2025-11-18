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
#include "specificworker.h"
#include "specificworker.h"
#include <cmath>
#include <ranges>
#include <cppitertools/groupby.hpp>

SpecificWorker::SpecificWorker(const ConfigLoader& configLoader, TuplePrx tprx, bool startup_check) : GenericWorker(configLoader, tprx)
{
	this->startup_check_flag = startup_check;
	if(this->startup_check_flag)
	{
		this->startup_check();
	}
	else
	{
		#ifdef HIBERNATION_ENABLED
			hibernationChecker.start(500);
		#endif

		// Example statemachine:
		/***
		//Your definition for the statesmachine (if you dont want use a execute function, use nullptr)
		states["CustomState"] = std::make_unique<GRAFCETStep>("CustomState", period,
															std::bind(&SpecificWorker::customLoop, this),  // Cyclic function
															std::bind(&SpecificWorker::customEnter, this), // On-enter function
															std::bind(&SpecificWorker::customExit, this)); // On-exit function

		//Add your definition of transitions (addTransition(originOfSignal, signal, dstState))
		states["CustomState"]->addTransition(states["CustomState"].get(), SIGNAL(entered()), states["OtherState"].get());
		states["Compute"]->addTransition(this, SIGNAL(customSignal()), states["CustomState"].get()); //Define your signal in the .h file under the "Signals" section.

		//Add your custom state
		statemachine.addState(states["CustomState"].get());
		***/

		statemachine.setChildMode(QState::ExclusiveStates);
		statemachine.start();

		auto error = statemachine.errorString();
		if (error.length() > 0){
			qWarning() << error;
			throw error;
		}
	}
}

SpecificWorker::~SpecificWorker()
{
	std::cout << "Destroying SpecificWorker" << std::endl;
}


void SpecificWorker::initialize()
{
    std::cout << "initialize worker" << std::endl;

	if (this->startup_check_flag)
	{
		this->startup_check();
	} else
	{

		this->setupUi(this);

	}

	connect(pushButton_startstop, SIGNAL(clicked()), this, SLOT(doStartStop()));
	connect(viewer, &AbstractGraphicViewer::new_mouse_coordinates, this, &SpecificWorker::new_target_slot);

}


void SpecificWorker::compute()
{
	auto data = read_data();

	// compute corners
	const auto &[corners, lines] = room_detector.compute_corners(data, &viewer->scene);
	const auto center_opt = room_detector.estimate_center_from_walls(lines);
	draw_lidar(data, center_opt);
	// match corners  transforming first nominal corners to robot's frame
	const auto match = hungarian.match(corners,
											  nominal_rooms[0].transform_corners_to(robot_pose.inverse()));


	// compute max of  match error
	float max_match_error = 99999.f;
	if (not match.empty())
	{
		const auto max_error_iter = std::ranges::max_element(match, [](const auto &a, const auto &b)
			{ return std::get<2>(a) < std::get<2>(b); });
		max_match_error = static_cast<float>(std::get<2>(*max_error_iter));
		time_series_plotter->addDataPoint(match_error_graph,max_match_error);
		//print_match(match, max_match_error); //debugging
	}


	// update robot pose
	if (localised)
		update_robot_pose(corners, match);


	// Process state machine
	RetVal ret_val = process_state(data, corners, match, viewer);
	auto [st, adv, rot] = ret_val;
	state = st;


	// Send movements commands to the robot constrained by the match_error
	//qInfo() << __FUNCTION__ << "Adv: " << adv << " Rot: " << rot;
	move_robot(adv, rot, max_match_error);


	// draw robot in viewer
	robot_room_draw->setPos(robot_pose.translation().x(), robot_pose.translation().y());
	const double angle = qRadiansToDegrees(std::atan2(robot_pose.rotation()(1, 0), robot_pose.rotation()(0, 0)));
	robot_room_draw->setRotation(angle);
}

RoboCompLidar3D::TPoints SpecificWorker::read_data()
{
	auto data = lidar3d_proxy->getLidarDataWithThreshold2d("helios", 12000, 1);

	return door_detector.filter_points(data.points, &viewer->scene);
}



RoboCompLidar3D::TPoints SpecificWorker::filter_isolated_points(const RoboCompLidar3D::TPoints &points, float d) // set to 200mm
{
	if (points.empty()) return {};

	const float d_squared = d * d;  // Avoid sqrt by comparing squared distances
	std::vector<bool> hasNeighbor(points.size(), false);

	// Create index vector for parallel iteration
	std::vector<size_t> indices(points.size());
	std::iota(indices.begin(), indices.end(), size_t{0});

	// Parallelize outer loop - each thread checks one point
	std::for_each(std::execution::par, indices.begin(), indices.end(), [&](size_t i)
		{
			const auto& p1 = points[i];
			// Sequential inner loop (avoid nested parallelism)
			for (auto &&[j,p2] : iter::enumerate(points))
			{
				if (i == j) continue;
				const float dx = p1.x - p2.x;
				const float dy = p1.y - p2.y;
				if (dx * dx + dy * dy <= d_squared)
				{
					hasNeighbor[i] = true;
					break;
				}
			}
	});

	// Collect results
	std::vector<RoboCompLidar3D::TPoint> result;
	result.reserve(points.size());
	for (auto &&[i, p] : iter::enumerate(points))
		if (hasNeighbor[i])
			result.push_back(points[i]);
	return result;
}

void SpecificWorker::draw_lidar(const auto &points,  std::optional<Eigen::Vector2d> center_opt)
{
	static std::vector<QGraphicsItem*> draw_points;
	for (const auto &p : draw_points)
	{
		viewer->scene.removeItem(p);
		delete p;
	}
	draw_points.clear();

	const QColor color("LightGreen");
	const QPen pen(color, 10);
	//const QBrush brush(color, Qt::SolidPattern);
	for (const auto &p : points)
	{
		const auto dp = viewer->scene.addRect(-25, -25, 50, 50, pen);
		dp->setPos(p.x, p.y);
		draw_points.push_back(dp);   // add to the list of points to be deleted next time
	}

	auto center_values = center_opt.value();
	const auto center = viewer->scene.addEllipse(-100, -100, 200, 200, QPen(QColor("red")), QBrush(QColor("red")));
	center->setPos(center_values.x(), center_values.y());
	draw_points.push_back(center);
}

void SpecificWorker::update_robot_position()
{
	try
	{
		robot_room_draw->setPos(robot_pose.translation().x(), robot_pose.translation().y());
		double angle = std::atan2(robot_pose.rotation()(1, 0), robot_pose.rotation()(0, 0));
		robot_room_draw->setRotation(qRadiansToDegrees(angle));
	}
	catch (const Ice::Exception &e){std::cout << e.what() << std::endl;}
}


void SpecificWorker::new_target_slot(QPointF p){}

SpecificWorker::RetVal SpecificWorker::process_state(const RoboCompLidar3D::TPoints &data, const Corners &corners, const Match &match, AbstractGraphicViewer *viewer)
{
	switch(state)
	{
		case STATE::IDLE:               return localise(match);
		case STATE::LOCALISE:           return localise(match);
		case STATE::GOTO_DOOR:          return goto_door(data);
		case STATE::TURN:               return turn(corners);
		case STATE::ORIENT_TO_DOOR:     return orient_to_door(data);
		case STATE::GOTO_ROOM_CENTER:   return goto_room_center(data);
		case STATE::CROSS_DOOR:         return cross_door(data);
	}
}

//////////////////////////////////////////////////////////////////////////////////////////////////////

void SpecificWorker::emergency()
{
    std::cout << "Emergency worker" << std::endl;
    //emergencyCODE
    //
    //if (SUCCESSFUL) //The componet is safe for continue
    //  emmit goToRestore()
}



//Execute one when exiting to emergencyState
void SpecificWorker::restore()
{
    std::cout << "Restore worker" << std::endl;
    //restoreCODE
    //Restore emergency component

}


int SpecificWorker::startup_check()
{
	std::cout << "Startup check" << std::endl;
	QTimer::singleShot(200, QCoreApplication::instance(), SLOT(quit()));
	return 0;
}

void SpecificWorker::doStartStop()
{
	if (state == STATE::IDLE)
	{
		this->pushButton_startstop->setText("Stop");
		return;
	}

	state = STATE::IDLE;
	this->pushButton_startstop->setText("Start");
}


/**************************************/
// From the RoboCompDifferentialRobot you can call this methods:
// RoboCompDifferentialRobot::void this->differentialrobot_proxy->correctOdometer(int x, int z, float alpha)
// RoboCompDifferentialRobot::void this->differentialrobot_proxy->getBasePose(int x, int z, float alpha)
// RoboCompDifferentialRobot::void this->differentialrobot_proxy->getBaseState(RoboCompGenericBase::TBaseState state)
// RoboCompDifferentialRobot::void this->differentialrobot_proxy->resetOdometer()
// RoboCompDifferentialRobot::void this->differentialrobot_proxy->setOdometer(RoboCompGenericBase::TBaseState state)
// RoboCompDifferentialRobot::void this->differentialrobot_proxy->setOdometerPose(int x, int z, float alpha)
// RoboCompDifferentialRobot::void this->differentialrobot_proxy->setSpeedBase(float adv, float rot)
// RoboCompDifferentialRobot::void this->differentialrobot_proxy->stopBase()

/**************************************/
// From the RoboCompDifferentialRobot you can use this types:
// RoboCompDifferentialRobot::TMechParams

/**************************************/
// From the RoboCompLaser you can call this methods:
// RoboCompLaser::TLaserData this->laser_proxy->getLaserAndBStateData(RoboCompGenericBase::TBaseState bState)
// RoboCompLaser::LaserConfData this->laser_proxy->getLaserConfData()
// RoboCompLaser::TLaserData this->laser_proxy->getLaserData()

/**************************************/
// From the RoboCompLaser you can use this types:
// RoboCompLaser::LaserConfData
// RoboCompLaser::TData

