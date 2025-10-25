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

	this->dimensions = QRectF(-6000, -3000, 12000, 6000);
	viewer = new AbstractGraphicViewer(this->frame, this->dimensions);
	this->resize(900,450);
	viewer->show();
	const auto rob = viewer->add_robot(ROBOT_LENGTH, ROBOT_LENGTH, 0, 190, QColor("Blue"));
	robot_polygon = std::get<0>(rob);

	connect(viewer, &AbstractGraphicViewer::new_mouse_coordinates, this, &SpecificWorker::new_target_slot);

}


void SpecificWorker::compute()
{
	try
	{
		auto data = lidar3d_proxy->getLidarDataWithThreshold2d("helios", 5000, 1);
		if (data.points.empty()) return;

		const auto filter_values = filter_isolated_points(data.points, 200);
		if (filter_values.empty())	return;

		calculateDistances(filter_values);

		draw_lidar(filter_values,&viewer->scene);
		draw_collisions(&viewer->scene);

		doStateMachine();

		update_robot_position();
	}
	catch (const Ice::Exception &e)
	{
		std::cout << e << " " << "Conexion con laser" << std::endl;
	}
}

void SpecificWorker::draw_lidar(const auto &points, QGraphicsScene* scene)
{
	static std::vector<QGraphicsItem*> draw_points;
	for (const auto &p : draw_points)
	{
		scene->removeItem(p);
		delete p;
	}
	draw_points.clear();

	const QColor color("LightGreen");
	const QPen pen(color, 10);
	//const QBrush brush(color, Qt::SolidPattern);
	for (const auto &p : points)
	{
		const auto dp = scene->addRect(-25, -25, 50, 50, pen);
		dp->setPos(p.x, p.y);
		draw_points.push_back(dp);   // add to the list of points to be deleted next time
	}
}

void SpecificWorker::draw_collisions(QGraphicsScene* scene)
{
	static std::vector<QGraphicsItem*> draw_points;
	for (const auto &p : draw_points)
	{
		scene->removeItem(p);
		delete p;
	}
	draw_points.clear();

	// FRONT LINE
	QLineF frontLine(QPointF(-ROBOT_LENGTH/2, front_distance), QPointF(ROBOT_LENGTH/2, front_distance));
	draw_points.push_back(scene->addLine(frontLine, QPen(QColor("Red"), 30)));

	// RIGHT LINE
	QLineF rightLine(QPointF(right_distance, width_distances), QPointF(right_distance, -width_distances));
	draw_points.push_back(scene->addLine(rightLine, QPen(QColor("Red"), 30)));

	// LEFT LINE
	QLineF leftLine(QPointF(-left_distance, width_distances), QPointF(-left_distance, -width_distances));
	draw_points.push_back(scene->addLine(leftLine, QPen(QColor("Red"), 30))); // LEFT
}


void SpecificWorker::update_windows_values(QGraphicsScene* scene)
{

}

void SpecificWorker::update_robot_position()
{
	try
	{
		RoboCompGenericBase::TBaseState bState;
		omnirobot_proxy->getBaseState(bState);
		robot_polygon->setRotation(bState.alpha*180/M_PI);
		robot_polygon->setPos(bState.x, bState.z);

		std::cout << bState.alpha << " " << bState.x << " " << bState.z << std::endl;
	}
	catch (const Ice::Exception &e){std::cout << e.what() << std::endl;}
}


void SpecificWorker::new_target_slot(QPointF p){}

void SpecificWorker::calculateDistancesOLD(const RoboCompLidar3D::TPoints &points)
{
	float front_distance_aux = 9999, left_distance_aux = 9999, right_distance_aux = 9999;
	for (const auto &p : points)
	{
		float angle = atan2(p.y, p.x);

		float d = std::hypot(p.x, p.y);
		if (angle < (std::numbers::pi)*9/16 && angle > (std::numbers::pi)*7/16) front_distance_aux = std::min(front_distance_aux, d);
		if (angle < -(std::numbers::pi)*15/16 || angle > (std::numbers::pi)*15/16) left_distance_aux = std::min(left_distance_aux, d);
		if ((angle < (std::numbers::pi)*1/16 && angle > 0) || ( angle > -(std::numbers::pi)*1/16 && angle < 0)) right_distance_aux = std::min(right_distance_aux, d);
	}

	front_distance = front_distance_aux;
	right_distance = right_distance_aux;
	left_distance = left_distance_aux;
}

void SpecificWorker::calculateDistances(const RoboCompLidar3D::TPoints &points)
{
	auto robot_section = ROBOT_LENGTH/2+50;
	float front_min = 9999, left_min = 9999, right_min = 9999;

	RoboCompLidar3D::TPoint right_initPoint;	right_initPoint.x = 9999;
	const RoboCompLidar3D::TPoint *rightUpper_point = &right_initPoint;
	const RoboCompLidar3D::TPoint *rightLower_point = &right_initPoint;

	RoboCompLidar3D::TPoint left_initPoint;	left_initPoint.x = -9999;
	const RoboCompLidar3D::TPoint *leftUpper_point = &left_initPoint;
	const RoboCompLidar3D::TPoint *leftLower_point = &left_initPoint;
	for (const auto &point: points)
	{
		//FRONT
		if (point.x < robot_section && point.x > -robot_section && point.y >= 0)
		{
			if (point.y < front_min)	front_min = point.y;
			continue;
		}

		// RIGHT
		if (point.y < ROBOT_LENGTH && point.y > -ROBOT_LENGTH && point.x >= 0)
		{
			// UPPER
			if (point.y < width_distances*2 && point.y > width_distances)
			{
				if (point.x < rightUpper_point->x)
				{
					rightUpper_point = &point;
				}
				continue;
			}
			// MIDDLE
			if (point.y < width_distances && point.y > -width_distances)
			{
				if (point.x < right_min)	right_min = point.x;
				continue;
			}
			// LOWER
			if (point.y < -width_distances && point.y > -width_distances*2)
			{
				if (point.x < rightLower_point->x)	rightLower_point = &point;
				continue;
			}
		}

		// LEFT
		if (point.y < ROBOT_LENGTH && point.y > -ROBOT_LENGTH && point.x <= 0)
		{
			// UPPER
			if (point.y < width_distances*2 && point.y > width_distances)
			{
				if (point.x > leftUpper_point->x)	leftUpper_point = &point;
				continue;
			}
			// MIDDLE
			if (point.y < width_distances && point.y > -width_distances)
			{
				if (-point.x < left_min)	left_min = -point.x;
				continue;
			}
			// LOWER
			if (point.y < -width_distances && point.y > -width_distances*2)
			{
				if (point.x > leftLower_point->x)	leftLower_point = &point;
				continue;
			}
		}
	}

	front_distance = front_min;
	right_distance = right_min;
	left_distance = left_min;

	right_angle = atan2(rightUpper_point->y - rightLower_point->y, rightUpper_point->x - rightLower_point->x) - M_PI_2;
	left_angle = atan2(leftUpper_point->y - leftLower_point->y, leftUpper_point->x - leftLower_point->x) - M_PI_2;

	qInfo() << "distancia Frontal:" << front_distance;
	qInfo() << "Distancia derecha: "<< right_distance;
	qInfo() << "distancia izquierda: " << left_distance;
	qInfo() << "Right angle:" << right_angle;
	qInfo() << "Left angle:" << left_angle;
}

void SpecificWorker::doStateMachine()
{
	switch (state)
	{
		case State::SPIRAL:
			spiralState();
			break;
		case State::FORWARD:
			forwardState();
			break;
		case State::TURN:
			turnState();
			break;
		case State::FOLLOW_WALL:
			follow_WallState();
			break;
	}
}
void SpecificWorker::forwardState()
{
	if (front_distance < OBSTACLE_DIST)
	{
		state = State::TURN;
		omnirobot_proxy->setSpeedBase(0,0,0);
		qInfo() << "Obstaculo encontrado, girando...";
		return;
	}
	omnirobot_proxy->setSpeedBase(0,MAX_ADV,0);
}

static int time_to_turn = 10;

void SpecificWorker::turnState()
{
	if (front_distance > OBSTACLE_DIST)
	{
		direccionGiro = (rand() % 2 == 0);
		current_rotation = INIT_ROTATION;
		current_velocity = INIT_VELOCITY;
		state = stateRandomizer();
		return;
	}

	if (time_to_turn++ < 10)
	{
		return;
	}
	time_to_turn = 0;

	if (right_distance > left_distance)
	{
		omnirobot_proxy->setSpeedBase(0,0,MAX_ROT);
	}
	else
	{
		omnirobot_proxy->setSpeedBase(0,0,-MAX_ROT);
	}
}

void SpecificWorker::follow_WallState()
{
	if (front_distance < OBSTACLE_DIST)
	{
		state = State::TURN;
		return;
	}

	if (right_angle < 0.1f || left_angle > 0.1f)
	{
		
	}

	omnirobot_proxy->setSpeedBase(0,MAX_ADV,0);

	if (right_distance < OBSTACLE_DIST || left_distance < OBSTACLE_DIST)
	{
		state = State::TURN;
	}
}

void SpecificWorker::spiralState()
{
	if(front_distance < OBSTACLE_DIST || current_rotation < 0 || current_velocity < 10)
	{
		omnirobot_proxy->setSpeedBase(0,0,0);
		state = State::TURN;
		return;
	}

	current_rotation += 0.0025;
	current_velocity -= 1;

	if (direccionGiro)
		omnirobot_proxy->setSpeedBase(0,current_velocity,current_rotation);
	else
		omnirobot_proxy->setSpeedBase(0,current_velocity,-current_rotation);
}

State SpecificWorker::stateRandomizer()
{
	int random_num = rand() % 3;
	switch (random_num)
	{
	case 0:
		return State::FORWARD;
	case 1:
		return State::FOLLOW_WALL;
	case 2:
		return State::SPIRAL;
	}
	return State::TURN;
}


std::optional<RoboCompLidar3D::TPoints> SpecificWorker::filter_min_distance_cppintertools(const RoboCompLidar3D::TPoints &points)
{
	RoboCompLidar3D::TPoints result; result.reserve(points.size());

	for (auto&& [angle, group] : iter::groupby(points, [](const auto& p)
	{
		float multiplier = std::pow(10.0f, 2); return std::floor(p.phi * multiplier) / multiplier;
	}))
	{
		auto min_it = std::min_element(std::begin(group), std::end(group),
			[](const auto& a, const auto& b){ return a.r < b.r; });
		result.emplace_back(RoboCompLidar3D::TPoint{.x=min_it->x, .y=min_it->y, .phi=min_it->phi});
	}
	return result;
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

