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

	this->setupUi(this);
	this->dimensions = QRectF(-6000, -3000, 12000, 6000);
	viewer = new AbstractGraphicViewer(this->frame, this->dimensions);
	viewer_room = new AbstractGraphicViewer(this->frame_room, nominal_room.rect);
	this->resize(900,450);
	this->show();
	const auto rob = viewer->add_robot(ROBOT_LENGTH, ROBOT_LENGTH, 0, 190, QColor("Blue"));
	robot_polygon = std::get<0>(rob);
	const auto rob_room= viewer_room->add_robot(ROBOT_LENGTH, ROBOT_LENGTH, 0, 190, QColor("Blue"));
	room_draw_robot = std::get<0>(rob_room);
	viewer_room->scene.addRect(nominal_room.rect, QPen(Qt::black, 30));
	viewer_room->fitInView(nominal_room.rect, Qt::KeepAspectRatio);

	robot_pose.setIdentity();
	robot_pose.translate(Eigen::Vector2d(0.0,0.0));

	connect(pushButton_startstop, SIGNAL(clicked()), this, SLOT(doStartStop()));
	connect(viewer, &AbstractGraphicViewer::new_mouse_coordinates, this, &SpecificWorker::new_target_slot);

}


void SpecificWorker::compute()
{
	read_data();

	doStateMachine();

	update_robot_position();
}

void SpecificWorker::read_data()
{
	try
	{
		auto data = lidar3d_proxy->getLidarDataWithThreshold2d("helios", 12000, 1);
		if (data.points.empty()) return;

		const auto filter_values = filter_isolated_points(data.points, 200);
		if (filter_values.empty())	return;

		calculateDistances(filter_values);
		
		draw_lidar(filter_values,&viewer->scene);

		auto measurements_corners = room_detector.compute_corners(filter_values, &viewer->scene);
		auto nominal_corners_on_robot_frame = nominal_room.transform_corners_to(robot_pose.inverse());
		auto hungarian_match =  hungarian.match(measurements_corners, nominal_corners_on_robot_frame);

		Eigen::MatrixXd W(nominal_room.corners.size() * 2, 3);
		Eigen:: VectorXd b(nominal_room.corners.size() * 2);

		for (auto &&[i, m]: hungarian_match | iter::enumerate)
		{
			auto &[meas_c, nom_c, distance] = m;
			auto &[p_meas, __, ___] = meas_c;
			auto &[p_nom, ____, _____] = nom_c;
			b(2 * i) = p_nom.x() - p_meas.x();
			b(2 * i + 1) = p_nom.y() - p_meas.y();
			W.block<1, 3>(2 * i, 0) << 1.0, 0.0, -p_meas.y();
			W.block<1, 3>(2 * i +1, 0) << 0.0, 1.0, p_meas.x();

			qInfo() << p_meas << p_nom << distance;
		}
		qInfo() << "-----";
		const Eigen::Vector3d r = (W.transpose() * W).inverse() * W.transpose() * b;
		W.transpose() * b;
		// std::cout << r << std::endl;
		// qInfo() << "--------------------";

		if (r.array().isNaN().any())
			return;

		robot_pose.translate(Eigen::Vector2d(r(0), r(1)));
		robot_pose.rotate(r[2]);

		room_draw_robot->setPos(robot_pose.translation().x(), robot_pose.translation().y());
		double angle = std::atan2(robot_pose.rotation()(1, 0), robot_pose.rotation()(0, 0));
		room_draw_robot->setRotation(qRadiansToDegrees(angle));

		draw_collisions(&viewer->scene);
		update_windows_values();
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


	const QColor color2("cyan");
	for(const auto &[p, _, __] : nominal_room.transform_corners_to(robot_pose.inverse()))
	{
		const auto i = scene->addEllipse(-100, -100, 200, 200, QPen(color2), QBrush(color2));
		i->setPos(p.x(), p.y());
		draw_points.push_back(i);
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


void SpecificWorker::update_windows_values()
{
	this->lcdNumber_leftdist->display(left_distance);
	this->lcdNumber_leftangle->display(left_angle);
	this->lcdNumber_frontdist->display(front_distance);
	this->lcdNumber_rightdist->display(right_distance);
	this->lcdNumber_rightangle->display(right_angle);
	switch (state)
	{
		case State::FOLLOW_WALL:
			this->label_state->setText("Follow Wall");
			break;
		case State::FORWARD:
			this->label_state->setText("Forward");
			break;
		case State::SPIRAL:
			this->label_state->setText("Spiral");
			break;
		case State::TURN:
			this->label_state->setText("Turn");
			break;
		default:
			this->label_state->setText("Off");
			break;
	}
	this->lcdNumber_xpos->display(robot_pose.translation().x());
	this->lcdNumber_ypos->display(robot_pose.translation().y());
	this->lcdNumber_angle->display(qRadiansToDegrees(std::atan2(robot_pose.rotation()(1, 0), robot_pose.rotation()(0, 0))));
}

void SpecificWorker::update_robot_position()
{
	try
	{
		RoboCompGenericBase::TBaseState bState;
		omnirobot_proxy->getBaseState(bState);
		//robot_polygon->setRotation(bState.alpha*180/M_PI);
		robot_polygon->setPos(bState.x, bState.z);

		std::cout << bState.alpha << " " << bState.x << " " << bState.z << std::endl;
	}
	catch (const Ice::Exception &e){std::cout << e.what() << std::endl;}
}


void SpecificWorker::new_target_slot(QPointF p){}

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
			if (point.y < width_distances && point.y > -width_distances)
			{
				if (point.x < right_min)	right_min = point.x;
				// UPPER
				if (point.y < width_distances && point.y > 0)
				{
					if (point.x < rightUpper_point->x)	rightUpper_point = &point;
					continue;
				}
				// LOWER
				if (point.y < 0 && point.y > -width_distances)
				{
					if (point.x < rightLower_point->x)	rightLower_point = &point;
				}
				continue;
			}
			continue;
		}

		// LEFT
		if (point.y < ROBOT_LENGTH && point.y > -ROBOT_LENGTH && point.x <= 0)
		{
			// MIDDLE
			if (point.y < width_distances && point.y > -width_distances)
			{
				if (-point.x < left_min)	left_min = -point.x;
				// UPPER
				if (point.y < width_distances && point.y > 0)
				{
					if (point.x > leftUpper_point->x)	leftUpper_point = &point;
					continue;
				}
				// LOWER
				if (point.y < 0 && point.y > -width_distances)
				{
					if (point.x > leftLower_point->x)	leftLower_point = &point;
				}
				continue;
			}
			continue;
		}
	}

	front_distance = front_min;
	right_distance = right_min;
	left_distance = left_min;

	right_angle = 0.0f;
	left_angle = 0.0f;

	if (rightUpper_point != &right_initPoint && rightLower_point != &right_initPoint)
		right_angle = atan2(rightUpper_point->y - rightLower_point->y, rightUpper_point->x - rightLower_point->x) - M_PI_2;
	if (leftUpper_point != &left_initPoint && leftLower_point != &left_initPoint)
		left_angle = atan2(leftUpper_point->y - leftLower_point->y, leftUpper_point->x - leftLower_point->x) - M_PI_2;
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
		case State::OFF:
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

void SpecificWorker::turnState()
{
	if (front_distance > OBSTACLE_DIST)
	{
		right_turn = right_distance > left_distance;
		current_rotation = INIT_ROTATION;
		current_velocity = INIT_VELOCITY;
		state = stateRandomizer();
		return;
	}

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
		omnirobot_proxy->setSpeedBaseAsync(0, 0, 0);
		state = State::TURN;
		return;
	}

	const float Kp_angle = 1.0f;   // Para corrección de orientación

	// Calculamos corrección por ángulo
	float rotation_angle_correction = 0.0f;
	rotation_angle_correction -= Kp_angle * right_angle;
	rotation_angle_correction -= Kp_angle * left_angle;

	// Velocidad angular final
	float rotation = rotation_angle_correction;

	// Limitamos la velocidad de giro
	if (rotation > MAX_ROT) rotation = MAX_ROT;
	if (rotation < -MAX_ROT) rotation = -MAX_ROT;
	if (rotation > -0.01f && rotation < 0.1f) rotation = 0;

	omnirobot_proxy->setSpeedBaseAsync(0, MAX_ADV, rotation);
}

void SpecificWorker::spiralState()
{
	if(front_distance < OBSTACLE_DIST || current_rotation >= 1.0f  || current_rotation <= -1.0f || current_velocity < 350)
	{
		omnirobot_proxy->setSpeedBase(0,0,0);
		state = State::TURN;
		return;
	}

	current_rotation += (MAX_ROT - current_rotation) * 0.05;
	current_velocity *= 0.99;

	if (right_turn)
		omnirobot_proxy->setSpeedBase(0,current_velocity,current_rotation);
	else
		omnirobot_proxy->setSpeedBase(0,current_velocity,-current_rotation);
}

State SpecificWorker::stateRandomizer()
{
	int random_num = rand() % 2;
	switch (random_num)
	{
	case 0:
		return State::SPIRAL;
	case 1:
		return State::FOLLOW_WALL;
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

void SpecificWorker::doStartStop()
{
	if (state == State::OFF)
	{
		state = stateRandomizer();
		this->pushButton_startstop->setText("Stop");
		return;
	}

	state = State::OFF;
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

