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
	std::cout << "Initialize worker" << std::endl;
	if(this->startup_check_flag)
	{
		this->startup_check();
	}
	else
	{
		///////////// Your code ////////
		// Viewer
		viewer = new AbstractGraphicViewer(this->frame, params.GRID_MAX_DIM);
		auto [r, e] = viewer->add_robot(params.ROBOT_WIDTH, params.ROBOT_LENGTH, 0, 100, QColor("Blue"));
		robot_draw = r;
		//viewer->show();


		viewer_room = new AbstractGraphicViewer(this->frame_room, params.GRID_MAX_DIM);
		auto [rr, re] = viewer_room->add_robot(params.ROBOT_WIDTH, params.ROBOT_LENGTH, 0, 100, QColor("Blue"));
		robot_room_draw = rr;
		//viewer_room->show();
		show();


		// initialise robot pose
		robot_pose.setIdentity();
		robot_pose.translate(Eigen::Vector2d(0.0,0.0));


		// time series plotter for match error
		TimeSeriesPlotter::Config plotConfig;
		plotConfig.title = "Maximum Match Error Over Time";
		plotConfig.yAxisLabel = "Error (mm)";
		plotConfig.timeWindowSeconds = 15.0; // Show a 15-second window
		plotConfig.autoScaleY = false;       // We will set a fixed range
		plotConfig.yMin = 0;
		plotConfig.yMax = 1000;
		time_series_plotter = std::make_unique<TimeSeriesPlotter>(frame_plot_error, plotConfig);
		match_error_graph = time_series_plotter->addGraph("", Qt::blue);


		// stop robot
		move_robot(0, 0);

		connect(pushButton_stop, &QPushButton::clicked, this, &SpecificWorker::doStartStop);
	}
}



void SpecificWorker::compute() {
	auto data = read_data();

	if (!door_detector.get_current_door().has_value())
	{
		qInfo() << "No door detected";
		return;
	}
	auto door = door_detector.get_current_door().value();

	estimated_center = center_estimator.estimate(data);

	// compute corners
	const auto &[corners, lines] = room_detector.compute_corners(data);
	draw_lidar(data, estimated_center, &viewer->scene);

	auto [nominal_room, match, max_match_error] = compute_match(corners);

	time_series_plotter->addDataPoint(match_error_graph,max_match_error);

	// update robot pose
	if (localised)
	{
		label_localised->setText("True");
		draw_current_room(nominal_room, &viewer_room->scene);
		update_robot_pose(corners, match);
	}
	else
		label_localised->setText("False");



	// Process state machine
	RetVal ret_val = process_state(data, door);
	auto [st, adv, rot] = ret_val;
	state = st;


	// Send movements commands to the robot
	//qInfo() << __FUNCTION__ << "Adv: " << adv << " Rot: " << rot;
	move_robot(adv, rot);


	// draw robot in viewer
	robot_room_draw->setPos(robot_pose.translation().x(), robot_pose.translation().y());
	const double angle = qRadiansToDegrees(std::atan2(robot_pose.rotation()(1, 0), robot_pose.rotation()(0, 0)));
	robot_room_draw->setRotation(angle);

	// update GUI
	label_state->setText(to_string(state));
	time_series_plotter->update();
	lcdNumber_adv->display(adv);
	lcdNumber_rot->display(rot);
	lcdNumber_x->display(robot_pose.translation().x());
	lcdNumber_y->display(robot_pose.translation().y());
	lcdNumber_angle->display(angle);
	last_time = std::chrono::high_resolution_clock::now();
}

SpecificWorker::RetVal SpecificWorker::goto_door(const RoboCompLidar3D::TPoints &points, const Door &door)
{
	auto door_center = door.center();

	Eigen::Vector2f v = door.p2 - door.p1;

	Eigen::Vector2f perp(-v.y(), v.x());
	perp.normalize();

	Eigen::Vector2f pA = door_center + perp * params.DOOR_REACHED_DIST;
	Eigen::Vector2f pB = door_center - perp * params.DOOR_REACHED_DIST;

	auto target_local = (pA.norm() < pB.norm()) ? pA : pB;

	// Usamos el umbral definido en los parametros
	if (target_local.norm() < params.RELOCAL_CENTER_EPS)
	{
		qInfo() << "Llegada a la puerta (Distancia:" << door_center.norm() << "mm). Cambiando a CROSS_DOOR.";
		// Paramos elrobot (0,0) y cambiamos de estado
		return {STATE::ORIENT_TO_DOOR, 0.f, 0.f};
	}

	auto [adv, rot] = robot_controller(target_local);
	return {STATE::GOTO_DOOR, adv, rot};
}

SpecificWorker::RetVal SpecificWorker::turn()
{
	auto [detected,left_right] = rc::ImageProcessor::check_colour_patch_in_image(camera360rgb_proxy, "red");
	if (detected)
	{
		localised = true;
		return 	{STATE::GOTO_DOOR, 0,0};
	}

	auto rot = params.RELOCAL_ROT_SPEED;
	if (left_right == -1)	rot = -rot;

	return 	{STATE::TURN, 0, rot};
}

SpecificWorker::RetVal SpecificWorker::orient_to_door(const RoboCompLidar3D::TPoints &points, const Door &door)
{
	auto door_center = door.center();

	float angle_to_door = std::atan2(door_center.y(), door_center.x());

	if(angle_to_door < M_PI_2 + params.RELOCAL_DELTA && angle_to_door > M_PI_2 - params.RELOCAL_DELTA)
	{
		// Está mirando a la puerta -> Avanzar
		return {STATE::CROSS_DOOR, 0, 0};
	}

	// No está orientado -> Girar hacia la puerta
	auto [adv, rot] = robot_controller(door_center);

	return {STATE::ORIENT_TO_DOOR, 0.0f, rot};
}

SpecificWorker::RetVal SpecificWorker::goto_room_center(const RoboCompLidar3D::TPoints &points)
{
	if (estimated_center.has_value()) {
		if (estimated_center.value().norm() < params.RELOCAL_CENTER_EPS) {
			return {STATE::TURN, 0.f, 0.f};
		}
		Eigen::Vector2f target = estimated_center.value().cast<float>();

		auto[adv, rot] = robot_controller(target);

		return {STATE::GOTO_ROOM_CENTER, adv, rot}; //Estado actual y velociades calculadas
	}else { //No se detecta el cendttro
		return {STATE::GOTO_ROOM_CENTER, 0.f, 0.f};
	}
}

SpecificWorker::RetVal SpecificWorker::cross_door(const RoboCompLidar3D::TPoints &points, const Door &door)
{
	auto door_center = door.center();

	// initialise robot pose
	localised = false;
	robot_pose.setIdentity();
	robot_pose.translate(Eigen::Vector2d(0.0,0.0));

	Eigen::Vector2f v = door.p2 - door.p1;

	Eigen::Vector2f perp(-v.y(), v.x());
	perp.normalize();

	Eigen::Vector2f pA = door_center + perp * params.DOOR_REACHED_DIST;
	Eigen::Vector2f pB = door_center - perp * params.DOOR_REACHED_DIST;

	auto target_local = pA.y() > pB.y() ? pA : pB;

	// Usamos el umbral definido en los parametros
	if (target_local.norm() < params.RELOCAL_CENTER_EPS)
	{
		// Paramos elrobot (0,0) y cambiamos de estado
		return {STATE::GOTO_ROOM_CENTER, 0.f, 0.f};
	}

	auto [adv, rot] = robot_controller(target_local);
	return {STATE::CROSS_DOOR, adv, rot};
}

SpecificWorker::RetVal SpecificWorker::update_pose(const Corners &corners, const Match &match)
{

}

SpecificWorker::RetVal SpecificWorker::process_state(const RoboCompLidar3D::TPoints &data, const Door &door)
{
	switch(state) {
		case STATE::IDLE:               return {STATE::IDLE, 0, 0};

		case STATE::GOTO_ROOM_CENTER:   return goto_room_center(data);
		case STATE::TURN:               return turn();
		case STATE::GOTO_DOOR:          return goto_door(data, door);
		case STATE::ORIENT_TO_DOOR:     return orient_to_door(data, door);
		case STATE::CROSS_DOOR:         return cross_door(data, door);
		default:						return {STATE::IDLE, 0, 0};
	}
}

void SpecificWorker::draw_lidar(const auto &points,  std::optional<Eigen::Vector2d> center_opt, QGraphicsScene *scene)
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

	auto center_values = center_opt.value();
	const auto center = scene->addEllipse(-100, -100, 200, 200, QPen(QColor("red")), QBrush(QColor("red")));
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

void SpecificWorker::draw_current_room(const NominalRoom &room, QGraphicsScene *scene)
{
	static std::vector<QGraphicsItem*> draw_points;
	for (const auto &p : draw_points) {
		scene->removeItem(p);
		delete p;
	}
	draw_points.clear();

	// draw room in viewer_room
	auto dp = viewer_room->scene.addRect(room.rect(), QPen(Qt::black, 30));
	draw_points.push_back(dp);
}

RoboCompLidar3D::TPoints SpecificWorker::read_data()
{
	auto data = lidar3d_proxy->getLidarDataWithThreshold2d("helios", 12000, 1);

	return door_detector.filter_points(data.points, &viewer->scene);
}

void SpecificWorker::print_match(const Match &match, const float error) const
{

}

float SpecificWorker::compute_match_error(const Match &match)
{
	if (match.empty())
		return std::numeric_limits<float>::infinity();

	double sum = std::transform_reduce(
		match.begin(), match.end(),
		0.0,
		std::plus<double>{},        // reduce
		[](const auto &m){          // transform
			return std::get<2>(m);
		}
	);

	return sum / match.size();
}

std::tuple<NominalRoom, Match, float> SpecificWorker::compute_match(const Corners &corners)
{
	std::vector<std::tuple<NominalRoom, Match, float>> matches;
	for (const auto &nominal_room : nominal_rooms) {
		auto match = hungarian.match(corners, nominal_room.transform_corners_to(robot_pose.inverse()));
		matches.push_back({nominal_room,
			match,
			compute_match_error(match)});
	}
	auto it = std::ranges::min_element(matches, {}, [](auto &t){ return std::get<2>(t); });

	return *it;
}

bool SpecificWorker::update_robot_pose(const Corners &corners, const Match &match)
{
	//Resolvemos la pose matematica
	Eigen::Vector3d inc_pose_params = solve_pose(corners, match);


	// Construir transformación incremental
	Eigen::Isometry2d Tinc = Eigen::Isometry2d::Identity();
	Tinc.translate(Eigen::Vector2d(inc_pose_params.x(), inc_pose_params.y()));
	Tinc.rotate(inc_pose_params.z());

	// Componer pose global
	robot_pose = robot_pose * Tinc;

	return true;
}

void SpecificWorker::move_robot(float adv, float rot)
{
	this->omnirobot_proxy->setSpeedBase(0, adv, rot);
}

Eigen::Vector3d SpecificWorker::solve_pose(const Corners &corners, const Match &match)
{
	Eigen::MatrixXd W(corners.size() * 2, 3);
	Eigen::VectorXd b(corners.size() * 2);

	for (auto &&[i, m] : match | iter::enumerate)
	{
		auto &[meas_c, nom_c, distance] = m;
		auto &[p_meas, __, ___] = meas_c;
		auto &[p_nom, ____, _____] = nom_c;

		b(2*i)     = p_nom.x() - p_meas.x();
		b(2*i + 1) = p_nom.y() - p_meas.y();

		W.block<1,3>(2*i,     0) << 1.0, 0.0, -p_meas.y();
		W.block<1,3>(2*i + 1, 0) << 0.0, 1.0,  p_meas.x();
	}

	// Estimación robusta de mínimos cuadrados
	Eigen::Vector3d r = W.colPivHouseholderQr().solve(b);

	return r;
}

std::tuple<float, float> SpecificWorker::robot_controller(const Eigen::Vector2f &target) {
	float distance = target.norm(); //Distancia euclidea

	auto x = target.x();
	auto y = target.y();

	float new_x = y;
	float new_y = -x;


	float angle = -std::atan2(new_y, new_x); //Angulo hacia el objetivo

	const float Kv = 0.5f; //Ganancia lineal
	const float Kw = 1.5f; //Ganancia angular

	// Calcular velocidades
	float adv = Kv * distance;
	float rot = Kw * angle;

	// Saturacion con los limites definidos
	if (adv > params.MAX_ADV_SPEED) adv = params.MAX_ADV_SPEED;
	if (rot > params.MAX_ROT_SPEED) rot = params.MAX_ROT_SPEED;
	if (rot < -params.MAX_ROT_SPEED) rot = -params.MAX_ROT_SPEED;

	// Si el angulo se muy grande, gira sin avanzar.
	// Esto evita movimientos extraños en la espiral si en objetivo esta detras o al lado
	if (std::abs(angle) > 0.4f) adv = 0.f;

	return std::make_tuple(adv, rot);
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
		// Reiniciar pose
		localised = false;
		robot_pose.setIdentity();
		robot_pose.translate(Eigen::Vector2d(0.0, 0.0));

		state = STATE::GOTO_ROOM_CENTER;

		pushButton_stop->setText("Stop");
		qInfo() << "Robot iniciado. Estado: GOTO_ROOM_CENTER"; // Log para depurar
		return;
	}

	// Caso contrario: Detener
	pushButton_stop->setText("Start");
	state = STATE::IDLE;

	move_robot(0, 0);
	qInfo() << "Robot detenido. Estado: IDLE";
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

