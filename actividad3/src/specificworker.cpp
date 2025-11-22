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
		// draw room in viewer_room
		viewer_room->scene.addRect(nominal_rooms[0].rect(), QPen(Qt::black, 30));
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
		move_robot(0, 0, 0);
	}
}



void SpecificWorker::compute()
{
	auto data = read_data();

	// compute corners
	const auto &[corners, lines] = room_detector.compute_corners(door_detector.filter_points(data,&viewer->scene), &viewer->scene);
	const auto center_opt = room_detector.estimate_center_from_walls();
	draw_lidar(door_detector.filter_points(data,&viewer->scene), center_opt);
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

	// update GUI
	time_series_plotter->update();
	lcdNumber_adv->display(adv);
	lcdNumber_rot->display(rot);
	lcdNumber_x->display(robot_pose.translation().x());
	lcdNumber_y->display(robot_pose.translation().y());
	lcdNumber_angle->display(angle);
	last_time = std::chrono::high_resolution_clock::now();
}

SpecificWorker::RetVal SpecificWorker::goto_door(const RoboCompLidar3D::TPoints &points)
{

}

SpecificWorker::RetVal SpecificWorker::turn(const Corners &corners)
{

}

SpecificWorker::RetVal SpecificWorker::orient_to_door(const RoboCompLidar3D::TPoints &points)
{

}

SpecificWorker::RetVal SpecificWorker::goto_room_center(const RoboCompLidar3D::TPoints &points)
{
	//Encontramos el centro geometrico de la habitacion
	std::optional<Eigen::Vector2d> center_opt = room_detector.estimate_center_from_walls();

	if (center_opt.has_value()) {
		Eigen::Vector2f target = center_opt.value().cast<float>();

		auto[adv, rot] = robot_controller(target);

		return {STATE::GOTO_ROOM_CENTER, adv, rot}; //Estado actual y velociades calculadas
	}else { //No se detecta el cendttro
		return {STATE::GOTO_ROOM_CENTER, 0.f, 0.f};
	}
}

SpecificWorker::RetVal SpecificWorker::cross_door(const RoboCompLidar3D::TPoints &points)
{

}

SpecificWorker::RetVal SpecificWorker::update_pose(const Corners &corners, const Match &match)
{

}

SpecificWorker::RetVal SpecificWorker::process_state(const RoboCompLidar3D::TPoints &data, const Corners &corners, const Match &match, AbstractGraphicViewer *viewer)
{
	switch(state) {
		case STATE::IDLE:               return std::make_tuple(STATE::IDLE, 0, 0);
		case STATE::GOTO_DOOR:          return goto_door(data);
		case STATE::TURN:               return turn(corners);
		case STATE::ORIENT_TO_DOOR:     return orient_to_door(data);
		case STATE::GOTO_ROOM_CENTER:   return goto_room_center(data);
		case STATE::CROSS_DOOR:         return cross_door(data);
	}
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

void SpecificWorker::print_match(const Match &match, const float error) const
{
	// Me gustaría ver dos tetas muy grandres
}

bool SpecificWorker::update_robot_pose(const Corners &corners, const Match &match)
{
	//Resolvemos la pose matematica
	Eigen::Vector3d new_pose_params = solve_pose(corners, match);

	//Actualizamos la variable robot_pose
	//new_pose_params contiene (x, y, theta)
	robot_pose.setIdentity();
	robot_pose.translation() << new_pose_params.x(), new_pose_params.y();
	robot_pose.linear() = Eigen::Rotation2Dd(new_pose_params.z()).toRotationMatrix();

	return true;
}

void SpecificWorker::move_robot(float adv, float rot, float max_match_error)
{
	this->omnirobot_proxy->setSpeedBase(0, adv, rot);
}

Eigen::Vector3d SpecificWorker::solve_pose(const Corners &corners, const Match &match) {
	Eigen::MatrixXd W(corners.size() * 2, 3);
	Eigen:: VectorXd b(corners.size() * 2);

	for (auto &&[i, m]: match | iter::enumerate)
	{
		auto &[meas_c, nom_c, distance] = m;
		auto &[p_meas, __, ___] = meas_c;
		auto &[p_nom, ____, _____] = nom_c;
		b(2 * i) = p_nom.x() - p_meas.x();
		b(2 * i + 1) = p_nom.y() - p_meas.y();
		W.block<1, 3>(2 * i, 0) << 1.0, 0.0, -p_meas.y();
		W.block<1, 3>(2 * i +1, 0) << 0.0, 1.0, p_meas.x();

		// qInfo() << p_meas << p_nom << distance;
	}
	qInfo() << "-----";
	const Eigen::Vector3d r = (W.transpose() * W).inverse() * W.transpose() * b;

	return r;
}

/*std::tuple<float, float> SpecificWorker::robot_controller(const Eigen::Vector2f &target) {
    // 1. Calcular Distancia (d) y Error Angular (theta_e) [cite: 436]
    float d = target.norm();
    float theta_e = std::atan2(target.y(), target.x());

    // ---------------------------------------------------------
    // PARÁMETROS DE SINTONIZACIÓN (Tuning Parameters)
    // ---------------------------------------------------------
    // Nota: El documento indica que las unidades son milímetros

    // Ganancias del control rotacional
    const float Kp = 1.5f; // [cite: 275] (Antes Kw)
    // const float Kd = 0.0f; // Se puede añadir si tienes el error previo

    // Parámetros del Freno Angular (Gaussian Angle-Brake) [cite: 342]
    // sigma = 45 grados (aprox 0.78 rad) es un buen valor inicial.
    const float sigma = M_PI / 4.0f;

    // Parámetros del Freno de Distancia (Sigmoid Distance-Brake) [cite: 425]
    // d_stop: Distancia donde empieza a frenar realmente (ej. 500mm)
    const float d_stop = 500.0f;
    // k: Pendiente de frenado.
    // IMPORTANTE: Como trabajamos en mm, k debe ser pequeño (ej. 0.02 para mm equivale a k=20 para metros)
    const float k_sigmoid = 0.02f;

    // ---------------------------------------------------------
    // CÁLCULO DE VELOCIDADES
    // ---------------------------------------------------------

    // 2. Control de Rotación (Solo P por ahora, ecuación w = Kp*theta) [cite: 437]
    float w = Kp * theta_e;

    // 3. Freno Angular (Gaussiana)
    // Reduce la velocidad si el robot no está alineado con el objetivo
    // f_theta tiende a 1 si theta_e es 0, tiende a 0 si theta_e es grande.
    float f_theta = std::exp( - (theta_e * theta_e) / (2.0f * sigma * sigma) );

    // 4. Freno de Distancia (Sigmoide)
    // Reduce la velocidad cuando el robot se acerca al objetivo (d -> d_stop)
    float f_d = 1.0f / (1.0f + std::exp( -k_sigmoid * (d - d_stop) ));

    // 5. Velocidad Lineal Final (v)
    // Modulamos la velocidad máxima con ambos frenos
    float v = params.MAX_ADV_SPEED * f_theta * f_d;

    // ---------------------------------------------------------
    // SATURACIÓN Y SEGURIDAD
    // ---------------------------------------------------------

    // Comprobar si hemos llegado (Zona muerta final)
    if (d < params.RELOCAL_CENTER_EPS) {
       return std::make_tuple(0.f, 0.f);
    }

    // Saturación de velocidad angular (la lineal ya está escalada por v_max)
    if (w > params.MAX_ROT_SPEED) w = params.MAX_ROT_SPEED;
    if (w < -params.MAX_ROT_SPEED) w = -params.MAX_ROT_SPEED;

    // Logging para depuración (opcional)
    // qInfo() << "Dist:" << d << " Ang:" << theta_e << " f_theta:" << f_theta << " f_d:" << f_d << " v:" << v;

    return std::make_tuple(v, w);
}*/

std::tuple<float, float> SpecificWorker::robot_controller(const Eigen::Vector2f &target) {
	float distance = target.norm(); //Distancia euclidea

	auto x = target.x();
	auto y = target.y();

	float new_x = y;
	float new_y = -x;


	float angle = -std::atan2(new_y, new_x); //Angulo hacia el objetivo

	const float Kv = 0.5f; //Ganancia lineal
	const float Kw = 1.5f; //Ganancia angular

	// Comprobar si hemos llegado
	if (distance < params.RELOCAL_CENTER_EPS) {
		return std::make_tuple(0.f, 0.f);
	}

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
		return;
	}

	state = STATE::IDLE;
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

