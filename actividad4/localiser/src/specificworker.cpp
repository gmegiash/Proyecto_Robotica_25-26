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
		// Viewer
		viewer = new AbstractGraphicViewer(this->frame, params.GRID_MAX_DIM);
		auto [r, e] = viewer->add_robot(params.ROBOT_WIDTH, params.ROBOT_LENGTH, 0, 100, QColor("Blue"));
		robot_draw = r;

		viewer_room = new AbstractGraphicViewer(this->frame_room, params.GRID_MAX_DIM);
		auto [rr, re] = viewer_room->add_robot(params.ROBOT_WIDTH, params.ROBOT_LENGTH, 0, 100, QColor("Blue"));
		robot_room_draw = rr;
		show();

		// initialise robot pose
		robot_pose.setIdentity();
		robot_pose.translate(Eigen::Vector2f(0.0,0.0));

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
	const auto data = read_data();

	estimated_center = center_estimator.estimate(data);

	// update robot pose
	if (localised)
	{
		label_localised->setText("True");
		draw_current_room(&viewer_room->scene);
		if (const auto res = update_robot_pose(data, true); res.has_value())
		{
			robot_pose = res.value().first;
			const auto max_match_error = res.value().second;
			const auto &robot_pos = robot_pose.translation();
			robot_room_draw->setPos(robot_pos.x(),robot_pos.y());
			const double angle = qRadiansToDegrees(std::atan2(robot_pose.rotation()(1, 0), robot_pose.rotation()(0, 0)));
			robot_room_draw->setRotation(angle);
			robot_room_draw->show();
			time_series_plotter->addDataPoint(match_error_graph,max_match_error);
		}
	}
	else {
		label_localised->setText("False");
		robot_room_draw->hide();
	}

	// Process state machine
	RetVal ret_val = process_state(data);
	auto [st, adv, rot] = ret_val;
	state = st;
	// Send movements commands to the robot
	move_robot(adv, rot);

	draw_lidar(data, &viewer->scene);
	draw_room_center(&viewer->scene);

	static std::vector<QGraphicsItemGroup*> to_remove;

	for (const auto item_group: to_remove)
	{
		delete item_group;
	}
	to_remove.clear();

	for (const auto item_group: to_draw)
	{
		item_group->show();
		to_remove.push_back(item_group);
	}
	to_draw.clear();

	// update GUI
	label_state->setText(to_string(state));
	time_series_plotter->update();
	lcdNumber_adv->display(adv);
	lcdNumber_rot->display(rot);
	lcdNumber_x->display(robot_pose.translation().x());
	lcdNumber_y->display(robot_pose.translation().y());
	//lcdNumber_angle->display(angle);
	last_time = std::chrono::high_resolution_clock::now();
	lcdNumber_current_room->display(current_room); lcdNumber_current_door->display(current_door);
	lcdNumber_entering_room->display(door_crossing.entering_room_index); lcdNumber_entering_door->display(door_crossing.entering_room_index);
	lcdNumber_leaving_room->display(door_crossing.leaving_room_index); lcdNumber_leaving_door->display(door_crossing.leaving_door_index);
}

SpecificWorker::RetVal SpecificWorker::process_state(const RoboCompLidar3D::TPoints &points)
{
	switch(state) {
		case STATE::IDLE:               return {STATE::IDLE, 0, 0};
		case STATE::GOTO_ROOM_CENTER:   return goto_room_center();
		case STATE::TURN:               return turn(points);
		case STATE::GOTO_DOOR:          return goto_door(points);
		case STATE::ORIENT_TO_DOOR:     return orient_to_door(points);
		case STATE::CROSS_DOOR:         return cross_door(points);
		default:						return {STATE::IDLE, 0, 0};
	}
}

SpecificWorker::RetVal SpecificWorker::goto_room_center()
{
	if (estimated_center.has_value()) {
		if (estimated_center.value().norm() < params.RELOCAL_CENTER_EPS) {
			return {STATE::TURN, 0.f, 0.f};
		}
		Eigen::Vector2f target = estimated_center.value().cast<float>();

		auto[adv, rot] = robot_controller(target);

		if (door_crossing.valid) door_crossing.track_entering_door(door_detector.doors());
		return {STATE::GOTO_ROOM_CENTER, adv, rot}; //Estado actual y velociades calculadas
	}

	return {STATE::GOTO_ROOM_CENTER, 0.f, 0.f};
}

SpecificWorker::RetVal SpecificWorker::turn(const RoboCompLidar3D::TPoints &points)
{
	if (door_crossing.valid) door_crossing.track_entering_door(door_detector.doors());
	auto [detected,room_idx,left_right] = rc::ImageProcessor::check_colour_patch_in_image(camera360rgb_proxy);
	if (detected)
	{
		current_room = room_idx;
		localised = true;

		if (const auto res = update_robot_pose(points, true); res.has_value())
		{
			robot_pose = res.value().first;
			const auto max_match_error = res.value().second;
			time_series_plotter->addDataPoint(match_error_graph,max_match_error);
		}

		if (auto &room = nominal_rooms.at(current_room); !room.visited)
		{
			qInfo() << "Habitación " << current_room << " visitada por primera vez. Guardando puertas.";
			room.visited = true;

			const auto &detected_doors = door_detector.doors();
			room.doors.clear(); // Limpiamos para llenar con datos reales

			// Transformamos las puertas detectadas (locales) a globales y las guardamos
			for(const auto &d : detected_doors)
			{
				// Convertir p1 y p2 a coordenadas globales usando la pose actual del robot
				Eigen::Vector2f p1_g = room.get_projection_of_point_on_closest_wall(robot_pose * d.p1);
				Eigen::Vector2f p2_g = room.get_projection_of_point_on_closest_wall(robot_pose * d.p2);
				float p1_angle = atan2(p1_g.y(), p1_g.x());
				float p2_angle = atan2(p2_g.y(), p2_g.x());

				room.doors.emplace_back(p1_g, p1_angle, p2_g, p2_angle);
			}
		}

		if (door_crossing.valid)
		{
			door_crossing.set_entering_data(current_room, nominal_rooms);

			auto &leaving_door_info = nominal_rooms[door_crossing.leaving_room_index].doors[door_crossing.leaving_door_index];
			leaving_door_info.connects_to_room = door_crossing.entering_room_index;
			leaving_door_info.connects_to_door = door_crossing.entering_door_index;
			leaving_door_info.visited = true;

			auto &entering_door_info = nominal_rooms[door_crossing.entering_room_index].doors[door_crossing.entering_door_index];
			entering_door_info.connects_to_room = door_crossing.leaving_room_index;
			entering_door_info.connects_to_door = door_crossing.leaving_door_index;
			entering_door_info.visited = true;
		}

		if (current_door = choose_next_door(); current_door != -1)
		{
			return {STATE::GOTO_DOOR, 0, 0};
		}

		return {STATE::IDLE, 0, 0};
	}

	float rot = params.RELOCAL_ROT_SPEED;
	if (left_right == -1) rot = -rot; // Girar hacia la izquierda si la pista lo indica

	return {STATE::TURN, 0, rot};
}

unsigned long SpecificWorker::choose_next_door() const
{
	const auto &doors = nominal_rooms[current_room].doors;

	std::vector<unsigned long> candidates;
	for (const auto [i, door] : doors | iter::enumerate)
	{
		if (!door.visited)
		{
			candidates.push_back(i);
		}
	}

	if (candidates.empty())
	{
		for (const auto [i, door] : doors | iter::enumerate)
		{
			candidates.push_back(i);
		}
	}

	if (!candidates.empty())
	{
		const int random_idx = std::rand() % candidates.size();
		const unsigned long chosen_door_idx = candidates[random_idx];

		return chosen_door_idx;
	}

	return -1;
}

SpecificWorker::RetVal SpecificWorker::goto_door(const RoboCompLidar3D::TPoints &points)
{
	if (current_door == -1)
	{
		return {STATE::IDLE, 0, 0};
	}

	const auto &door = nominal_rooms[current_room].doors[current_door];

	const auto target_global = door.center_before(robot_pose.translation(),params.DIST_TARGET_BEFORE_DOOR);
	const auto target_local = robot_pose.inverse() * target_global;

	const auto target_group = viewer->scene.createItemGroup({});
	target_group->hide();
	to_draw.push_back(target_group);

	const QColor color("Cyan");
	const QPen pen(color, 10);
	const auto target_item = new QGraphicsEllipseItem(-25, -25, 100, 100, target_group);
	target_item->setPen(pen);
	target_item->setBrush(QBrush(color));
	target_item->setPos(target_local.x(), target_local.y());

	if (target_local.norm() < params.DOOR_REACHED_DIST) return {STATE::ORIENT_TO_DOOR,0,0};

	const auto [adv, rot] = robot_controller(target_local);

	return {STATE::GOTO_DOOR, adv, rot};
}

SpecificWorker::RetVal SpecificWorker::orient_to_door(const RoboCompLidar3D::TPoints &points)
{
	if (current_door == -1)
	{
		return {STATE::IDLE, 0, 0};
	}

	const auto &door = nominal_rooms[current_room].doors[current_door];

	const auto target_global = door.center();
	const auto target_local = robot_pose.inverse() * target_global;

	const auto direct_angle = atan2(target_local.y(), target_local.x()) - M_PI_2;
	if (!(direct_angle > params.RELOCAL_MAX_ORIENTED_ERROR || direct_angle < -params.RELOCAL_MAX_ORIENTED_ERROR))
	{
		return {STATE::CROSS_DOOR,0,0};
	}

	const auto [adv,rot] = robot_controller(target_local);
	return {STATE::ORIENT_TO_DOOR, 0, rot};
}

SpecificWorker::RetVal SpecificWorker::cross_door(const RoboCompLidar3D::TPoints &points)
{
	static std::chrono::time_point<std::chrono::high_resolution_clock> start;
    static bool started = false;

    // 1. INICIALIZACIÓN
    if (!started)
	{
    	localised = false;
		door_crossing = DoorCrossing(current_room, current_door);
		start = std::chrono::high_resolution_clock::now();
		started = true;
	}

    // 2. COMPROBAR TIEMPO DE CRUCE A CIEGAS
    if (const auto elapsedTime = std::chrono::high_resolution_clock::now() - start; elapsedTime > std::chrono::milliseconds(params.BLIND_CROSS_TIME))
    {
		started = false;

		const auto &crossed_room_values = nominal_rooms[current_room];
    	const auto &crossed_door_values = crossed_room_values.doors[current_door];

		// Si la puerta por la que salimos tiene información de destino...
		if (crossed_door_values.visited)
		{
			// A. Recuperamos índices y objetos de la NUEVA habitación
			const auto &connected_room_idx = crossed_door_values.connects_to_room;
			const auto &connected_room = nominal_rooms[connected_room_idx];

			// B. Recuperamos la puerta en la NUEVA habitación (donde estamos ahora)
			// Ojo: usamos 'connects_to_door' para saber cuál es la puerta correspondiente en la nueva sala
			const auto &connected_door_idx = crossed_door_values.connects_to_door;
			const auto &connected_door = connected_room.doors[connected_door_idx];

			// C. ACTUALIZACIÓN CRÍTICA DE POSE
			current_room = connected_room_idx;

			// 1. Reset completo (limpiamos basura de la sala anterior)
			robot_pose.setIdentity();

			// 2. Traslación: Nos ponemos en el centro de la puerta de la nueva sala
			const auto connected_door_center = connected_door.center();
			robot_pose.translate(connected_door_center);

			// 3. Rotación: EL CAMBIO CLAVE
			// Queremos mirar HACIA el centro (0,0). Vector = (0-x, 0-y)
			const auto entering_angle = std::atan2(-connected_door_center.y(), -connected_door_center.x());

			// Asignación directa a la matriz de rotación (más seguro que .rotate())
			robot_pose.linear() = Eigen::Rotation2Df(entering_angle).toRotationMatrix();

			qDebug()	<< "Cruce completado. Nueva Sala:" << current_room
						<< "Puerta cruzada: " << connected_door_idx
						<< "Pos:" << connected_door_center.x() << connected_door_center.y()
						<< "Angulo Entrada (rad):" << entering_angle;
			localised = true;
		}
		else
		{
			// Puerta desconocida -> Perdidos (o inicio de exploración)
			qWarning() << "Cruzando a territorio desconocido. Reseteando a 0,0.";
			localised = false;
			robot_pose.setIdentity();
			robot_pose.translate(Eigen::Vector2f(0.0,0.0));
		}

		return {STATE::GOTO_ROOM_CENTER, 0, 0};
    }

    return {STATE::CROSS_DOOR, params.BLIND_ADV_VELOCITY, 0};
}

SpecificWorker::RetVal SpecificWorker::update_pose(const Corners &corners, const Match &match)
{
	// Validamos que tengamos suficientes correspondencias
	if (match.empty() || match.size() < 3) {
		qWarning() << __FUNCTION__ <<"Not enought matches to estimate pose";
		return {STATE::LOCALISE, 0.0f, 0.0f};
	}

	// Resolvemos la corrección de pose
	Eigen::Vector3d pose_correction = solve_pose(corners, match);

	// Validar que no hay NaN (error numérico)
	if (pose_correction.array().isNaN().any()) {
		qWarning() << __FUNCTION__ << "NaN values in pose correction";
		return {STATE::LOCALISE, 0.0f, 0.0f};
	}

	// Aplicamos la correción a la pose del robot
	robot_pose.translate(Eigen::Vector2f(pose_correction(0), pose_correction(1)));
	robot_pose.rotate(Eigen::Rotation2Df(pose_correction(2)));

	// Calculamos el error máximo de las correspondencias
	const auto max_error_iter = std::ranges::max_element(match, [](const auto &a, const auto &b) {
		return std::get<2>(a) < std::get<2>(b);
	});

	float max_match_error = std::get<2>(*max_error_iter);

	// Log para debugging
	qDebug() << __FUNCTION__ << "Max match error:" << max_match_error;

	// PASO 6: Transicionar según el error
	if (max_match_error < params.RELOCAL_DONE_MATCH_MAX_ERROR)
	{
		// Localización completada exitosamente
		qInfo() << "Localization successful! Error:" << max_match_error;
		return {STATE::TURN, 0.0f, 0.0f};
	}
	else if (max_match_error < 2000.0f)
	{
		// Localización en progreso, continuar refinando
		return {STATE::LOCALISE, 0.0f, 0.0f};
	}
	else
	{
		// Error muy grande, volver a centrar en la habitación
		qWarning() << "Large match error, returning to center";
		return {STATE::GOTO_ROOM_CENTER, 0.0f, 0.0f};
	}
}

SpecificWorker::RetVal SpecificWorker::localise(const RoboCompLidar3D::TPoints &points, QGraphicsScene *scene) {
	// initialise robot pose at origin. Necessary to reset pose accumulation
	robot_pose.setIdentity();
	robot_pose.translate(Eigen::Vector2f(0.0,0.0));
	localised = false;

	// if error high but not at room centre, go to centering step
	// compute mean of LiDAR points aFs room center estimate
	if (const auto center = center_estimator.estimate(points); center.has_value()) {
		if (center.value().norm() > params.RELOCAL_CENTER_EPS)
			return {STATE::GOTO_ROOM_CENTER, 0.0f, 0.0f};

		// if close enough to center -> stop and move to TURN
		if (center.value().norm() < params.RELOCAL_CENTER_EPS)
			return {STATE::TURN, 0.0f, 0.0f};
	}
	qWarning() << __FUNCTION__ << "Not able to estimate room center from walls, continue localising.";
	return {STATE::LOCALISE, 0.0f, 0.0f};
}

void SpecificWorker::draw_lidar(const auto &points, QGraphicsScene *scene)
{
	QGraphicsItemGroup* lidar_group = scene->createItemGroup({});
	lidar_group->hide();
	to_draw.push_back(lidar_group);

	const QColor color("LightGreen");
	const QPen pen(color, 1);
	const QBrush brush(color);

	//const QBrush brush(color, Qt::SolidPattern);

	for (const auto &p : points)
	{
		const auto point_item = new QGraphicsRectItem(-25, -25, 50, 50, lidar_group);
		point_item->setPen(pen);
		point_item->setBrush(brush);
		point_item->setPos(p.x, p.y);
	}
}

void SpecificWorker::draw_room_center(QGraphicsScene *scene) {
	QGraphicsItemGroup* center_group = scene->createItemGroup({});
	center_group->hide();
	to_draw.push_back(center_group);

	const QColor color("Red");
	const QPen pen(color, 1);
	const QBrush brush(color);

	const auto center_item = new QGraphicsEllipseItem(-25, -25, 100, 100, center_group);
	center_item->setPen(pen);
	center_item->setBrush(brush);
	center_item->setPos(estimated_center->x(), estimated_center->y());
}

void SpecificWorker::update_robot_position()
{
	try
	{
		robot_room_draw->setPos(robot_pose.translation().x(), robot_pose.translation().y());
		const double angle = std::atan2(robot_pose.rotation()(1, 0), robot_pose.rotation()(0, 0));
		robot_room_draw->setRotation(qRadiansToDegrees(angle));
	}
	catch (const Ice::Exception &e){std::cout << e.what() << std::endl;}
}

void SpecificWorker::draw_current_room(QGraphicsScene *scene)
{
	if (current_room == -1)	return;

	QGraphicsItemGroup* room_group = scene->createItemGroup({});
	room_group->hide();
	to_draw.push_back(room_group);

	const auto room_number = new QGraphicsTextItem(QString::number(current_room),room_group);
	room_number->setPos(0,0);
	QRectF room_number_label = room_number->boundingRect();
	room_number->setTransformOriginPoint(room_number_label.center());
	room_number->setTransform(QTransform::fromScale(-30, 30));
	room_number->setRotation(180);

	const QPen room_rect_pen(Qt::black, 30);
	const QColor color("Red");
	const QPen door_pen(color, 45);

	auto &room_to_draw = nominal_rooms[current_room];

	const auto room_rect_item = new QGraphicsRectItem(room_to_draw.rect(),room_group);
	room_rect_item->setPen(room_rect_pen);

	for (const auto &[index,door] : room_to_draw.doors | iter::enumerate)
	{
		const auto door_p1 = room_to_draw.get_projection_of_point_on_closest_wall(door.p1);
		const auto door_p2 = room_to_draw.get_projection_of_point_on_closest_wall(door.p2);
		const auto door_center = door.center();
		const auto room_door_item = new QGraphicsLineItem(door_p1.x(), door_p1.y(), door_p2.x(), door_p2.y(),room_group);
		room_door_item->setPen(door_pen);

		const auto door_label = new QGraphicsTextItem("Door:"+QString::number(index)+"\n"+
			"Room(Door):"+QString::number(door.connects_to_room)+"("+QString::number(door.connects_to_door)+")"+"\n"+
			"Pos:"+QString::number(door.center().x())+","+QString::number(door.center().y()),room_group);
		QRectF bounds = door_label->boundingRect();
		door_label->setPos(door_center.x()-bounds.width()/2, door_center.y());
		door_label->setTransformOriginPoint(bounds.center());
		door_label->setTransform(QTransform::fromScale(10, -10));
	}
}

RoboCompLidar3D::TPoints SpecificWorker::read_data()
{
	auto data = lidar3d_proxy->getLidarDataWithThreshold2d("helios", 12000, 1);

	return door_detector.filter_points(data.points);
}

std::optional<std::pair<Eigen::Affine2f, float>> SpecificWorker::update_robot_pose(const RoboCompLidar3D::TPoints &data,
																				  bool transform_corners)
{
	const auto &[corners, lines] = room_detector.compute_corners(data, to_draw, &viewer->scene);
	// match corners  transforming first nominal corners to robot's frame
	Match match;
	if (transform_corners)
		match = hungarian.match(corners, nominal_rooms[current_room].transform_corners_to(robot_pose.inverse()));
	else
		match = hungarian.match(corners, nominal_rooms[current_room].corners());

	if (match.empty() or match.size() < 3)
		return {};

	const auto max_error_iter = std::ranges::max_element(match, [](const auto &a, const auto &b)
	  { return std::get<2>(a) < std::get<2>(b); });

	const auto max_match_error = std::get<2>(*max_error_iter);

	// create matrices W and b for pose estimation
	auto r = solve_pose(corners, match);
	if (r.array().isNaN().any())
	{
		qWarning() << __FUNCTION__ << "NaN values in r ";
		return {};
	}

	auto r_pose_copy = robot_pose;
	r_pose_copy.translate(Eigen::Vector2f(r(0), r(1)));
	r_pose_copy.rotate(r[2]);
	return {{r_pose_copy, max_match_error}};
}

void SpecificWorker::move_robot(float adv, float rot) const {
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

std::tuple<float, float> SpecificWorker::robot_controller(const Eigen::Vector2f &target) const {
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
		robot_pose.translate(Eigen::Vector2f(0.0, 0.0));

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

