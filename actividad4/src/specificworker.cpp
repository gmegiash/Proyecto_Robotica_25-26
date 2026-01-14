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
	auto data = read_data();

	if (!door_detector.get_current_door().has_value())
	{
		qInfo() << "No door detected";
		return;
	}
	auto door = door_detector.get_current_door().value();

	estimated_center = center_estimator.estimate(data);

	// compute corners
	const auto &[corners, lines] = room_detector.compute_corners(data, &viewer->scene);
	draw_lidar(data, estimated_center, &viewer->scene);

	auto [nominal_room, match, max_match_error] = compute_match(corners);

	// update robot pose
	if (localised)
	{
		label_localised->setText("True");
		draw_current_room(nominal_room, &viewer_room->scene);
		if (const auto res = update_robot_pose(current_room, corners, true); res.has_value())
		{
			robot_pose = res.value().first;
			max_match_error = res.value().second;
			time_series_plotter->addDataPoint(match_error_graph,max_match_error);
		}
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
	static auto last_valid_time = std::chrono::steady_clock::now();
	static float last_adv = 0.0f;
	static float last_rot = 0.0f;
	const int BLIND_THREASHOLD_MS = 2000; //2 segundos de persistencia

	QGraphicsScene *scene = &viewer->scene;

	Doors doors;

	if ((doors = door_detector.doors()).empty()) {
		//Calculamos cuanto tiempo hace que perdimos la puerta
		auto now = std::chrono::steady_clock::now();
		auto elapsed_ms = std::chrono::duration_cast<std::chrono::milliseconds>(now - last_valid_time).count();

		if (elapsed_ms < BLIND_THREASHOLD_MS) {
			//Seguimos con la ultima velocidad conocida
			return {STATE::GOTO_DOOR, last_adv, last_rot};
		}
		else {
			//Se acaba el tiempo, paramos
			qInfo() << __FUNCTION__ << "No detecto puerta desde hace " << elapsed_ms << "ms. Parando. ";
			return {STATE::GOTO_DOOR, 0.f, 0.f};
		}
	}

	//Hemos visto puertas -> Reseteamos el temporizados
	last_valid_time = std::chrono::steady_clock::now();

	//Seleccion de la mejor puerta
	Door target_door;
	if (localised && current_room != -1 && current_door != -1){
		//Si esta localizado, buscamos la que coincide con la puerta nominal del mapa
		const auto dn = nominal_rooms[current_room].doors[current_door];
		const auto sd = std::ranges::min_element(doors, [dn, this](const auto &a, const auto &b)
			{ return (a.center() - robot_pose.inverse() * dn.center_global()).norm() < (b.center() - robot_pose.inverse() * dn.center_global()).norm(); });
		target_door = *sd;
	}
	else {
		//Si no esta localizado, cogemos la que terngamos mas enfrente
		const auto sd = std::ranges::min_element(doors, [](const auto &a, const auto &b)
			{ return abs(a.p1_angle) < abs(b.p1_angle); });
		target_door = *sd;
	}

	//calculamos el punto objetivo antes de cruzar la puerta
	const auto target = target_door.center_before(robot_pose.translation(), params.DIST_TARGET_BEFORE_DOOR);
	const auto dist_to_door = target.norm();

	static QGraphicsItem *door_target_draw = nullptr;
	if (door_target_draw != nullptr)
		scene->removeItem(door_target_draw);

	door_target_draw = scene->addEllipse(-50, -50, 100, 100, QPen(Qt::magenta), QBrush(Qt::magenta));
	door_target_draw->setPos(target.x(), target.y());

	//Condicion de exito
	if (dist_to_door < params.DOOR_REACHED_DIST) {
		last_adv = 0;
		last_rot = 0;
		return{STATE::ORIENT_TO_DOOR, 0.f, 0.f};
	}

	//Controlador
	const auto &[adv, rot] = robot_controller(target);

	last_adv = adv;
	last_rot = rot;

	return {STATE::GOTO_DOOR, adv, rot};
}

SpecificWorker::RetVal SpecificWorker::turn()
{
	auto [detected,room_idx, left_right] = rc::ImageProcessor::check_colour_patch_in_image(camera360rgb_proxy);
	if (detected)
	{
		current_room = room_idx;
		localised = true;

		if (!nominal_rooms[current_room].visited)
		{
			qInfo() << "Habitación " << current_room << " visitada por primera vez. Guardando puertas.";
			nominal_rooms[current_room].visited = true;

			auto detected_doors = door_detector.doors();
			nominal_rooms[current_room].doors.clear(); // Limpiamos para llenar con datos reales

			// Transformamos las puertas detectadas (locales) a globales y las guardamos
			for(const auto &d : detected_doors)
			{
				// Convertir p1 y p2 a coordenadas globales usando la pose actual del robot
				Eigen::Vector2f p1_g = robot_pose * d.p1;
				Eigen::Vector2f p2_g = robot_pose * d.p2;

				// Guardamos una nueva puerta "Nominal" en el mapa
				Door global_door;
				global_door.p1 = p1_g;
				global_door.p2 = p2_g;
				nominal_rooms[current_room].doors.push_back(global_door);
			}
		}

		if (door_crossing.valid)
		{
			// Recuperamos índices de donde veníamos
			int prev_room_idx = door_crossing.leaving_room_index;
			int prev_door_idx = door_crossing.leaving_door_index;
			int curr_door_idx = door_crossing.entering_door_index;

			// Actualizamos el grafo: La puerta de atrás conecta con la de aquí
			nominal_rooms[prev_room_idx].doors[prev_door_idx].connects_to_room = current_room;
			nominal_rooms[current_room].doors[curr_door_idx].connects_to_room = prev_room_idx;

			qInfo() << "CONEXIÓN REALIZADA: Room " << prev_room_idx << " <--> Room " << current_room;

			door_crossing.valid = false; // Ya hemos consumido el evento de cruce
		}

		current_door = choose_next_door(current_room);

		if (current_door != -1)
			return {STATE::GOTO_DOOR, 0, 0};

		return {STATE::IDLE, 0, 0};

	}

	float rot = params.RELOCAL_ROT_SPEED;
	if (left_right == -1) rot = -rot; // Girar hacia la izquierda si la pista lo indica

	return {STATE::TURN, 0, rot};
}

int SpecificWorker::choose_next_door(int room_idx) const
{
	const auto &doors = nominal_rooms[room_idx].doors;

	// --- FASE 1: EXPLORACIÓN ---
	// Objetivo: Ir a donde nunca hemos ido.
	// Buscamos puertas que tengan 'connects_to_room == -1'.
	// Esto significa que sabemos que hay puerta, pero no qué hay detrás.
	std::vector<int> unknown_doors;
	for (int i = 0; i < doors.size(); ++i)
	{
		if (doors[i].connects_to_room == -1)
		{
			unknown_doors.push_back(i);
		}
	}

	// Si hay puertas desconocidas, elegimos una (puede ser la primera o aleatoria entre ellas)
	if (!unknown_doors.empty())
	{
		qInfo() << "Estrategia: Explorar puerta desconocida.";
		return unknown_doors[0];
	}

	// --- FASE 2: NAVEGACIÓN / ANTI-BUCLE ---
	// Si llegamos aquí, es que YA CONOCEMOS todas las puertas de esta habitación.
	// Estamos en una habitación "resuelta". Debemos salir para seguir buscando en otro lado.

	// Regla: Intentar NO volver inmediatamente por la puerta por la que acabamos de entrar (current_door),
	// a menos que sea un callejón sin salida (solo 1 puerta).

	if (doors.size() > 1)
	{
		// Buscamos cualquier puerta que NO sea la actual
		for (int i = 0; i < doors.size(); ++i)
		{
			if (i != current_door) // current_door es por la que entramos tras el TURN
			{
				qInfo() << "Estrategia: Evitar volver atrás, tomando ruta alternativa.";
				return i;
			}
		}
	}

	// --- FASE 3: CALLEJÓN SIN SALIDA ---
	// Si solo hay 1 puerta y ya la conocemos (es por la que entramos),
	// no hay opción: tenemos que dar la vuelta.
	qInfo() << "Estrategia: Callejón sin salida, volviendo.";
	return 0;
}

SpecificWorker::RetVal SpecificWorker::orient_to_door(const RoboCompLidar3D::TPoints &points, const Door &door)
{
	// 1. Re-detectar puertas locales para asegurar precisión a corta distancia
	auto detected_doors = door_detector.doors();

	// Safety: Si perdemos visual de la puerta (oclusión/ruido), mantenemos un giro suave de búsqueda
	if (detected_doors.empty()) {
		return {STATE::ORIENT_TO_DOOR, 0.f, 0.2f};
	}

	// 2. Seleccionar la puerta física más cercana (asumimos que es la que tenemos delante)
	auto it = std::ranges::min_element(detected_doors, [](const Door &a, const Door &b){
	   return a.center().norm() < b.center().norm();
	});
	const Door &target_local_door = *it;

	// 3. Calcular el ángulo hacia el centro de la puerta
	// Queremos que el centro de la puerta esté en ángulo 0 (eje X del robot)
	auto center = target_local_door.center();
	float angle_to_center = std::atan2(center.y(), center.x());

	// 4. Comprobar alineación
	// params.RELOCAL_MAX_ORIENTED_ERROR suele ser ~0.1 rad (5-6 grados)
	if (std::abs(angle_to_center) < params.RELOCAL_MAX_ORIENTED_ERROR)
	{
		qInfo() << __FUNCTION__ << "Alineado con puerta (Error:" << angle_to_center << " rad). Cruzando.";
		// Paramos rotación y pasamos a cruzar
		return {STATE::CROSS_DOOR, 0.f, 0.f};
	}

	// 5. Girar para corregir el error angular
	// Controlador P simple: velocidad proporcional al error
	float Kp = 1.2f;
	float rot = Kp * angle_to_center;

	// Saturación para evitar giros bruscos
	if (rot > params.MAX_ROT_SPEED) rot = params.MAX_ROT_SPEED;
	if (rot < -params.MAX_ROT_SPEED) rot = -params.MAX_ROT_SPEED;

	// Solo rotamos, no avanzamos
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

		door_crossing.track_entering_door(door_detector.doors());
		return {STATE::GOTO_ROOM_CENTER, adv, rot}; //Estado actual y velociades calculadas
	}

	return {STATE::GOTO_ROOM_CENTER, 0.f, 0.f};
}

SpecificWorker::RetVal SpecificWorker::cross_door(const RoboCompLidar3D::TPoints &points, const Door &door)
{
	//Variables para el tiempo
	static bool first_time = time;
	static std::chrono::time_point<std::chrono::high_resolution_clock> start;

	//Constantes de la manioblra
	const float CROSS_SPEED = 500.0f;
	const long CROSS_TIME_MS = 3000; //3 segundos -> recorre 1.5 metros aprox

	if (first_time) {
		first_time = false;
		start = std::chrono::high_resolution_clock::now();

		localised = false;

		return {STATE::CROSS_DOOR, CROSS_SPEED, 0.0f};
	}

	const auto elapsed = std::chrono::high_resolution_clock::now() - start;
	const auto elapsed_ms = std::chrono::duration_cast<std::chrono::milliseconds>(elapsed).count();

	if (elapsed_ms <= CROSS_TIME_MS) {
		return {STATE::CROSS_DOOR, CROSS_SPEED, 0.0f};
	}

	first_time = true;

	if (current_room < 0 || current_door < 0) {
		qWarning() << __FUNCTION__ << "Indices de habitacion/puerta invalidos. Cambiamos a LOCALISE.";
		return {STATE::LOCALISE, 0.f, 0.f};
	}

	//Obtenemos informacion de la puerta que acabamos de cruzar
	const auto &leaving_door = nominal_rooms[current_room].doors[current_door];
	int next_room_idx = leaving_door.connects_to_room;

	//Decision de estado

	//Entrando a una habitacion visitada
	if (next_room_idx >= 0 && next_room_idx < (int)nominal_rooms.size() && nominal_rooms[next_room_idx].visited) {
		//Actualizamos indices a la nueva habitacion
		int next_door_idx = leaving_door.connects_to_door;
		current_room = next_room_idx;
		current_door = next_door_idx;

		//Recalculamos la pose del robot en la nueva habitacion
		const auto &entering_door = nominal_rooms[current_room].doors[current_door];
		Eigen::Vector2f door_center = entering_door.center_global();

		//Inicializamos la pose del robot relativa a esa puerta
		robot_pose.setIdentity();
		door_center.y() -= 500; //Ajuste manual segun referencia
		robot_pose.translate(door_center);

		//Ya estamos localizados en la nueva habitacion
		localised = true;
		qInfo() << "Entrando a habitacion conocida " << current_room << ". Localizando. ";
		return {STATE::GOTO_ROOM_CENTER, 0.f, 0.f};
	}

	//Entrando en habitacion desconocida
	else {
		//Guardamos informacion para el algoritmo de 'DoorCrossing'
		door_crossing = DoorCrossing{current_room, current_door};

		//Marcamos la puerta de salida como visitada
		nominal_rooms[current_room].doors[current_door].visited = true;

		qInfo() << "Entrando a habitacion desconocida. Iniciando LOCALISE.";
		return {STATE::LOCALISE, 0.f, 0.f};
	}
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

SpecificWorker::RetVal SpecificWorker::process_state(const RoboCompLidar3D::TPoints &data, const Door &door)
{
	switch(state) {
		case STATE::IDLE:               return {STATE::IDLE, 0, 0};

		case STATE::GOTO_ROOM_CENTER:   return goto_room_center(data);
		case STATE::TURN:               return turn();
		case STATE::GOTO_DOOR:          return goto_door(data, door);
		case STATE::ORIENT_TO_DOOR:     return orient_to_door(data, door);
		case STATE::CROSS_DOOR:         return cross_door(data, door);
		//case STATE::LOCALISE: 			return localise(data, )
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

	auto puertas = door_detector.doors();

}

RoboCompLidar3D::TPoints SpecificWorker::read_data()
{
	auto data = lidar3d_proxy->getLidarDataWithThreshold2d("helios", 12000, 1);

	return door_detector.filter_points(data.points);
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

std::optional<std::pair<Eigen::Affine2f, float>> SpecificWorker::update_robot_pose(int room_index,
																				  const Corners &corners,
																				  bool transform_corners)
{
	// match corners  transforming first nominal corners to robot's frame
	Match match;
	if (transform_corners)
		match = hungarian.match(corners, nominal_rooms[room_index].transform_corners_to(robot_pose.inverse()));
	else
		match = hungarian.match(corners, nominal_rooms[room_index].corners());


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

