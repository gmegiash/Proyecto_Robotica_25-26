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

//	Constantes de distancias
const float MAX_ADV = 600.0f;      // velocidad avance
const float MAX_ROT = 0.8f;        // velocidad rotación
const float OBSTACLE_DIST = 700;  // mm
const float WALL_DIST = 700;       // mm


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
		auto data = lidar3d_proxy->getLidarDataWithThreshold2d("helios", 50000, 1);
		if (data.points.empty()) return;

		const auto filter_data = filter_min_distance_cppintertools(data.points);
		qInfo() << filter_data.value().size();
		if (!filter_data.has_value())	return;

		auto filter_values = filter_data.value();

		draw_lidar(filter_values,&viewer->scene);

		auto distancias = calculateDistances(filter_values);
		//
		// qInfo() << "distancia Frontal:" << std::get<distFront>(distancias);
		// qInfo() << "Distancia derecha: "<< std::get<distRight>(distancias);
		// qInfo() << "distancia izquierda:" << std::get<distLeft>(distancias);

		// qInfo() << "Distancia derecha: "<< calculateDistRight(filter_data.value()) << "distancia Frontal:" << calculateDistForward(filter_data.value()) <<"distancia izquierda:" << calculateDistLeft(filter_data.value());

		// const QColor color2("aqua");
		// const QColor color3("aqua");
		// const auto dp1 = viewer->scene.addRect(-25, -25, 50, 50, QPen(QColor("aqua"), 10));
		// const auto dp2 = viewer->scene.addRect(-25, -25, 50, 50, QPen(QColor("red"), 10));
		// const auto dp3 = viewer->scene.addRect(-25, -25, 50, 50, QPen(QColor("yellow"), 10));
		// dp1->setPos(0, distFrontal);
		// dp2->setPos(distDer, 0);
		// dp3->setPos(-distIzq, 0);
		// qInfo() << "Distancia derecha: "<< distDer << "distancia Frontal:" << distFrontal <<"distancia izquierda:" << distIzq;

		switch (state)
		{
			case State::FORWARD:
			forwardState(distancias);
				break;
			case State::TURN:
			turnState(distancias);
				break;
			case State::FOLLOW_WALL:
				break;
		}

		//	Máquina de estados
		// switch (state)
		// {
		// 	case State::FORWARD:

		// 		} break;
		//
		// 	case State::TURN:
		// 		static int count = 0;
		// 		if (distFrontal > OBSTACLE_DIST * 2 && count > 10)
		// 		{
		// 			state = State::FOLLOW_WALL;
		// 			qInfo() << "Sigue la corriente bro y no te separes de la pared...";
		// 		} else{
		// 			omnirobot_proxy->setSpeedBase(0,0,MAX_ROT);
		// 			qInfo() << "Girando...";
		// 			count++;
		// 		} break;
		//
		// 	case State::FOLLOW_WALL:
		// 		if (distDer < WALL_DIST * 0.8)
		// 		{
		// 			//Gira izquierda
		// 			omnirobot_proxy->setSpeedBase(0, MAX_ADV * 0.8f, 0.3f);
		// 		} else if (distDer > WALL_DIST * 1.2)
		// 			{
		// 			//Gira derecha
		// 			omnirobot_proxy->setSpeedBase(0, MAX_ADV * 0.8f, -0.3f);
		// 			} else{
		// 			//Avanza recto
		// 			omnirobot_proxy->setSpeedBase(0, MAX_ADV, 0);
		// 			}
		//
		// 		if (distFrontal < OBSTACLE_DIST)
		// 		{
		// 			state = State::TURN;
		// 		}
		//
		// 	break;
		// }

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

void SpecificWorker::new_target_slot(QPointF p){}


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

// RoboCompLidar3D::TPoints SpecificWorker::filter_isolated_points(const RoboCompLidar3D::TPoints &points, float d)
// {
// 	if (points.empty()) return {};
//
// 	const float d_squared = d * d;  // Avoid sqrt by comparing squared distances
// 	std::vector<bool> hasNeighbor(points.size(), false);
//
// 	// Create index vector for parallel iteration
// 	std::vector<size_t> indices(points.size());
// 	std::iota(indices.begin(), indices.end(), size_t{0});
//
// 	// Parallelize outer loop - each thread checks one point
// 	std::for_each(std::execution::par, indices.begin(), indices.end(), [&](size_t i)
// 		{
// 			const auto& p1 = points[i];
// 			// Sequential inner loop (avoid nested parallelism)
// 			for (auto &&[j,p2] : iter::enumerate(points))
// 			{
// 				if (i == j) continue;
// 				const float dx = p1.x - p2.x;
// 				const float dy = p1.y - p2.y;
// 				if (dx * dx + dy * dy <= d_squared)
// 				{
// 					hasNeighbor[i] = true;
// 					break;
// 				}
// 			}
// 	});
//
// 	// Collect results
// 	std::vector<RoboCompLidar3D::TPoint> result;
// 	result.reserve(points.size());
// 	for (auto &&[i, p] : iter::enumerate(points))
// 		if (hasNeighbor[i])
// 			result.push_back(points[i]);
// 	return result;
// }


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

void SpecificWorker::forwardState(std::tuple<float,float,float> distances)
{
	if (std::get<distFront>(distances) < OBSTACLE_DIST)
	{
		state = State::TURN;
		omnirobot_proxy->setSpeedBase(0,0,0);
		qInfo() << "Obstaculo encontrado, girando...";
		return;
	}

	omnirobot_proxy->setSpeedBase(0,MAX_ADV,0);

}

void SpecificWorker::turnState(std::tuple<float,float,float> distances)
{
	if (std::get<distFront>(distances) > OBSTACLE_DIST * 2)
	{
		state = State::FORWARD;
		return;
	}

	if (std::get<distRight>(distances) > std::get<distLeft>(distances))
	{
		omnirobot_proxy->setSpeedBase(0,0,MAX_ROT);
	}
	else
	{
		omnirobot_proxy->setSpeedBase(0,0,-MAX_ROT);
	}
}

void SpecificWorker::follow_WallState(std::tuple<float,float,float> distances)
{

}

float SpecificWorker::calculateDistForward(const RoboCompLidar3D::TPoints &points)
{
	return std::get<distFront>(calculateDistances(points));
}

float SpecificWorker::calculateDistLeft(const RoboCompLidar3D::TPoints &points)
{
	return std::get<distLeft>(calculateDistances(points));
}

float SpecificWorker::calculateDistRight(const RoboCompLidar3D::TPoints &points)
{
	return std::get<distRight>(calculateDistances(points));
}

std::tuple<float,float,float> SpecificWorker::calculateDistances(const RoboCompLidar3D::TPoints &points)
{
	float distFrontal = 9999, distIzq = 9999, distDer = 9999;
	for (const auto &p : points)
	{
		float angle = atan2(p.y, p.x);

		float d = std::hypot(p.x, p.y);
		if (angle < (std::numbers::pi)*9/16 && angle > (std::numbers::pi)*7/16) distFrontal = std::min(distFrontal, d);
		if (angle < -(std::numbers::pi)*15/16 || angle > (std::numbers::pi)*15/16) distIzq = std::min(distIzq, d);
		if ((angle < (std::numbers::pi)*1/16 && angle > 0) || ( angle > -(std::numbers::pi)*1/16 && angle < 0)) distDer = std::min(distDer, d);
	}

	qInfo() << "distancia Frontal:" << distFrontal;
	qInfo() << "Distancia derecha: "<< distDer;
	qInfo() << "distancia izquierda:" << distIzq;

	return {distIzq, distFrontal, distDer};
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

