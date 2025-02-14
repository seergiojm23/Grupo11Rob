/*
 *    Copyright (C) 2024 by YOUR NAME HERE
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
#include <ranges>
#include "specificworker.h"

/**
* \brief Default constructor
*/
SpecificWorker::SpecificWorker(TuplePrx tprx, bool startup_check) : GenericWorker(tprx) {
    this->startup_check_flag = startup_check;
    //	QLoggingCategory::setFilterRules("*.debug=false\n");
}

/**
* \brief Default destructor
*/
SpecificWorker::~SpecificWorker() {
    std::cout << "Destroying SpecificWorker" << std::endl;
}

bool SpecificWorker::setParams(RoboCompCommonBehavior::ParameterList params) {
    return true;
}

void SpecificWorker::initialize() {
    std::cout << "Initialize worker" << std::endl;
    if (this->startup_check_flag) {
        this->startup_check();
    } else {
        ///////////// Your code ////////
        // Viewer
        viewer = new AbstractGraphicViewer(this->frame, params.GRID_MAX_DIM); //Crea un viewer con el frame y el tamaño del grid
        auto [r, e] = viewer->add_robot(params.ROBOT_WIDTH, params.ROBOT_LENGTH, 0, 100, QColor("Blue"));
        //Esto lo que hace es que el robot se dibuje en la posición 0,100 y de color azul y se guarda en la variable robot_draw
        robot_draw = r;
        viewer->show();//Muestra el viewer

        ///////////////////////////////
#ifdef HIBERNATION_ENABLED
			hibernationChecker.start(500);
#endif

        this->setPeriod(STATES::Compute, 100);
    }
}

void SpecificWorker::compute() {
    RoboCompLidar3D::TData ldata; //Variable para guardar los datos del LIDAR
    try { ldata = lidar3d_proxy->getLidarData("bpearl", 0, 2 * M_PI, 1); } catch (const Ice::Exception &e) {
        std::cout << e << std::endl;
    }
    //Esto lo que hace es que se obtienen los datos del LIDAR y se guardan en la variable ldata

    RoboCompLidar3D::TPoints p_filter; //Variable para guardar los puntos filtrados
    std::ranges::copy_if(ldata.points, std::back_inserter(p_filter),
                         [](auto &a) { return a.z < 500 and a.distance2d > 200; });

    //Esto lo que hace es que se copian los puntos del LIDAR que cumplen la condición en p_filter

    draw_lidar(p_filter, &viewer->scene); //Dibuja los puntos del LIDAR que cumplen la condición en el viewer

    /// Añade máquina de estados con los métodos forward, turn, spiral y follow_wall
    RetVal ret_val; //Variable para guardar el valor de retorno de la máquina de estados

    switch (state) {
        case STATE::FORWARD: {
            ret_val = forward(p_filter);
            break;
        }
        case STATE::TURN: {
            ret_val = turn(p_filter);
            break;
        }

        case STATE::SPIRAL: {
            ret_val = spiral(p_filter);
            break;
        }

        case STATE::FOLLOW_WALL: {
            ret_val = follow_wall(p_filter);
            break;
        }
    }
    /// unpack  the tuple
    auto [st, adv, rot] = ret_val; //Desempaqueta el valor de retorno de la máquina de estados
    state = st; //Guarda el estado en la variable state
    //Las tuplas son de la forma (estado, velocidad de avance, velocidad de rotación)

    /// Send movements commands to the robot
    try { omnirobot_proxy->setSpeedBase(0, adv, rot); } catch (const Ice::Exception &e) { std::cout << e << std::endl; }
    //Esto lo que hace es que se envían los comandos de movimiento al robot con la velocidad de avance y rotación
}

///////////////////////////////////////////////////////////////////////////////////////////////////


/**
 * Analyzes the filtered points to determine whether to continue moving forward or to stop and turn.
 *
 * This method examines the central part of the `filtered_points` vector to find the minimum distance
 * point within that range. If the minimum distance is less than the width of the robot, it indicates
 * an obstacle is too close, prompting a state change to `TURN` and stopping motion. Otherwise,
 * the robot continues to move forward.
 *
 * @param filtered_points A vector of filtered points representing the robot's perception of obstacles.
 * @return A `RetVal` tuple consisting of the state (`FORWARD` or `TURN`), speed, and rotation.
 */
SpecificWorker::RetVal SpecificWorker::forward(auto &points) {
    // check if the central part of the filtered_points vector has a minimum lower than the size of the robot
    auto offset_begin = closest_lidar_index_to_given_angle(points, -params.LIDAR_FRONT_SECTION);
    auto offset_end = closest_lidar_index_to_given_angle(points, params.LIDAR_FRONT_SECTION);

    //El objetivo de encontrar el primer punto a la izquierda y el primero a la derecha es delimitar un rango de puntos que representen la zona frontal del robot.

    if (offset_begin and offset_end) { //Si hay lecturas válidas
        auto min_point = std::min_element(std::begin(points) + offset_begin.value(),
                                          std::begin(points) + offset_end.value(), [](auto &a, auto &b) {
                                              return a.distance2d < b.distance2d;
                                          });
        //Busca el punto más cercano dentro del sector frontal del LIDAR, entre offset_begin y offset_end

        //Si el punto más cercano está a una distancia menor que el umbral de parada, se detiene y cambia de estado
        if (min_point != points.end() and min_point->distance2d < params.STOP_THRESHOLD)
            return RetVal(STATE::TURN, 0.f, 0.f); // stop and change state if obstacle detected
        else
            return RetVal(STATE::FORWARD, params.MAX_ADV_SPEED, 0.f);
    } else // no valid readings
    {
        qWarning() << "No valid readings. Stopping";
        return RetVal(STATE::FORWARD, 0.f, 0.f);
    }
}

/**
 * @brief Checks if the central part of the provided filtered points is free to proceed and determines the next state.
 *
 * This function inspects the central third of the filtered points vector to find the point with the minimum distance.
 * If the minimum distance in this central region is greater than twice the robot's width, the robot will switch to
 * the FORWARD state. Otherwise, it will continue to TURN.
 *
 * @param filtered_points A vector containing points with distance information used for making navigation decisions.
 * @returns A tuple containing the next state (FORWARD or TURN), and speed values.
 */
SpecificWorker::RetVal SpecificWorker::turn(auto &points) {
    // Instantiate the random number generator and distribution
    static std::mt19937 gen(rd());
    static std::uniform_int_distribution<int> dist(0, 1);
    static bool first_time = true; // Controla si es la primera vez que gira
    static int cont = 0;
    static int sign = 1;

    static std::random_device rd;
    static std::uniform_int_distribution<int> distri(0, 4);

    int randomvalue = distri(gen);

    // Comprobar si la parte central estrecha de los puntos filtrados está libre para avanzar
    auto offset_begin = closest_lidar_index_to_given_angle(points, -params.LIDAR_FRONT_SECTION);
    auto offset_end = closest_lidar_index_to_given_angle(points, params.LIDAR_FRONT_SECTION);

    // Salir si no hay lecturas válidas
    if (not offset_begin or not offset_end) {
        qWarning() << "No valid readings. Stopping";
        return RetVal(STATE::FORWARD, 0.f, 0.f);
    }

    // Encontrar el punto mínimo en la parte central
    auto min_point = std::min_element(std::begin(points) + offset_begin.value(),
                                      std::begin(points) + offset_end.value(), [](auto &a, auto &b) {
                                          return a.distance2d < b.distance2d;
                                      });

    // Si hay suficiente espacio en frente, decidir si hacer FOLLOW_WALL o SPIRAL
    if (min_point != std::end(points) and min_point->distance2d > params.ADVANCE_THRESHOLD) {
        if (cont < 4) {
            cont++; // Marcar que ya ha girado por primera vez
            return RetVal(STATE::FOLLOW_WALL, 0.f, 0.f); // La primera vez va a FOLLOW_WALL
        } else {
            if(randomvalue == 0) {
                return RetVal(STATE::FORWARD, 0.f, 0.f); // La primera vez va a FOLLOW_WALL
            }
            else {
                return RetVal(STATE::SPIRAL, 0.f, 0.f); // En las siguientes veces irá a SPIRAL
            }
        }
    }

    // Continuar girando
    auto min_point_all = std::ranges::min_element(points, [](auto &a, auto &b) { return a.distance2d < b.distance2d; });

    // Si el phi del min_point_all es negativo, girar a la derecha; si no, a la izquierda. Si está cerca de cero, gira aleatoriamente
    if (first_time) {
        if (min_point_all->phi < 0.1 and min_point_all->phi > -0.1) {
            sign = dist(gen);
            sign = (sign == 0) ? -1 : 1;
        } else {
            sign = (min_point_all->phi > 0) ? -1 : 1; // Determinar dirección según el ángulo
        }
        first_time = false; // Marcar que ya ha girado la primera vez
    }

    return RetVal(STATE::TURN, 0.f, sign * params.MAX_ROT_SPEED);
}


SpecificWorker::RetVal SpecificWorker::spiral(auto &points) {
    // check if the central part of the filtered_points vector has a minimum lower than the size of the robot
    auto offset_begin = closest_lidar_index_to_given_angle(points, -params.LIDAR_FRONT_SECTION);
    auto offset_end = closest_lidar_index_to_given_angle(points, params.LIDAR_FRONT_SECTION);
    static int cont = 0;

    if (offset_begin and offset_end) {
        auto min_point = std::min_element(std::begin(points) + offset_begin.value(),
                                          std::begin(points) + offset_end.value(), [](auto &a, auto &b) {
                                              return a.distance2d < b.distance2d;
                                          });
        if (min_point != points.end() and min_point->distance2d < params.STOP_THRESHOLD) {
            cont++;
            return RetVal(STATE::TURN, 0.f, 0.f); // stop and change state if obstacle detected
        }
        else {
            static float spiral_adv_speed = 100.f;
            static float spiral_rot_speed = params.MAX_ROT_SPEED;


            spiral_adv_speed = std::min(spiral_adv_speed + 10.f, params.MAX_ADV_SPEED);
            spiral_rot_speed = std::max(spiral_rot_speed - 0.001f, 0.001f);

            static std::mt19937 gen(rd());
            static std::uniform_int_distribution<int> dist(0, 1);
            static int sign = 1;

            auto min_point_all = std::ranges::min_element(points, [](auto &a, auto &b) { return a.distance2d < b.distance2d; });
            if (min_point_all->phi < 0.1 and min_point_all->phi > -0.1) {
                sign = dist(gen);
                sign = (sign == 0) ? -1 : 1;
            } else {
                sign = (min_point_all->phi > 0) ? -1 : 1; // Determinar dirección según el ángulo
            }

            if(cont == 7) {
                spiral_adv_speed = 100.f;
                spiral_rot_speed = params.MAX_ROT_SPEED;
                cont = 0;
            }
            return RetVal(STATE::SPIRAL, spiral_adv_speed, sign * spiral_rot_speed);
        }
    } else {
        qWarning() << "No valid readings. Stopping";
        return RetVal(STATE::FORWARD, 0.f, 0.f);
    }
}

SpecificWorker::RetVal SpecificWorker::follow_wall(auto &points) {
    // Encontrar los índices correspondientes al ángulo frontal del LIDAR
    auto offset_begin = closest_lidar_index_to_given_angle(points, -params.LIDAR_FRONT_SECTION);
    auto offset_end = closest_lidar_index_to_given_angle(points, params.LIDAR_FRONT_SECTION);
    if (offset_begin and offset_end) {
        auto min_point = std::min_element(std::begin(points) + offset_begin.value(),
                                          std::begin(points) + offset_end.value(), [](auto &a, auto &b) {
                                              return a.distance2d < b.distance2d;
                                          });
        if (min_point != points.end() and min_point->distance2d < params.STOP_THRESHOLD)
            return RetVal(STATE::TURN, 0.f, 0.f); // stop and change state if obstacle detected
        else {
            // Determinar la posición de la pared y ajustar la velocidad de avance y rotación
            if (min_point->phi < 0 and min_point->distance2d > params.REFERENCE_DISTANCE) {
                return RetVal(STATE::FOLLOW_WALL, params.STOP_THRESHOLD, -0.3 * params.MAX_ROT_SPEED);
                // Gira a la izquierda (lejos)
            } else if (min_point->phi > 0 and min_point->distance2d > params.REFERENCE_DISTANCE) {
                return RetVal(STATE::FOLLOW_WALL, params.STOP_THRESHOLD, 0.3 * params.MAX_ROT_SPEED);
                // Gira a la derecha (lejos)
            } else if (min_point->phi < 0 and min_point->distance2d < params.REFERENCE_DISTANCE) {
                return RetVal(STATE::FOLLOW_WALL, params.MAX_ADV_SPEED, 0.3 * params.MAX_ROT_SPEED);
                // Gira a la derecha (cerca)
            } else if (min_point->phi > 0 and min_point->distance2d < params.REFERENCE_DISTANCE) {
                return RetVal(STATE::FOLLOW_WALL, params.MAX_ADV_SPEED, -0.3 * params.MAX_ROT_SPEED);
                // Gira a la izquierda (cerca)
            }
        }
    } else {
        qWarning() << "No valid readings. Stopping";
        return RetVal(STATE::FORWARD, 0.f, 0.f); // Detener si no hay lecturas válidas
    }
    return RetVal(STATE::FORWARD, 0.f, 0.f); // Detener si no hay lecturas válidas
}

/**
 * Draws LIDAR points onto a QGraphicsScene.
 *
 * This method clears any existing graphical items from the scene, then iterates over the filtered
 * LIDAR points to add new items. Each LIDAR point is represented as a colored rectangle. The point
 * with the minimum distance is highlighted in red, while the other points are drawn in green.
 *
 * @param filtered_points A collection of filtered points to be drawn, each containing the coordinates
 *                        and distance.
 * @param scene A pointer to the QGraphicsScene where the points will be drawn.
 */
void SpecificWorker::draw_lidar(auto &filtered_points, QGraphicsScene *scene) {
    static std::vector<QGraphicsItem *> items; // store items so they can be shown between iterations

    // remove all items drawn in the previous iteration
    for (auto i: items) {
        scene->removeItem(i);
        delete i;
    }
    items.clear();

    auto color = QColor(Qt::green);
    auto brush = QBrush(QColor(Qt::green));
    for (const auto &p: filtered_points) {
        auto item = scene->addRect(-50, -50, 100, 100, color, brush);
        item->setPos(p.x, p.y);
        items.push_back(item);
    }
    // compute and draw minimum distance point
    auto p_min = std::ranges::min_element(filtered_points, [](auto &a, auto &b) {
        return a.distance2d < b.distance2d;
    });
    auto item = scene->addRect(-150, -150, 300, 300, QColor(Qt::red), QBrush(QColor(Qt::red)));
    item->setPos(p_min->x, p_min->y);
    items.push_back(item);


    // Draw two lines coming out from the robot at angles given by params.LIDAR_OFFSET
    // Calculate the end points of the lines
    //float angle1 = params.LIDAR_FRONT_SECTION / 2.f;
    auto res_right = closest_lidar_index_to_given_angle(filtered_points, params.LIDAR_FRONT_SECTION / 2.f);
    auto res_left = closest_lidar_index_to_given_angle(filtered_points, -params.LIDAR_FRONT_SECTION / 2.f);
    if (res_right and res_left) {
        float right_line_length = filtered_points[res_right.value()].distance2d;
        float left_line_length = filtered_points[res_left.value()].distance2d;
        float angle1 = filtered_points[res_left.value()].phi;
        float angle2 = filtered_points[res_right.value()].phi;
        QLineF line_left{
            QPointF(0.f, 0.f),
            robot_draw->mapToScene(left_line_length * sin(angle1), left_line_length * cos(angle1))
        };
        QLineF line_right{
            QPointF(0.f, 0.f),
            robot_draw->mapToScene(right_line_length * sin(angle2), right_line_length * cos(angle2))
        };
        QPen left_pen(Qt::blue, 10); // Blue color pen with thickness 3
        QPen right_pen(Qt::red, 10); // Blue color pen with thickness 3
        auto line1 = scene->addLine(line_left, left_pen);
        auto line2 = scene->addLine(line_right, right_pen);
        items.push_back(line1);
        items.push_back(line2);
    } else
        std::cout << res_right.error() << " " << res_left.error() << std::endl;

    // update UI
    lcdNumber_minangle->display(atan2(p_min->x, p_min->y));
    lcdNumber_mindist->display(p_min->distance2d);
}

/**
 * @brief Calculates the index of the closest lidar point to the given angle.
 *
 * This method searches through the provided list of lidar points and finds the point
 * whose angle (phi value) is closest to the specified angle. If a matching point is found,
 * the index of the point in the list is returned. If no point is found that matches the condition,
 * an error message is returned.
 *
 * @param points The collection of lidar points to search through.
 * @param angle The target angle to find the closest matching point.
 * @return std::expected<int, string> containing the index of the closest lidar point if found,
 * or an error message if no such point exists.
 */
std::expected<int, string> SpecificWorker::closest_lidar_index_to_given_angle(const auto &points, float angle) {
    // search for the point in points whose phi value is closest to angle
    auto res = std::ranges::find_if(points, [angle](auto &a) { return a.phi > angle; });
    if (res != std::end(points))
        return std::distance(std::begin(points), res);
    else
        return std::unexpected("No closest value found in method <closest_lidar_index_to_given_angle>");
}

///////////////////////////////////////////////////////////////////////////////////////////////////
void SpecificWorker::emergency() {
    std::cout << "Emergency worker" << std::endl;
    //computeCODE
    //
    //if (SUCCESSFUL)
    //  emmit goToRestore()
}

//Execute one when exiting to emergencyState
void SpecificWorker::restore() {
    std::cout << "Restore worker" << std::endl;
    //computeCODE
    //Restore emergency component
}

int SpecificWorker::startup_check() {
    std::cout << "Startup check" << std::endl;
    QTimer::singleShot(200, qApp, SLOT(quit()));
    return 0;
}

/**************************************/
// From the RoboCompLidar3D you can call this methods:
// this->lidar3d_proxy->getLidarData(...)
// this->lidar3d_proxy->getLidarDataArrayProyectedInImage(...)
// this->lidar3d_proxy->getLidarDataProyectedInImage(...)
// this->lidar3d_proxy->getLidarDataWithThreshold2d(...)

/**************************************/
// From the RoboCompLidar3D you can use this types:
// RoboCompLidar3D::TPoint
// RoboCompLidar3D::TDataImage
// RoboCompLidar3D::TData

/**************************************/
// From the RoboCompOmniRobot you can call this methods:
// this->omnirobot_proxy->correctOdometer(...)
// this->omnirobot_proxy->getBasePose(...)
// this->omnirobot_proxy->getBaseState(...)
// this->omnirobot_proxy->resetOdometer(...)
// this->omnirobot_proxy->setOdometer(...)
// this->omnirobot_proxy->setOdometerPose(...)
// this->omnirobot_proxy->setSpeedBase(...)
// this->omnirobot_proxy->stopBase(...)

/**************************************/
// From the RoboCompOmniRobot you can use this types:
// RoboCompOmniRobot::TMechParams