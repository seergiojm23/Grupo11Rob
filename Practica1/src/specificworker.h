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

/**
	\brief
	@author authorname
*/



#ifndef SPECIFICWORKER_H
#define SPECIFICWORKER_H

//#define HIBERNATION_ENABLED

#include <genericworker.h>
#include "abstract_graphic_viewer/abstract_graphic_viewer.h"
#include <expected>
#include <random>


class SpecificWorker : public GenericWorker
{
    Q_OBJECT
    public:
        SpecificWorker(TuplePrx tprx, bool startup_check);
        ~SpecificWorker();
        bool setParams(RoboCompCommonBehavior::ParameterList params);

    public slots:
        void initialize();
        void compute();
        void emergency();
        void restore();
        int startup_check();

    private:
        struct Params
        {
            float ROBOT_WIDTH = 460;  // mm //Esto es el ancho del robot
            float ROBOT_LENGTH = 480;  // mm //Esto es el largo del robot
            float MAX_ADV_SPEED = 1000; // mm/s //Esto es la velocidad máxima de avance
            float MAX_ROT_SPEED = 1; // rad/s //Esto es la velocidad máxima de rotación
            float STOP_THRESHOLD = MAX_ADV_SPEED*0.7; // mm //Esto es la distancia de parada
            float ADVANCE_THRESHOLD = ROBOT_WIDTH * 2; // mm //Esto es la distancia de avance
            float LIDAR_OFFSET = 9.f/10.f; // eight tenths of vector's half size //Esto es el offset del LIDAR que se usa para calcular la distancia de los puntos
            float LIDAR_FRONT_SECTION = 0.5; // rads, aprox 30 degrees //Esto es la sección frontal del LIDAR
            float REFERENCE_DISTANCE = ADVANCE_THRESHOLD * 1.1; // mm //Esto es la distancia de referencia
            std::string LIDAR_NAME_LOW = "bpearl"; //Esto es el nombre del LIDAR de abajo
            std::string LIDAR_NAME_HIGH = "helios"; //Esto es el nombre del LIDAR de arriba
            QRectF GRID_MAX_DIM{-5000, 2500, 10000, -5000}; //Esto es la dimensión máxima de la cuadrícula

        };
        Params params;

        bool startup_check_flag; //Esto es la bandera de comprobación de inicio
        AbstractGraphicViewer *viewer; //Esto es el visor gráfico

        // state machine
        enum class STATE {FORWARD, TURN, SPIRAL, FOLLOW_WALL}; //Esto es la máquina de estados
        STATE state = STATE::FORWARD; //Esto es el estado

        using RetVal = std::tuple<STATE, float, float>; //Esto es el tipo de retorno
        RetVal forward(auto &filtered_points);
        RetVal turn(auto &filtered_points);
        RetVal spiral(auto &filtered_points);
        RetVal follow_wall(auto &filtered_points);
        void draw_lidar(auto &filtered_points, QGraphicsScene *scene); //Esto es para dibujar los puntos del LIDAR
        QGraphicsPolygonItem* robot_draw; //Esto es para dibujar el robot
        std::expected<int, string> closest_lidar_index_to_given_angle(const auto &points, float angle); //Esto es para calcular el índice del punto del LIDAR más cercano al ángulo dado

        // random number generator
        std::random_device rd;
};

#endif