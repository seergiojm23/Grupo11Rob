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
#include <Eigen/Dense>
#include "abstract_graphic_viewer/abstract_graphic_viewer.h"
#include <algorithm>
#include <unordered_set>
#include <functional>
#include <tuple>
#include <tuple>
#include <boost/functional/hash.hpp>


class SpecificWorker : public GenericWorker
{
	Q_OBJECT
	public:
		//using GridPosition = std::optional<std::tuple<int, int>>;
		// Alias para representar una posición en la cuadrícula
		using GridPosition = std::optional<std::tuple<int, int>>;

		SpecificWorker(TuplePrx tprx, bool startup_check);
		~SpecificWorker();
		bool setParams(RoboCompCommonBehavior::ParameterList params);

		RoboCompGrid2D::Result Grid2D_getPaths(RoboCompGrid2D::TPoint source, RoboCompGrid2D::TPoint target);


	public slots:
		void initialize();
		void new_mouse_coordinates(QPointF);

	void update_grid(const std::vector<Eigen::Vector2f> &ldata_bpearl);

	void clear_grid();

	std::vector<QPointF> dijkstraAlgorithm(GridPosition start, GridPosition target);

	void draw_path(const vector<QPointF> & path, QGraphicsScene *scene);

	void compute();
		void emergency();
		void restore();
		int startup_check();
	private:
		bool startup_check_flag;
		std::vector<QPointF> path;

	struct Params
	{
		float ROBOT_WIDTH = 460;  // mm
		float ROBOT_LENGTH = 480;  // mm
		float MAX_ADV_SPEED = 1900; // mm/s
		float MAX_ROT_SPEED = 2; // rad/s
		float SEARCH_ROT_SPEED = 0.9; // rad/s
		float STOP_THRESHOLD = 700; // mm
		float ADVANCE_THRESHOLD = ROBOT_WIDTH * 3; // mm
		float LIDAR_FRONT_SECTION = 0.2; // rads, aprox 12 degrees
		// person
		float PERSON_MIN_DIST = 1200; // mm
		int MAX_DIST_POINTS_TO_SHOW = 300; // points to show in plot
		// lidar
		std::string LIDAR_NAME_LOW = "bpearl";
		std::string LIDAR_NAME_HIGH = "helios";
		QRectF GRID_MAX_DIM{-5000, 2500, 10000, -5000};
		// control track
		float acc_distance_factor = 2;
		float k1 = 1.1;  // proportional gain for the angle error;
		float k2 = 0.5; // proportional gain for derivative of the angle error;
	};
	Params params;

	// Definir el enum para los tipos de celda
	enum class State {
		Empty,      // Celda vacía
		Occupied,   // Celda ocupada
		Unknown,		//Desconocida
	};

	// Definir la estructura de la celda
	struct TCell {
		State state;  // Tipo de celda (usamos el enum CellType)
		QGraphicsRectItem *item;
		bool changed = false;
	};

	struct Node
	{
		GridPosition pos;
		float cost;

		// Sobrecarga de operador para ordenar en la priority_queue
		bool operator>(const Node &other) const
		{
			return cost > other.cost;
		}
	};


	// Declarar la cuadrícula como un arreglo 2D de celdas
	// Definir las dimensiones de la cuadrícula
	static constexpr int GRID_WIDTH_MM = 5000;
	static constexpr int GRID_HEIGHT_MM = 5000;
	static constexpr int CELL_SIZE_MM = 50;  // Tamaño de la celda en milímetros

	// Calcular el número de celdas en cada dimensión
	static constexpr int NUM_CELLS_X = GRID_WIDTH_MM / CELL_SIZE_MM;
	static constexpr int NUM_CELLS_Y = GRID_HEIGHT_MM / CELL_SIZE_MM;
	std::array<std::array<TCell, NUM_CELLS_X>, NUM_CELLS_Y> grid;

		// lidar
		std::vector<Eigen::Vector2f> read_lidar_bpearl();

		// draw
		AbstractGraphicViewer *viewer;
		void draw_lidar(auto &filtered_points, QGraphicsScene *scene);
		QGraphicsPolygonItem *robot_draw;

		TCell getCell(auto x, auto y);
		void setCell(auto x, auto y, TCell cell);
		QPointF get_lidar_point(int i, int j);
		std::optional<std::tuple<int, int>> get_grid_index(float x, float y);

		void regruesadoObstaculo();


};

#endif
