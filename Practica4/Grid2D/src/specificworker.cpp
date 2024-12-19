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
#include "specificworker.h"

#include <queue>
#include <cppitertools/enumerate.hpp>
#include <cppitertools/range.hpp>

/**
* \brief Default constructor
*/
SpecificWorker::SpecificWorker(TuplePrx tprx, bool startup_check) : GenericWorker(tprx)
{
	this->startup_check_flag = startup_check;
	// Uncomment if there's too many debug messages
	// but it removes the possibility to see the messages
	// shown in the console with qDebug()
//	QLoggingCategory::setFilterRules("*.debug=false\n");
}

/**
* \brief Default destructor
*/
SpecificWorker::~SpecificWorker()
{
	std::cout << "Destroying SpecificWorker" << std::endl;
}

bool SpecificWorker::setParams(RoboCompCommonBehavior::ParameterList params)
{
//	THE FOLLOWING IS JUST AN EXAMPLE
//	To use innerModelPath parameter you should uncomment specificmonitor.cpp readConfig method content
//	try
//	{
//		RoboCompCommonBehavior::Parameter par = params.at("InnerModelPath");
//		std::string innermodel_path = par.value;
//		innerModel = std::make_shared(innermodel_path);
//	}
//	catch(const std::exception &e) { qFatal("Error reading config params"); }
	

	return true;
}

void SpecificWorker::initialize()
{
	//Cuestionable
	static bool initialized = false; // Verifica si ya se inicializó
	if (initialized) return;
	initialized = true;
	//Fin Cuestionable
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
		//viewer->setStyleSheet("background-color: lightGray;");
		this->resize(800, 700);
		viewer->show();

		for(auto &&[i, row]: grid | iter::enumerate)
			for (auto &&[j, celda] : row | iter::enumerate)
			{
				celda.item = viewer->scene.addRect(CELL_SIZE_MM/2, CELL_SIZE_MM/2, CELL_SIZE_MM, CELL_SIZE_MM,
					 QPen(QColor("White"), 10), QBrush(QColor("Light Gray")));
				celda.item->setPos(get_lidar_point(i, j));
				celda.state = State::Unknown;
			}
		#ifdef HIBERNATION_ENABLED
			hibernationChecker.start(500);
		#endif

		//Para indicar con el ratón el punto del grid de la coordenada a donde debe dirigirse el robot mediante Dijkstra
		connect(viewer, SIGNAL(new_mouse_coordinates(QPointF)), this, SLOT(new_mouse_coordinates(QPointF)));

		this->setPeriod(STATES::Compute, 100);
		//this->setPeriod(STATES::Emergency, 500);
	}
}


void SpecificWorker::compute()
{
	if (parar_compute)
		return;

	//read bpearl (lower) lidar and draw
	auto ldata_bpearl = read_lidar_bpearl();
	if(ldata_bpearl.empty()) { qWarning() << __FUNCTION__ << "Empty bpearl lidar data"; return; };
	draw_lidar(ldata_bpearl, &viewer->scene);


	clear_grid();
	// Update grid
	std::lock_guard<std::mutex> lg (mutex);
	update_grid(ldata_bpearl);
	regruesadoObstaculo();

	draw_path(path, &viewer->scene);
	qDebug() <<path.size() << "path size";

	// clear

}

void SpecificWorker::update_grid(const std::vector<Eigen::Vector2f> &ldata_bpearl)
{
	const QBrush brush(QColor("White"));

	for (const auto &p : ldata_bpearl)
	{
		const auto salto = p.norm() / (CELL_SIZE_MM / 4); // Número de celdas en esa distancia
		const auto r = p.normalized();

		// Marcar celdas como vacías (Empty)
		for (const auto s : iter::range(0.f, p.norm(), salto))
		{
			const auto step = r * s;
			auto index_opt = get_grid_index(step.x(), step.y());

			// Verificar si el índice es válido antes de acceder
			if (index_opt.has_value())
			{
				const auto &[i, j] = index_opt.value();
				grid[i][j].state = State::Empty;
				grid[i][j].item->setBrush(brush);
			}
		}

		// Marcar la celda final como ocupada (Occupied)
		auto final_index_opt = get_grid_index(p.x(), p.y());
		if (final_index_opt.has_value())
		{
			const auto &[i, j] = final_index_opt.value();
			grid[i][j].state = State::Occupied;
			grid[i][j].item->setBrush(QBrush(QColor("Red")));
		}
	}
}


void SpecificWorker::clear_grid()
{
	const QBrush brush(QColor("Light Gray"));
	for (auto &row : grid)
		for (auto &[state, item, changed] : row) {
			item->setBrush(brush);
			state = State::Unknown;
			changed = false;
		}
}

// std::vector<QPointF> SpecificWorker::dijkstraAlgorithm(GridPosition start, GridPosition target)
// {
//     // Verificar que las posiciones de inicio y objetivo son válidas
//     if (!start.has_value() || !target.has_value())
//     {
//         qWarning() << "Posición de inicio o de objetivo no válida.";
//         return {};
//     }
//
//     auto [start_x, start_y] = start.value();
//     auto [target_x, target_y] = target.value();
//
//     // Verificar límites del inicio y el objetivo
//     if (start_x < 0 || start_y < 0 || start_x >= NUM_CELLS_X || start_y >= NUM_CELLS_Y ||
//         target_x < 0 || target_y < 0 || target_x >= NUM_CELLS_X || target_y >= NUM_CELLS_Y)
//     {
//         qWarning() << "Inicio u objetivo fuera de los límites del mapa.";
//         return {};
//     }
//
//     // Crear una matriz para indicar celdas no transitables + margen
//     std::vector<bool> non_traversable(NUM_CELLS_X * NUM_CELLS_Y, false);
//
//     // Ajustar el margen de obstáculos. Ejemplo: 3 celdas
//     const int MARGIN = 2;
//     for (int i = 0; i < NUM_CELLS_X; ++i)
//     {
//         for (int j = 0; j < NUM_CELLS_Y; ++j)
//         {
//             if (grid[i][j].state == State::Occupied)
//             {
//                 for (int dx = -MARGIN; dx <= MARGIN; ++dx)
//                 {
//                     for (int dy = -MARGIN; dy <= MARGIN; ++dy)
//                     {
//                         int nx = i + dx;
//                         int ny = j + dy;
//                         if (nx >= 0 && ny >= 0 && nx < NUM_CELLS_X && ny < NUM_CELLS_Y)
//                         {
//                             non_traversable[nx * NUM_CELLS_Y + ny] = true;
//                         }
//                     }
//                 }
//             }
//         }
//     }
//
//     // Si el objetivo es intransitable, retornar vacío
//     if (non_traversable[target_x * NUM_CELLS_Y + target_y])
//     {
//         qWarning() << "El objetivo está dentro de una zona no transitable.";
//         return {};
//     }
//
//     // Vectores para distancias y predecesores
//     std::vector<int> dist(NUM_CELLS_X * NUM_CELLS_Y, std::numeric_limits<int>::max());
//     std::vector<int> previous(NUM_CELLS_X * NUM_CELLS_Y, -1);
//
//     auto index = [=](int x, int y) { return x * NUM_CELLS_Y + y; };
//
//     int start_index = index(start_x, start_y);
//     int target_index = index(target_x, target_y);
//
//     dist[start_index] = 0;
//
//     // Movimientos (arriba, abajo, izquierda, derecha)
//     //const std::array<std::pair<int,int>,4> directions = {{{1,0},{-1,0},{0,1},{0,-1}}};
// 	const std::array<std::pair<int,int>,8> directions = {{{1,0},{-1,0},{0,1},{0,-1}, {1,1}, {1,-1}, {-1,1}, {-1,-1}}};
//     // Cola de prioridad para Dijkstra
//     using PQItem = std::pair<int,int>; // <dist, index>
//     std::priority_queue<PQItem, std::vector<PQItem>, std::greater<>> pq;
//     pq.push({0, start_index});
//
//     bool found = false;
//
//     while(!pq.empty())
//     {
//         auto [current_dist, current] = pq.top();
//         pq.pop();
//
//         if(current == target_index)
//         {
//             found = true;
//             break;
//         }
//
//         // Si ya tenemos una distancia mejor, seguimos
//         if(current_dist > dist[current])
//             continue;
//
//         int cx = current / NUM_CELLS_Y;
//         int cy = current % NUM_CELLS_Y;
//
//         for (auto [dx, dy] : directions)
//         {
//             int nx = cx + dx;
//             int ny = cy + dy;
//
//             if (nx < 0 || ny < 0 || nx >= NUM_CELLS_X || ny >= NUM_CELLS_Y)
//                 continue;
//
//             int n_index = index(nx, ny);
//
//             if (non_traversable[n_index])
//                 continue;
//
//             int new_dist = current_dist + 1;
//             if (new_dist < dist[n_index])
//             {
//                 dist[n_index] = new_dist;
//                 previous[n_index] = current;
//                 pq.push({new_dist, n_index});
//             }
//         }
//     }
//
//     if(!found)
//     {
//         qWarning() << "No se encontró un camino válido hasta el objetivo.";
//         return {};
//     }
//
//     // Reconstrucción del camino
//     std::vector<int> path_indices;
//     for(int at = target_index; at != -1; at = previous[at])
//         path_indices.push_back(at);
//
//     std::reverse(path_indices.begin(), path_indices.end());
//
//     // Convertir índices a (x,y)
//     std::vector<std::tuple<int,int>> path_cells;
//     path_cells.reserve(path_indices.size());
//     for(auto idx : path_indices)
//     {
//         int x = idx / NUM_CELLS_Y;
//         int y = idx % NUM_CELLS_Y;
//         path_cells.push_back({x,y});
//     }
//
//     // Convertir a QPointF
//     std::vector<QPointF> path_points;
//     path_points.reserve(path_cells.size());
//     for (const auto &[x, y] : path_cells)
//         path_points.emplace_back(get_lidar_point(x, y));
//
//     return path_points;
// }

std::vector<QPointF> SpecificWorker::dijkstraAlgorithm(GridPosition start, GridPosition target)
{
    // Verificar que las posiciones de inicio y objetivo son válidas
    if (!start.has_value() || !target.has_value())
    {
        qWarning() << "Posición de inicio o de objetivo no válida.";
        return {};
    }

    auto [start_x, start_y] = start.value();
    auto [target_x, target_y] = target.value();

    // Verificar límites del inicio y el objetivo
    if (start_x < 0 || start_y < 0 || start_x >= NUM_CELLS_X || start_y >= NUM_CELLS_Y ||
        target_x < 0 || target_y < 0 || target_x >= NUM_CELLS_X || target_y >= NUM_CELLS_Y)
    {
        qWarning() << "Inicio u objetivo fuera de los límites del mapa.";
        return {};
    }

    // Crear una matriz para indicar celdas no transitables + margen
    std::vector<bool> non_traversable(NUM_CELLS_X * NUM_CELLS_Y, false);

    // Ajustar el margen de obstáculos. Ejemplo: 2 celdas
    const int MARGIN = 2;
    for (int i = 0; i < NUM_CELLS_X; ++i)
    {
        for (int j = 0; j < NUM_CELLS_Y; ++j)
        {
            if (grid[i][j].state == State::Occupied)
            {
                // Marcar la celda ocupada y sus celdas vecinas dentro del margen como no transitables
                for (int dx = -MARGIN; dx <= MARGIN; ++dx)
                {
                    for (int dy = -MARGIN; dy <= MARGIN; ++dy)
                    {
                        int nx = i + dx;
                        int ny = j + dy;
                        if (nx >= 0 && ny >= 0 && nx < NUM_CELLS_X && ny < NUM_CELLS_Y)
                        {
                            non_traversable[nx * NUM_CELLS_Y + ny] = true;
                        }
                    }
                }
            }
        }
    }

    // Si el objetivo es intransitable, retornar vacío
    if (non_traversable[target_x * NUM_CELLS_Y + target_y])
    {
        qWarning() << "El objetivo está dentro de una zona no transitable.";
        return {};
    }

    // Vectores para distancias y predecesores
    std::vector<int> dist(NUM_CELLS_X * NUM_CELLS_Y, std::numeric_limits<int>::max());
    std::vector<int> previous(NUM_CELLS_X * NUM_CELLS_Y, -1);

    auto index = [=](int x, int y) { return x * NUM_CELLS_Y + y; };

    int start_index = index(start_x, start_y);
    int target_index = index(target_x, target_y);

    dist[start_index] = 0;

    // Movimientos (arriba, abajo, izquierda, derecha, y diagonales)
    const std::array<std::pair<int,int>,8> directions = {{{1,0},{-1,0},{0,1},{0,-1}, {1,1}, {1,-1}, {-1,1}, {-1,-1}}};

    // Cola de prioridad para Dijkstra
    using PQItem = std::pair<int,int>; // <dist, index>
    std::priority_queue<PQItem, std::vector<PQItem>, std::greater<>> pq;
    pq.push({0, start_index});

    bool found = false;

    while(!pq.empty())
    {
        auto [current_dist, current] = pq.top();
        pq.pop();

        if(current == target_index)
        {
            found = true;
            break;
        }

        // Si ya tenemos una distancia mejor, seguimos
        if(current_dist > dist[current])
            continue;

        int cx = current / NUM_CELLS_Y;
        int cy = current % NUM_CELLS_Y;

        for (auto [dx, dy] : directions)
        {
            int nx = cx + dx;
            int ny = cy + dy;

            // Verificar si la nueva casilla está dentro de los límites y no es intransitable
            if (nx < 0 || ny < 0 || nx >= NUM_CELLS_X || ny >= NUM_CELLS_Y)
                continue;

            int n_index = index(nx, ny);

            // Si la celda no es transitables, saltarla
            if (non_traversable[n_index])
                continue;

            int new_dist = current_dist + 1;
            if (new_dist < dist[n_index])
            {
                dist[n_index] = new_dist;
                previous[n_index] = current;
                pq.push({new_dist, n_index});
            }
        }
    }

    if(!found)
    {
        qWarning() << "No se encontró un camino válido hasta el objetivo.";
        return {};
    }

    // Reconstrucción del camino
    std::vector<int> path_indices;
    for(int at = target_index; at != -1; at = previous[at])
        path_indices.push_back(at);

    std::reverse(path_indices.begin(), path_indices.end());

    // Convertir índices a (x,y)
    std::vector<std::tuple<int,int>> path_cells;
    path_cells.reserve(path_indices.size());
    for(auto idx : path_indices)
    {
        int x = idx / NUM_CELLS_Y;
        int y = idx % NUM_CELLS_Y;
        path_cells.push_back({x,y});
    }

    // Convertir a QPointF
    std::vector<QPointF> path_points;
    path_points.reserve(path_cells.size());
    for (const auto &[x, y] : path_cells)
        path_points.emplace_back(get_lidar_point(x, y));

    return path_points;
}






void SpecificWorker::draw_path(const vector<QPointF> &path, QGraphicsScene *scene)
{
	static std::vector<QGraphicsItem*> items;   // store items so they can be shown between iterations

	// remove all items drawn in the previous iteration
	for(auto i: items)
	{
		scene->removeItem(i);
		delete i;
	}
	items.clear();

	const QBrush brush(QColor("Blue"));
	for (const auto &[x, y] : path)
	{
		auto i = scene->addEllipse(-25, -25, 50, 50, QPen (Qt::blue), brush);
		i->setPos(x,y);
		items.push_back(i);
	}

}


//////////////////////////////////////////////////////////////////
/// YOUR CODE HERE
//////////////////////////////////////////////////////////////////
// Read the BPEARL lidar data and filter the points
std::vector<Eigen::Vector2f> SpecificWorker::read_lidar_bpearl()
{
	try
	{
		auto ldata =  lidar3d_proxy->getLidarData("bpearl", 0, 2*M_PI, 1);
		// filter points according to height and distance
		std::vector<Eigen::Vector2f>  p_filter;
		for(const auto &a: ldata.points)
		{
			if(a.z < 500 and a.distance2d > 200)
				p_filter.emplace_back(a.x, a.y);
		}
		return p_filter;
	}
	catch(const Ice::Exception &e){std::cout << e << std::endl;}
	return {};
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
void SpecificWorker::draw_lidar(auto &filtered_points, QGraphicsScene *scene)
{
	static std::vector<QGraphicsItem*> items;   // store items so they can be shown between iterations

	// remove all items drawn in the previous iteration
	for(auto i: items)
	{
		scene->removeItem(i);
		delete i;
	}
	items.clear();

	auto color = QColor(Qt::darkGreen);
	auto brush = QBrush(QColor(Qt::darkGreen));
	for(const auto &p : filtered_points)
	{
		auto item = scene->addRect(-25, -25, 50, 50, color, brush);
		item->setPos(p.x(), p.y());
		items.push_back(item);
	}
}

SpecificWorker::TCell SpecificWorker::getCell(auto x, auto y)
{
	return grid[x][y];
}

void SpecificWorker::setCell(auto x, auto y, TCell value)
{
	grid[x][y] = value;
}

QPointF SpecificWorker::get_lidar_point(int i, int j)
{

	float x = GRID_WIDTH_MM/NUM_CELLS_X * i - (GRID_WIDTH_MM/2);
	float y = GRID_WIDTH_MM/NUM_CELLS_Y * j - (GRID_WIDTH_MM/2);

	return QPointF(x, y);
}

std::optional<std::tuple<int, int>> SpecificWorker::get_grid_index(float x, float y)
{
	// Cálculo inicial de las coordenadas de la cuadrícula
	int i = (static_cast<float>(NUM_CELLS_X) / GRID_WIDTH_MM) * x + (NUM_CELLS_X / 2);
	int j = (static_cast<float>(NUM_CELLS_Y) / GRID_WIDTH_MM) * y + (NUM_CELLS_Y / 2);

	// Comprobación de límites antes de aplicar el clamping
	if (i < 0 || i >= NUM_CELLS_X || j < 0 || j >= NUM_CELLS_Y)
	{
		return std::nullopt; // Retornar opcional vacío si está fuera de los límites
	}

	// Retornar las coordenadas dentro de los límites
	return std::make_tuple(i, j);
}

void SpecificWorker::regruesadoObstaculo()
{
	const QBrush brush(QColor("Red"));

	for(auto &&[i, row]: grid | iter::enumerate)
		for (auto &&[j, celda] : row | iter::enumerate)
		{
			if (celda.state == State::Occupied and not celda.changed) {
				// for (auto &[x, y]: directions)
				// {
				for (int x = -3; x < 4; x++)
					for (int y = -3; y < 4; y++)
					{
						auto sumai = i + x;
						auto sumaj = j + y;
						if (sumai > 0 and sumai < NUM_CELLS_X and sumaj > 0 and sumaj < NUM_CELLS_Y and not grid[sumai][sumaj].changed )
						{
							grid[sumai][sumaj].state = State::Occupied;
							grid[sumai][sumaj].item->setBrush(brush);
							grid[sumai][sumaj].changed = true;
						}
				}
			}
		}
}


void SpecificWorker::emergency()
{
    std::cout << "Emergency worker" << std::endl;
	//computeCODE
	//
	//if (SUCCESSFUL)
    //  emmit goToRestore()
}

//Execute one when exiting to emergencyState
void SpecificWorker::restore()
{
    std::cout << "Restore worker" << std::endl;
	//computeCODE
	//Restore emergency component

}

int SpecificWorker::startup_check()
{
	std::cout << "Startup check" << std::endl;
	QTimer::singleShot(200, qApp, SLOT(quit()));
	return 0;
}

void SpecificWorker::new_mouse_coordinates(QPointF p)
{
	qDebug() << "New mouse coordinates" << p;
	std::optional<std::tuple<int, int>> meta = get_grid_index(p.x(), p.y());
	std::optional<std::tuple<int, int>> salida = get_grid_index(0.f, 0.f);

	path = dijkstraAlgorithm(salida, meta);
}

void SpecificWorker::BorrarPersona(const std::tuple<int, int> &p)
{
	parar_compute.store(true);

	const auto &[x, y] = p;

	for (int i = -4; i < 5; i++)
		for (int j = -4; j < 5; j++)
		{
			if (x + i < NUM_CELLS_X and x+i > 0 and y + j < NUM_CELLS_Y and y + j > 0 )
				grid[x + i][y + j].state = State::Empty;
		}
}

//////////////////////////////////////////////////////////////////////////////////////////////////////

RoboCompGrid2D::Result SpecificWorker::Grid2D_getPaths(RoboCompGrid2D::TPoint source, RoboCompGrid2D::TPoint target)
{
	auto p = get_grid_index(target.x, target.y);

	// Si no hay persona, no retornamos un camino vacío directamente,
	// sino que devolvemos el último path calculado.
	if (not p.has_value())
	{
		qWarning() << "No se ha encontrado persona en la posición solicitada. Devolviendo último path calculado.";
		RoboCompGrid2D::Result res1;
		for (const auto &[x, y] : path) // 'path' ya es un miembro de la clase
		{
			res1.path.emplace_back(x, y);
		}
		return res1;
	}

	std::optional<std::tuple<int, int>> meta = get_grid_index(target.x, target.y);
	std::optional<std::tuple<int, int>> salida = get_grid_index(0.f, 0.f);

	BorrarPersona(p.value());

	// Si hay persona, calculamos un nuevo path con Dijkstra
	std::lock_guard<std::mutex> lg (mutex);
	auto new_path = dijkstraAlgorithm(salida, meta);

	if (!new_path.empty())
	{
		path = new_path;
	}

	parar_compute.store(false);

	RoboCompGrid2D::Result res;
	for (const auto &[x, y] : path)
	{
		res.path.emplace_back(x, y);
	}

	return res;

#ifdef HIBERNATION_ENABLED
	hibernation = true;
#endif
	//implementCODE
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
// From the RoboCompGrid2D you can use this types:
// RoboCompGrid2D::TPoint
// RoboCompGrid2D::Result

