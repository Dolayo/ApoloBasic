#include "EGKRRT.h"
#include <algorithm>
#include "ShipState.h"
#include "defines.h"

//! -------------------------------------- Planner -------------------------------------- 

bool EGKRRT::setStartAndGoalStates(RobotState* start_, RobotState* goal_)
{
	if (SBPathPlanner::setStartAndGoalStates(start_, goal_))
	{
		_tree->rootTree(start_);
		*goal = *goal_;
		return true;
	}
	return false;
}

bool EGKRRT::testingPlan()
{
	ShipState* EGKgoal = dynamic_cast<ShipState*>(goal);
	EGKgoal->setYaw(TEST_YAW);
	this->_tree->_vertexes.push_back(EGKgoal);
	RobotState* addedNode = _tree->addNode(goal);
	if (addedNode && addedNode->isEqual(goal))
		//if (dynamic_cast<ShipState*>(addedNode)->isSamePos(goal))
		{	
			solved = true;
			//retrive path
			RobotPath pathA = _tree->getPathFromRoot(addedNode);

			//rearrange the states
			delete path;

			path = new RobotPath(pathA);

			//path->filterLoops(); //clean loops if any 
			return true;
		}
	return false;
}

bool EGKRRT::computePlan(int maxiterations)
{
	if (solved)return true;

	for (int i = 0; i < maxiterations; i++)
	{
		RobotState* node = getNextSample();

		//--! Testing purposes !--//
		if(i==0)
			dynamic_cast<ShipState*>(node)->setPose(Vector3D(8, 9, 0));
		if(i==1)
			dynamic_cast<ShipState*>(node)->setPose(Vector3D(1, 4, 0));
		if (i == 2)
			dynamic_cast<ShipState*>(node)->setPose(Vector3D(8, 2, 0));
		if (i == 3)
			dynamic_cast<ShipState*>(node)->setPose(Vector3D(8, -8, 0));
		if (i == 4)
			dynamic_cast<ShipState*>(node)->setPose(Vector3D(3, -9, 0));
		if (i == 5)
			dynamic_cast<ShipState*>(node)->setPose(Vector3D(1, -4, 0));

		/*if (i == 0)
			dynamic_cast<ShipState*>(node)->setPose(Vector3D(8, -8, 0));
		if (i == 1)
			dynamic_cast<ShipState*>(node)->setPose(Vector3D(7, -3, 0));
		if (i == 2)
			dynamic_cast<ShipState*>(node)->setPose(Vector3D(5, 2, 0));*/
		//--! Testing purposes !--//


		if (!node)continue;

		RobotState* addedNode = _tree->addNode(node);

		if (addedNode) 
		{
			ShipState* EGK_addedNode = dynamic_cast<ShipState*>(addedNode);
			if (EGK_addedNode && EGK_addedNode-> isSamePos(goal))
			{
				solved = true;

				_tree->PopulateVertexes();

				//retrive each path
				EGKRobotPath* pathA = _tree->GetPathFromRoot(EGK_addedNode); //esto hay que cambiarlo para que se genere un EGKpath, hay que hacerlo con punteros y no con objetos tal cual

				//rearrange the states
				delete path;

				path = &(*pathA);

				path->filterLoops(); //clean loops if any 

				return solved;
			}
			
		}
		delete node;
	}

	_tree->PopulateVertexes();
	return false;
}

//! -------------------------------------- Tree -------------------------------------- 

RobotState* EGKRRT::EGKtree::addNode(RobotState* node)
{
	/*RobotState* in = dynamic_cast<ShipState*>(node);
	if (!in)
		return nullptr;*/

	// node es en realidad un ShipState*
	// primer nodo del segmento que tratamos de crear al añadir el nodo in
	RobotState* initNode;
	RobotState* n = node->clone();

	// tomo el punto de conexion: estado y segmento
	// closest_path es el segmento del arbol mas cercano al nodo que tratamos de añadir(in/n)
	// initNode almacena el nodo de dicho segmento que esta mas cerca de n
	PathSegment* closest_path = getClosestPathSegment(n, &initNode);

	// Si no hay ningun segmento es que el arbol esta vacio, solo tiene el origen
	if (closest_path == 0)
	{
		initNode = _root;
		if (dynamic_cast<ShipState*>(initNode))
			dynamic_cast<ShipState*>(initNode)->setCost(0);

	}

	// Compruebo si n puede tener vecinos, si es asi los almaceno en neighbors, y busco el 
	// de menor coste el cual lo almaceno en initNode
	vector<RobotState*> neighbors;

	if ((_radius > n->distanceTo(initNode)) && (_paths.size() != 0))
	{
		getNeighbors(n, &neighbors);

		closest_path = getBest(neighbors, &initNode);
	}

	// El segmento mas cercano aux queda dividido en dos por el nuevo segmento a añadir, por lo que
	// se añade un nuevo segmento newPathA que va antes de initNode

	// newPathA es el segmento que va desde el principio de aux(antiguo) a initNode
	// closest_path se cambia para que empiece en initNode pero mantenga su nodo final

	// La division en dos segmentos solo tiene sentido si el nodo mas cercano es parte intermedia de un segmento
	// si es el principio o final de un segmento no habra division en dos segmentos

	if (closest_path != nullptr)
	{
		if (_paths.size() != 0 && !(closest_path->_init->isEqual(initNode)) && !(closest_path->_end->isEqual(initNode)))
		{
			bool b_aux = false;

			// Creamos el path desde el inicio del antiguo segmento hasta el punto intermedio initNode
			PathSegment* newPathA = EGKpath::createPath(closest_path->_init, initNode, b_aux);


			// Agregamos todos los nodos intermedios recien creados del nuevo segmento createPath a la lista global de nodos del arbol
			for (RobotState* i : newPathA->_inter)
				add(i);

			erase_vertex(closest_path->_end);

			//_vertexes.push_back(newPathA->_end);

			newPathA->_init = closest_path->_init;
			closest_path->_init = newPathA->_end;//closest_path->_init = initNode;

			newPathA->_parent = closest_path->_parent;
			closest_path->_parent = newPathA;

			//newPathA->_end = closest_path->_init;

			PathSegment* aux_path = EGKpath::createPath(closest_path->_init, closest_path->_end, b_aux);
		

			closest_path->_inter.assign(aux_path->_inter.begin(), aux_path->_inter.end());
			closest_path->_end = aux_path->_end;

			delete aux_path;

			//_vertexes.push_back(closest_path->_end);

			for (RobotState* i : closest_path->_inter)
				add(i);

			_paths.push_back(newPathA);	

			initNode = closest_path->_init;
		}
	}

	bool success;

	// este segmento sera el que una al nuevo nodo con el segmento que contiene al nodo de 
	// menor coste
	// creamos el nuevo segmento desde initNode hasta n
	//dynamic_cast<ShipState*>(initNode)->setVels(Vector3D(V_MAX,0,0));
	EGKpath* newPath = EGKpath::createPath(initNode, n, success, NUM_ITER_PATH, true);

	//SOlo para debugeo, para ver con que angulo acaba
	double yaw = dynamic_cast<ShipState*>(newPath->_end)->getYaw() * 180/PI;
	//~SOlo para debugeo
	// 
	//_vertexes.push_back(newPath->_end);

	for (RobotState* i : newPath->_inter)
		add(i);

	// Si el nuevo segmento esta vacio, eliminamos todo 
	if (newPath->size() == 0) 
	{
		delete newPath;
		delete n;

		return 0;//no state created
	}

	// inicializo el nuevo segmento con el nodo de menor coste
	newPath->_init = initNode;

	//el padre del nuevo segmento es el segmento que contiene al nodo de menor coste
	newPath->_parent = findPath4Node(initNode);

	// si no se ha llegado al final destruyo la copia
	if (!success)
		delete n;

	// añado el nuevo path al arbol
	_paths.push_back(newPath);

	if ((_paths.size() != 0) && (neighbors.size() != 0))
	{
		Reconnect(neighbors, newPath->_end);
	}

	// devuelvo el extremo del nuevo segmento añadido
	return newPath->_end;

}

void EGKRRT::EGKtree::Reconnect(vector<RobotState*>& v_nei, RobotState* xnew)
{
	// Ordeno los vecinos de mayor a menor coste
	sort(
			v_nei.begin(), 
			v_nei.end(), 
			[](RobotState* const s1, RobotState* const s2)
			{
				if (dynamic_cast<ShipState*>(s1) && dynamic_cast<ShipState*>(s2))
					return dynamic_cast<ShipState*>(s1)->getCost() > dynamic_cast<ShipState*>(s2)->getCost();
				else return false;
			}
		);

	for (RobotState* v : v_nei)
	{
		ShipState* Xnew = dynamic_cast<ShipState*>(xnew);
		ShipState* vecino = dynamic_cast<ShipState*>(v);
		if (Xnew && vecino)
		{
			bool b_reach = false;

			// El nuevo segmento que une a Xnew con el vecino
			EGKpath* newRePath = EGKpath::createPath(Xnew, vecino, b_reach);
			if (newRePath->_inter.empty())
				continue;
			double distance = newRePath->getLength();

			// Si el coste de un vecino es mayor que la distancia a Xnew mas el coste de Xnew, creo el nuevo segmento de Xnew al vecino
			if (((distance + Xnew->getCost()) < vecino->getCost()) && b_reach)
			{

				// Buscamos a que segmento pertenece este vecino
				PathSegment* oldPath = findPath4Node(vecino);
				if (!oldPath)
				{
					delete newRePath;
					continue;
				}

				//bool debug_a = erase_vertex(oldPath->_init);
				//bool debug_b = erase_vertex(oldPath->_end);

				newRePath->_init = Xnew;
				oldPath->_init = newRePath->_end;//oldPath->_init = vecino;

				newRePath->_parent = findPath4Node(Xnew);

				oldPath->_parent = newRePath;

				//newRePath->_end = oldPath->_init;

				// Deleteamos los nodos de la lista del arbol y de la lista de vecinos
				for (RobotState* oldState_i : oldPath->_inter)
				{
					if (!(oldState_i == vecino))
					{
						for (int j = 0; j < _nodes.size(); ++j)
						{
							if (oldState_i == _nodes[j])
							{
								//delete nodes[j];
								_nodes.erase(_nodes.begin() + j);
								break;
							}
						}
					}

					else break;
				}

				// borro los punteros de old path que iban antes del vecino
				//oldPath->_inter.erase(oldPath->_inter.begin(), oldPath->_inter.begin() + pos_v);
				bool b_temp = false;

				EGKpath* temp = EGKpath::createPath(oldPath->_init, oldPath->_end, b_temp);

				oldPath->_inter.assign(temp->_inter.begin(), temp->_inter.end());

				oldPath->_end = temp->_end;

				delete temp;

				if (oldPath->_end == nullptr)
					oldPath->_end = oldPath->_init;

				for (RobotState* i: newRePath->_inter)
						add(i);

				//_vertexes.push_back(oldPath->_init);
				//_vertexes.push_back(oldPath->_end);
				

				_paths.push_back(newRePath);
			}

			else
			{
				// Existe un choque o el vecino no tiene un coste interesante
				delete newRePath;
				continue;
			}
		}
	}
}

//double EGKRRT::EGKtree::distance(RobotState* rs, PathSegment* path, RobotState** mnode)
//{
//	ShipState* p = dynamic_cast<ShipState*>(rs);
//	ShipState* mn = dynamic_cast<ShipState*>(path->_init);
//	if (!p || !mn)
//		return -1.0;
//
//	bool b_aux = false;
//
//	EGKpath* path_aux = EGKpath::createPath(path->_init, p, b_aux);
//
//	double minimal = path_aux->getLength();
//
//	if (!b_aux)
//		minimal *= 1.5;
//
//	//end belongs to the path
//	
//	for (RobotState* i: path->_inter)
//	{
//		ShipState* i_aux = dynamic_cast<ShipState*>(i);
//		if (i_aux)
//		{
//			path_aux = EGKpath::createPath(i_aux, p, b_aux);
//
//			double val = path_aux->getLength();
//
//			if (!b_aux)
//				val *= 1.5;// Sustituyo al rayo de visibilidad
//
//			if (val < minimal)
//			{
//				mn = i_aux;
//				minimal = val;
//			}
//		}
//		else
//			continue;
//	}
//	if (mnode)
//		*mnode = mn;
//	return minimal;
//}

EGKRobotPath* EGKRRT::EGKtree::GetPathFromRoot(ShipState* n)
{
	EGKRobotPath* path = new EGKRobotPath();// hay que incluirlo en los destructores
	ShipState* rs;
	PathSegment* p = nullptr;

	// Busco el path que contiene en su _end al nodo GOAL
	for (PathSegment* i: _paths)
		if (i->_end == n) 
		{ 
			p = i; 
			break; 
		}

	if (p == nullptr)
		return path;

	EGKpath* p_egk = dynamic_cast<EGKpath*>(p);

	if (p_egk)
	{
		for (auto a : p_egk->_sequence)
		{
			_sequence_solution.push_back(a);
		}
	}

	for (int i = (int)p->_inter.size() - 1; i >= 0; i--)
		path->path.insert(path->path.begin(), (p->_inter)[i]);

	while (p->_parent != nullptr) 
	{
		//busco el nodo de inicio
		p = p->_parent;

		for (int i = (int)p->_inter.size() - 1; i >= 0; i--)
			path->path.insert(path->path.begin(), p->_inter[i]);

		for (int i = (int)p_egk->_sequence.size() - 1; i >= 0; i--)
			_sequence_solution.insert(_sequence_solution.begin(), p_egk->_sequence[i]);
	}

	path->path.insert(path->path.begin(), _root);

	return path;
}

double EGKRRT::EGKtree::distance(RobotState* p, PathSegment* path, RobotState** mnode)
{
	RobotState* mn = path->_init;
	
	if (dynamic_cast<ShipState*>(path->_init))
	{
		Vector3D ghost_pos = dynamic_cast<ShipState*>(path->_init)->getGhostPos();

		if (dynamic_cast<ShipState*>(p))
		{
			double minimal = dynamic_cast<ShipState*>(p)->distanceTo(ghost_pos);
			double val;
			//end belongs to the path
			for (RobotState* i: path->_inter)
			{
				if (dynamic_cast<ShipState*>(i))
				{
					ghost_pos = dynamic_cast<ShipState*>(i)->getGhostPos();
					val = dynamic_cast<ShipState*>(p)->distanceTo(ghost_pos);
					if (val < minimal) {
						mn = i;
						minimal = val;
					}
				}
				else continue;
			}

			if (mnode)*mnode = mn;
			return minimal;
		}
	}
	return -1.0;
}

RDTstar::RDTtree::PathSegment* EGKRRT::EGKtree::getBest(vector<RobotState*>& v_nei, RobotState** best)
{

	if (v_nei.size() == 0)
		return nullptr;

	*best = v_nei[0];

	double min_cost = 0;

	if (dynamic_cast<ShipState*>(v_nei[0]))
	{
		min_cost = dynamic_cast<ShipState*>(v_nei[0])->getCost();
	}


	PathSegment* bestPath = _paths[0];

	for (unsigned int i = 1; i < v_nei.size(); i++)
	{
		double cost_i;
		if (dynamic_cast<ShipState*>(v_nei[i]))
		{
			cost_i = dynamic_cast<ShipState*>(v_nei[i])->getCost();

			if (cost_i < min_cost)
			{
				min_cost = cost_i;
				*best = v_nei[i];
			}
		}
	}

	bestPath = findPath4Node(*best);

	if (bestPath) return bestPath;
	else return nullptr;
}

//RDTstar::RDTtree::PathSegment* EGKRRT::EGKtree::getClosestPathSegment(RobotState* n, RobotState** minstate)
//{
//	*minstate = 0;
//	if (_paths.size() == 0)return 0;
//	RobotState* ms;
//	double dist, minimun = distance(n, _paths[0], minstate);
//	PathSegment* minPath = _paths[0];
//	for (unsigned int i = 1; i < _paths.size(); i++) {
//		dist = distance(n, _paths[i], &ms);
//		if (dist < minimun) {
//			minimun = dist;
//			minPath = _paths[i];
//			*minstate = ms;
//		}
//	}
//	return minPath;
//}

//RDTstar::RDTtree::PathSegment* EGKRRT::EGKtree::getBest(vector<RobotState*>& v_nei, RobotState** best)
//{
//
//	if (v_nei.size() == 0)
//		return nullptr;
//
//	*best = v_nei[0];
//
//	double min_cost = 0;
//
//	if (dynamic_cast<ShipState*>(v_nei[0]))
//		min_cost = dynamic_cast<ShipState*>(v_nei[0])->getCost();
//
//
//
//	PathSegment* bestPath = _paths[0];
//
//	for (unsigned int i = 1; i < v_nei.size(); i++)
//	{
//		double cost_i;
//		if (dynamic_cast<ShipState*>(v_nei[i]))
//		{
//			cost_i = dynamic_cast<ShipState*>(v_nei[i])->getCost();
//
//			if (cost_i < min_cost)
//			{
//				min_cost = cost_i;
//				*best = v_nei[i];
//			}
//		}
//	}
//
//	bestPath = findPath4Node(*best);
//
//	if (bestPath) return bestPath;
//	else return nullptr;
//}

void EGKRRT::EGKtree::PopulateVertexes()
{
	for (auto p : _paths)
	{
		_vertexes.push_back(p->_init);
		_vertexes.push_back(p->_end);
	}
}

//! -------------------------------------- Path -------------------------------------- 
//! 
EGKRRT::EGKtree::EGKpath* EGKRRT::EGKtree::EGKpath::createPath(RobotState* p_init, RobotState* p_end, bool& b_success, int niter, bool b_ensure_yaw)
{
	EGKpath* p_newPath = new EGKpath();
	ShipState* p_initState = dynamic_cast<ShipState*>(p_init);
	ShipState* p_finalState = dynamic_cast<ShipState*>(p_end);

	if (!p_initState || !p_finalState)
			return nullptr;


	p_newPath->appendState(p_initState);

	p_newPath->_init = p_initState;

	
	
	p_initState->placeRobot();

	ShipState* p_newState = nullptr;/* , * p_prevState = p_init;*/

	b_success = false; //solo se pone a true si se logra la solucion

	// Registramos el valor relativo inicial de la orientacion
	double init_yaw = p_finalState->getYaw() - p_initState->getYaw();

	bool b_yaw_ensured = false;

	for (int n = 0; n < niter; ++n)
	{
		if (p_initState->isSamePos(p_finalState))
		{
			b_success = true;
			p_newPath->appendState(p_initState);
			p_newPath->_end = p_newState;
			double ang = 180*(p_initState->getYaw())/PI;//DEBUG
			return p_newPath;
		}

		// Obtain the control action
		// Si el angulo de alejamiento ya se ha conseguido, no hace falta seguir usando la opcion de asegurar yaw
		if (b_yaw_ensured)
			b_ensure_yaw = false;

		std::vector<double> v_ctrlAct = p_newPath->navigation(p_initState, p_finalState, init_yaw, b_yaw_ensured, b_ensure_yaw);

		//if (p_newPath->isGhostThere(p_initState, p_finalState))
		//{
		//	v_ctrlAct[0] = 0.0;
		//	v_ctrlAct[1] = 0.0;
		//	v_ctrlAct[2] = 0.0;//Just for simple dynamics
		//} 

		// Propagate the control action
		bool b_success_prop = p_initState->propagate(v_ctrlAct, DELTA_T, &p_newState);

		if (b_success_prop)
		{
			//ShipState* auxtemp = new ShipState();
			p_newState->setCost(p_newState->distanceTo(p_initState) + p_initState->getCost());

			
			p_newPath->appendState(p_newState);
			p_newPath->_end = p_newState;
			p_newPath->appendCtrlAct(v_ctrlAct);

			p_initState = p_newState;
		}
		else
			return p_newPath;
	}

	return p_newPath;
}

std::vector<double> EGKRRT::EGKtree::EGKpath::navigation(RobotState* p_initState, RobotState* p_finalState, double& ar_init_yaw, bool& b_yaw_ensured, bool b_ensure_yaw)
{
	std::vector<double> v_auxCtrlAct;

	ShipState* p_ShipInitState = nullptr;
	ShipState* p_ShipFinalState = nullptr;

	if (dynamic_cast<ShipState*>(p_initState) && dynamic_cast<ShipState*>(p_finalState))
	{
		p_ShipInitState = dynamic_cast<ShipState*>(p_initState);
		p_ShipFinalState = dynamic_cast<ShipState*>(p_finalState);
	}
	else
		return v_auxCtrlAct;
	
	double distance = p_ShipInitState->distanceTo(p_ShipFinalState);

	// OLD
	/*double angAbsFinalState = std::atan(p_ShipFinalState->getPose().y / p_ShipFinalState->getPose().x);
	double angle = angAbsFinalState - p_ShipInitState->getPose().z;*/

	double x_Rel = p_ShipFinalState->getPose().x - p_ShipInitState->getPose().x;
	double y_Rel = p_ShipFinalState->getPose().y - p_ShipInitState->getPose().y;

	//double angle = PI - std::atan(y_Rel/x_Rel);
	// 
	//Debemos distinguir en que cuadrante esta el goal respecto a init
	Quadrant quad = Quadrant::first;

	if (x_Rel < 0.0 && y_Rel >= 0.0)
	{
		quad = Quadrant::second;
	}

	else
		if(x_Rel >= 0.0 && y_Rel < 0.0)
		{
			quad = Quadrant::fourth;
		}
		else
			if (x_Rel < 0.0 && y_Rel < 0.0)
			{
				quad = Quadrant::third;
			}

	// Angulo en coordenadas absolutas entre el punto final y el estado inicial, sin considerar la orientacion del robot
	double angle = 0.0;

	switch(quad)
	{
		case Quadrant::first:
		{
			angle = std::atan(y_Rel / x_Rel);
			break;
		}
		case Quadrant::second:
		{
			angle = std::atan(y_Rel / x_Rel);
			angle = PI + angle;
			break;
		}
		case Quadrant::third:
		{
			angle = std::atan(y_Rel / x_Rel);
			angle = angle - PI;
			break;
		}
		case Quadrant::fourth:
		{
			angle = std::atan(y_Rel / x_Rel);
			break;
		}

	}

	// Angulo en coordenadas relativas al robot teniendo en cuenta la orientacion
	angle = angle - p_ShipInitState->getYaw();

	if (std::abs(angle) > PI)
	{
		if (angle > 0.0)
			angle = angle - 2 * PI;
		else
			angle = 2 * PI + angle;
	}

	// Determinamos en que zona se encuentra el destino respecto al sistema de referencia del robot
	ZoneType zone = ZoneType::right;

	if (angle > 0)
		zone = ZoneType::left;

	if ((distance < DIST1) && (std::abs(angle) < THETA1))
		zone = ZoneType::central;

	else
	{
		if ((distance < DIST2) && (std::abs(angle) < THETA2))
			zone = ZoneType::central;

		else
		{
			if((distance > DIST2) && (std::abs(angle) < THETA3))
				zone = ZoneType::central;
		}
	}

	//! +++++++++++++++++++++ ONLY for testing purposes
	if (b_ensure_yaw)
	{
		b_yaw_ensured = true;
		//_p_circ = new Circunference(p_ShipInitState, p_ShipFinalState);
		_p_spline = new Spline(p_ShipInitState, p_ShipFinalState);
	}
	//! //! +++++++++++++++++++++ ONLY for testing purposes

	if(b_yaw_ensured)//b_yaw_ensured
	{
		//generateCtrlActCirc(p_ShipInitState, quad, zone, v_auxCtrlAct);
		generateCtrlActSpline(p_ShipInitState, quad, zone, v_auxCtrlAct);
		/*if(_p_spline->IsOk())
			generateCtrlActSpline(p_ShipInitState, quad, zone, v_auxCtrlAct);
		else
		{
			bool nope = true;
		}*/

		if(v_auxCtrlAct.size()==0)
		{
			v_auxCtrlAct.push_back(0.);
			v_auxCtrlAct.push_back(0.);
			v_auxCtrlAct.push_back(0.);
		}
		return v_auxCtrlAct;// dale una vuelta a como es el flujo trambolico de toda esta funcion
	}
	

	//switch (zone)
	//{
	//case ZoneType::right:
	//	{
	//	if (b_ensure_yaw)
	//	{
	//		double yaw_rel = p_ShipFinalState->getYaw() - p_ShipInitState->getYaw();

	//		if (std::abs(yaw_rel) < std::abs(REENTRY_ANGLE_K * ar_init_yaw))
	//		{
	//			if (yaw_rel > 0.0)// Si el angulo es positivo, objetivo a la izda, giro a la derecha
	//			{
	//				v_auxCtrlAct.push_back(THRUSTX);
	//				v_auxCtrlAct.push_back(0.);
	//				v_auxCtrlAct.push_back(-THRUSTW);
	//			}
	//			else// Si el angulo es negativo, objetivo a la dcha, giro a la izquierda
	//			{
	//				v_auxCtrlAct.push_back(THRUSTX);
	//				v_auxCtrlAct.push_back(0.);
	//				v_auxCtrlAct.push_back(THRUSTW);
	//			}
	//		}
	//		else
	//		{
	//			b_yaw_ensured = true;
	//			_p_circ = new Circunference(p_ShipInitState, p_ShipFinalState);

	//			v_auxCtrlAct.push_back(THRUSTX);
	//			v_auxCtrlAct.push_back(0.);
	//			v_auxCtrlAct.push_back(-THRUSTW);
	//		}

	//	}
	//	else
	//	{
	//		v_auxCtrlAct.push_back(THRUSTX);
	//		v_auxCtrlAct.push_back(0.);
	//		v_auxCtrlAct.push_back(-THRUSTW);
	//	}

	//	break;
	//	}

	//case ZoneType::central:
	//	{
	//		double distance = p_ShipInitState->distanceTo(p_ShipFinalState);
	//		double yaw_rel = p_ShipFinalState->getYaw() - p_ShipInitState->getYaw();

	//		if (b_ensure_yaw)
	//		{
	//			if(std::abs(yaw_rel) < std::abs(REENTRY_ANGLE_K * ar_init_yaw))
	//			{
	//				if (yaw_rel > 0.0)// Si el angulo es positivo, objetivo a la izda, giro a la derecha
	//				{
	//					v_auxCtrlAct.push_back(THRUSTX);
	//					v_auxCtrlAct.push_back(0.);
	//					v_auxCtrlAct.push_back(-THRUSTW);
	//				}
	//				else// Si el angulo es negativo, objetivo a la dcha, giro a la izquierda
	//				{
	//					v_auxCtrlAct.push_back(THRUSTX);
	//					v_auxCtrlAct.push_back(0.);
	//					v_auxCtrlAct.push_back(THRUSTW);
	//				}
	//			}
	//			else
	//			{
	//				b_yaw_ensured = true;
	//				_p_circ = new Circunference(p_ShipInitState, p_ShipFinalState);

	//				v_auxCtrlAct.push_back(THRUSTX);
	//				v_auxCtrlAct.push_back(0.);
	//				v_auxCtrlAct.push_back(0.);
	//			}
	//		}
	//		else
	//		{
	//			if (b_yaw_ensured)
	//			{
	//				// MINIAJUSTES
	//				/*if ((distance < DIST_ADJ2) && (distance > DIST_ADJ1))
	//				{
	//					if(yaw_rel > YAW_TOL)
	//					{
	//						v_auxCtrlAct.push_back(THRUSTX);
	//						v_auxCtrlAct.push_back(0.);
	//						v_auxCtrlAct.push_back(-THRUSTW);
	//					}
	//					else
	//						if (yaw_rel < -YAW_TOL)
	//						{
	//							v_auxCtrlAct.push_back(THRUSTX);
	//							v_auxCtrlAct.push_back(0.);
	//							v_auxCtrlAct.push_back(THRUSTW);
	//						}
	//						else
	//						{
	//							v_auxCtrlAct.push_back(THRUSTX);
	//							v_auxCtrlAct.push_back(0.);
	//							v_auxCtrlAct.push_back(0.);
	//						}
	//					
	//				}
	//				else
	//				{
	//					v_auxCtrlAct.push_back(THRUSTX);
	//					v_auxCtrlAct.push_back(0.);
	//					v_auxCtrlAct.push_back(0.);
	//				}*/
	//				/*v_auxCtrlAct.push_back(THRUSTX);
	//				v_auxCtrlAct.push_back(0.);
	//				v_auxCtrlAct.push_back(0.);*/

	//				//! Seguimiento de circunferencia
	//				

	//			}
	//			else
	//			{
	//				v_auxCtrlAct.push_back(THRUSTX);
	//				v_auxCtrlAct.push_back(0.);
	//				v_auxCtrlAct.push_back(0.);
	//			}
	//		}
	//			
	//			break;
	//	}
	//case ZoneType::left:
	//	{
	//	if (b_ensure_yaw)
	//	{
	//		double yaw_rel = p_ShipFinalState->getYaw() - p_ShipInitState->getYaw();
	//		if (std::abs(yaw_rel) < std::abs(REENTRY_ANGLE_K * ar_init_yaw))
	//		{
	//			if (yaw_rel > 0.0)// Si el angulo es positivo, objetivo a la izda, giro a la derecha
	//			{
	//				v_auxCtrlAct.push_back(THRUSTX);
	//				v_auxCtrlAct.push_back(0.);
	//				v_auxCtrlAct.push_back(-THRUSTW);
	//			}
	//			else// Si el angulo es negativo, objetivo a la dcha, giro a la izquierda
	//			{
	//				v_auxCtrlAct.push_back(THRUSTX);
	//				v_auxCtrlAct.push_back(0.);
	//				v_auxCtrlAct.push_back(THRUSTW);
	//			}
	//		}
	//		else
	//		{
	//			b_yaw_ensured = true;
	//			_p_circ = new Circunference(p_ShipInitState, p_ShipFinalState);

	//			v_auxCtrlAct.push_back(THRUSTX);
	//			v_auxCtrlAct.push_back(0.);
	//			v_auxCtrlAct.push_back(THRUSTW);
	//		}

	//	}
	//	else
	//	{
	//		v_auxCtrlAct.push_back(THRUSTX);
	//		v_auxCtrlAct.push_back(0.);
	//		v_auxCtrlAct.push_back(THRUSTW);
	//	}

	//	break;
	//	}

	//}

	//return v_auxCtrlAct;
}

bool EGKRRT::EGKtree::EGKpath::isGhostThere(ShipState* donkey, ShipState* carrot, Vector2D& newPos)
{
	bool b_ret = false;

	Vector3D accs = donkey->getAccs();
	Vector3D vels = donkey->getVels();
	Vector3D pos = donkey->getPose();

	double vx = vels.x;
	double vy = vels.y;

	vx = std::abs(vx);
	vy = std::abs(vy);

	double t_stop = 0.0;

	if ((vx != 0.0) && (vy != 0.0) && (accs.x!=0.0) && (accs.y!=0.0))
		t_stop = (-1.0 * vx * (accs.x / accs.y) - vy) / (accs.y * (1 + (accs.x * accs.x) / (accs.y * accs.y)));

	else
		if (vx != 0.0 && accs.x != 0.0)
			t_stop = vx / std::abs(accs.x);
		else
			if((vy != 0.0) && (accs.y != 0.0))
				t_stop = vy / accs.y;

	// Transformar las velocidades relativas a absolutas
	double vabs_x = donkey->getVels().x * cos(donkey->getYaw());
	double vabs_y = donkey->getVels().x * sin(donkey->getYaw());

	double roz = accs.x;//el rozamiento es el mismo para la x que para la y, de momento

	double aabs_x = roz * cos(donkey->getYaw());
	double aabs_y = roz * sin(donkey->getYaw());

	// MRUA
	double new_pos_x = pos.x + vabs_x * t_stop + 0.5 * aabs_x * t_stop * t_stop;
	double new_pos_y = pos.y + vabs_y * t_stop + 0.5 * aabs_y * t_stop * t_stop;

	newPos = Vector2D(new_pos_x, new_pos_y);

	//std::vector<double> empty_ctrlAct {0, 0, 0};
	//ShipState* p_auxState = nullptr;
	//bool b_success = donkey->propagate(empty_ctrlAct, t_stop, &p_auxState);
	b_ret = carrot->isSamePos(Vector3D(new_pos_x, new_pos_y, 0.0));
	if (b_ret)
		double hola = 1;
	return b_ret;
}

double EGKRRT::EGKtree::EGKpath::getLength()
{
	double ret = 0.0;

	if(!(this->_inter.empty()))
		for (size_t i = 0; i < (this->_inter.size() - 1); ++i)
			ret += this->_inter[i]->distanceTo(this->_inter[i + 1]);

	return ret;
}

bool EGKRRT::EGKtree::EGKpath::generateCtrlActCirc(ShipState* ap_initState, Quadrant& ar_quad, ZoneType& ar_zone, std::vector<double>& ar_ctrl_act)
{
	//! Determinamos la posicion del robot respecto a la curva
	CurveZone the_zone = _p_circ->StateZone(ap_initState).first;
	bool b_outside = _p_circ->StateZone(ap_initState).second;

	switch(the_zone)
	{
		case CurveZone::Inner:
		{
			std::tuple<double, bool, bool> tuple_rel_ang = _p_circ->getRelativeAng(ap_initState);

			double relative_ang = std::get<0>(tuple_rel_ang);

			bool b_pointing_outside = std::get<1>(tuple_rel_ang);

			bool b_is_center_left = std::get<2>(tuple_rel_ang);

		
			if (std::abs(relative_ang) < YAW_TOL)
			{
				//! avance recto
				ar_ctrl_act.push_back(THRUSTX);
				ar_ctrl_act.push_back(0.);
				ar_ctrl_act.push_back(0.);
				return true;
			}
			else
			{
				//! Mirando hacia afuera
				if (b_pointing_outside)
				{
					//! Centro a la izquierda
					if (b_is_center_left)
					{
						//! Ligeramente fuera de la circunferencia
						if (b_outside)
						{
							//! giro izquierda
							ar_ctrl_act.push_back(THRUSTX);
							ar_ctrl_act.push_back(0.);
							ar_ctrl_act.push_back(THRUSTW);
						}
						//! Ligeramente dentro de la circunferencia
						else
						{
							//! Avance recto
							ar_ctrl_act.push_back(THRUSTX);
							ar_ctrl_act.push_back(0.);
							ar_ctrl_act.push_back(0.);
						}
						
					}
					//!Centro a la derecha
					else
					{
							//! Ligeramente fuera de la circunferencia
							if (b_outside)
							{
								//! giro derecha
								ar_ctrl_act.push_back(THRUSTX);
								ar_ctrl_act.push_back(0.);
								ar_ctrl_act.push_back(-THRUSTW);
							}
							//! Ligeramente dentro de la circunferencia
							else
							{
								//! Avance recto
								ar_ctrl_act.push_back(THRUSTX);
								ar_ctrl_act.push_back(0.);
								ar_ctrl_act.push_back(0.);
							}
					}
					
					return true;
				}
				//! Mirando hacia dentro
				else
				{
					//! Centro a la izquierda
					if (b_is_center_left)
					{
						//! Ligeramente fuera de la circunferencia
						if (b_outside)
						{
							//! Avance recto
							ar_ctrl_act.push_back(THRUSTX);
							ar_ctrl_act.push_back(0.);
							ar_ctrl_act.push_back(0.);
						}
						//! Ligeramente dentro de la circunferencia
						else
						{
							//! giro derecha
							ar_ctrl_act.push_back(THRUSTX);
							ar_ctrl_act.push_back(0.);
							ar_ctrl_act.push_back(0.);
						}
					}
					//!Centro a la derecha
					else
					{
						//! Ligeramente fuera de la circunferencia
						if (b_outside)
						{
							//! Avance recto
							ar_ctrl_act.push_back(THRUSTX);
							ar_ctrl_act.push_back(0.);
							ar_ctrl_act.push_back(0.);
						}
						//! Ligeramente dentro de la circunferencia
						else
						{
							//! giro izquierda
							ar_ctrl_act.push_back(THRUSTX);
							ar_ctrl_act.push_back(0.);
							ar_ctrl_act.push_back(THRUSTW);
						}
					}
					return true;
				}
			}
			break;
		}
		case CurveZone::Medium:
		{
			//! Fuera de la circunferencia
			if (b_outside)
			{
				switch (ar_zone)
				{
				case ZoneType::central: // Punto final en frente
					{
						//! Avance recto
						ar_ctrl_act.push_back(THRUSTX);
						ar_ctrl_act.push_back(0.);
						ar_ctrl_act.push_back(0.);
						return true;
						break;
					}
					case ZoneType::right: // Punto final a la derecha
					{
						//! Giro a la izquierda
						ar_ctrl_act.push_back(THRUSTX);
						ar_ctrl_act.push_back(0.);
						ar_ctrl_act.push_back(-THRUSTW);
						return true;
						break;
					}
					//! Punto final detras a la izquierda
					case ZoneType::left:
					{
						//! Giro a la izquierda
						ar_ctrl_act.push_back(THRUSTX);
						ar_ctrl_act.push_back(0.);
						ar_ctrl_act.push_back(THRUSTW);
						break;
					}
				}
			}
			//! Dentro de la circunferencia
			else
			{
				switch (ar_zone)
				{
					case ZoneType::central: // Punto final en frente
					{
						//! Avance recto
						ar_ctrl_act.push_back(THRUSTX);
						ar_ctrl_act.push_back(0.);
						ar_ctrl_act.push_back(0.);
						return true;
						break;
					}
					case ZoneType::right: // Punto final a la derecha
					{
						//! Giro a la izquierda
						ar_ctrl_act.push_back(THRUSTX);
						ar_ctrl_act.push_back(0.);
						ar_ctrl_act.push_back(-THRUSTW);
						return true;
						break;
					}
					//! Punto final detras a la izquierda
					case ZoneType::left:
					{
						//! Giro a la izquierda
						ar_ctrl_act.push_back(THRUSTX);
						ar_ctrl_act.push_back(0.);
						ar_ctrl_act.push_back(THRUSTW);
						break;
					}
				}
			}

			break;
		}
		case CurveZone::Outer:
		{
			//! Fuera de la circunferencia
			if (b_outside)
			{
				switch (ar_quad)
				{
					case Quadrant::fourth: // Punto final a la derecha
					{
						//! Giro a la derecha
						ar_ctrl_act.push_back(THRUSTX);
						ar_ctrl_act.push_back(0.);
						ar_ctrl_act.push_back(-THRUSTW);
						return true;
						break;
					}
					case Quadrant::first: // Punto final a la izquierda
					{
						//! Giro a la izquierda
						ar_ctrl_act.push_back(THRUSTX);
						ar_ctrl_act.push_back(0.);
						ar_ctrl_act.push_back(THRUSTW);
						return true;
						break;
					}
					//! Punto final detras a la derecha
					case Quadrant::third:
					{
						//! Giro a la derecha
						ar_ctrl_act.push_back(THRUSTX);
						ar_ctrl_act.push_back(0.);
						ar_ctrl_act.push_back(-THRUSTW);
						break;
					}
					//! Punto final detras a la izquierda
					case Quadrant::second:
					{
						//! Giro a la izquierda
						ar_ctrl_act.push_back(THRUSTX);
						ar_ctrl_act.push_back(0.);
						ar_ctrl_act.push_back(THRUSTW);
						break;
					}
				}
			}
			//! Dentro de la circunferencia
			else
			{
				switch (ar_quad)
				{
					//! Punto final a la derecha
					case Quadrant::fourth:
					{
						//! Giro a la izquierda
						ar_ctrl_act.push_back(THRUSTX);
						ar_ctrl_act.push_back(0.);
						ar_ctrl_act.push_back(THRUSTW);
						break;
					}
					//! Punto final a la izquierda
					case Quadrant::first:
					{
						//! Giro a la derecha
						ar_ctrl_act.push_back(THRUSTX);
						ar_ctrl_act.push_back(0.);
						ar_ctrl_act.push_back(-THRUSTW);
						break;
					}
					//! Punto final detras a la derecha
					case Quadrant::third:
					{
						//! Giro a la izquierda
						ar_ctrl_act.push_back(THRUSTX);
						ar_ctrl_act.push_back(0.);
						ar_ctrl_act.push_back(THRUSTW);
						break;
					}
					//! Punto final detras a la izquierda
					case Quadrant::second:
					{
						//! Giro a la derecha
						ar_ctrl_act.push_back(THRUSTX);
						ar_ctrl_act.push_back(0.);
						ar_ctrl_act.push_back(-THRUSTW);
						break;
					}
				}
			}
			
			break;
		}
	}
}

//std::vector<double> EGKRRT::EGKtree::EGKpath::navigationOrient(RobotState* ap_initState, circunference* ap_circ)
//{
//	std::vector<double> v_auxCtrlAct;
//
//	ShipState* p_ShipInitState = dynamic_cast<ShipState*>(ap_initState);
//	
//	if (!p_ShipInitState)
//		return v_auxCtrlAct;
//	
//	Vector3D init_pose = p_ShipInitState->getPose();
//	
//	if(ap_circ->PointBelongs(init_pose))
//	{
//		double angle_rel = ap_circ->getAng(/*No me acuerdo que argumentos tenia*/);
//		if(std::abs(angle_rel)<YAW_TOL)
//		{
//			// Avance recto	
//		}
//		else
//		{
//			if(angle_rel > 0.0)
//			{
//				// Circunferencia a la izquierda, giro a la izquierda
//			}
//			else
//			{
//			// Circunferencia a la derecha, giro a la derecha
//			}
//		}
//		
//	}
//	else
//	{	
//		// Giro hacia la circunferencia
//		ap_circ->IsPointInside(init_pose)
//		{
//			// Punto dentro de la circunferencia
//		}
//		else
//		{
//			// Punto fuera de la circunferencia
//		}
//	}
//}

//! -------------------------------------- Circunference -------------------------------------- 

EGKRRT::EGKtree::EGKpath::Circunference::Circunference(RobotState* ap_init, RobotState* ap_goal)
{
	ShipState* p_EGK_init = dynamic_cast<ShipState*>(ap_init);
	ShipState* p_EGK_goal = dynamic_cast<ShipState*>(ap_goal);

	if (!(p_EGK_init) || (!p_EGK_goal))
		_b_is_Ok = false;

	//! Construcción de la circunferencia
	if (_b_is_Ok)
	{
		double yaw_goal = p_EGK_goal->getYaw();
		Vector3D pos_goal = p_EGK_goal->getPose();

		Vector3D pos_init = p_EGK_init->getPose();

		Vector2D unit_yaw_goal(cos(yaw_goal), sin(yaw_goal));

		Vector2D unit_yaw_perp = unit_yaw_goal.perpendicularVector();

		//! Pendiente de la recta que pasa por goal y el centro de la circunferencia
		double M = tan(unit_yaw_perp.y / unit_yaw_perp.x); 

		//! Anclaje de la recta que pasa por goal y el centro de la circunferencia
		double n = pos_goal.y - M * pos_goal.x;

		//! Coordenada X del centro de la circunferencia
		_center.x = ((pos_goal.x)*(pos_goal.x) - (pos_init.x)*(pos_init.x) + (pos_goal.y)*(pos_goal.y) - (pos_init.y)*(pos_init.y) - 2*(pos_goal.y - pos_init.y) * n) /
			(2 * (pos_goal.x - pos_init.x) + 2 * (pos_goal.y - pos_init.y) * M);

		//! Coordenada Y del centro de la circunferencia
		_center.y = M * _center.x + n;

		//! Radio de la circunferencia
		_radius = sqrt((pos_goal.x - _center.x)*(pos_goal.x - _center.x) + (pos_goal.y - _center.y)* (pos_goal.y - _center.y));
	}
}

//! Zona: dentro, cerca, lejos. Si es positivo es fuera de la circunferencia y si es negativo es dentro.
std::pair <CurveZone, bool> EGKRRT::EGKtree::EGKpath::Circunference::StateZone(RobotState* ap_init) const
{
	CurveZone the_zone = CurveZone::Outer;

	ShipState* p_EGK_init = dynamic_cast<ShipState*>(ap_init);

	if (!p_EGK_init)
		throw ERRORNULL;

	//! Calculamos la distancia a la circunferencia
	double distance = getDistance(p_EGK_init);

	//! Pertenece a la circunferencia
	if (distance < CIRC_TOL_INNER_OUTSIDE)
		the_zone = CurveZone::Inner;

	else
	{
		//! Pertenece a la zona intermedia
		if (distance < CIRC_TOL_OUTER)
			the_zone = CurveZone::Medium;
	}

	return std::make_pair(the_zone, distance > 0.0);
}

//! El retorno es el angulo relativo, si mira hacia afura, y si el centro esta a la izquierda
std::tuple<double, bool, bool> EGKRRT::EGKtree::EGKpath::Circunference::getRelativeAng(RobotState* ap_init) const
{
	ShipState* p_EGK_init = dynamic_cast<ShipState*>(ap_init);

	if (!p_EGK_init)
		throw ERRORNULL;

	Vector3D init_pos = p_EGK_init->getPose();

	double init_yaw = p_EGK_init->getYaw();

	if (StateZone(p_EGK_init).first == CurveZone::Inner)
	{
		//! Vector tangente a la circunferencia que pasa por init

		Vector2D init_2_center( _center.x - init_pos.x, _center.y - init_pos.y);

		//init_2_center = init_2_center.normalize();

		Vector2D init_tan = init_2_center.perpendicularVector();

		Vector2D init_dir(cos(init_yaw), sin(init_yaw));

		//init_dir = init_dir.normalize();

		//! Producto escalar
		double ang = acos((init_tan.x * init_dir.x + init_tan.y * init_dir.y) / (init_tan.module() * init_dir.module()));

		if (ang > (PI / 2))
		{
			ang = PI - ang;
			init_tan.x = -init_tan.x;
			init_tan.y = -init_tan.y;

		}
			
		
		//! Producto vectorial: vector direccion por vector tangente V x U: Vx.Uy - Ux.Vy
		bool sgn1 = (init_dir.x * init_tan.y - init_tan.x * init_dir.y) > 0.0; 

		//!Producto vectorial: vector direccion por vector  init_2_center V x U: Vx.Uy - Ux.Vy
		bool sgn2 = (init_dir.x * init_2_center.y - init_2_center.x * init_dir.y) > 0.0;

		//! Si sgn2 es positivo el centro esta a la izquierda, -> Si sgn1 es positivo la direccion apunta hacia fuera de la circunferencia
		//!													   -> Si sgn1 es negativo la direccion apunta hacia dentro de la circunferencia
		//! Si sgn2 es negativo el centro esta a la derecha, -> Si sgn1 es positivo la direccion apunta hacia dentro de la circunferencia
		//!													 -> Si sgn1 es negativo la direccion apunta hacia fuera de la circunferencia
		
		return std::make_tuple(ang, sgn1&&sgn2, sgn2);
	}
}

double EGKRRT::EGKtree::EGKpath::Circunference::getDistance(RobotState* ap_init) const
{
	ShipState* p_EGK_init = dynamic_cast<ShipState*>(ap_init);

	if (!p_EGK_init)
		throw ERRORNULL;

	Vector2D init_pose(p_EGK_init->getPose().x, p_EGK_init->getPose().y);

	return (init_pose - _center).module() - _radius;
}

//! -------------------------------------- Spline -------------------------------------- 

EGKRRT::EGKtree::EGKpath::Spline::Spline(RobotState* ap_init, RobotState* ap_goal)
{
	ShipState* p_EGK_init = dynamic_cast<ShipState*>(ap_init);
	ShipState* p_EGK_goal = dynamic_cast<ShipState*>(ap_goal);

	if (!(p_EGK_init) || (!p_EGK_goal))
		throw ERRORNULL;

	//! Construcción de la Spline
	
	//! Extraemos las posiciones inicial y final
	Vector2D pos_init(p_EGK_init->getPose().x, p_EGK_init->getPose().y);
	Vector2D pos_goal(p_EGK_goal->getPose().x, p_EGK_goal->getPose().y);

	//! Extraemos las direcciones inicial y final
	double yaw_init = p_EGK_init->getYaw();
	double yaw_goal = p_EGK_goal->getYaw();
	Vector2D unit_yaw_init(cos(yaw_init), sin(yaw_init));
	Vector2D unit_yaw_goal(-cos(yaw_goal), -sin(yaw_goal));

	//! Extraemos las velocidades inicial y final
	Vector3D vel_init = p_EGK_init->getVels();
	Vector3D vel_goal = p_EGK_goal->getVels();

	//! Calculo del corte de las rectas que forman las direcciones inicial y final
	//! Calculamos las rectas que forman los puntos inicial y final y sus direcciones: y = Mx + n
	double M_init = tan(unit_yaw_init.y / unit_yaw_init.x);
	double n_init = pos_init.y - M_init * pos_init.x;

	double M_goal = tan(unit_yaw_goal.y / unit_yaw_goal.x);
	double n_goal = pos_goal.y - M_goal * pos_goal.x;
	//! y = m1x +n1
	//! y = m2x + n2
	//! m2x - m1x = n1-n2
	//! x = n1-n2 / m2-m1
	Vector2D intersec_point((n_init - n_goal)/(M_goal - M_init), M_init*pos_init.x + n_init);

	//! Distancias del punto de interseccion al punto inicial
	double dist_intersec = intersec_point.distance(Vector2D(pos_init.x, pos_init.y));

	Vector2D ghost_pos;

	EGKRRT::EGKtree::EGKpath::isGhostThere(p_EGK_init, p_EGK_goal, ghost_pos);

	double dist_ghost = (ghost_pos - pos_init).module();

	if (dist_ghost > (dist_intersec*0.8))
		_b_is_Ok = false;

	_b_is_Ok = true;

	if (_b_is_Ok)
	{
		//! El codigo comentado corresponde con la construccion de la obsoleta spline en funcion de 2 puntos de control
		//! Calculo de los puntos de control
		/*_p0.x = pos_init.x;
		_p0.y = pos_init.y;
		_p1.x = pos_init.x + dist_p1 * unit_yaw_init.x;
		_p1.y = pos_init.y + dist_p1 * unit_yaw_init.y;
		_p2.x = pos_goal.x - dist_p2 * unit_yaw_goal.x;
		_p2.y = pos_goal.y - dist_p2 * unit_yaw_goal.y;
		_p3.x = pos_goal.x;
		_p3.y = pos_goal.y;*/
		
		_p0.x = X_START;
		_p0.y = Y_START;
		/*_p1.x = 20.0;
		_p1.y = 30.0;
		_p2.x = 30.0;
		_p2.y = 20.0;*/
		_p3.x = X_GOAL;
		_p3.y = Y_GOAL;

		//! Calculamos los coeficientes del polinomio del tercer grado
		/*_cx = 3.0 * (_p1.x - _p0.x);
		_bx = 3.0 * (_p2.x - _p1.x) - _cx;
		_ax = _p3.x - _p0.x - _cx - _bx;
		_cy = 3.0 * (_p1.y - _p0.y);
		_by = 3.0 * (_p2.y - _p1.y) - _cy;
		_ay = _p3.y - _p0.y - _cy - _by;
		_ax = 2 * (_p0.x - _p3.x) + unit_yaw_init.x + unit_yaw_goal.x;
		_bx = 3 * (_p3.x - _p0.x) - (2 * unit_yaw_init.x + unit_yaw_goal.x);
		_cx = unit_yaw_init.x;
		_ay = 2 * (_p0.y - _p3.y) + unit_yaw_init.y + unit_yaw_goal.y;
		_by = 3 * (_p3.y - _p0.y) - (2 * unit_yaw_init.y + unit_yaw_goal.y);
		_cy = unit_yaw_init.y;*/

		double dist_init_goal = (pos_goal - pos_init).module();

		double mx1 = K_DEBUG_INIT * dist_init_goal * cos(yaw_init), my1 = K_DEBUG_INIT * dist_init_goal * sin(yaw_init),
			   mx2 = K_DEBUG_GOAL * dist_init_goal * cos(yaw_goal), my2 = K_DEBUG_GOAL * dist_init_goal * sin(yaw_goal);
		                

		_ax = 2 * (_p0.x - _p3.x) + mx1 + mx2;
		_bx = 3 * (_p3.x - _p0.x) - (2 * mx1 + mx2);
		_cx = mx1;

		_ay = 2 * (_p0.y - _p3.y) + my1 + my2;
		_by = 3 * (_p3.y - _p0.y) - (2 * my1 + my2);
		_cy = my1;
	}
}

bool EGKRRT::EGKtree::EGKpath::generateCtrlActSpline(ShipState* ap_initState, Quadrant& ar_quad, ZoneType& ar_zone, std::vector<double>& ar_ctrl_act)
{
	_p_spline->CalculateDistance(ap_initState);
	//! Determinamos la posicion del robot respecto a la curva
	std::pair<CurveZone, bool> state_zone = _p_spline->StateZone(ap_initState);
	CurveZone the_zone = state_zone.first;
	bool b_outside = !(state_zone.second);

	switch (the_zone)
	{
	case CurveZone::Inner:
	{
		std::tuple<double, bool, bool> tuple_rel_ang = _p_spline->getRelativeAng(ap_initState);

		double relative_ang = std::get<0>(tuple_rel_ang);

		bool b_pointing_outside = std::get<1>(tuple_rel_ang);

		bool b_is_center_left = std::get<2>(tuple_rel_ang);


		if (std::abs(relative_ang) < YAW_TOL)
		{
			//! avance recto
			double t_near = _p_spline->getTnear();
			if (t_near > T_LOW && t_near < T_TOP)
			{
				double vel_surge = ap_initState->getVels().x;

				if ((vel_surge > VX_SLOW) && (_p_spline->getSTOP()))
				{
					ar_ctrl_act.push_back(THRUSTX_spline);
					ar_ctrl_act.push_back(0.);
					ar_ctrl_act.push_back(0.);
				}
				else
				{
					_p_spline->setSTOP(false);

					ar_ctrl_act.push_back(THRUSTX);
					ar_ctrl_act.push_back(0.);
					ar_ctrl_act.push_back(0.);
				}
			}
			else
			{
				ar_ctrl_act.push_back(THRUSTX);
				ar_ctrl_act.push_back(0.);
				ar_ctrl_act.push_back(0.);
			}

			return true;
		}
		else
		{
			//! Mirando hacia afuera
			if (b_pointing_outside)
			{
				//! Centro a la izquierda
				if (b_is_center_left)
				{
					//! Ligeramente fuera de la circunferencia
					if (b_outside)
					{
						//! giro izquierda
						double t_near = _p_spline->getTnear();
						if (t_near > T_LOW && t_near < T_TOP)
						{
							ar_ctrl_act.push_back(THRUSTX_spline);
							ar_ctrl_act.push_back(0.);
							ar_ctrl_act.push_back(THRUSTW_spline);
						}
						else
						{
							ar_ctrl_act.push_back(THRUSTX);
							ar_ctrl_act.push_back(0.);
							ar_ctrl_act.push_back(THRUSTW);
						}
					}
					//! Ligeramente dentro de la circunferencia
					else
					{
						//! giro izquierda
						double t_near = _p_spline->getTnear();
						if (t_near > T_LOW && t_near < T_TOP)
						{
							ar_ctrl_act.push_back(THRUSTX_spline);
							ar_ctrl_act.push_back(0.);
							ar_ctrl_act.push_back(THRUSTW_spline);
						}
						else
						{
							ar_ctrl_act.push_back(THRUSTX);
							ar_ctrl_act.push_back(0.);
							ar_ctrl_act.push_back(THRUSTW);
						}
					}

				}
				//!Centro a la derecha
				else
				{
					//! Ligeramente fuera de la circunferencia
					if (b_outside)
					{
						//! giro derecha
						double t_near = _p_spline->getTnear();
						if (t_near > T_LOW && t_near < T_TOP)
						{
							ar_ctrl_act.push_back(THRUSTX_spline);
							ar_ctrl_act.push_back(0.);
							ar_ctrl_act.push_back(-THRUSTW_spline);
						}
						else
						{
							ar_ctrl_act.push_back(THRUSTX);
							ar_ctrl_act.push_back(0.);
							ar_ctrl_act.push_back(-THRUSTW);
						}
					}
					//! Ligeramente dentro de la circunferencia
					else
					{
						//! giro derecha
						double t_near = _p_spline->getTnear();
						if (t_near > T_LOW && t_near < T_TOP)
						{
							ar_ctrl_act.push_back(THRUSTX_spline);
							ar_ctrl_act.push_back(0.);
							ar_ctrl_act.push_back(-THRUSTW_spline);
						}
						else
						{
							ar_ctrl_act.push_back(THRUSTX);
							ar_ctrl_act.push_back(0.);
							ar_ctrl_act.push_back(-THRUSTW);
						}
					}
				}

				return true;
			}
			//! Mirando hacia dentro
			else
			{
				//! Centro a la izquierda
				if (b_is_center_left)
				{
					//! Ligeramente fuera de la circunferencia
					if (b_outside)
					{
						//! giro derecha
						double t_near = _p_spline->getTnear();
						if (t_near > T_LOW && t_near < T_TOP)
						{
							ar_ctrl_act.push_back(THRUSTX_spline);
							ar_ctrl_act.push_back(0.);
							ar_ctrl_act.push_back(-THRUSTW_spline);
						}
						else
						{
							ar_ctrl_act.push_back(THRUSTX);
							ar_ctrl_act.push_back(0.);
							ar_ctrl_act.push_back(-THRUSTW);
						}
					}
					//! Ligeramente dentro de la circunferencia
					else
					{
						//! giro derecha
						double t_near = _p_spline->getTnear();
						if (t_near > T_LOW && t_near < T_TOP)
						{
							ar_ctrl_act.push_back(THRUSTX_spline);
							ar_ctrl_act.push_back(0.);
							ar_ctrl_act.push_back(-THRUSTW_spline);
						}
						else
						{
							ar_ctrl_act.push_back(THRUSTX);
							ar_ctrl_act.push_back(0.);
							ar_ctrl_act.push_back(-THRUSTW);
						}
					}
				}
				//!Centro a la derecha
				else
				{
					//! Ligeramente fuera de la circunferencia
					if (b_outside)
					{
						//! giro izquierda
						double t_near = _p_spline->getTnear();
						if (t_near > T_LOW && t_near < T_TOP)
						{
							ar_ctrl_act.push_back(THRUSTX_spline);
							ar_ctrl_act.push_back(0.);
							ar_ctrl_act.push_back(THRUSTW_spline);
						}
						else
						{
							ar_ctrl_act.push_back(THRUSTX);
							ar_ctrl_act.push_back(0.);
							ar_ctrl_act.push_back(THRUSTW);
						}
					}
					//! Ligeramente dentro de la circunferencia
					else
					{
						//! giro izquierda
						double t_near = _p_spline->getTnear();
						if (t_near > T_LOW && t_near < T_TOP)
						{
							ar_ctrl_act.push_back(THRUSTX_spline);
							ar_ctrl_act.push_back(0.);
							ar_ctrl_act.push_back(THRUSTW_spline);
						}
						else
						{
							ar_ctrl_act.push_back(THRUSTX);
							ar_ctrl_act.push_back(0.);
							ar_ctrl_act.push_back(THRUSTW);
						}
					}
				}
				return true;
			}
		}
		break;
	}
	case CurveZone::Medium:
	{
		bool t_near_left = _p_spline->IsTNearLeft(ap_initState);
		//! La curva esta a la izquierda
		if (t_near_left)
		{
			//! Giro a la izquierda
			double t_near = _p_spline->getTnear();
			if (t_near > T_LOW && t_near < T_TOP)
			{
				ar_ctrl_act.push_back(THRUSTX);
				ar_ctrl_act.push_back(0.);
				ar_ctrl_act.push_back(THRUSTW_spline);
			}
			else
			{
				ar_ctrl_act.push_back(THRUSTX);
				ar_ctrl_act.push_back(0.);
				ar_ctrl_act.push_back(THRUSTW);
			}
			return true;
			break;
		}
		//! la curva esta a la derecha
		else
		{
			//! Giro a la derecha
			double t_near = _p_spline->getTnear();
			if (t_near > T_LOW && t_near < T_TOP)
			{
				ar_ctrl_act.push_back(THRUSTX);
				ar_ctrl_act.push_back(0.);
				ar_ctrl_act.push_back(-THRUSTW_spline);
			}
			else
			{
				ar_ctrl_act.push_back(THRUSTX);
				ar_ctrl_act.push_back(0.);
				ar_ctrl_act.push_back(-THRUSTW);
			}
			return true;
			break;
		}

		//! Fuera de la circunferencia
		//if (b_outside)
		//{
		//	
		//	switch (ar_zone)
		//	{
		//	case ZoneType::central: // Punto final en frente
		//	{
		//		//! Avance recto
		//		double t_near = _p_spline->getTnear();
		//		if (t_near > T_LOW && t_near < T_TOP)
		//		{
		//			double vel_surge = ap_initState->getVels().x;
		//			if ((vel_surge > VX_SLOW) && (_p_spline->getSTOP()))
		//			{
		//				ar_ctrl_act.push_back(THRUSTX_spline);
		//				ar_ctrl_act.push_back(0.);
		//				ar_ctrl_act.push_back(0.);
		//			}
		//			else
		//			{
		//				_p_spline->setSTOP(false);
		//				ar_ctrl_act.push_back(THRUSTX);
		//				ar_ctrl_act.push_back(0.);
		//				ar_ctrl_act.push_back(0.);
		//			}
		//		}
		//		else
		//		{
		//			ar_ctrl_act.push_back(THRUSTX);
		//			ar_ctrl_act.push_back(0.);
		//			ar_ctrl_act.push_back(0.);
		//		}
		//		return true;
		//		break;
		//	}
		//	case ZoneType::right: // Punto final a la derecha
		//	{
		//		//! Giro a la derecha
		//		double t_near = _p_spline->getTnear();
		//		if (t_near > T_LOW && t_near < T_TOP)
		//		{
		//			ar_ctrl_act.push_back(THRUSTX_spline);
		//			ar_ctrl_act.push_back(0.);
		//			ar_ctrl_act.push_back(-THRUSTW_spline);
		//		}
		//		else
		//		{
		//			ar_ctrl_act.push_back(THRUSTX);
		//			ar_ctrl_act.push_back(0.);
		//			ar_ctrl_act.push_back(-THRUSTW);
		//		}
		//		return true;
		//		break;
		//	}
		//	//! Punto final detras a la izquierda
		//	case ZoneType::left:
		//	{
		//		//! Giro a la izquierda
		//		double t_near = _p_spline->getTnear();
		//		if (t_near > T_LOW && t_near < T_TOP)
		//		{
		//			ar_ctrl_act.push_back(THRUSTX_spline);
		//			ar_ctrl_act.push_back(0.);
		//			ar_ctrl_act.push_back(THRUSTW_spline);
		//		}
		//		else
		//		{
		//			ar_ctrl_act.push_back(THRUSTX);
		//			ar_ctrl_act.push_back(0.);
		//			ar_ctrl_act.push_back(THRUSTW);
		//		}
		//		break;
		//	}
		//	}
		//}
		////! Dentro de la circunferencia
		//else
		//{
		//	switch (ar_zone)
		//	{
		//	case ZoneType::central: // Punto final en frente
		//	{
		//		//! Avance recto
		//		double t_near = _p_spline->getTnear();
		//		if (t_near > T_LOW && t_near < T_TOP)
		//		{
		//			double vel_surge = ap_initState->getVels().x;
		//			if ((vel_surge > VX_SLOW) && (_p_spline->getSTOP()))
		//			{
		//				ar_ctrl_act.push_back(THRUSTX_spline);
		//				ar_ctrl_act.push_back(0.);
		//				ar_ctrl_act.push_back(0.);
		//			}
		//			else
		//			{
		//				_p_spline->setSTOP(false);
		//				ar_ctrl_act.push_back(THRUSTX);
		//				ar_ctrl_act.push_back(0.);
		//				ar_ctrl_act.push_back(0.);
		//			}
		//		}
		//		else
		//		{
		//			ar_ctrl_act.push_back(THRUSTX);
		//			ar_ctrl_act.push_back(0.);
		//			ar_ctrl_act.push_back(0.);
		//		}
		//		return true;
		//		break;
		//	}
		//	case ZoneType::right: // Punto final a la derecha
		//	{
		//		//! Giro a la derecha
		//		double t_near = _p_spline->getTnear();
		//		if (t_near > T_LOW && t_near < T_TOP)
		//		{
		//			ar_ctrl_act.push_back(THRUSTX_spline);
		//			ar_ctrl_act.push_back(0.);
		//			ar_ctrl_act.push_back(-THRUSTW_spline);
		//		}
		//		else
		//		{
		//			ar_ctrl_act.push_back(THRUSTX);
		//			ar_ctrl_act.push_back(0.);
		//			ar_ctrl_act.push_back(-THRUSTW);
		//		}
		//		return true;
		//		break;
		//	}
		//	//! Punto final detras a la izquierda
		//	case ZoneType::left:
		//	{
		//		//! Giro a la izquierda
		//		double t_near = _p_spline->getTnear();
		//		if (t_near > T_LOW && t_near < T_TOP)
		//		{
		//			ar_ctrl_act.push_back(THRUSTX_spline);
		//			ar_ctrl_act.push_back(0.);
		//			ar_ctrl_act.push_back(THRUSTW_spline);
		//		}
		//		else
		//		{
		//			ar_ctrl_act.push_back(THRUSTX);
		//			ar_ctrl_act.push_back(0.);
		//			ar_ctrl_act.push_back(THRUSTW);
		//		}
		//		break;
		//	}
		//	}
		//}

		break;
	}
	}
}

Vector2D EGKRRT::EGKtree::EGKpath::Spline::Spfunction(double a_t)
{
	double tSquared = a_t * a_t;
	double tCubed = tSquared * a_t;

	double x = (_ax * tCubed) + (_bx * tSquared) + (_cx * a_t) + _p0.x;
	double y = (_ay * tCubed) + (_by * tSquared) + (_cy * a_t) + _p0.y;

	return Vector2D(x, y);
}

void EGKRRT::EGKtree::EGKpath::Spline::CalculateDistance(RobotState* ap_init)
{
	ShipState* p_EGK_init = dynamic_cast<ShipState*>(ap_init);

	if (!p_EGK_init)
		throw ERRORNULL;

	Vector2D pos(p_EGK_init->getPose().x, p_EGK_init->getPose().y);

	//! MINIMOS CUADRADOS
	//! Primeras 3 estimaciones para interpolar el polinomio cuadratico de la distancia al cuadrado
	double approx1 = _t_near;
	double approx2 = _t_near + 0.05;
	double approx3 = _t_near + 0.1;

	if (approx3 > 1.0)
	{
		approx1 = _t_near;
		approx2 = _t_near + 0.05;
		approx3 = 1.0;
	}

	if (approx2 > 1.0)
	{
		approx1 = _t_near - 0.05;
		approx2 = _t_near;
		approx3 = 1.0;
	}
		
	std::vector<double> ts{ approx1, approx2, approx3 };
	int k = 0;

	// t_min es el minimo del polinomio que interpola t1, t2, t3
	double t_min = QuadraticMin(pos, ts[0], ts[1], ts[2]);

	//double Pt_min = QuadraticPolynom(pos, t_min, ts[0], ts[1], ts[2]); creo que no es necesario
	double Pt1 = QuadraticPolynom(pos, ts[0], ts[0], ts[1], ts[2]);
	double Pt2 = QuadraticPolynom(pos, ts[1], ts[0], ts[1], ts[2]);
	double Pt3 = QuadraticPolynom(pos, ts[2], ts[0], ts[1], ts[2]);

	std::vector<double> Pts{ Pt1 ,Pt2 ,Pt3 };

	std::vector<double> aux_debug;
	aux_debug.push_back(t_min);

	while (k < 4)
	{
		//! Buscamos el maximo entre P(t1), P(t2), P(t3) para descartarlo en favor de P(t_min)
		int t_max = 0;
		double val_max = 0.0;

		for (int i=0;i<Pts.size();++i)
			if (val_max < Pts[i])
			{
				val_max = Pts[i];
				t_max = i;
			}

		//! Sustituimos el valor mas alto de las 3 aproximaciones t1, t2, t3 por este valor minimo t_min
		ts[t_max] = t_min;

		//Pt_min = QuadraticPolynom(pos, t_min, ts[0], ts[1], ts[2]); creo que no es necesario
		Pt1 = QuadraticPolynom(pos, ts[0], ts[0], ts[1], ts[2]);
		Pt2 = QuadraticPolynom(pos, ts[1], ts[0], ts[1], ts[2]);
		Pt3 = QuadraticPolynom(pos, ts[2], ts[0], ts[1], ts[2]);
		t_min = QuadraticMin(pos, ts[0], ts[1], ts[2]);

		aux_debug.push_back(t_min);
		++k;
	}

	//! NEWYON-RAPHSON
	int nr = 0;
	 
	while (nr < 4)
	{
		t_min = t_min - (Newton1(t_min, pos)) / (Newton2(t_min, pos));
		aux_debug.push_back(t_min);
		++nr;
	}

	if (t_min < 0.0)
		t_min = 0.000001;
	if (t_min > 1.0)
		t_min = 0.999999;

	_distance = getDistanceT(pos, t_min);
	_t_near = t_min;

	std::pair<double, double> distance_test = getDistanceTest(ap_init);

	double dif_dist = std::abs(_distance - distance_test.first);
	double dif_percent = std::abs(_distance - distance_test.first) / distance_test.first;
	if (dif_percent > 1.0)
	{
		_distance = distance_test.first;
		_t_near = distance_test.second;
	}
}

std::pair<double, double> EGKRRT::EGKtree::EGKpath::Spline::getDistanceTest(RobotState* ap_init)
{
	ShipState* p_EGK_init = dynamic_cast<ShipState*>(ap_init);

	if (!p_EGK_init)
		throw ERRORNULL;

	double ret_distance = -1.0;

	Vector2D pos(p_EGK_init->getPose().x, p_EGK_init->getPose().y);

	double guardalai = 0.0;
	ret_distance = getDistanceT(pos, guardalai);
	
	for (double i = 0.01; i < 1.0; i+=0.01)
	{
		double aux_dist = getDistanceT(pos, i);

		if (aux_dist < ret_distance)
		{
			ret_distance = aux_dist;
			guardalai = i;
		}
	}
	_t_near = guardalai;

	return std::pair<double, double>(ret_distance, guardalai);
}

double EGKRRT::EGKtree::EGKpath::Spline::QuadraticMin(Vector2D& pos, double& t1, double& t2, double& t3)
{
	//! En principio creamos una variable para debuguear, pero luego pon return directamente
	double y23 = t2 * t2 - t3 * t3;
	double y31 = t3 * t3 - t1 * t1;
	double y12 = t1 * t1 - t2 * t2;
	double t23 = t2 - t3;
	double t31 = t3 - t1;
	double t12 = t1 - t2;
	
	double num = y23 * getSqDistanceT(pos, t1) + y31 * getSqDistanceT(pos, t2) + y12 * getSqDistanceT(pos, t3);
	double den = t23 * getSqDistanceT(pos, t1) + t31 * getSqDistanceT(pos, t2) + t12 * getSqDistanceT(pos, t3);
	double ret = 0.5 * (num) / (den);
	return ret;
}

double EGKRRT::EGKtree::EGKpath::Spline::getDistanceT(Vector2D& pos, double& t)
{
	//! En principio creamos una variable para debuguear, pero luego pon return directamente
	// double ret = (Spfunction(t) - pos).module();
	Vector2D point = Spfunction(t);
	double ret = std::sqrt((point.x - pos.x)* (point.x - pos.x) + (point.y - pos.y)* (point.y - pos.y));
	return ret;
}

double EGKRRT::EGKtree::EGKpath::Spline::getSqDistanceT(Vector2D& pos, double& t)
{
	//! En principio creamos una variable para debuguear, pero luego pon return directamente
	// double ret = (Spfunction(t) - pos).module();
	Vector2D point = Spfunction(t);
	double ret = (point.x - pos.x) * (point.x - pos.x) + (point.y - pos.y) * (point.y - pos.y);
	return ret;
}

double EGKRRT::EGKtree::EGKpath::Spline::QuadraticPolynom(Vector2D& pos, double& t, double& t1, double& t2, double& t3)
{
	//! En principio creamos una variable para debuguear, pero luego pon return directamente
	double ret = (((t - t2) * (t - t3)) / ((t1 - t2) * (t1 - t3))) * getSqDistanceT(pos, t1)
		+ (((t - t1) * (t - t3)) / ((t2 - t1) * (t2 - t3))) * getSqDistanceT(pos, t2)
		+ (((t - t1) * (t - t2)) / ((t3 - t1) * (t3 - t2))) * getSqDistanceT(pos, t3);
	return ret;
}

double EGKRRT::EGKtree::EGKpath::Spline::Newton1(double& t, Vector2D& pos)
{
	Vector2D aux = Spfunction(t);
	return 2 * ( 3 * _ax * t * t + 2 * _bx * t + _cx) * (aux.x - pos.x) +
		   2 * ( 3 * _ay * t * t + 2 * _by * t + _cy) * (aux.y - pos.y);
}

double EGKRRT::EGKtree::EGKpath::Spline::Newton2(double& t, Vector2D& pos)
{
	Vector2D aux = Spfunction(t);
	return 2 * ((6 * _ax * t + 2 * _bx) * (aux.x - pos.x) + (3 * _ax * t * t + 2 * _bx * t + _cx) * (3 * _ax * t * t + 2 * _bx * t + _cx)) +
		   2 * ((6 * _ay * t + 2 * _by) * (aux.y - pos.y) + (3 * _ay * t * t + 2 * _by * t + _cy) * (3 * _ay * t * t + 2 * _by * t + _cy));
}

std::pair <CurveZone, bool> EGKRRT::EGKtree::EGKpath::Spline::StateZone(RobotState* ap_init)
{
	CurveZone the_zone = CurveZone::Outer;

	ShipState* p_EGK_init = dynamic_cast<ShipState*>(ap_init);

	if (!p_EGK_init)
		throw ERRORNULL;

	//! Preguntamos si el punto esta en la parte concava de la spline
	bool b_inside = IsInside(ap_init);

	//! Usamos la distancia de pertenencia en funcion de si estamos dentro o fuera de la spline
	double dist_belong_spline = CIRC_TOL_INNER_INSIDE;

	if (!b_inside)
		dist_belong_spline = CIRC_TOL_INNER_OUTSIDE;

	//! Calculamos la distancia a la curva
	double distance = getDistance();

	//! Pertenece a la curva
	if (distance < dist_belong_spline)
		the_zone = CurveZone::Inner;

	//! no pertenece a la curva
	else
		the_zone = CurveZone::Medium;

	return std::make_pair(the_zone, b_inside);
}

bool EGKRRT::EGKtree::EGKpath::Spline::IsInside(RobotState* ap_init)
{
	ShipState* p_EGK_init = dynamic_cast<ShipState*>(ap_init);

	if (!p_EGK_init)
		throw ERRORNULL;

	Vector2D pos(p_EGK_init->getPose().x, p_EGK_init->getPose().y);

	double t_min = _t_near;

	double t_aux = t_min + 0.1;
	if (t_aux > 1.0)
	{
		t_aux = t_min - 0.1;
	}

	Vector2D min_point = Spfunction(t_min);
	Vector2D aux_point = Spfunction(t_aux);

	Vector2D min2aux = aux_point - min_point;
	Vector2D min2pos = pos - min_point;

	double angle = std::acos((min2aux.x * min2pos.x + min2aux.y * min2pos.y) / (min2aux.module() * min2pos.module()));

	return angle < PI / 2.0;

}

bool EGKRRT::EGKtree::EGKpath::Spline::IsTNearLeft(RobotState* ap_init)
{
	ShipState* p_EGK_init = dynamic_cast<ShipState*>(ap_init);

	if (!p_EGK_init)
		throw ERRORNULL;

	Vector2D pos(p_EGK_init->getPose().x, p_EGK_init->getPose().y);

	Vector2D t_near_pos = Spfunction(_t_near);

	Vector2D pos2near = t_near_pos - pos;

	double yaw_init = p_EGK_init->getYaw();

	Vector2D unit_yaw_init(cos(yaw_init), sin(yaw_init));

	bool sgn = (unit_yaw_init.x * pos2near.y - pos2near.x * unit_yaw_init.y) > 0.0;

	return sgn;
	
}

std::tuple<double, bool, bool> EGKRRT::EGKtree::EGKpath::Spline::getRelativeAng(RobotState* ap_init)
{
	ShipState* p_EGK_init = dynamic_cast<ShipState*>(ap_init);

	if (!p_EGK_init)
		throw ERRORNULL;

	Vector3D init_pos = p_EGK_init->getPose();

	double init_yaw = p_EGK_init->getYaw();

	if (StateZone(p_EGK_init).first == CurveZone::Inner)
	{
		//! Vector tangente a la circunferencia que pasa por init

		Vector2D init_tan = SpfirstD(getTnear());
		//init_tan = init_tan.normalize();
		Vector2D init_dir(cos(init_yaw), sin(init_yaw));

		//init_dir = init_dir.normalize();

		//! Producto escalar
		double ang = acos((init_tan.x * init_dir.x + init_tan.y * init_dir.y) / (init_tan.module() * init_dir.module()));

		if (ang > (PI / 2))
		{
			ang = PI - ang;
			init_tan.x = -init_tan.x;
			init_tan.y = -init_tan.y;

		}
		Vector2D perp_init_tan = init_tan.perpendicularVector();

		//! Producto vectorial: vector direccion por vector tangente V x U: Vx.Uy - Ux.Vy
		bool sgn1 = (init_dir.x * init_tan.y - init_tan.x * init_dir.y) > 0.0;

		//!Producto vectorial: vector direccion por vector  init_2_center V x U: Vx.Uy - Ux.Vy
		bool sgn2 = (init_dir.x * perp_init_tan.y - perp_init_tan.x * init_dir.y) > 0.0;

		//!--------------- A veces apuntara a fuera de la Splie y a veces a dentro, hay que ver como hacer que siempre apunte a dentro ---------------

		//! Si sgn2 es positivo el centro esta a la izquierda, -> Si sgn1 es positivo la direccion apunta hacia fuera de la curva
		//!													   -> Si sgn1 es negativo la direccion apunta hacia dentro de la curva
		//! Si sgn2 es negativo el centro esta a la derecha, -> Si sgn1 es positivo la direccion apunta hacia dentro de la curva
		//!													 -> Si sgn1 es negativo la direccion apunta hacia fuera de la curva
		bool pointing_outside = false;
		if (sgn2)
		{
			if (sgn1)
				pointing_outside = true;
		}
			
		else
		{
			if (!sgn1)
				pointing_outside = true;
		}
			

		return std::make_tuple(ang, pointing_outside, sgn2);
	}
}

Vector2D EGKRRT::EGKtree::EGKpath::Spline::SpfirstD(double&& t)
{
	double firstD_x = 3 * _ax * t * t + 2 * _bx * t + _cx;
	double firstD_y = 3 * _ay * t * t + 2 * _by * t + _cy;

	return Vector2D(firstD_x, firstD_y);
}

//! ----------------- Drawing methods --------------------------------------

void EGKRRT::drawGL()
{
	if (_tree)
		_tree->drawGL();
}

void EGKRRT::EGKtree::drawGL()
{
	//pinta las trayectorias: nodos y lineas que las unen
	unsigned int i;
	glLineWidth(1);
	glColor3f(1, 0, 1);
	for (i = 0; i < _paths.size(); i++)_paths[i]->drawGL();
	//for (i = 0; i < _nodes.size(); i++)_nodes[i]->drawGL();
	for (i = 0; i < _vertexes.size(); i++)
	{
		if (_vertexes[i] == nullptr)
			continue;
		_vertexes[i]->drawGL();
	}

	if (_root)_root->drawGL();
}

void EGKRRT::EGKtree::EGKpath::drawGL()
{
	if (_p_circ)
		_p_circ->drawGL();

	if (_p_spline)
		_p_spline->drawGL();

	if (this->_init == nullptr || this->_end == nullptr)
		return;
	vector<double> v;

	v = _init->getSample();

	if (v.size() < 2)return;
	glDisable(GL_LIGHTING);
	glBegin(GL_LINE_STRIP);
	glColor3f(0.1F, 0.1F, 0.8F);
	glVertex3f(v[0], v[1], 0);

	for (int i = 0; i < (int)_inter.size(); i++)
	{
		v = _inter[i]->getSample();
		glVertex3f(v[0], v[1], 0);
	}

	v = _end->getSample();

	glVertex3f(v[0], v[1], 0);

	glEnd();
	glEnable(GL_LIGHTING);
}

void EGKRRT::EGKtree::EGKpath::Circunference::drawGL()
{
	glDisable(GL_LIGHTING);
	glBegin(GL_LINE_STRIP);

	glLineWidth(5);
	glColor3f(0.2, 0.8F, 1.0F);
	for (int i = 0; i < 360; ++i)
	{
		double x = _center.x + _radius * cos(i * PI / 180);
		double y = _center.y + _radius * sin(i * PI / 180);// dale una vuelta a como pintar la circunferencia entera
		glVertex3f(x, y, 0.0);
	}
	glEnd();
	glEnable(GL_LIGHTING);
}

void EGKRRT::EGKtree::EGKpath::Spline::drawGL()
{
	glDisable(GL_LIGHTING);
	glBegin(GL_LINE_STRIP);
	glColor3f(0.8F, 0.1F, 0.1F);
	glLineWidth(5);

	for (double i = 0.0; i < 1.0; i+=0.001)
	{
		Vector2D point = Spfunction(i);
		glVertex3f(point.x, point.y, 0.0);
	}
	glEnd();
	glEnable(GL_LIGHTING);
}

void EGKRobotPath::drawGL()
{
	if (path.size() < 2)
		return;

	glLineWidth(3);
	glColor3f(0.2, 1.0F, 0.2F);//1, 0.2F, 0.2F
	glDisable(GL_LIGHTING);
	glBegin(GL_LINE_STRIP);
	for (RobotState* i: path)
		glVertex3f(dynamic_cast<ShipState*>(i)->getPose().x, dynamic_cast<ShipState*>(i)->getPose().y, 0.0);
	
	glEnd();
	glEnable(GL_LIGHTING);
}
