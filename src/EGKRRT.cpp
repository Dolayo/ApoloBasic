#include "EGKRRT.h"
#include <algorithm>
#include "ShipState.h"
#include "defines.h"


//double EGKRRT::EGKtree::distance(RobotState* p, PathSegment* path, RobotState** mnode)
//{
//
//} en un principio SI HACE FALTA reescribir la funcion distancia, llamando a create path para ver la distancia real

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
	RobotState* addedNode = _tree->addNode(goal);
	if (addedNode)
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

		if (!node)continue;

		RobotState* addedNode = _tree->addNode(node);

		if (addedNode) 
		{
			if (addedNode->isEqual(goal))
			{
				solved = true;

				//retrive each path
				RobotPath pathA = _tree->getPathFromRoot(addedNode);

				//rearrange the states
				delete path;

				path = new RobotPath(pathA);

				//path->filterLoops(); //clean loops if any 
			}
			
		}
		delete node;
	}
	return false;
}

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
	// aux es el segmento del arbol mas cercano al nodo que tratamos de añadir(in/n)
	// initNode almacena el nodo de dicho segmento que esta mas cerca de n
	PathSegment* aux = getClosestPathSegment(n, &initNode);

	// Si no hay ningun segmento es que el arbol esta vacio, solo tiene el origen
	if (aux == 0)
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

		aux = getBest(neighbors, &initNode);
	}

	// El segmento mas cercano aux queda dividido en dos por el nuevo segmento a añadir, por lo que
	// se añade un nuevo segmento newPathA que va antes de initNode

	// newPathA es el segmento que va desde el principio de aux(antiguo) a initNode
	// Aux se cambia para que empiece en initNode pero mantenga su nodo final

	// La division en dos segmentos solo tiene sentido si el nodo mas cercano es parte intermedia de un segmento
	// si es el principio o final de un segmento no habra division en dos segmentos

	if (aux != nullptr)
	{
		if (_paths.size() != 0 && !(aux->_init->isEqual(initNode)) && !(aux->_end->isEqual(initNode)))
		{
			bool b_aux = false;

			PathSegment* newPathA = EGKpath::createPath(aux->_init, initNode, b_aux);

			for (RobotState* i : newPathA->_inter)
				add(i);

			newPathA->_init = aux->_init;
			aux->_init = initNode;

			newPathA->_parent = aux->_parent;
			aux->_parent = newPathA;

			newPathA->_end = aux->_init;

			PathSegment* aux_path = EGKpath::createPath(aux->_init, aux->_end, b_aux);
		

			aux->_inter.assign(aux_path->_inter.begin(), aux_path->_inter.end());

			for (RobotState* i : aux->_inter)
				add(i);

			_paths.push_back(newPathA);	
		}
	}

	bool success;

	// este segmento sera el que una al nuevo nodo con el segmento que contiene al nodo de 
	// menor coste
	// creamos el nuevo segmento desde initNode hasta n
	EGKpath* newPath = EGKpath::createPath(initNode, n, success);

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

				newRePath->_init = Xnew;
				oldPath->_init = vecino;

				newRePath->_parent = findPath4Node(Xnew);

				oldPath->_parent = newRePath;

				newRePath->_end = oldPath->_init;

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
				
				for (RobotState* i: newRePath->_inter)
						add(i);

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

std::vector<double> EGKRRT::EGKtree::EGKpath::navigation(RobotState* p_initState, RobotState* p_finalState)
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

	switch (zone)
	{
	case ZoneType::right:
		{
			v_auxCtrlAct.push_back(THRUSTX);
			v_auxCtrlAct.push_back(0.);
			v_auxCtrlAct.push_back(-THRUSTW);
			break;
		}
	case ZoneType::central:
		{
			v_auxCtrlAct.push_back(THRUSTX);
			v_auxCtrlAct.push_back(0.);
			v_auxCtrlAct.push_back(0.);
			break;
		}
	case ZoneType::left:
		{
			v_auxCtrlAct.push_back(THRUSTX);
			v_auxCtrlAct.push_back(0.);
			v_auxCtrlAct.push_back(THRUSTW);
			break;
		}

	}

	return v_auxCtrlAct;
}

EGKRRT::EGKtree::EGKpath* EGKRRT::EGKtree::EGKpath::createPath(RobotState* p_init, RobotState* p_end, bool& b_success, int niter)
{
	EGKpath* p_newPath = new EGKpath;
	ShipState* p_initState = dynamic_cast<ShipState*>(p_init);
	ShipState* p_finalState = dynamic_cast<ShipState*>(p_end);

	p_newPath->appendState(p_initState);

	p_newPath->_init = p_initState;

	if (!p_initState || !p_finalState)
		return nullptr;
	
	p_initState->placeRobot();

	ShipState* p_newState = nullptr;/* , * p_prevState = p_init;*/

	b_success = false; //solo se pone a true si se logra la solucion

	for (int n = 0; n < 1800; ++n)
	{
		if (p_initState->isSamePos(p_finalState))
		{
			b_success = true;
			p_newPath->appendState(p_initState);
			p_newPath->_end = p_newState;
			return p_newPath;
		}

		

		// Obtain the control action
		std::vector<double> v_ctrlAct = p_newPath->navigation(p_initState, p_finalState);
		if (p_newPath->isGhostThere(p_initState, p_finalState))
		{
			v_ctrlAct[0] = 0.0;
			v_ctrlAct[1] = 0.0;
			v_ctrlAct[2] = 0.0;//Just for simple dynamics
		}

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

bool EGKRRT::EGKtree::EGKpath::isGhostThere(ShipState* donkey, ShipState* carrot)
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

double EGKRRT::EGKtree::distance(RobotState* rs, PathSegment* path, RobotState** mnode)
{
	ShipState* p = dynamic_cast<ShipState*>(rs);
	ShipState* mn = dynamic_cast<ShipState*>(path->_init);
	if (!p || !mn)
		return -1.0;

	bool b_aux = false;

	EGKpath* path_aux = EGKpath::createPath(path->_init, p, b_aux);

	double minimal = path_aux->getLength();

	if (!b_aux)
		minimal *= 1.5;

	//end belongs to the path
	
	for (RobotState* i: path->_inter)
	{
		ShipState* i_aux = dynamic_cast<ShipState*>(i);
		if (i_aux)
		{
			path_aux = EGKpath::createPath(i_aux, p, b_aux);

			double val = path_aux->getLength();

			if (!b_aux)
				val *= 1.5;// Sustituyo al rayo de visibilidad

			if (val < minimal)
			{
				mn = i_aux;
				minimal = val;
			}
		}
		else
			continue;
	}
	if (mnode)
		*mnode = mn;
	return minimal;
}

//RDTstar::RDTtree::PathSegment* EGKRRT::EGKtree::getBest(vector<RobotState*>& v_nei, RobotState** best)
//{
//	sort(
//			v_nei.begin(), 
//			v_nei.end(), 
//			[](RobotState* const s1, RobotState* const s2)
//			{
//				if (dynamic_cast<ShipState*>(s1) && dynamic_cast<ShipState*>(s2))
//					return dynamic_cast<ShipState*>(s1)->getCost() > dynamic_cast<ShipState*>(s2)->getCost();
//				else return false;
//			}
//		);
//
//	return nullptr;
//}

RDTstar::RDTtree::PathSegment* EGKRRT::EGKtree::getClosestPathSegment(RobotState* n, RobotState** minstate)
{
	*minstate = 0;
	if (_paths.size() == 0)return 0;
	RobotState* ms;
	double dist, minimun = distance(n, _paths[0], minstate);
	PathSegment* minPath = _paths[0];
	for (unsigned int i = 1; i < _paths.size(); i++) {
		dist = distance(n, _paths[i], &ms);
		if (dist < minimun) {
			minimun = dist;
			minPath = _paths[i];
			*minstate = ms;
		}
	}
	return minPath;
}

RDTstar::RDTtree::PathSegment* EGKRRT::EGKtree::getBest(vector<RobotState*>& v_nei, RobotState** best)
{

	if (v_nei.size() == 0)
		return nullptr;

	*best = v_nei[0];

	double min_cost = 0;

	if (dynamic_cast<ShipState*>(v_nei[0]))
		min_cost = dynamic_cast<ShipState*>(v_nei[0])->getCost();



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

void EGKRobotPath::drawGL()
{
	if (path.size() < 2)return;

	glLineWidth(3);
	glColor3f(0.2F, 1.0F, 0.2F);//1, 0.2F, 0.2F
	glDisable(GL_LIGHTING);
	glBegin(GL_LINE_STRIP);
	for (RobotState* i: path)
		glVertex3f(dynamic_cast<ShipState*>(i)->getPose().x, dynamic_cast<ShipState*>(i)->getPose().y, 0.0);
	

	glEnd();
	glEnable(GL_LIGHTING);
}