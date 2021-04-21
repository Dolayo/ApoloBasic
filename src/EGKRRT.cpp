#include "EGKRRT.h"
#include <algorithm>
#include "ShipState.h"
#include "defines.h"


//double EGKRRT::EGKtree::distance(RobotState* p, PathSegment* path, RobotState** mnode)
//{
//
//} en un principio no hace falta reescribir la funcion distancia.

RobotState* EGKRRT::EGKtree::addNode(RobotState* node)
{
	RobotState* in = dynamic_cast<ShipState*>(node);
	if (!in)
		return nullptr;
	// primer nodo del segmento que tratamos de crear al añadir el nodo in
	RobotState* initNode;
	RobotState* n = in->clone();

	// tomo el punto de conexion: estado y segmento
	// aux es el segmento del arbol mas cercano al nodo que tratamos de añadir(in/n)
	// initNode almacena el nodo de dicho segmento que esta mas cerca de n
	PathSegment* aux = getClosestPathSegment(n, &initNode);

	// Si no hay ningun segmento es que el arbol esta vacio, solo tiene el origen
	if (aux == 0)
	{
		initNode = _root;
		if (dynamic_cast<WBStar*>(initNode))
		{
			dynamic_cast<WBStar*>(initNode)->setCost(0);
			//root_aux->setParent(0);
		}
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


	if (_paths.size() != 0 && !(aux->_init->isEqual(initNode)) && !(aux->_end->isEqual(initNode)))
	{
		PathSegment* newPathA = new PathSegment;

		newPathA->_init = aux->_init;
		aux->_init = initNode;

		newPathA->_parent = aux->_parent;
		aux->_parent = newPathA;

		newPathA->_end = aux->_init;

		// Primero localizo la posicion de initNode en aux
		int pos_i = 0;

		for (int i = 0; i < aux->size(); ++i)
		{
			if ((*aux)[i]->isEqual(initNode))
			{
				pos_i = i;
			}
		}

		//Meto los nodos de aux que van antes de initNode en el nuevo segmento newPathA
		for (int i = 0; i <= pos_i; ++i)
		{
			newPathA->_inter.push_back((*aux)[i]);
		}

		// borro los estados de aux que iban antes de initNode
		aux->_inter.erase(aux->_inter.begin(), aux->_inter.begin() + pos_i);

		for (RobotState* nodoi : aux->_inter)
		{
			if ((dynamic_cast<WBStar*>(nodoi)) && dynamic_cast<WBStar*>(aux->_init))
			{
				dynamic_cast<WBStar*>(nodoi)->setCost(nodoi->distanceTo(aux->_init) + dynamic_cast<WBStar*>(aux->_init)->getCost());
			}
		}
		_paths.push_back(newPathA);
	}

	bool success;

	// este segmento sera el que una al nuevo nodo con el segmento que contiene al nodo de 
	// menor coste
	// creamos el nuevo segmento desde initNode hasta n
	EGKpath* newPath = EGKpath::createPath(initNode, n, success, N_ITER);

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

	// si se ha llegado al final incluyo la copia, si no... la destruyo
	if (success)
		newPath->appendState(n); 

	else 
		delete n; 

	// añado al nuevo segmento hasta donde ha sido posible llegar
	newPath->_end = newPath->last();

	// Establezco los costes para cada nodo
	for (RobotState* i: newPath->_inter)
	{
		if (dynamic_cast<WBStar*>(i) && dynamic_cast<WBStar*>(newPath->_init))
			dynamic_cast<WBStar*>(i)->setCost(i->distanceTo(newPath->_init) + dynamic_cast<WBStar*>(newPath->_init)->getCost());
		
		add(i);
	}

	// añado el nuevo path al arbol
	_paths.push_back(newPath);

	if ((_paths.size() != 0) && (neighbors.size() != 0))
	{
		Reconnect(neighbors, newPath->_end);
	}

	// devuelvo el extremo del nuevo segmento añadido
	return newPath->_end;

}

void EGKRRT::EGKtree::Reconnect(vector<RobotState*>& v_nei, RobotState* Xnew)
{
	// Ordeno los vecinos de mayor a menor coste

	sort(v_nei.begin(), v_nei.end(), [](RobotState* const s1, RobotState* const s2)
		{
			if (dynamic_cast<WBStar*>(s1) && dynamic_cast<WBStar*>(s2))
				return dynamic_cast<WBStar*>(s1)->getCost() > dynamic_cast<WBStar*>(s2)->getCost();
			else return false;
		});

	for (RobotState* vecino : v_nei)
	{
		if ((dynamic_cast<WBStar*>(Xnew)) && (dynamic_cast<WBStar*>(vecino)))
		{
			// Si el coste de un vecino es mayor que la distancia a Xnew mas el coste de Xnew, creo el nuevo segmento de Xnew al vecino
			if ((vecino->distanceTo(Xnew) + dynamic_cast<WBStar*>(Xnew)->getCost()) < (dynamic_cast<WBStar*>(vecino)->getCost()))
			{
				// El nuevo segmento que une a Xnew con el vecino
				EGKpath* newRePath = new EGKpath;

				// Buscamos a que segmento pertenece este vecino
				PathSegment* oldPath = findPath4Node(vecino);
				if (!oldPath)
				{
					delete newRePath;
					continue;
				}

				bool success;

				// creamos el nuevo segmento desde Xnew hasta el vecino
				newRePath = EGKpath::createPath(Xnew, vecino, success, N_ITER);

				if (success) { newRePath->appendState(vecino); }
				else
				{
					// Existe un choque
					delete newRePath;
					continue;
				}

				newRePath->_init = Xnew;
				oldPath->_init = vecino;

				newRePath->_parent = findPath4Node(Xnew);

				oldPath->_parent = newRePath;

				newRePath->_end = oldPath->_init;

				// Primero localizo la posicion del vecino en oldPath
				int pos_v = 0;
				for (int j = 0; j < oldPath->size(); ++j)
				{
					if ((*oldPath)[j] == vecino)
					{
						pos_v = j;
					}
				}
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
				oldPath->_inter.erase(oldPath->_inter.begin(), oldPath->_inter.begin() + pos_v);

				//relleno el nuevo segmento
				if ((dynamic_cast<WBStar*>(newRePath->_init)) &&
					(dynamic_cast<WBStar*>(oldPath->_init)))
				{
					for (RobotState* auxState_i : newRePath->_inter)
					{
						if (dynamic_cast<WBStar*>(auxState_i))
						{
							dynamic_cast<WBStar*>(auxState_i)->setCost(auxState_i->distanceTo(newRePath->_init) + dynamic_cast<WBStar*>(newRePath->_init)->getCost());
						}
						add(auxState_i);
					}

					// Cambio los costes de los nodos de oldPath ahora que empieza en vecino
					for (RobotState* oldState_i : oldPath->_inter)
					{
						if (dynamic_cast<WBStar*>(oldState_i))
						{
							dynamic_cast<WBStar*>(oldState_i)->setCost(oldState_i->distanceTo(oldPath->_init) + dynamic_cast<WBStar*>(oldPath->_init)->getCost());
						}
					}
				}
				_paths.push_back(newRePath);
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

	double angAbsFinalState = std::atan(p_ShipFinalState->getPose().y / p_ShipFinalState->getPose().x);

	double angle = angAbsFinalState - p_ShipInitState->getPose().z;

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
			v_auxCtrlAct.push_back(0.);
			v_auxCtrlAct.push_back(-THRUSTY);
			break;
		}
	case ZoneType::central:
		{
			v_auxCtrlAct.push_back(THRUSTX);
			v_auxCtrlAct.push_back(0.);
			break;
		}
	case ZoneType::left:
		{
			v_auxCtrlAct.push_back(0.);
			v_auxCtrlAct.push_back(THRUSTY);
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

	if (!p_initState || !p_finalState)
		return nullptr;
	
	p_initState->placeRobot();

	ShipState* p_newState = nullptr;/* , * p_prevState = p_init;*/
	double dist, antdist;
	b_success = false; //solo se pone a true is se logra la solucion
	dist = p_finalState->distanceTo(p_initState);

	for (int n = 0; n < niter; ++n)
	{
		if (p_initState->isEqual(p_finalState))
		{
			b_success = true;
			return p_newPath;
		}
		
		// Obtain the control action
		std::vector<double> v_ctrlAct = p_newPath->navigation(p_initState, p_finalState);
		if(p_newPath->isGhostThere(p_initState, p_finalState))
		// Propagate the control action
		b_success = p_initState->propagate(v_ctrlAct, DELTA_T, p_newState);

		if (b_success)
		{
			p_initState = p_newState;
			p_newPath->appendState(p_initState);
			p_newPath->appendCtrlAct(v_ctrlAct);
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
	double vx = donkey->getVels().x;
	double vy = donkey->getVels().y;
	double w = donkey->getVels().z;
	double t_stop = (-1.0 * vx * (accs.x/accs.y) - vy) / (accs.y * (1 + (accs.x * accs.x)/(accs.y*accs.y)));
	// Falta propagar la accion de control vacia con es ese tiempo y ver si el estado es igual a la zanahoria
	return b_ret;
}

double EGKRRT::EGKtree::distance(RobotState* rs, PathSegment* path, RobotState** mnode)
{
	ShipState* p = dynamic_cast<ShipState*>(rs);
	ShipState* mn = dynamic_cast<ShipState*>(path->_init);
	if (!p || !mn)
		return -1.0;

	double minimal = p->distanceTo(path->_init);
	double val;
	//end belongs to the path
	for (RobotState* i: path->_inter)
	{
		ShipState* i_aux = dynamic_cast<ShipState*>(i);
		if (i_aux)
		{
			val = p->distanceTo(i_aux);
			if (val < minimal) 
			{
				mn = i_aux;
				minimal = val;
			}
		}
	}
	if (mnode)
		*mnode = mn;
	return minimal;
}


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

	//Meter la modificacion de usar el createPath para ver si es posible llegar en menos iteraciones

	return minPath;
}