#include "rdtstar.h"
//////////////////////////////////////////////////////////////////////////////////// WBStar

RobotState* WBStar::createStateFromSample(vector<double> values)

{
	//valid conditions: there is a robt and a world defined, there are 3 or 4 doubles
	if (!world)return 0;
	if (!robot)return 0;
	if (values.size() < 3)return 0;
	//two options: a 3D position, and a 3D position + orientation
	double val(-11 * PI / 12), inc(PI / 6), half_inc(PI / 12.5);
	bool nuevoSector = false;
	Transformation3D t;
	int i = 0;
	bool flag = true;
	double hmax;
	WheeledBaseSim* r = robot;
	/*primero selecciono la altura más alta valida... que sera a partir de quien genero*/
	//Si tengo 3 argumentos... la altura queda determinada por el más alto de los validos
	//si tengo más de 3 (4), la altura queda determinada por la orientación indicada (si es valida, claro)
	if (values.size() == 3)
	{
		//bucle para detectar el hmax
		for (i = 0; i < 12; i++)
		{
			t = Transformation3D(values[0], values[1], values[2], Z_AXIS, i * inc + half_inc);
			if (checkPose(t)) { ///revisar ¡que pasa con t!
				if (flag)hmax = t.position.z;
				flag = false;
				if (t.position.z > hmax)hmax = t.position.z;
			}
		}
		if (flag)return 0; //no valid poses for that coordinates. 
	}
	else
	{ //hmax is the z, after the drop, if there is no valid pose.. returns 0
		t = Transformation3D(values[0], values[1], values[2], Z_AXIS, values[3]);
		if (checkPose(t))hmax = t.position.z;
		else return 0;

	}
	WBStar aux(r, world, 0);
	//Create the valid sectors
	hmax += 2 * r->getWheelRadius();
	for (i = 0; i < 12; i++)
	{
		t = Transformation3D(values[0], values[1], hmax, Z_AXIS, i * inc + half_inc);
		if (checkPose(t)) { ///revisar ¡que pasa con t!
			//creo un sector
			Sector newSector;
			newSector.min = i * inc;
			newSector.max = (i + 1) * inc;
			newSector.pose = t;
			if (t.position.z > hmax)hmax = t.position.z;
			aux.sectors.push_back(newSector);
		}
	}


	//utilizo el pos mas alto y elimino todos los sectores que esten por debajo una distancia mayor que
	//la admitida por la gota que es el radio de la rueda del robot

	for (i = (int)aux.sectors.size() - 1; i >= 0; i--)
	if (hmax - aux.sectors[i].pose.position.z > (4 * r->getWheelRadius()))aux.sectors.erase(aux.sectors.begin() + i);

	if (aux.sectors.empty())return 0;

	//At his point we obtain the medium position value(X, Y are the same but, 
	Vector3D newPose(0, 0, 0);
	int numvs = aux.sectors.size();
	double factorns = 1.0 / ((double)numvs);
	for (i = 0; i < numvs; i++)newPose = newPose + ((aux.sectors[i].pose.position) * factorns);


	//fusión de sectores
	i = 1;
	while (i < (int)aux.sectors.size())
	{
		if (aux.sectors[i].min - aux.sectors[i - 1].max < EPS) {
			aux.sectors[i - 1].max = aux.sectors[i].max;
			aux.sectors.erase(aux.sectors.begin() + i);
		}
		else i++;
	}
	//creo el nuevo estado
	aux.pose = newPose;
	return new WBStar(aux);

}

WBStar* WBStar::createStateFromPoint3D(double x, double y, double z)
{
	vector<double> aux(3);
	aux[0] = x; aux[1] = y; aux[2] = z;
	return (WBStar*)createStateFromSample(aux);
}

RobotState* WBStar::createStateFromCurrentRobotState()
{
	if (!world)return 0;
	if (!robot)return 0;
	vector<double> v(4);
	Transformation3D t = robot->getAbsoluteT3D();
	double r, p, y;

	v[0] = t.position.x;
	v[1] = t.position.y;
	v[2] = t.position.z;
	t.orientation.getRPY(r, p, y);
	v[3] = y;
	WBStar* aux = (WBStar*)createStateFromSample(v);
	if (aux)aux->specialize(t);
	return aux;
}

RobotState* WBStar::clone()
{
	return new WBStar(*this);
}

WBStar::~WBStar()
{
}

//////////////////////////////////////////////////////////////////////////////////// Planner

bool RDTstar::setStartAndGoalStates(RobotState *start_, RobotState *goal_)
{
if(SBPathPlanner::setStartAndGoalStates(start_, goal_))
	{
		treeA->rootTree(start_);
		treeB->rootTree(goal_);
		return true;
	}
return false;
}

bool RDTstar::computePlan(int maxiterations)
{
	if(solved)return true;
	for(int i=0;i<maxiterations;i++)
	{
		RobotState *node=getNextSample();
		if(!node)continue;
		RobotState *addedNodeA=treeA->addNode(node);
		if(addedNodeA){
			RobotState *addedNodeB=treeB->addNode(addedNodeA);
			if(addedNodeB){
				if(addedNodeA->isEqual(addedNodeB))
				{
					RobotPath pathA,pathB;
					//retrive each path 
					if(treeA==&treeStart){
						pathA=treeA->getPathFromRoot(addedNodeA);
						pathB=treeB->getPathFromRoot(addedNodeB);
						}
					else{
						pathB=treeA->getPathFromRoot(addedNodeA);
						pathA=treeB->getPathFromRoot(addedNodeB);
					}
					//rearrange the states
					delete path;
					path=new RobotPath(pathA);
					solved=true;
					for(int j=(int)pathB.path.size()-1;j>=0;j--)
							path->path.push_back(pathB.path[j]);
					path->filterLoops(); //clean loops if any 
					return true;
				}
			}
		}
		if(treeA->getNumNodes()>treeB->getNumNodes())
		{
			RDTtree *auxt=treeA;
			treeA=treeB;
			treeB=auxt;		
		}
		delete node;
	}
	return false;
}
// Computa la distancia del estado p al segmento path, y almacena el nodo de ese path en mnode(inout)

double RDTstar::RDTtree::distance(RobotState *p, PathSegment *path, RobotState **mnode)
{
	RobotState *mn=path->init;
	double minimal=p->distanceTo(path->init);
	double val;
	//end belongs to the path
	for(int i=0;i<(int)path->inter.size();i++)
	{
		val=p->distanceTo(path->inter[i]);
		if(val<minimal){
			mn=path->inter[i];
			minimal=val;
		}

	}
	
	if(mnode)*mnode=mn;
	return minimal;
}

RobotPath RDTstar::RDTtree::getPathFromRoot(RobotState *n)
{
RobotPath path;
RobotState *rs;
PathSegment *p=0;

	int i;
	for(i=0;i<(int)paths.size();i++)if(paths[i]->end==n){p=paths[i];break;}
	if(p==0)return path;
	for(i=(int)p->inter.size()-1;i>=0;i--)
		path.path.insert(path.path.begin(),(p->inter)[i]);
	rs=p->init;
	while(p->parent!=0){
		//busco el nodo de inicio
		p=p->parent;
		for(int j=0;j<(int)p->inter.size();j++)if(rs==p->inter[j])
		{
			path.path.insert(path.path.begin(),rs);
			for(int k=j-1;k>=0;k--)
				path.path.insert(path.path.begin(),(p->inter)[k]);
		}
		rs=p->init;
	}
	path.path.insert(path.path.begin(),root);
	return path;

}
// Encuentra el segmento del arbol mas cercano al estado n y almacena el nodo de ese segmento en minstate(inout)
RDTstar::RDTtree::PathSegment *RDTstar::RDTtree::getClosestPathSegment(RobotState *n,RobotState **minstate)
{
	*minstate=0;
	if(paths.size()==0)return 0;
	RobotState *ms;
	double dist,minimun=distance(n,paths[0],minstate);
	PathSegment *minPath=paths[0];
	for(unsigned int i=1;i<paths.size();i++){
		dist=distance(n,paths[i],&ms);
		if(dist<minimun){
			minimun=dist;
			minPath=paths[i];
			*minstate=ms;
		}
	}

	return minPath;
}

RobotState *RDTstar::RDTtree::addNode(RobotState *in)
{
	//primero miro que no exista un nodo equivalente ya en el arbol
//	for(int i=0;i<(int)nodes.size();i++)
//		if(in->isEqual(nodes[i]))return 0;
	
    // primer nodo del segmento que tratamos de crear al añadir el nodo in
	RobotState *initNode;
	RobotState *n=in->clone();
	
	// tomo el punto de conexion: estado y segmento
    // aux es el segmento del arbol mas cercano al nodo que tratamos de añadir(in/n)
    // initNode almacena el nodo de dicho segmento que esta mas cerca de n
	PathSegment *aux=getClosestPathSegment(n,&initNode);

    // Si no hay ningun segmento es que el arbol esta vacio, solo tiene el origen
	if(aux==0)
	{
		initNode = root;
		if (dynamic_cast<WBStar*>(initNode) != NULL)
		{
			dynamic_cast<WBStar*>(initNode)->setCost(0);
			//root_aux->setParent(0);
		}
	}
	// Compruebo si n puede tener vecinos, si es asi los almaceno en neighbors, y busco el 
	// de menor coste el cual lo almaceno en initNode
	vector<RobotState*> neighbors;

	if((radius > n->distanceTo(initNode)) && (paths.size()!=0)) 
	{

		getNeighbors(n, &neighbors);

		aux = getBest(neighbors, &initNode);

	// El segmento mas cercano aux queda dividido en dos por el nuevo segmento a añadir, por lo que
	// hay que cambiar el padre de los nodos posteriores a initNode hasta el final de aux
	// pero solo en caso de que initNode no sea el principio o el final de aux

	// newPathA es el segmento que va desde el principio de aux a initNode
	// newPathB es el segmento que va de initNode hasta el final de aux

		if (paths.size() != 0 && !(aux->init->isEqual(initNode)) && !(aux->end->isEqual(initNode)))
		{

			PathSegment* newPathA = new PathSegment;
			

			newPathA->init = aux->init;
			aux->init = initNode;

			newPathA->parent = aux->parent;
			aux->parent = newPathA;

			newPathA->end = aux->init;

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
			for (int i = 0; i<=pos_i; ++i)
			{
				newPathA->inter.push_back((*aux)[i]);
			}

			//FALTA BORRAR LOS NODOS DE LA LISTA DE NODOS DEL ARBOL

			// borro los estados de aux que iban antes de initNode
			aux->inter.erase(aux->inter.begin(), aux->inter.begin() + pos_i-1);

			paths.push_back(newPathA);
		}		
	}


	


	//este segmento sera el que una al nuevo nodo con el segmento que contiene al nodo de 
	// menor coste
	PathSegment *newPath=new PathSegment;

	// inicializo el nuevo segmento con el nodo de menor coste
	newPath->init=initNode;

	//el padre del nuevo segmento es el segmento que contiene al nodo de menor coste
	newPath->parent=aux;

	bool success;
	// creamos el nuevo segmento desde initNode hasta n
	RobotPath *auxPath=RobotPath::createPath(initNode,n,success);

	// Si el nuevo segmento esta vacio, eliminamos todo 
	if(auxPath->size()==0){
		delete newPath;
		delete auxPath;
		delete n;

		return 0;//no state created
	}
	// si se ha llegado al final incluyo la copia, si no... la destruyo
	if(success){auxPath->add(n);}
	
	else{delete n;} 

	// añado al nuevo segmento hasta donde ha sido posible llegar
	newPath->end=auxPath->last();
	
	if (dynamic_cast<WBStar*>(newPath->end) != NULL && dynamic_cast<WBStar*>(initNode) != NULL)
	{
		(dynamic_cast<WBStar*>(newPath->end)->setCost(newPath->end->distanceTo(initNode) + dynamic_cast<WBStar*>(initNode)->getCost()));
	}

	// relleno el nuevo segmento 
	for(int i=0;i<auxPath->size();i++)
	{
		if (i != 0 && dynamic_cast<WBStar*>((*auxPath)[i]) != NULL && dynamic_cast<WBStar*>(newPath->init) != NULL)
		{
			dynamic_cast<WBStar*>((*auxPath)[i])->setCost((*auxPath)[i]->distanceTo(newPath->init) + dynamic_cast<WBStar*>(newPath->init)->getCost());
		}

		add((*auxPath)[i]);

		newPath->inter.push_back((*auxPath)[i]);

	}

	// añado el nuevo path al arbol
	paths.push_back(newPath);

	if(paths.size()!=0)
	{
		Reconnect(&neighbors, newPath->end);
	}
	
	// devuelvo el extremo del nuevo segmento añadido
	return newPath->end;



}

bool RDTstar::RDTtree::rootTree(RobotState *rot)
{	
	//clear graph
	for(unsigned int i=0;i<nodes.size();i++)delete nodes[i];
	nodes.clear();
	paths.clear();
	root=0;
	root=rot->clone();
	if(root->isValid()==false)
	{
		delete root;
		root=0;
		return false;
	}
	nodes.push_back(root);
	return true;
}

RDTstar::RDTtree::PathSegment* RDTstar::RDTtree::findPath4Node( RobotState* node)
{

	if (paths.size() != 0)
	{
		for (auto i_path : paths)
		{
			for (auto j_node : i_path->inter)
			{
				if (j_node->isEqual(node)) return i_path;
				
			}
		}
		return nullptr;
	}

	else return nullptr;
}
bool RDTstar::RDTtree::PathSegment::isEqual(PathSegment* p)
{
	bool c = true;
	if (this->size() == p->size())
	{
		for (int i = 0; i < this->size(); ++i)
		{
			c = c && (*this)[i]->isEqual((*p)[i]);
		}

		return c;
	}
	else return false;
}
void RDTstar::RDTtree::getNeighbors(RobotState *Xnew, vector<RobotState*> *v_nei)
{
	
	for(auto i_path: paths)
	{
		for(auto j_node: i_path->inter)
		{
			if(j_node->distanceTo(Xnew)<radius) v_nei->push_back(j_node);
		}
	}
}

RDTstar::RDTtree::PathSegment* RDTstar::RDTtree::getBest(vector<RobotState*> v_nei, RobotState **best)
{

	if(v_nei.size()==0)return nullptr;

	*best= v_nei[0];

	double min_cost=0;

	if(dynamic_cast<WBStar*>(v_nei[0]))
	{
		min_cost  = dynamic_cast<WBStar*>(v_nei[0])->getCost();
	}
	

	PathSegment* bestPath = paths[0];

	for(unsigned int i=1; i < v_nei.size(); i++)
	{
		double cost_i;
		if (dynamic_cast<WBStar*>(v_nei[i]))
		{
			cost_i = dynamic_cast<WBStar*>(v_nei[i])->getCost();

			if(cost_i < min_cost)
			{
				min_cost=cost_i;
				*best=v_nei[i];
			}

		}
		

		
	}

	bestPath=findPath4Node(*best);

	if (bestPath) return bestPath;
	else return nullptr;
}

void RDTstar::RDTtree::Reconnect( vector<RobotState*> *v_nei, RobotState* Xnew)
{
	// Ordeno los vecinos de mayor a menor coste
	
	sort(v_nei->begin(),v_nei->end(), [](RobotState* const s1, RobotState* const s2)
		{
			if (dynamic_cast<WBStar*>(s1) && dynamic_cast<WBStar*>(s2))
				return dynamic_cast<WBStar*>(s1)->getCost() > dynamic_cast<WBStar*>(s2)->getCost();
			else return false;
		});

	
	//for(auto vecino: *v_nei)cout<<vecino->getCost()<<endl;
	// Continua aqui: fijate si puedes eliminar de algun otro lado los nodos que estarían en oldPath

	for(int i=0;i< v_nei->size(); ++i)
	{
		if (dynamic_cast<WBStar*>(Xnew) != NULL && dynamic_cast<WBStar*>((*v_nei)[i]) != NULL)
		{
			// Si el coste de un vecino es mayor que la distancia a Xnew mas su coste, creo el nuevo segmento de Xnew al vecino
			if((*v_nei)[i]->distanceTo(Xnew) + dynamic_cast<WBStar*>(Xnew)->getCost() < dynamic_cast<WBStar*>((*v_nei)[i])->getCost())
			{
				// El nuevo segmento que une a Xnew con el vecino
				PathSegment* newRePath = new PathSegment;

				// Buscamos a que segmento pertenece este vecino
				PathSegment* oldPath = findPath4Node((*v_nei)[i]);
				if (!oldPath) 
				{
					delete newRePath;
					continue;
				}

				bool success;

				// creamos el nuevo segmento desde Xnew hasta el vecino
				RobotPath* auxRePath = RobotPath::createPath(Xnew, (*v_nei)[i], success);

				if (success) { auxRePath->add((*v_nei)[i]); }
				else
				{
					cout << " Existe un choque" << endl;
					delete auxRePath;
					delete newRePath;
					continue;
				}
	
				newRePath->init = Xnew;
				oldPath->init = (*v_nei)[i];

				newRePath->parent = findPath4Node(Xnew);
				if (!(newRePath->parent)) 
				{
					delete auxRePath;
					delete newRePath;
					continue;
				}
				oldPath->parent = newRePath;

				
				// Primero localizo la posicion del vecino
				int pos_v = 0;
				for (int j = 0; j < oldPath->size(); ++j)
				{
					if ((*oldPath)[j]->isEqual((*v_nei)[i]))
					{
						pos_v = j;
					}
				}
				//FALTA BORRAR LOS NODOS DE LA LISTA DE NODOS DEL ARBOL
				// borro los estados de old path que iban antes del vecino
				oldPath->inter.erase(oldPath->inter.begin(), oldPath->inter.begin() + pos_v-1);

				auxRePath->add(oldPath->init);

				newRePath->end = auxRePath->last();

				//relleno el nuevo segmento
				if (( dynamic_cast<WBStar*>(auxRePath->path.front()) )&&
					( dynamic_cast<WBStar*>(oldPath->init) ))
				{
					for (int j = 0; j < auxRePath->size(); j++)
					{
						if (( j != 0 )&&( dynamic_cast<WBStar*>((*auxRePath)[j]) ))
						{
							dynamic_cast<WBStar*>((*auxRePath)[j])->setCost((*auxRePath)[j]->distanceTo(newRePath->init) + dynamic_cast<WBStar*>(newRePath->init)->getCost());
						}
						add((*auxRePath)[j]);
						newRePath->inter.push_back((*auxRePath)[j]);
					}

					for (int j = 0; j < oldPath->size(); j++)
					{
						if (j != 0 && ( dynamic_cast<WBStar*>((*oldPath)[j]) ))
						{
							dynamic_cast<WBStar*>((*oldPath)[j])->setCost((*oldPath)[j]->distanceTo(oldPath->init) + dynamic_cast<WBStar*>(oldPath->init)->getCost());
						}
					}
				}
				paths.push_back(newRePath);
			}
		}
	}
}

void RDTstar::RDTtree::deletePath(PathSegment* p)
{
	for (int i = 0; i < paths.size(); ++i)
	{
		if (paths[i]->isEqual(p))
		{
			delete paths[i];
			paths.erase(paths.begin() + i);
		}
	}

}

//////////////Painting methods
void RDTstar::RDTtree::PathSegment::drawGL()
{
	vector<double> v;
	v=init->getSample();
	if(v.size()<2)return;
	glDisable(GL_LIGHTING);
	glBegin(GL_LINE_STRIP);
		if(v.size()==2)glVertex3f(v[0],v[1],0);
		else glVertex3f(v[0],v[1],v[2]);
		for(int i=0;i<(int)inter.size();i++){
			v=inter[i]->getSample();
			if(v.size()==2)glVertex3f(v[0],v[1],0);
			else glVertex3f(v[0],v[1],v[2]);		
		}
		v=end->getSample();
		if(v.size()==2)glVertex3f(v[0],v[1],0);
		else glVertex3f(v[0],v[1],v[2]);
	glEnd();
	glEnable(GL_LIGHTING);
}
void RDTstar::RDTtree::drawGL()
{
//pinta las trayectorias: nodos y lineas que las unen
unsigned int i;
glLineWidth(1);
glColor3f(1,1,0);
for(i=0;i<paths.size();i++)paths[i]->drawGL();
for(i=0;i<nodes.size();i++)nodes[i]->drawGL();
if(root)root->drawGL();

}
void RDTstar::drawGL()
{
	if(treeA) treeA->drawGL();
	if(treeB) treeB->drawGL();

}