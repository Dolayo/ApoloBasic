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
		WBStar* root_aux = dynamic_cast<WBStar*>(initNode);
		if (root_aux != NULL)
		{
			root_aux->setCost(0);
			//root_aux->setParent(0);
		}
	}
	// Compruebo si n puede tener vecinos, si es asi los almaceno, y busco el 
	// de menor coste el cual lo almaceno en initNode
	vector<RobotState*> neighbors;

	if((radius > n->distanceTo(initNode)) && (paths.size()!=0)) 
	{

		getNeighbors(n, &neighbors);

		aux = getBest(neighbors, &initNode);
		
	}

	// El segmento mas cercano aux queda dividido en dos por el nuevo segmento a añadir, por lo que
	// hay que cambiar el padre de los nodos posteriores a initNode hasta el final de aux
	// pero solo en caso de que initNode no sea el principio o el final de aux

	// newPathA es el segmento que va desde el principio de aux a initNode
	// newPathB es el segmento que va de initNode hasta el final de aux
	PathSegment* newPathA = new PathSegment;
	PathSegment* newPathB = new PathSegment;

	if(paths.size()!=0 && !(aux->init->isEqual(initNode)) && !(aux->end->isEqual(initNode)))
	{
		newPathA->init = aux->init;
		newPathB->init = initNode;

		newPathA->parent = aux->parent;
		newPathB->parent = newPathA;

		bool successA;
		bool successB;

		RobotPath* auxPathA = RobotPath::createPath(newPathA->init, newPathB->init, successA);
		RobotPath* auxPathB = RobotPath::createPath(newPathB->init, aux->end, successB);

		auxPathA->add(newPathB->init);
		auxPathB->add(aux->end);

		newPathA->end = auxPathA->last();
		newPathB->end = auxPathB->last();

		for (int i = 0; i < auxPathA->size(); i++)
		{
			if (i != 0 && dynamic_cast<WBStar*>((*auxPathA)[i]) != NULL && dynamic_cast<WBStar*>(auxPathA->path.front()) != NULL)
			{
				dynamic_cast<WBStar*>((*auxPathA)[i])->setCost( (*auxPathA)[i]->distanceTo( auxPathA->path.front() ) + dynamic_cast<WBStar*>(auxPathA->path.front())->getCost() );
				//dynamic_cast<WBStar*>((*auxPathA)[i])->setParent(dynamic_cast<WBStar*>(auxPathA->path.front()));
			}

			newPathA->inter.push_back((*auxPathA)[i]);
		}

		for (int i = 0; i < auxPathB->size(); i++)
		{
			if (i != 0 && dynamic_cast<WBStar*>((*auxPathB)[i]) != NULL && dynamic_cast<WBStar*>(auxPathB->path.front()) != NULL)
			{
				dynamic_cast<WBStar*>((*auxPathB)[i])->setCost((*auxPathB)[i]->distanceTo(auxPathB->path.front()) + dynamic_cast<WBStar*>(auxPathB->path.front())->getCost());
				//dynamic_cast<WBStar*>((*auxPathB)[i])->setParent(dynamic_cast<WBStar*>(auxPathB->path.front()));
			}

			newPathB->inter.push_back((*auxPathB)[i]);
		}

		// Dado que el id de los segmentos se basa en el tamaño del vector segmentos del arbol
		// el id del antiguo segmento aux se lo queda newPathA y newPathB tiene un id como si se añadiese un segmento nuevo
		newPathA->id = aux->id;
		newPathB->id = paths.size() + 1;
		deletePath(aux->id);
		paths.push_back(newPathA);
		paths.push_back(newPathB);
	}

	else
	{
		delete newPathA;
		delete newPathB;
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
		//(dynamic_cast<WBStar*>(newPath->end)->setParent(dynamic_cast<WBStar*>(initNode)));
	}

	// relleno el nuevo segmento 
	for(int i=0;i<auxPath->size();i++)
	{
		//if(i!=0)(*auxPath)[i]->setCost((*auxPath)[i]->distanceTo(auxPath->path.front()) + initNode->getCost());

		if (i != 0 && dynamic_cast<WBStar*>((*auxPath)[i]) != NULL && dynamic_cast<WBStar*>(initNode) != NULL)
		{
			dynamic_cast<WBStar*>((*auxPath)[i])->setCost((*auxPath)[i]->distanceTo(auxPath->path.front()) + dynamic_cast<WBStar*>(initNode)->getCost());
		}

		add((*auxPath)[i]);

		newPath->inter.push_back((*auxPath)[i]);

	}

	// Pongo al primer punto de newPath como padre del resto de puntos del segmento
	/*
	for(int i=0; i<newPath->inter.size(); ++i)
	{
		if(i!=0 && dynamic_cast<WBStar*>(newPath->inter[i])!=NULL && dynamic_cast<WBStar*>(newPath->inter.front())!=NULL)
		{
			dynamic_cast<WBStar*>( newPath->inter[i])->setParent( dynamic_cast<WBStar*>( newPath->inter.front() ) );
		} 
	}
	*/
	

	// añado el nuevo path al arbol
	paths.push_back(newPath);

	newPath->id = paths.size();

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

	if(paths.size()!=0)
	{
		for(auto i_path: paths)
		{
			for(auto j_node: i_path->inter)
			{
				if(j_node->isEqual(node)) return i_path;
			}
		}
	}
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

	if(v_nei.size()==0)return 0;

	*best= v_nei[0];

	double min_cost;

	if(dynamic_cast<WBStar*>(v_nei[0]) != NULL)
	{
		min_cost  = dynamic_cast<WBStar*>(v_nei[0])->getCost();
	}
	

	PathSegment* bestPath = paths[0];

	for(unsigned int i=1; i < v_nei.size(); i++)
	{
		double cost_i;
		if (dynamic_cast<WBStar*>(v_nei[i]) != NULL)
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

	return bestPath;
}

void RDTstar::RDTtree::Reconnect( vector<RobotState*> *v_nei, RobotState* Xnew)
{
	// Ordeno los vecinos de menor a mayor coste
	
	sort(v_nei->begin(),v_nei->end(), [](RobotState* const s1, RobotState* const s2)
		{if (dynamic_cast<WBStar*>(s1) != NULL && dynamic_cast<WBStar*>(s2) != NULL) return dynamic_cast<WBStar*>(s1)->getCost() > dynamic_cast<WBStar*>(s2)->getCost();});

	//for(auto vecino: *v_nei)cout<<vecino->getCost()<<endl;
	// Continua aqui: fijate si puedes eliminar de algun otro lado los nodos que estarían en oldPath

	for(int i=0;i< v_nei->size(); ++i)
	{

		// Si el coste de un vecino es mayor que la distancia a Xnew mas su coste, creo el nuevo segmento del vecino a Xnew
		if (dynamic_cast<WBStar*>(Xnew) != NULL && dynamic_cast<WBStar*>((*v_nei)[i]) != NULL)
		{
			if((*v_nei)[i]->distanceTo(Xnew) + dynamic_cast<WBStar*>(Xnew)->getCost() < dynamic_cast<WBStar*>((*v_nei)[i])->getCost())
			{
				PathSegment* newRePathA = new PathSegment;
				PathSegment* newRePathB = new PathSegment;


				PathSegment* oldPath = findPath4Node((*v_nei)[i]);

				for(int i =0;i<oldPath->size();++i)
				{
					if (dynamic_cast<WBStar*>(oldPath->inter[i]) != NULL)
					{
						cout<<"i: "<<i<<endl;
						cout<< dynamic_cast<WBStar*>(oldPath->inter[i])->getCost()<<endl;
					}				
				}

				int n_pos = 0;

				bool success;

				// creamos el nuevo segmento desde Xnew hasta el vecino
				RobotPath* auxRePathA = RobotPath::createPath(Xnew, (*v_nei)[i], success);

				if (success) { auxRePathA->add((*v_nei)[i]); }
				else
				{
					cout << " Existe un choque" << endl;
					delete auxRePathA;
					continue;
				}
				// Creacion de los 2 nuevos segmentos
				// COntinuacion: Comprueba que esta bien la creacion de los 2 nuevos segmentos y debuguea en windows
				newRePathA->init = Xnew;
				newRePathB->init = (*v_nei)[i];

				newRePathA->parent = findPath4Node(Xnew);
				newRePathB->parent = newRePathA;


				bool successB;

				//RobotPath *auxPathA=RobotPath::createPath(newPathA->init, newPathB->init, successA);
				RobotPath* auxRePathB = RobotPath::createPath(newRePathB->init, oldPath->end, successB);

				auxRePathA->add(newRePathB->init);
				auxRePathB->add(oldPath->end);

				newRePathA->end = auxRePathA->last();
				newRePathB->end = auxRePathB->last();

				if (dynamic_cast<WBStar*>((*auxRePathA)[i]) != NULL && dynamic_cast<WBStar*>((*auxRePathB)[i]) != NULL && dynamic_cast<WBStar*>(auxRePathA->path.front()) != NULL && dynamic_cast<WBStar*>(auxRePathB->path.front()) != NULL)
				{
					for (int i = 0; i < auxRePathA->size(); i++)
					{
						if (i != 0)
						{
							dynamic_cast<WBStar*>((*auxRePathA)[i])->setCost((*auxRePathA)[i]->distanceTo(auxRePathA->path.front()) + dynamic_cast<WBStar*>(auxRePathA->path.front())->getCost());
						}

						/*if (i != 0 && (*auxRePathA)[i] != NULL && auxRePathA->path.front() != NULL)
						{
							dynamic_cast<WBStar*>((*auxRePathA)[i])->setParent(dynamic_cast<WBStar*>(auxRePathA->path.front()));
						}*/

						newRePathA->inter.push_back((*auxRePathA)[i]);
					}

					for (int i = 0; i < auxRePathB->size(); i++)
					{
						if (i != 0)
						{
							dynamic_cast<WBStar*>((*auxRePathB)[i])->setCost((*auxRePathB)[i]->distanceTo(auxRePathB->path.front()) + dynamic_cast<WBStar*>(auxRePathB->path.front())->getCost());
						}

						/*if (i != 0 && (*auxRePathB)[i] != NULL && auxRePathB->path.front() != NULL)
						{
							dynamic_cast<WBStar*>((*auxRePathB)[i])->setParent(dynamic_cast<WBStar*>(auxRePathB->path.front()));
						}*/

						newRePathB->inter.push_back((*auxRePathB)[i]);
					}
	
				}

				newRePathA->id = oldPath->id;
				newRePathB->id = paths.size() + 1;
				deletePath(oldPath->id);
				paths.push_back(newRePathA);
				paths.push_back(newRePathB);
			}
		
		}
	}


}

RDTstar::RDTtree::PathSegment* RDTstar::RDTtree::getPathByID(int n)
{
	for (int i = 0; i < paths.size(); ++i)
	{
		if (paths[i]->id == n) return paths[i];
	}
}

void RDTstar::RDTtree::deletePath(int n)
{
	for (int i = 0; i < paths.size(); ++i)
	{
		if (paths[i]->id == n) paths.erase(paths.begin() + i);
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