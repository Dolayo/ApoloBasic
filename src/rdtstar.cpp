#include "rdtstar.h"

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
		initNode=root;
		initNode->setCost(0);
		cout<<" Es esto?"<<endl;
		WBStar* aux_dynamic = dynamic_cast<WBStar*>(initNode);
		if(aux_dynamic!=NULL)aux_dynamic->setParent(0);
		cout<<" NO"<<endl;
	}
	// Compruebo si n puede tener vecinos, si es asi los almaceno, y busco el 
	// de menor coste el cual lo almaceno en initNode
	vector<RobotState*> neighbors;

	if((radius > n->distanceTo(initNode)) && (paths.size()!=0)) 
	{

		getNeighbors(n, &neighbors);

		aux = getBest(neighbors, &initNode);
		
	}

	n->setCost(n->distanceTo(initNode) + initNode->getCost());

	// El segmento mas cercano aux queda dividido por el nuevo segmento a añadir, por lo que
	// hay que eliminar dicho segmento y sustituirlo por dos segmentos que vayan desde el principio de aux
	// a initNode, y de initNode hasta el final ed aux

	PathSegment* newPathA=new PathSegment;
	PathSegment* newPathB=new PathSegment;
	
	if(paths.size()!=0)
	{
		/*cout<<" Empieza el primer cambio"<<endl;
		WBStar* actual_p;
		if(initNode!=NULL)actual_p = (dynamic_cast<WBStar*>(initNode))->getParent();
		cout<<" primer dynamic_cast correcto"<<endl;

		// Posicion de initNode en el segmento aux
		unsigned int j;

		for(int i=0; i<aux->inter.size(); ++i)
		{
			if(initNode->isEqual(aux->inter[i])) j = i+1;
		}
		cout<<" Posicion j conocida"<<j<<endl;
		bool same_p;
		if(aux->inter[j]!=NULL)same_p = (dynamic_cast<WBStar*>( aux->inter[j]))->getParent()->isEqual(actual_p);
		cout<<" Variable same_p"<<same_p<<endl;
		while(same_p && (j<aux->inter.size()))
		{
			if( aux->inter[j] != NULL)
			{	
				++j;
				(dynamic_cast<WBStar*>( aux->inter[j]))->setParent(dynamic_cast<WBStar*>(initNode));
				same_p = (dynamic_cast<WBStar*>( aux->inter[j]))->getParent()->isEqual(actual_p);
			}
		}
		cout<<" While completado"<<j<<endl;
		*/

		newPathA->init=aux->init;
		newPathB->init=initNode;

		newPathA->parent=aux->parent;
		newPathB->parent=newPathA;

		bool successA;
		bool successB;

		RobotPath *auxPathA=RobotPath::createPath(newPathA->init, newPathB->init, successA);
		RobotPath *auxPathB=RobotPath::createPath(newPathB->init, aux->end,successB);
		
		auxPathA->add(newPathB->init);
		auxPathB->add(aux->end);

		newPathA->end=auxPathA->last();
		newPathB->end=auxPathB->last();

		for(int i=0;i<auxPathA->size();i++)
		{
			if(i!=0)(*auxPathA)[i]->setCost((*auxPathA)[i]->distanceTo(auxPathA->path.front()) + auxPathA->path.front()->getCost());

			if(i!=0 && (*auxPathA)[i]!=NULL && auxPathA->path.front()!= NULL) dynamic_cast<WBStar*>((*auxPathA)[i])->setParent(dynamic_cast<WBStar*>(auxPathA->path.front()));

			newPathA->inter.push_back((*auxPathA)[i]);
		}

		for(int i=0;i<auxPathB->size();i++)
		{
			if(i!=0)(*auxPathB)[i]->setCost((*auxPathB)[i]->distanceTo(auxPathB->path.front()) + auxPathB->path.front()->getCost());

			if(i!=0 && (*auxPathB)[i]!=NULL && auxPathB->path.front()!= NULL) dynamic_cast<WBStar*>((*auxPathB)[i])->setParent(dynamic_cast<WBStar*>(auxPathB->path.front()));

			newPathB->inter.push_back((*auxPathB)[i]);
		}

		newPathA->id = aux->id;
		newPathB->id = paths.size()+1;
		deletePath(aux->id);
		paths.push_back(newPathA);
		paths.push_back(newPathB);

	}


	//este segmento sera el que una al nuevo nodo con el segmento que contiene al nodo de 
	// menor coste
	PathSegment *newPath=new PathSegment;

	// inicializo el nuevo segmento con el nodo de menor coste
	newPath->init=initNode;

	//el padre del nuevo segmento es el segmento que contiene al nodo de menor coste
	newPath->parent=newPathA;

	// Asigno el id del path en funcion del orden en que es insertado en el arbol
	newPath->id = paths.size()+1;
	
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


	// relleno el nuevo segmento 
	for(int i=0;i<auxPath->size();i++)
	{
		if(i!=0)(*auxPath)[i]->setCost((*auxPath)[i]->distanceTo(auxPath->path.front()) + initNode->getCost());

		add((*auxPath)[i]);

		newPath->inter.push_back((*auxPath)[i]);

	}

	// Pongo al primer punto de newPath como padre del resto de puntos del segmento
	cout<<" Y esto?"<<endl;
	for(int i=0; i<newPath->inter.size(); ++i)
	{
		if(i!=0 && ((newPath->inter[i])!=NULL) && (newPath->inter.front()!=NULL))
		{
			(dynamic_cast<WBStar*>( newPath->inter[i]))->setParent(dynamic_cast<WBStar*>(newPath->inter.front()));
		} 
	}
	cout<<" NO"<<endl;


	
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
	double min_cost = v_nei[0]->getCost();

	PathSegment* bestPath = paths[0];

	for(unsigned int i=1; i < v_nei.size(); i++)
	{
		double cost_i = v_nei[i]->getCost();

		if(cost_i < min_cost)
		{
			min_cost=cost_i;
			*best=v_nei[i];
		}
	}

	bestPath=findPath4Node(*best);

	return bestPath;
}

void RDTstar::RDTtree::Reconnect( vector<RobotState*> *v_nei, RobotState* Xnew)
{
	// Ordeno los vecinos de menor a mayor coste
	sort(v_nei->begin(),v_nei->end(), [](RobotState* const s1, RobotState* const s2){return s1->getCost()>s2->getCost();});
	for(auto vecino: *v_nei)cout<<vecino->getCost()<<endl;
	cout<<" Vecinos ordenados"<<endl;

	PathSegment *newRePath=new PathSegment;

	PathSegment* newRePathA=new PathSegment;
	PathSegment* newRePathB=new PathSegment;

	for(int i=0;i< v_nei->size(); ++i)
	{
		
		// Si el coste de un vecino es mayor que la distancia a Xnew mas su coste, creo el nuevo segmento del vecino a Xnew
		if((*v_nei)[i]->distanceTo(Xnew) + Xnew->getCost() < (*v_nei)[i]->getCost())
		{
			cout<<" Hemos encontrado un vecino que quiere un cambio"<<endl;
			// Eliminamos el resto del segmento que va a partir del vecino en cuestion

			PathSegment* oldPath = findPath4Node((*v_nei)[i]);

			cout<<" Hemos encontrado el pathsegment del vecino"<<endl;
			cout<<" NUmero de nodos del path segment: "<< oldPath->size()<<endl;
			for(int i = 0;i<oldPath->size();++i)
			{
				cout<<"i: "<<i<<endl;
				cout<<oldPath->inter[i]->getCost()<<endl;
			}

			cout<<" Nuestro vecino tiene un coste de: "<<(*v_nei)[i]->getCost() <<endl;

			int n_pos = 0;

			//Nos quedamos con el vecino que ademas de tener el coste más bajo este libre de obstaculos 
			bool success;

			// creamos el nuevo segmento desde Xnew hasta el vecino
			RobotPath *auxRePathA=RobotPath::createPath(Xnew,(*v_nei)[i],success);

			if(success){auxRePathA->add((*v_nei)[i]);}
			else
			{
				cout<<" Existe un choque"<<endl;
				delete auxRePathA;
				continue;
			}
			 // Creacion de los 2 nuevos segmentos
			 // COntinuacion: Comprueba que esta bien la creacion de los 2 nuevos segmentos y debuguea en windows
			newRePathA->init=Xnew;
			newRePathB->init=(*v_nei)[i];

			newRePathA->parent = findPath4Node(Xnew);
			newRePathB->parent = newRePathA;

			
			bool successB;

			//RobotPath *auxPathA=RobotPath::createPath(newPathA->init, newPathB->init, successA);
			RobotPath *auxRePathB=RobotPath::createPath(newRePathB->init, oldPath->end,successB);
			
			auxRePathA->add(newRePathB->init);
			auxRePathB->add(oldPath->end);

			newRePathA->end=auxRePathA->last();
			newRePathB->end=auxRePathB->last();

			for(int i=0;i<auxRePathA->size();i++)
			{
				if(i!=0)(*auxRePathA)[i]->setCost((*auxRePathA)[i]->distanceTo(auxRePathA->path.front()) + auxRePathA->path.front()->getCost());

				if(i!=0 && (*auxRePathA)[i]!=NULL && auxRePathA->path.front()!= NULL) dynamic_cast<WBStar*>((*auxRePathA)[i])->setParent(dynamic_cast<WBStar*>(auxRePathA->path.front()));

				newRePathA->inter.push_back((*auxRePathA)[i]);
			}

			for(int i=0;i<auxRePathB->size();i++)
			{
				if(i!=0)(*auxRePathB)[i]->setCost((*auxRePathB)[i]->distanceTo(auxRePathB->path.front()) + auxRePathB->path.front()->getCost());

				if(i!=0 && (*auxRePathB)[i]!=NULL && auxRePathB->path.front()!= NULL) dynamic_cast<WBStar*>((*auxRePathB)[i])->setParent(dynamic_cast<WBStar*>(auxRePathB->path.front()));

				newRePathB->inter.push_back((*auxRePathB)[i]);
			}

			newRePathA->id = oldPath->id;
			newRePathB->id = paths.size()+1;
			deletePath(oldPath->id);
			paths.push_back(newRePathA);
			paths.push_back(newRePathB);
			
		}
	}


}

RDTstar::RDTtree::PathSegment* RDTstar::RDTtree::getPathByID(int n)
{
	for(int i=0;i<paths.size();++i)
	{
		if(paths[i]->id == n) return paths[i];
	}
}

void RDTstar::RDTtree::deletePath(int n)
{
	for(int i=0;i<paths.size();++i)
	{
		if(paths[i]->id==n) paths.erase(paths.begin()+i);
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