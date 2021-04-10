#include "EGKRRT.h"
#include <algorithm>
#include "ShipState.h"

#define DELTA_T 0.1 //seconds

//double EGKRRT::EGKtree::distance(RobotState* p, PathSegment* path, RobotState** mnode)
//{
//
//} en un principio no hace falta reescribir la funcion distancia.

RobotState* EGKRRT::EGKtree::addNode(RobotState* n)
{

}

void EGKRRT::EGKtree::Reconnect(vector<RobotState*>& v_nei, RobotState* Xnew)
{

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
	


	return v_auxCtrlAct;
}

EGKRRT::EGKtree::EGKpath* EGKRRT::EGKtree::EGKpath::createPath(RobotState* p_init, RobotState* p_end, bool& b_success, int niter)
{
	EGKpath* p_newPath = new EGKpath;
	ShipState* p_initState = nullptr;
	ShipState* p_finalState = nullptr;

	if (dynamic_cast<ShipState*>(p_init) && dynamic_cast<ShipState*>(p_end))
	{
		p_initState = dynamic_cast<ShipState*>(p_init);
		p_finalState = dynamic_cast<ShipState*>(p_end);
	}
	else
	{
		return nullptr;
	}
	
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
		// Make the control action
		std::vector<double> v_ctrlAct = p_newPath->navigation(p_initState, p_finalState);
		
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


