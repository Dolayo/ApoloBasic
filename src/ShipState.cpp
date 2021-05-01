#include "ShipState.h"
#include "defines.h"


bool ShipState::isEqual(RobotState* n)
{
	//Equal if compatible
	// For now, compatible if less than a constant
	ShipState* naux = dynamic_cast<ShipState*>(n);
	if (!naux)return false;
	if (_ship == 0)return false;

	Vector3D dif_pose = naux->_pose - _pose;
	Vector3D dif_vel = naux->_vel - _vel;
	if ((dif_pose.module() < POSE_TOL) && (dif_vel.module() < VEL_TOL))
		return true;
	else return false;
}

bool ShipState::isSamePos(RobotState* n)
{
	//Equal if compatible
	// For now, compatible if less than a constant
	ShipState* naux = dynamic_cast<ShipState*>(n);
	if (!naux)return false;
	if (_ship == 0)return false;

	Vector3D dif_pose = naux->_pose - _pose;
	dif_pose.z = 0.0;
	if ((dif_pose.module() < POSE_TOL))
		return true;
	else return false;
}



//For now, we will use just the differences between vectors of speed and position

double ShipState::distanceTo(RobotState* p)
{

	ShipState* naux = dynamic_cast<ShipState*>(p);
	if (!naux)return 10000.0;

	Vector3D dif_pos = naux->_pose - _pose;
	
	double val = dif_pos.module();

	Segment3D segm = Segment3D(Vector3D(_pose.x, _pose.y,0), Vector3D(naux->_pose.x, naux->_pose.y, 0));

	_ship->setIntersectable(false);

	//penalties the intersection
	if (_world->segmentIntersection(segm, 0))
		val *= 1.5;

	return val;
}

bool ShipState::isEqualToCurrentRobotState()
{
	Vector3D dif_pose = _ship->getPos()- _pose;
	Vector3D dif_vel = _ship->getVels() - _vel;
	if ((dif_pose.module() < POSE_TOL) && (dif_vel.module() < VEL_TOL))
		return true;
	else return false;
}

RobotState* ShipState::createStateFromCurrentRobotState()
{
	if (!_world)return 0;
	if (!_ship)return 0;
	vector<double> v(3);
	Vector3D t = _ship->getPos();

	//v[0] = t.position.x;
	//v[1] = t.position.y;

	v[0] = t.x;
	v[1] = t.y;
	v[2] = t.z;

	RobotState* aux = createStateFromSample(v);
	dynamic_cast<ShipState*>(aux)->setVels(_ship->getVels());
	return aux;
}

RobotState* ShipState::createStateFromPoint3D(double x, double y, double yaw)
{
	vector<double> aux(3);
	aux[0] = x; aux[1] = y; aux[2] = yaw;
	return (ShipState*)createStateFromSample(aux);
}

//creates a ship state with a set of doubles. if invalid return false
RobotState* ShipState::createStateFromSample(vector<double> values)
{
	/*Values: 
		[0]: x
		[1]: y
		[2]: z not used
		[3]: yaw
		[4]: Vx
		[5]: Vy
		[6]: Vw
	*/

	//valid conditions: there is a robt and a world defined
	if (!_world)return 0;
	if (!_ship)return 0;
	if (values.size() < 3)return 0;

	Ship* s = _ship;
	ShipState aux(s, _world);
	aux._pose = Vector3D(values[0], values[1], 0.0);//x,y,z
	aux._yaw = values[2];

	return new ShipState(aux);
}

vector<double> ShipState::getSample()
{
	return vector<double>{_pose.x, _pose.y, _pose.z, _vel.x, _vel.y, _vel.z/*W*/};
}

bool ShipState::propagate(std::vector<double> v_auxCtrlAct, double delta_t, ShipState* p_retState)
{
	bool b_success;

	_ship->setThrusts(v_auxCtrlAct[0], v_auxCtrlAct[1], v_auxCtrlAct[2]);

	b_success = _ship->simpleDynamicsSim(delta_t);

	RobotState* aux_state = createStateFromCurrentRobotState();

	if (dynamic_cast<ShipState*>(aux_state))
		p_retState = dynamic_cast<ShipState*>(aux_state);
	else b_success = false;
	
	return b_success;
}

void ShipState::placeRobot()
{
	Transformation3D t(_pose.x, _pose.y, 0, Z_AXIS, _pose.z);
	_ship->setRelativeT3D(t);
	_ship->setPos(Vector3D(_pose.x, _pose.y, _pose.z));
}

void ShipState::placeRobotTowards(RobotState* target)
{
	ShipState* naux = dynamic_cast<ShipState*>(target);
	if (!naux)return;
	Vector3D dif = naux->_pose - _pose;
	double angle = atan2(dif.y, dif.x);

	Transformation3D t(_pose.x, _pose.y, _pose.z, Z_AXIS, angle);
	_ship->setRelativeT3D(t);

}

bool ShipState::moveRobotCloser(double stepSize)
{
	//No se necesita esta mierda pero es para que la clase no quede abstracta
	return true;
}

ShipState* ShipState::clone()
{
	return new ShipState(*this);
}

void ShipState::drawGL()
{
	glDisable(GL_LIGHTING);
	glPointSize(2);
	glColor3f(1, 1, 0);
	glBegin(GL_POINTS);
	glVertex3f(_pose.x, _pose.y, 0);
	glEnd();
	glColor3f(0, 0, 1);
}


