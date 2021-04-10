#include "ShipState.h"

//Eventually this will go in its own file of macros
constexpr auto POSE_TOL = 1.0;
constexpr auto VEL_TOL = 1.0;

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

//For now, we will use just the differences between vectors of speed and position

double ShipState::distanceTo(RobotState* p)
{

	ShipState* naux = dynamic_cast<ShipState*>(p);
	if (!naux)return 10000.0;

	Vector3D dif_pos = naux->_pose - _pose;
	Vector3D dif_vel = naux->_vel - _vel;
	
	/*std::map<string, double> dist;
	dist.insert(std::make_pair<string, double>("pos", dif_pos.module()));
	dist.insert(std::make_pair<string, double>("vel", dif_vel.module()));*/

	return dif_pos.module();
}

bool ShipState::isEqualToCurrentRobotState()
{
	//Transformation3D t=robot->getAbsoluteT3D(); 
	//return
	Transformation3D t = _ship->getAbsoluteT3D();

	Vector3D dif_pose = t.position - _pose;
	Vector3D dif_vel = _ship->getVels() - _vel;
	if ((dif_pose.module() < POSE_TOL) && (dif_vel.module() < VEL_TOL))
		return true;
	else return false;

}

RobotState* ShipState::createStateFromCurrentRobotState()
{
	if (!_world)return 0;
	if (!_ship)return 0;
	vector<double> v(7);
	Transformation3D t = _ship->getAbsoluteT3D();
	double r, p, y;

	v[0] = t.position.x;
	v[1] = t.position.y;
	// Warning: Z is included
	v[2] = t.position.z;

	t.orientation.getRPY(r, p, y);
	v[3] = y;

	v[4] = _ship->getU();
	v[5] = _ship->getV();
	v[6] = _ship->getW();
	ShipState* aux = dynamic_cast<ShipState*>(createStateFromSample(v));
	return aux;
}

RobotState* ShipState::createStateFromPoint3D(double x, double y, double z, double yaw, double Vx, double Vy, double W)
{
	vector<double> aux(7);
	aux[0] = x; aux[1] = y; aux[2] = z; aux[3] = yaw;
	aux[4] = Vx; aux[5] = Vy; aux[6] = W;
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
		[6]: W
	*/

	//valid conditions: there is a robt and a world defined
	if (!_world)return 0;
	if (!_ship)return 0;
	if (values.size() < 7)return 0;

	Ship* s = _ship;
	ShipState aux(s, _world);
	aux._pose = Vector3D(values[0], values[1], values[2]);//x,y,z
	aux._vel = Vector3D(values[4], values[5], values[6]);//Vx,Vy,Vz

	return new ShipState(aux);
}

vector<double> ShipState::getSample()
{
	return vector<double>{_pose.x, _pose.y, _pose.z, _vel.x, _vel.y, _vel.z/*W*/};
}

bool ShipState::propagate(std::vector<double> v_auxCtrlAct, double delta_t, ShipState* p_retState)
{
	bool b_success;

	_ship->setThrusts(v_auxCtrlAct[0], v_auxCtrlAct[1]);

	b_success = _ship->dynamicsSim(delta_t);

	if(dynamic_cast<ShipState*>(createStateFromCurrentRobotState()));
		p_retState =  dynamic_cast<ShipState*>(createStateFromCurrentRobotState());
	
	return b_success;
}

bool ShipState::moveRobotCloser(double stepSize)
{
	Transformation3D t = _ship->getAbsoluteT3D();
	Vector3D ux = t.orientation.getVectorU();
	Vector3D dif = _pose - t.position;

	//if the robot is in this node... doesn´t move
	if (isEqualToCurrentRobotState())return false;
}

void ShipState::placeRobot()
{
	Transformation3D t(_pose.x, _pose.y, _pose.z, Z_AXIS, _yaw);
	_ship->setRelativeT3D(t);
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

RobotState* ShipState::clone()
{
	return new ShipState(*this);
}

void ShipState::drawGL()
{
	glDisable(GL_LIGHTING);
	glPointSize(2);
	glColor3f(1, 1, 0);
	glBegin(GL_POINTS);
	glVertex3f(_pose.x, _pose.y, _pose.z);
	glEnd();
	glColor3f(0, 0, 1);
}


