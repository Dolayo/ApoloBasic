#include "ShipState.h"
#include "defines.h"


bool ShipState::isEqual(RobotState* n)
{
	//Equal if compatible
	// For now, compatible if less than a constant
	ShipState* naux = dynamic_cast<ShipState*>(n);

	if (!naux)
		return false;

	if (_ship == nullptr)
		return false;

	Vector3D dif_pose = naux->_pose - _pose;

	Vector3D aux(dif_pose[0], dif_pose[1], 0.0);

	double dif_yaw = dif_pose[2];

	Vector3D dif_vel = naux->_vel - _vel;


	if ((aux.module() < POSE_TOL) && std::abs(dif_yaw) < YAW_TOL /*&& (dif_vel.module() < VEL_TOL)*/)
		return true;

	else return false;
}

bool ShipState::isSamePos(RobotState* n, double pos_tol)
{
	//Equal if compatible
	// For now, compatible if less than a constant
	ShipState* naux = dynamic_cast<ShipState*>(n);
	if (!naux)return false;
	if (_ship == 0)return false;

	Vector3D dif_pose = naux->_pose - _pose;
	dif_pose.z = 0.0;
	if ((dif_pose.module() < pos_tol))
		return true;
	else return false;
}

bool ShipState::isSamePos(Vector3D p)
{
	//Equal if compatible
	// For now, compatible if less than a constant

	Vector3D dif_pose = p - _pose;
	dif_pose.z = 0.0;
	if ((dif_pose.module() < POSE_TOL))
		return true;
	else return false;
}



//For now, we will use just the differences between vectors of speed and position

double ShipState::distanceTo(RobotState* p)
{

	ShipState* naux = dynamic_cast<ShipState*>(p);
	//if (!naux)return 10000.0;

	Vector3D thisPos = this->getGhostPos();
	Vector3D nauxPos = naux->getGhostPos();
	Vector3D dif_pos = nauxPos - thisPos;
	
	double val = dif_pos.module();

	Segment3D segm = Segment3D(Vector3D(_pose.x, _pose.y, 1.0), Vector3D(naux->_pose.x, naux->_pose.y, 1.0));

	_ship->setIntersectable(false);

	//penalties the intersection
	if (_world->segmentIntersection(segm, 0))
		val *= 2.0;

	_ship->setIntersectable(true);

	return val;
}

double ShipState::distanceTo(Vector3D vec_pos)
{

	Vector3D thisPos = this->getGhostPos();

	Vector3D dif_pos = vec_pos - thisPos;

	double val = dif_pos.module();

	Segment3D segm = Segment3D(Vector3D(_pose.x, _pose.y, 1.0), Vector3D(vec_pos.x, vec_pos.y, 1.0));

	_ship->setIntersectable(false);

	//penalties the intersection
	if (_world->segmentIntersection(segm, 0))
		val *= 10;

	_ship->setIntersectable(true);

	return val;
}

bool ShipState::IsVisible(Vector2D vec_pos)
{
	bool b_ret = true;

	Segment3D segm = Segment3D(Vector3D(_pose.x, _pose.y, 1.0), Vector3D(vec_pos.x, vec_pos.y, 1.0));

	_ship->setIntersectable(false);

	b_ret = !(_world->segmentIntersection(segm, 0));

	_ship->setIntersectable(true);

	return b_ret;
}

double ShipState::SimpleDistance(RobotState* p)
{
	ShipState* naux = dynamic_cast<ShipState*>(p);
	//if (!naux)return 10000.0;
	Vector3D thisPos(_pose.x, _pose.y, 0.0);
	Vector3D nauxPos(naux->_pose.x, naux->_pose.y, 0.0);
	Vector3D dif_pos = nauxPos - thisPos;

	return dif_pos.module();
}

Vector3D ShipState::getGhostPos()
{
	Vector3D accs = getAccs();
	Vector3D vels = getVels();
	Vector3D pos = getPose();

	double vx = vels.x;
	double vy = vels.y;

	vx = std::abs(vx);
	vy = std::abs(vy);

	double t_stop = 0.0;

	if ((vx != 0.0) && (vy != 0.0) && (accs.x != 0.0) && (accs.y != 0.0))
		t_stop = (-1.0 * vx * (accs.x / accs.y) - vy) / (accs.y * (1 + (accs.x * accs.x) / (accs.y * accs.y)));

	else
		if (vx != 0.0 && accs.x != 0.0)
			t_stop = vx / std::abs(accs.x);
		else
			if ((vy != 0.0) && (accs.y != 0.0))
				t_stop = vy / accs.y;

	// Transformar las velocidades relativas a absolutas
	double vabs_x = getVels().x * cos(getYaw());
	double vabs_y = getVels().x * sin(getYaw());

	double roz = accs.x;//el rozamiento es el mismo para la x que para la y, de momento

	double aabs_x = roz * cos(getYaw());
	double aabs_y = roz * sin(getYaw());

	// MRUA
	double new_pos_x = pos.x + vabs_x * t_stop + 0.5 * aabs_x * t_stop * t_stop;
	double new_pos_y = pos.y + vabs_y * t_stop + 0.5 * aabs_y * t_stop * t_stop;

	return Vector3D(new_pos_x, new_pos_y, 0.0);
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
		[2]: yaw
	*/

	//valid conditions: there is a robt and a world defined
	if (!_world)return 0;
	if (!_ship)return 0;
	if (values.size() < 3)return 0;

	Ship* s = _ship;
	ShipState aux(s, _world);
	aux._pose = Vector3D(values[0], values[1], values[2]);//x,y,yaw
	aux._yaw = values[2];
	aux.setVels(_ship->getVels());

	return new ShipState(aux);
}

vector<double> ShipState::getSample()
{
	return vector<double>{_pose.x, _pose.y, _pose.z, _vel.x, _vel.y, _vel.z/*W*/};
}

bool ShipState::propagate(std::vector<double> v_auxCtrlAct, double delta_t, ShipState** p_retState, bool b_use_collision)
{
	// Booleano para comprobar que no se ha chocado con nada
	bool b_success;

	this->placeRobot();

	// Establecemos la accion de control a emplear
	this->_ship->setThrusts(v_auxCtrlAct[0], v_auxCtrlAct[1], v_auxCtrlAct[2]);

	//Simulamos al robot durante delta_t segundos con la accion de control v_auxCtrlAct
	b_success = this->_ship->simpleDynamicsSim(delta_t, b_use_collision);

	// Creamos el nuevo estado a partir de como queda el robot despues de aplicar la accion de control
	RobotState* aux_state = createStateFromCurrentRobotState();

	//Dejamos al robot del estado como estaba antes
	/*this->_ship->setPos(this->_pose);
	this->_ship->setVels(this->_vel);*/

	if (dynamic_cast<ShipState*>(aux_state))
		*p_retState = dynamic_cast<ShipState*>(aux_state);
	else b_success = false;
	
	return b_success;
}

void ShipState::placeRobot()
{
	Transformation3D t(_pose.x, _pose.y, 0, Z_AXIS, _pose.z);
	_ship->setRelativeT3D(t);
	_ship->setPos(Vector3D(_pose.x, _pose.y, _pose.z));
	_ship->setVels(Vector3D(_vel.x, _vel.y, _vel.z));
}

void ShipState::placeRobotTowards(RobotState* target)
{
	ShipState* naux = dynamic_cast<ShipState*>(target);
	if (!naux)return;
	Vector3D dif = naux->_pose - _pose;
	double angle = atan2(dif.y, dif.x);

	Transformation3D t(_pose.x, _pose.y, _pose.z, Z_AXIS, angle);
	_ship->setRelativeT3D(t);
	_ship->setYaw(angle);
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
	glPointSize(5);
	glColor3f(1, 1, 0);
	glBegin(GL_POINTS);
	glVertex3f(_pose.x, _pose.y, 0);
	glEnd();
	glColor3f(0, 0, 1);
}


