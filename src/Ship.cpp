#include "Ship.h"
namespace mr
{

		Ship::Ship(double w = 0.5, double l = 2, double m = 100, double c = 0.5) :
		_width(w), _length(l), _mass(m), _coeff(c), _speed(0), _rot_speed(0)
	{
		computeInertia();

		// Pintar el barquito
		vector<Vector2D> list_bod;
		PrismaticPart* hull = new PrismaticPart;

		list_bod.push_back(Vector2D(_width / 2, _width / 2));
		list_bod.push_back(Vector2D(_width / 4, -_width / 2));
		list_bod.push_back(Vector2D(-_width / 4, -_width / 2));
		list_bod.push_back(Vector2D(-_width / 2, _width / 2));
		hull->setPolygonalBase(list_bod);
		hull->setHeight(_length);
		hull->setRelativePosition(Vector3D(0, 0, _width / 2 + 0.001));
		hull->setRelativeOrientation(PI/2,0,0);
		hull->setColor(0.4, 0.4, 0.4);

		(*this) += hull;
		/*
		vector<Vector2D> list_head;
		Face tip;
		PrismaticPart* bow = new PrismaticPart;

		list_head.push_back(Vector2D(_width / 2, _width / 2));
		list_head.push_back(Vector2D(_width / 4, -_width / 2));
		list_head.push_back(Vector2D(-_width / 4, -_width / 2));
		list_head.push_back(Vector2D(-_width / 2, _width / 2));

		tip.addVertex(_width / 10, _width / 2);
		tip.addVertex(_width / 10, -((_width / 2) - (_width / 10)));
		tip.addVertex(-_width / 10, -((_width / 2) - (_width / 10)));
		tip.addVertex(-_width / 10, _width / 2);

		bow->setPolygonalBase(list_bod);
		bow->setPolygonalTop(tip);
		bow->setHeight(_length / 3);
		bow->setRelativePosition(Vector3D(0, 0, _width / 2));
		bow->setColor(0, 0, 1);

		(*this) += bow;
		*/
		vector<Vector2D> bridge_list;
		PrismaticPart* bridge = new PrismaticPart;

		bridge_list.push_back(Vector2D(_width / 2, _width / 2));
		bridge_list.push_back(Vector2D(_width / 4, -_width / 2));
		bridge_list.push_back(Vector2D(-_width / 4, -_width / 2));
		bridge_list.push_back(Vector2D(-_width / 2, _width / 2));
		bridge->setPolygonalBase(bridge_list);
		bridge->setHeight(_width);
		bridge->setRelativePosition(Vector3D(0, -_length/10, _width ));
		bridge->setRelativeOrientation(0, 0, 0);
		bridge->setColor(0.4, 0.4, 0.4);

		(*this) += bridge;
	}

	void Ship::computeInertia()
	{
		// De momento lo aproximamos por un cilindro
		_I = 0.25 * _mass * (_width / 2) * (_width / 2) + (1 / 12) * _mass * (_length / 2) * (_length / 2);
	}



	void Ship::simulate(double delta_t)
	{
		if (!_mass || !_I) return;

		double acc = (_thrust / _mass) - _coeff * _mass * 9.18;
		//Pensar un metodo para que la viscosidad solo se aplique cuando la 
		//velocidad sea distina de cero

		double tor = 0.1 * _thrust * _yaw;//Factor aun por determinar
		double rot_acc = tor / _I;

		double delta_x = _speed * delta_t + 0.5 * acc * delta_t * delta_t;

		double delta_th = _rot_speed * delta_t + 0.5 * rot_acc * delta_t * delta_t;;
		_move_success = false;

		Transformation3D position = getAbsoluteT3D();
		Transformation3D delta(delta_x * cos(delta_th), delta_x * sin(delta_th), 0, 0, 0, delta_th);

		Transformation3D newposition = position * delta;
		_speed = acc * delta_t;
		_rot_speed = rot_acc * delta_t;

		setAbsoluteT3D(newposition);

		World* world = getWorld();
		if (world) {
			if (world->checkCollisionWith(*this)) {
				setAbsoluteT3D(position); //no muevo el robot
				return;
			}
		}

		_move_success = true; //se pudo realizar el movimiento
	}

	void Ship::drawGL()
	{

		ComposedEntity::drawGL();
		return;
	}

	bool Ship::move(double t, double y)
	{
		_thrust = t;
		_yaw = y;
		if (_thrust > 5.0)_thrust = 5.0;
		if (_thrust < 0.0)_thrust = 0.0;
		if (_yaw > 45.0)_yaw = 45.0;
		if (_yaw < -45.0)_yaw = -45.0;

		return true;
	}

}
