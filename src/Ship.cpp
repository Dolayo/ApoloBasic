#include "Ship.h"
#include <math.h>

namespace mr
{
	Ship::Ship(double w = 0.5, double l = 2, double m = 100, double c = 0.5) :
		_width(w),
		_length(l),
		_mass(m),
		_J(0),
		_trueFlowDirection(0),
		_u(0),
		_v(0),
		_w(0),
		_x(0),
		_y(0),
		_yaw(0),
		_thrust_x(0),
		_thrust_y(0),
		_move_success(true)

	{

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
		hull->setRelativeOrientation(PI / 2, 0, PI / 2);
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
		bridge->setRelativePosition(Vector3D(_length / 15, 0, _width));
		bridge->setRelativeOrientation(0, 0, PI / 2);
		bridge->setColor(0.4, 0.4, 0.4);

		(*this) += bridge;
	}



	void Ship::simulate(double delta_t)
	{
		Vector2D F_air_drag;
		Vector2D F_water_drag;
		Vector2D F_drag_propulsion(_thrust_x, 0);

		Vector2D F_air_side;
		Vector2D F_water_side;
		Vector2D F_side_propulsion(0, _thrust_y);

		double M_air;
		double M_water;
		double M_rot;
		double M_prop = _thrust_y*_xp;


		double ax = (1 / _mass) * (F_air_drag + F_water_drag + F_drag_propulsion);
		double ay = (1 / _mass) * ();
		double aw = (1 / _J) * ();

		// MRUA
		double delta_x = _x + _u * delta_t + 0.5 * ax * delta_t * delta_t;
		double delta_y = _y + _u * delta_t + 0.5 * ay * delta_t * delta_t;

		// MCUA
		double delta_th = _yaw + _w * delta_t + 0.5 * aw * delta_t * delta_t;;
		_move_success = false;

		Transformation3D position = getAbsoluteT3D();
		Transformation3D delta(delta_x * cos(delta_th), delta_x * sin(delta_th), 0, 0, 0, delta_th);

		Transformation3D newposition = position * delta;
		//_speed = acc * delta_t;
		//_rot_speed = rot_acc * delta_t;

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

	bool Ship::move(double x, double y)
	{
		_thrust_x = x;
		_thrust_y = y;
		if (_thrust_x > 10.0)_thrust_x = 10.0;
		if (_thrust_x < 0.0)_thrust_x = 0.0;
		if (_thrust_y > 10.0)_thrust_y = 10;
		if (_thrust_y < 0.0)_thrust_y = 0.0;

		return true;
	}

	double Ship::coeff(const double& x, const char& fluid, const char& type)
	{
		// x is apparent flow angle in radians [-PI, PI]
		// fluid is either air(a) or water(w)
		// type is either drag(d), side(s), or yaw(y)

		if (fluid == 'a')
		{
			switch (type)
			{
			case 'd':
				//drag
				return (0.195738 + 0.518615 * std::abs(x) - 0.496029 * x * x +
					0.0941925 * std::abs(x) * std::abs(x) * std::abs(x) +
					1.86427 * sin(2 * PI * pow(std::abs(x) / PI, 1.05)) *
					exp(-2.17281 * (std::abs(x) - PI / 2) * (std::abs(x) - PI / 2)));
				break;

			case 's':
				// side
				if (x == PI / 2)
				{
					return 0.904313;
					break;
				}
				else
				{
					double sgn = (x > 0) ? 1 : -1;
					return (sgn * (12.3722 - 15.453 * std::abs(x) + 6.0261 * std::abs(x) -
						0.532325 * std::abs(x) * std::abs(x) * std::abs(x))
						* sin(std::abs(x)) *
						exp(-1.68668 * (std::abs(x) - PI / 2) * (std::abs(x) - PI / 2)));
					break;
				}


			case 'y':
				double sgn = (x > 0) ? 1 : -1;
				return (sgn * (0.710204 - 0.297196 * std::abs(x) +
					0.0857296 * std::abs(x) * std::abs(x)) *
					sin(2 * PI * pow(std::abs(x) / PI, 1.05)));
				break;

			default:
				// wrong type input
				return -1;
				break;
			}
		}
		else
		{
			if (fluid == 'w')
			{
				switch (type)
				{
				case 'd':
					return (0.245219 - 0.93044 * std::abs(x) + 0.745752 * std::abs(x) *
						std::abs(x) - 0.15915 * std::abs(x) * std::abs(x) *
						std::abs(x) + 2.79188 * sin(2 * std::abs(x)) *
						exp(-1.05667 * (std::abs(x) - PI / 2) * (std::abs(x) - PI / 2)));
					break;

				case 's':
					double sgn = (x > 0) ? 1 : -1;
					return (sgn * (0.115554 + 3.09423 * std::abs(x) - 0.984923 * std::abs(x) *
						std::abs(x)) * sin(std::abs(x)));
					break;

				case 'y':
					if (x == PI / 2)
					{
						return 2.545759;
						break;
					}
					double sgn = (x > 0) ? 1 : -1;
					return (sgn * (0.322986 + 0.317964 * std::abs(x) - 0.1021844 * std::abs(x)
						* std::abs(x)) * sin(2 * std::abs(x)));
					break;

				default:
					// wrong type input
					return -1;
					break;
				}
			}
			else
			{
				// wrong fluid input
				return -2;
			}
		}
	}

	double Ship::sdl(const double& x, const char& fluid, const char& type, const char& type2)
	{
		// Square Drag Law
		// x is apparent flow angle in radians [-PI, PI]
		// fluid is either air(a) or water(w)
		// type is eitcher force(f) or momentum(m)
		// type2 is either drag(d), side(s), or yaw(y)

		if (fluid == 'a')
		{
			double vr = sqrt(pow(_windSpeed * cos(_trueWindDirection - _yaw + PI / 2) - _u, 2)
			+ pow(_windSpeed * sin(_trueWindDirection - _yaw + PI / 2) - _v, 2));
			if (type == 'f')
			{
				return (0.5 * coeff(x, fluid, type2) * 15 * 1.228 * vr * vr);
			}
			else
			{
				if (type == 'm')
				{
					return (0.5 * coeff(x, fluid, type2) * 1 * 15 * 1.228 * vr * vr);
				}
				else
				{
					// wrong type input
					return -1;
				}
			}

		}
		else
		{
			if (fluid == 'w')
			{
				double vr = sqrt(pow(_waterSpeed * cos(_trueWaterDirection - _yaw + PI / 2) - _u, 2)
					+ pow(_waterSpeed * sin(_trueWaterDirection - _yaw + PI / 2) - _v, 2));
				if (type == 'f')
				{
					return (0.5 * coeff(x, fluid, type2) * 15 * 1.228 * vr * vr);
				}
				else
				{
					if (type == 'm')
					{
						return (0.5 * coeff(x, fluid, type2) * 1 * 15 * 1.228 * vr * vr);
					}
					else
					{
						// wrong type input
						return -1;
					}
				}
			}
			else
			{
				// wrong fluid input
				return -2;
			}
		}

	}


}
