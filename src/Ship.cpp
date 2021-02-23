#include "Ship.h"
namespace mr
{
	IMPLEMENT_MR_OBJECT(Ship)
	Ship::Ship(double w=0.5, double l=2, double m=100, double c=0.5):
		_width(w),_length(l),_mass(m), _coeff(c), _speed(0), _rot_speed(0)
	{
		computeInertia();
	}

	void Ship::computeInertia()
	{
		// De momento lo aproximamos por un cilindro
		_I = 0.25 * _mass * (_width / 2) * (_width / 2) + (1 / 12) * _mass * (_length / 2) * (_length / 2);
	}

	//serializers
	void Ship::writeToStream(Stream& stream)
	{
		SolidEntity::writeToStream(stream);
		stream << _width << _length << _I;
		stream << _thrust << _yaw;
	}

	void Ship::readFromStream(Stream& stream)
	{
		SolidEntity::readFromStream(stream);
		stream >> _width >> _length >> _I;
		stream >> _thrust >> _yaw;
		//specific initializations
	}

	void Ship::writeToXML(XMLElement* parent)
	{
		/*SolidEntity::writeToXML(parent);
		if (_thrust)
		{
			XMLVariable* _thrust = new XMLVariable("_thrust", XMLAux::string_Convert<double>(_thrust).c_str());
			parent->AddVariable(_thrust);
		}
		if (_yaw)
		{
			XMLVariable* _rotSpeed = new XMLVariable("_yaw", XMLAux::string_Convert<double>(_yaw).c_str());
			parent->AddVariable(_rotSpeed);
		}*/ //Me da error de plantilla
	}

	void Ship::readFromXML(XMLElement* parent)
	{
		SolidEntity::readFromXML(parent);

		if (parent->FindVariableZ("_thrust"))
		{
			_thrust = XMLAux::GetValueDouble(parent->FindVariableZ("_thrust"));

		}
		if (parent->FindVariableZ("_yaw"))
		{
			_yaw = XMLAux::GetValueDouble(parent->FindVariableZ("_yaw"));

		}
	}

	char* Ship::CreateXMLText()
	{
		XMLElement* elem = new XMLElement(0, "Ship");
		writeToXML(elem);
		return elem->CreateXMLText();
	}

	void Ship::loadFromXMLText(char* XmlText)
	{
		XML x;
		readFromXML(x.Paste(XmlText));
	}

	void Ship::simulate(double delta_t)
	{
		if (!_mass || !_I) return;

		double acc = (_thrust / _mass) - _coeff * _mass * 9.18;
		//Pensar un metodo para que la viscosidad solo se aplique cuando la 
		//velocidad sea distina de cero

		double tor = 0.1*_thrust * _yaw;//Factor aun por determinar
		double rot_acc = tor / _I;

		double delta_x = _speed * delta_t + 0.5 * acc * delta_t * delta_t;

		double delta_th = _rot_speed * delta_t + 0.5 * rot_acc * delta_t * delta_t;;
		_move_success = false;
		
		Transformation3D position = getAbsoluteT3D();
		Transformation3D delta(delta_x * cos(delta_th), delta_x * sin(delta_th), 0, 0, 0, delta_th);
		
		Transformation3D newposition = position * delta;
		_speed = acc * delta_t;
		_rot_speed = rot_acc * delta_t;

		if (computeGroundedLocation(newposition) == false)return;
		
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

	bool Ship::move(double t, double y)
	{
		_thrust = t;
		_yaw = y;


		return true;
	}

	void Ship::drawGL()
	{
		vector<Vector2D> list_bod;
		PrismaticPart hull;

		list_bod.push_back(Vector2D(_width / 2, _width / 2));
		list_bod.push_back(Vector2D(_width / 4, -_width / 2));
		list_bod.push_back(Vector2D(-_width / 4, -_width / 2));
		list_bod.push_back(Vector2D(-_width / 2, _width / 2));
		hull.setPolygonalBase(list_bod);
		hull.setHeight(_length);
		hull.setRelativePosition(Vector3D(0, 0, _width / 2));
		hull.setColor(1, 0, 0);

		vector<Vector2D> list_head;
		Face tip;
		PrismaticPart bow;

		list_head.push_back(Vector2D(_width / 2, _width / 2));
		list_head.push_back(Vector2D(_width / 4, -_width / 2));
		list_head.push_back(Vector2D(-_width / 4, -_width / 2));
		list_head.push_back(Vector2D(-_width / 2, _width / 2));

		tip.addVertex(_width / 10, _width / 2);
		tip.addVertex(_width / 10, -((_width / 2) - (_width / 10)));
		tip.addVertex(-_width / 10, -((_width / 2) - (_width / 10)));
		tip.addVertex(-_width / 10, _width / 2);

		bow.setPolygonalBase(list_bod);
		bow.setPolygonalBase(tip);
		bow.setHeight(_length/3);
		bow.setRelativePosition(Vector3D(0, 0, _width / 2));
		bow.setColor(1, 0, 0);

		vector<Vector2D> bridge_list;
		PrismaticPart bridge;

		bridge_list.push_back(Vector2D(_width / 2, _width / 2));
		bridge_list.push_back(Vector2D(_width / 4, -_width / 2));
		bridge_list.push_back(Vector2D(-_width / 4, -_width / 2));
		bridge_list.push_back(Vector2D(-_width / 2, _width / 2));
		bridge.setPolygonalBase(bridge_list);
		bridge.setHeight(_width);
		bridge.setRelativePosition(Vector3D(-_length/2.2, 0, _width));
		bridge.setColor(1, 0, 0);

		glPushMatrix();
		getAbsoluteT3D().transformGL();

		hull.drawGL();
		bow.drawGL();
		bridge.drawGL();

		glPopMatrix();
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

	bool Ship::computeGroundedLocation(Transformation3D& p, World* w)
	{

	}

	bool Ship::dropShip(Transformation3D& t, World* w)
	{

	}
}