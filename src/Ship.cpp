#include "Ship.h"
namespace mr
{
	IMPLEMENT_MR_OBJECT(Ship)
	Ship::Ship(double w, double l, double m):
		_width(w),_length(l),_mass(m)
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
		// CONTINUA AQUI
	}

}