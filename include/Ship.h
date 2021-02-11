#ifndef __SHIP_H_
#define __SHIP_H_

#include <iostream>
#include "gl/gltools.h"

#include "base\globject.h"
#include "base/logger.h"
#include "C:\mrcore\include\sim\composedentity.h"
#include "C:\mrcore\include\math\transformation2d.h"

#include "C:\mrcore\include\sim\world.h"
#include "C:\mrcore\include\sim\cylindricalpart.h"

namespace mr
{
	class Ship :public ComposedEntity
	{
		DECLARE_MR_OBJECT(Ship)
	public:
		//Serializers
		virtual void writeToStream(Stream& stream);
		virtual void readFromStream(Stream& stream);
		virtual void writeToXML(XMLElement* parent);
		virtual void readFromXML(XMLElement* parent);
		virtual char* CreateXMLText();
		virtual void loadFromXMLText(char* XmlText);

		// Constructor
		Ship(double w=0.5, double l=5.0, double m=100.0){}

		virtual bool move(double t, double y);
		virtual bool getPose3D(Pose3D& pose) { pose = getAbsoluteT3D(); return true; }
		bool computeGroundedLocation(Transformation3D& p, World* w = 0);
		virtual void setLocation(const Transformation3D& p)
		{
			setAbsoluteT3D(p);
		}
		virtual void simulate(double delta_t);//time inteval in seconds
		virtual void drawGL();
		bool getMoveSuccess() { return _move_success; }
		// Setters & Getters

		// Destructor
		virtual ~Ship(){}
	private:

		double _width;
		double _length;
		double _mass;
		double _I;

		// Inputs
		double _thrust;
		double _yaw;

		//utility bool that checks if last move was possible (not collision, not off ground)
		bool _move_success;

		
		void computeInertia();


	};
}

#endif