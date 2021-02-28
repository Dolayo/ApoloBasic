#ifndef __SHIP_H_
#define __SHIP_H_
/*
#include "C:\mrcore\include\math\transformation2d.h"

#include "C:\mrcore\include\sim\world.h"
#include "C:\mrcore\include\sim\cylindricalpart.h"
#include "C:\mrcore\include\sim\composedentity.h"
*/
#include <mrcore.h>


namespace mr
{
	class Ship :public ComposedEntity
	{
	public:
		
		// Constructor
		Ship(double w, double l, double m, double c);
		

		virtual bool move(double t, double y);
		virtual bool getPose3D(Pose3D& pose) { pose = getAbsoluteT3D(); return true; }
	
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
		double _coeff;
		double _speed;
		double _rot_speed;

		// State
		double _thrust;
		double _yaw;//cambiar por timon

		//utility bool that checks if last move was possible (not collision, not off ground)
		bool _move_success;

		
		void computeInertia();

		// Default Constructor and operators
		Ship();
		Ship& operator =(const Ship& s);
		Ship(Ship&);



	};
}

#endif