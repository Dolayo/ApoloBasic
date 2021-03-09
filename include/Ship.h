#ifndef __SHIP_H_
#define __SHIP_H_

#include <mrcore.h>


namespace mr
{
	class Ship :public ComposedEntity
	{
	public:
		
		// Constructor
		Ship(double w, double l, double m, double c);
		
		virtual bool move(double t, double r);
		double coeff(const double& x, const char& fluid, const char& type);
		double sdl(const char& fluid, const char& type, const char& type2);
		double Mfront(const char& fluid);
		double Mback(const char& fluid);
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
	protected:

		// --- Ship attributes ---

		double _width;
		double _length;
		double _mass;
		double _J;
		double _xp;
		double _Sair;
		double _Swater;
		double _vMax;
		double _Marm;

		// --- Environment ---

		double _trueWindDirection;
		double _trueWaterDirection;
		
		double _windSpeed;
		double _waterSpeed;
		double _alpha;

		double _alpha_water;
		double _alpha_air;
		
		double _Crs_water;
		double _Crs_air;
		
		double _ro_water;
		double _ro_air;

		// --- State ---

		// Velocities
		double _u;
		double _v;
		double _w;

		// Position
		double _x;
		double _y;

		// Orientation
		double _yaw;

		// Inputs
		double _thrust_x;
		double _thrust_y;
	

		//utility bool that checks if last move was possible (not collision, not off ground)
		bool _move_success;

		// Default Constructor and operators

	private:

		Ship();
		Ship& operator =(const Ship& s);
		Ship(Ship&);



	};
}

#endif