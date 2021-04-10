#ifndef __SHIP_H_
#define __SHIP_H_

#include <mrcore.h>


namespace mr
{
	class Ship :public ComposedEntity
	{
	public:
		
		// Constructor
		Ship();
		void setState(double x, double y, double yaw, double u, double v, double w);
		virtual bool setThrusts(double t, double r);
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
		virtual bool dynamicsSim(double delta_t);//time inteval in seconds
		virtual void simulate(double delta_t);
		virtual void drawGL();
		bool getMoveSuccess() { return _move_success; }

		// Setters & Getters

		void setX(const double& x) { _x = x; }
		double getX() { return _x; } const
			void setY(const double& y) { _y = y; }
		double getY() { return _y; } const
			void setYaw(const double& yaw) { _yaw = yaw; }
		double getYaw() { return _yaw; } const
		void setPos(Vector3D V)
		{
			_x = V.x;
			_y = V.y;
			_yaw = V.z;
		}
		Vector3D getPos() { return Vector3D(_x, _y, _yaw); }

		void setU(const double& u) { _u = u; }
		double getU() { return _u; } const
		void setV(const double& v) { _v = v; }
		double getV() { return _v; } const
		void setW(const double& w) { _w = w; }
		double getW() { return _w; } const

		void setVels(Vector3D V)
		{
			_u = V.x;
			_v = V.y;
			_w = V.z;
		}
		Vector3D getVels() { return Vector3D(_u, _v, _w); }

		const double getWind_Force_Drag() { return _Wind_Force_Drag; }
		const double getWater_Force_Drag() { return _Water_Force_Drag; }
		const double getWind_Force_Side() { return _Wind_Force_Side; }
		const double getWater_Force_Side() { return _Water_Force_Side; }
		const double getWind_Moment() { return _Wind_Moment; }
		const double getWater_Moment() { return _Water_Moment; }
		const double getRotational_Moment() { return _Rotational_Moment; }

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
		double _aMax;
		double _Marm;

		// --- Environment ---

		double _trueWindDirection;
		double _trueWaterDirection;
		
		double _windSpeed;
		double _waterSpeed;

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

		// Store the actions to show 
		double _Wind_Force_Drag;
		double _Water_Force_Drag;
		double _Wind_Force_Side;
		double _Water_Force_Side;
		double _Wind_Moment;
		double _Water_Moment;
		double _Rotational_Moment;
	

		//utility bool that checks if last move was possible (not collision, not off ground)
		bool _move_success;

		// Default Constructor and operators

	private:

		Ship& operator =(const Ship& s);
		Ship(Ship&);



	};
}

#endif