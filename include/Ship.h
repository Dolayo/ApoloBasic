#ifndef __SHIP_H_
#define __SHIP_H_

#include <mrcore.h>


namespace mr
{
	class Ship :public ComposedEntity
	{
	public:
		
		// Constructor
		Ship(double x=0.0, double y=0.0, double u=0.0, double v=0.0, double w=0.0);
		//Ship& operator =(const Ship& s) = delete;
		//Ship(Ship&) = delete;

		//copy constructor and operador = 
		const Ship& operator =(const Ship& s)
		{
			_width = s._width;
			_length = s._length;
			_mass = s._mass;
			_J = s._J;
			_xp = s._xp;
			_Sair = s._Sair;
			_Swater = s._Swater;
			_vMax = s._vMax;
			_aMax = s._aMax;
			_Marm = s._Marm;
			_trueWindDirection = s._trueWindDirection;
			_trueWaterDirection = s._trueWaterDirection;
			_windSpeed = s._windSpeed;
			_waterSpeed = _waterSpeed;
			_Crs_water = s._Crs_water;
			_Crs_air = s._Crs_air;
			_ro_water = s._ro_water;
			_ro_air = s._ro_air;
			_u = s._u;
			_v = s._v;
			_w = s._w;
			_x = s._x;
			_y = s._y;
			_yaw = s._yaw;
			_thrust_x = s._thrust_x;
			_thrust_y = s._thrust_y;
			_Wind_Force_Drag = s._Wind_Force_Drag;
			_Water_Force_Drag = s._Water_Force_Drag;
			_Wind_Force_Side = s._Wind_Force_Side;
			_Water_Force_Side = s._Water_Force_Side;
			_Wind_Moment = s._Wind_Moment;
			_Water_Moment = s._Water_Moment;
			_Rotational_Moment = s._Rotational_Moment;
			_move_success = s._move_success;

			_alpha_water = _Crs_water * _Swater * _ro_water / (_length / 2);
			_alpha_air = _Crs_air * _Sair * _ro_air / (_length / 2);
			return *this;
		}
		Ship(const Ship& s) { (*this) = s; }

		void setState(double x, double y, double yaw, double u, double v, double w);
		Vector3D simpleAccs();
		bool simpleDynamicsSim(double delta_t);
		virtual bool setThrusts(double t = 0.0, double r = 0.0, double w = 0.0);
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
		virtual void drawGL();
		bool getMoveSuccess() { return _move_success; }

		// Setters & Getters

		void setX(const double& x) { _x = x; }
		const double getX() { return _x; }
		void setY(const double& y) { _y = y; }
		const double getY() { return _y; }
		void setYaw(const double& yaw) { _yaw = yaw; }
		const double getYaw() { return _yaw; }
		void setPos(Vector3D V)
		{
			_x = V.x;
			_y = V.y;
			_yaw = V.z;
		}
		Vector3D getPos() { return Vector3D(_x, _y, _yaw); }

		void setU(const double& u) { _u = u; }
		const double getU() { return _u; }
		void setV(const double& v) { _v = v; }
		const double getV() { return _v; }
		void setW(const double& w) { _w = w; }
		const double getW() { return _w; }

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
		double _thrust_w;//Just for simple

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

	};
}

#endif