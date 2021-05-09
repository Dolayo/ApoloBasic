#ifndef __SHIP_STATE_H_
#define __SHIP_STATE_H_

#include <mrcore.h>
#include "Ship.h"

namespace mr 
{
	class ShipState : public RobotState
	{
		public:
			ShipState(Ship* s, World* w)
			{
				_ship = s;
				_world = w;
			}
			//copy constructor and operador = 
			const ShipState& operator =(const ShipState& s)
			{
				_ship = s._ship;
				_world = s._world;
				_pose = s._pose;
				_vel = s._vel;
				_yaw = s._yaw;
				return *this;
			}
			ShipState(const ShipState& s) { (*this) = s; }

			virtual bool isEqual(RobotState* n) override;

			bool isSamePos(RobotState* n);
			
			//metric used to evaluate de distances: it could be any measure, but
			//the less the better
			// yet to be determined along with the expert
			double distanceTo(RobotState* p) override;
			
			//returns true if the ship state is equal to the state
			virtual bool isEqualToCurrentRobotState() override;
			
			//create a new state from current ship position
			virtual RobotState* createStateFromCurrentRobotState()override;
			
			//creates a ship state with num gdl doubles. if invalid return false
			virtual RobotState* createStateFromSample(vector<double> values);
			
			//creates a ship at x,y,z and with Vx, Vy, W speed(0 default)
			RobotState* createStateFromPoint3D(double x=0, double y=0, double yaw=0);
			
			virtual bool propagate(std::vector<double> v_auxCtrlAct, double delta_t, ShipState** p_retState);

			//moves the Ship one step from the current position towards
			//the state. Local planner
			virtual bool moveRobotCloser(double stepSize);

			//place the Ship in a state
			virtual void placeRobot();

			//place the robot in a state but prepared for achieving the target state
			virtual void placeRobotTowards(RobotState* target);

			virtual vector<double> getSample();

			//check if it is a valid state
			virtual bool isValid() { return true; }

			virtual double getCost() { return _cost; }
			virtual void setCost(double c) { _cost = c; }

			virtual Vector3D getAccs()
			{
				return this->_ship->simpleAccs();
			}

			//Draw a RobotState representation
			virtual void drawGL();

			virtual ShipState* clone();

			const Vector3D getPose(){ return _pose;}
			void setPose(Vector3D p) { _pose = p; }

			const Vector3D getVels() { return _vel; }
			void setVels(Vector3D v) { _vel = v; }

	
			virtual ~ShipState()
			{

			}
		protected:
			Ship* _ship;
			World* _world;
			// Third value corresponds to orientation 
			Vector3D _pose{ 0.0, 0.0, 0.0 }; //medium value
			double _yaw{0};
			Vector3D _vel{0.0, 0.0, 0.0}; //medium value

			double _cost;
	};
}
#endif //__SHIP_STATE_H_
