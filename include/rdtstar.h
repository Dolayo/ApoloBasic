/**********************************************************************
 *
 * This code is part of the MRcore project
 * Author:   Daniel Olayo Andres
 * 
 * 
 * MRcore is licenced under the Common Creative License,
 * Attribution-NonCommercial-ShareAlike 3.0
 *
 * You are free:
 *   - to Share - to copy, distribute and transmit the work
 *   - to Remix - to adapt the work
 *
 * Under the following conditions:
 *   - Attribution. You must attribute the work in the manner specified
 *     by the author or licensor (but not in any way that suggests that
 *     they endorse you or your use of the work).
 *   - Noncommercial. You may not use this work for commercial purposes.
 *   - Share Alike. If you alter, transform, or build upon this work,
 *     you may distribute the resulting work only under the same or
 *     similar license to this one.
 *
 * Any of the above conditions can be waived if you get permission
 * from the copyright holder.  Nothing in this license impairs or
 * restricts the author's moral rights.
 *
 * It is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied 
 * warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR
 * PURPOSE.  
 **********************************************************************/

#ifndef __MRCORE__RDT_STAR__H
#define __MRCORE__RDT_STAR__H
#include <mrcore.h>
#include <algorithm>
#include "defines.h"
using namespace std;

namespace mr{

class WBStar: public WBState
{
	protected:

		double _cost;
	public:

		WBStar(WheeledBaseSim* r, World* w, double c):WBState(r, w),_cost(c){}

		const WBStar& operator=(const WBStar& n) {
			sectors = n.sectors;
			pose = n.pose;
			world = n.world;
			robot = n.robot;
			_cost = n._cost;
			return *this;
		}

		WBStar(const WBStar& a):WBState(a)
		{
			(*this) = a;
		}

		RobotState* WBStar::createStateFromSample(vector<double> values);
		WBStar* createStateFromPoint3D(double x, double y, double z);
		RobotState* WBStar::createStateFromCurrentRobotState();

		virtual double getCost() { return _cost; }
		virtual void setCost(double c) { _cost = c; }

		virtual RobotState* clone();
		virtual ~WBStar();
};


class RDTstar: public SBPathPlanner
{
protected:
  class RDTtree //an internal class for the tree
	{
	protected:

		class PathSegment{ //internal class for a segment of tree
			public:
			
			vector<RobotState *> _inter; //the path segment does not own the nodes
			RobotState *_init;
			RobotState *_end;
			PathSegment *_parent;
			const PathSegment &operator=(const PathSegment &n){
				_init=n._init;
				_end=n._end;
				_inter=n._inter;
				_parent=n._parent;
			}
			RobotState* operator[](int i){return _inter[i];}
			int size(){return (int)_inter.size();} 
			PathSegment() { _init = nullptr; _end = nullptr; _parent = nullptr; }
			PathSegment (const PathSegment &n){
				(*this)=n;
			}
			virtual void drawGL();
			
		};

		double _radius;
		bool _divided;
		void add(RobotState *n)
		{
			for(int i=0;i<(int)_nodes.size();i++)if(_nodes[i]==n)return;
				_nodes.push_back(n);
		}

	public:
		//returns the end added node or cero if none
		
		RobotState *_root;
		vector<PathSegment *> _paths;
		vector<RobotState *> _nodes; 

		RDTtree() { _root = 0; _radius = radius; _divided = false; }
		int getNumNodes(){return (int)_nodes.size();}
		virtual bool rootTree(RobotState *rot);
		virtual double distance(RobotState *p, PathSegment *path, RobotState **mnode=0);
		virtual PathSegment *getClosestPathSegment(RobotState *n,RobotState **minstate);
		virtual RobotState *addNode(RobotState *n);
		virtual RobotPath getPathFromRoot(RobotState *n);
		virtual void Reconnect( vector<RobotState*>& v_nei, RobotState* Xnew);
		virtual PathSegment* findPath4Node( RobotState* node);
		virtual void getNeighbors(RobotState *Xnew, vector<RobotState*> *v_nei);
		virtual PathSegment* getBest(vector<RobotState*>& v_nei, RobotState **best);
		virtual void drawGL();
	};

	  RDTtree _treeStart;
	  RDTtree _treeGoal;
	  RDTtree *_treeA;
	  RDTtree *_treeB;
public:
	RDTstar():SBPathPlanner(),_treeStart(),_treeGoal()
	{
		_treeA=&_treeStart;
		_treeB=&_treeGoal;
	}
  virtual bool computePlan(int maxiterations);
  virtual bool setStartAndGoalStates(RobotState *start_, RobotState *goal_);
  int getNumNodes(){return _treeStart.getNumNodes()+_treeGoal.getNumNodes();}
  virtual void drawGL();
};


}
#endif  //__MRCORE__RDT_STAR__H
