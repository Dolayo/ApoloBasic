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

#ifndef __EGKRRT__H_
#define __EGKRRT__H_

#include <mrcore.h>
#include "rdtstar.h"

namespace mr{
	
class ShipState;

class EGKRobotPath : public RobotPath
{
public:

	EGKRobotPath():RobotPath(){}
	virtual void drawGL() override;
};

class EGKRRT: public RDTstar
{
protected:
	class EGKtree : public RDTstar::RDTtree
	{
	protected:

		class EGKpath : public RDTstar::RDTtree::PathSegment
		{
			// Methods and attributes from RDTstar just to keep in mind
			/*
			vector<RobotState*> _inter; 
			RobotState* _init;
			RobotState* _end;
			PathSegment* _parent;
			RobotState* operator[](int i) { return _inter[i]; }
			int size() { return (int)_inter.size(); }
			PathSegment() { _init = 0; _end = 0; _parent = 0; }
			*/

			public:

				std::vector<std::vector<double>>_sequence;
				EGKpath():PathSegment(){}
				const EGKpath& operator=(const EGKpath& n) {
					_init = n._init;
					_end = n._end;
					_inter = n._inter;
					_parent = n._parent;
				}
				RobotState* operator[](int i) { return _inter[i]; }
				double getLength();
				EGKpath(const EGKpath& n){(*this) = n;}
				RobotState* last(){ return _inter.back(); }
				virtual void appendState(RobotState* p_aux){_inter.push_back(p_aux);}
				void appendCtrlAct(std::vector<double> v_aux) { _sequence.push_back(v_aux); }
				static EGKpath* createPath(RobotState* p_init, RobotState* p_end, bool& ar_success, int niter = 100, bool b_ensure_yaw = false);
				virtual std::vector<double> navigation(RobotState* p_initState, RobotState* p_finalState, 
														double& ar_init_yaw, bool& b_yaw_ensured, bool b_ensure_yaw = false);
				bool isGhostThere(ShipState* donkey, ShipState* carrot);
				void drawGL();
		};

	public:
		// Methods from RDTstar just to remember
		/*
		RobotState* _root;
		vector<PathSegment*> _paths;
		vector<RobotState*> _nodes;

		RDTtree() { _root = 0; _radius = 30; _divided = false; }
		int getNumNodes() { return (int)_nodes.size(); }
		virtual bool rootTree(RobotState* rot);
		virtual double distance(RobotState* p, PathSegment* path, RobotState** mnode = 0);
		virtual PathSegment* getClosestPathSegment(RobotState* n, RobotState** minstate);
		virtual RobotState* addNode(RobotState* n);
		virtual RobotPath getPathFromRoot(RobotState* n);
		virtual void Reconnect(vector<RobotState*>& v_nei, RobotState* Xnew);
		virtual PathSegment* findPath4Node(RobotState* node);
		virtual void getNeighbors(RobotState* Xnew, vector<RobotState*>* v_nei);
		virtual PathSegment* getBest(vector<RobotState*>& v_nei, RobotState** best);
		void drawGL();
	private:
		double _radius;
		bool _divided;
		void add(RobotState* n)
		{
			for (int i = 0; i < (int)_nodes.size(); i++)if (_nodes[i] == n)return;
			_nodes.push_back(n);
		}
		*/

		std::vector<RobotState*> _vertexes;

		std::vector<std::vector<double>> _sequence_solution;

		bool erase_vertex(RobotState* node)
		{
			bool b_ret = false;
			for (int i = 0; i < _vertexes.size(); ++i)
			{
				if (_vertexes[i]->isEqual(node))
				{
					_vertexes.erase(_vertexes.begin() + i);
					b_ret = true;
				}
			}
			return b_ret;
		}

		EGKtree() : RDTtree(){}
		//virtual double distance(RobotState* p, PathSegment* path, RobotState** mnode = 0) override;
		virtual RobotState* addNode(RobotState* node) override;
		virtual void Reconnect(vector<RobotState*>& v_nei, RobotState* Xnew) override;
		//virtual double distance(RobotState* rs, PathSegment* path, RobotState** mnode = nullptr) override;
		//virtual PathSegment* getClosestPathSegment(RobotState* n, RobotState** minstate) override;
		virtual PathSegment* getBest(vector<RobotState*>& v_nei, RobotState** best) override;
		EGKRobotPath* GetPathFromRoot(ShipState* n);
		double distance(RobotState* p, PathSegment* path, RobotState** mnode) override;
		void PopulateVertexes();
		virtual void drawGL() override;
	};

public:
	
	/*RDTstar() :SBPathPlanner(), _treeStart(), _treeGoal()
	{
		_treeA = &_treeStart;
		_treeB = &_treeGoal;
	}
	virtual bool computePlan(int maxiterations);
	virtual bool setStartAndGoalStates(RobotState* start_, RobotState* goal_);
	int getNumNodes() { return _treeStart.getNumNodes() + _treeGoal.getNumNodes(); }*/

	virtual bool setStartAndGoalStates(RobotState* start_, RobotState* goal_) override;
	virtual bool computePlan(int maxiterations) override;
	bool testingPlan();
	EGKRRT():RDTstar()
	{
		_tree = new EGKtree;
	}
	virtual void drawGL() override;


protected:
	EGKtree* _tree{nullptr};
};


}



#endif  //__EGKRRT__H
