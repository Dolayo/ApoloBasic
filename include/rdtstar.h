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

using namespace std;

namespace mr{

class WBStar: public WBState
{
	WBStar* parent;
	double cost;
	public:
		virtual double getCost() { return cost; }
		virtual void setCost(double c) { cost = c; }
		virtual WBStar* getParent(){return parent;}
		virtual void setParent(WBStar* p){parent = p;}
};


class RDTstar: public SBPathPlanner
{
protected:
  class RDTtree //an internal class for the tree
	{
		class PathSegment{ //internal class for a segment of tree
			public:
			
			int id;
			vector<RobotState *> inter; //the path segment does not own the nodes
			RobotState *init;
			RobotState *end;
			PathSegment *parent;
			const PathSegment &operator=(const PathSegment &n){
				init=n.init;
				end=n.end;
				inter=n.inter;
				parent=n.parent;
			}
			RobotState *operator[](int i){return inter[i];}
			int size(){return inter.size();} 
			PathSegment() { id = 0; init = 0; end = 0; parent = 0; }
			PathSegment (const PathSegment &n){
				(*this)=n;
			}
			void drawGL();
			
		};
	public:
		//returns the end added node or cero if none
		
		RobotState *root;
		vector<PathSegment *> paths;
		vector<RobotState *> nodes; 

		RDTtree(){root=0;radius=20;}
		int getNumNodes(){return (int)nodes.size();}
		bool rootTree(RobotState *rot);
		double distance(RobotState *p, PathSegment *path, RobotState **mnode=0);
		PathSegment *getClosestPathSegment(RobotState *n,RobotState **minstate);
		RobotState *addNode(RobotState *n);
		RobotPath getPathFromRoot(RobotState *n);
		void Reconnect( vector<RobotState*> *v_nei, RobotState* Xnew);
		PathSegment* findPath4Node( RobotState* node);
		void getNeighbors(RobotState *Xnew, vector<RobotState*> *v_nei);
		PathSegment* getBest(vector<RobotState*> v_nei, RobotState **best);
		PathSegment* getPathByID(int n);
		void deletePath(int n);
		void drawGL();
	private:
		double radius;
		void add(RobotState *n)
		{
			for(int i=0;i<(int)nodes.size();i++)if(nodes[i]==n)return;
				nodes.push_back(n);
		}
	};

	  RDTtree treeStart;
	  RDTtree treeGoal;
	  RDTtree *treeA;
	  RDTtree *treeB;
public:
	RDTstar():SBPathPlanner(),treeStart(),treeGoal()
	{
		treeA=&treeStart;
		treeB=&treeGoal;
	}
  bool computePlan(int maxiterations);
  virtual bool setStartAndGoalStates(RobotState *start_, RobotState *goal_);
  int getNumNodes(){return treeStart.getNumNodes()+treeGoal.getNumNodes();}
  virtual void drawGL();
};


}
#endif  //__MRCORE__RDT_STAR__H
