//
// Copyright (c) 2009-2015 Glen Berseth, Mubbasir Kapadia, Shawn Singh, Petros Faloutsos, Glenn Reinman, Rahul Shome
// See license.txt for complete license.
//


#ifndef __STEERLIB_A_STAR_PLANNER_H__
#define __STEERLIB_A_STAR_PLANNER_H__


#include <vector>
#include <stack>
#include <set>
#include <map>
#include <unordered_map> // added -Taichi
#include <limits> // added -Taichi
#include <algorithm> // added -Taichi
#include <functional> // added -Taichi
#include "SteerLib.h"

namespace SteerLib
{

	/*
		@function The AStarPlannerNode class gives a suggested container to build your search tree nodes.
		@attributes 
		f : the f value of the node
		g : the cost from the start, for the node
		point : the point in (x,0,z) space that corresponds to the current node
		parent : the pointer to the parent AStarPlannerNode, so that retracing the path is possible.
		@operators 
		The greater than, less than and equals operator have been overloaded. This means that objects of this class can be used with these operators. Change the functionality of the operators depending upon your implementation

	*/
	class STEERLIB_API AStarPlannerNode{
		public:
			double f;
			double g;
			Util::Point point;
			AStarPlannerNode* parent;
			int gridIndex;
			AStarPlannerNode(Util::Point _point, double _g, double _f, AStarPlannerNode* _parent)
			{
				f = _f;
				point = _point;
				g = _g;
				parent = _parent;
			}
			AStarPlannerNode(Util::Point _point, double _g, double _f, int _gridIndex, AStarPlannerNode* _parent)
			{
				f = _f;
				point = _point;
				g = _g;
				parent = _parent;
				gridIndex = _gridIndex;
			}
			AStarPlannerNode()
			{
			}
			bool operator<(AStarPlannerNode other) const
		    {
		        return this->f < other.f;
		    }
		    bool operator>(AStarPlannerNode other) const
		    {
		        return this->f > other.f;
		    }
		    bool operator==(AStarPlannerNode other) const
		    {
		        // return ((this->point.x == other.point.x) && (this->point.z == other.point.z));
		        return (this->gridIndex == other.gridIndex);
		    }

	};


	/* Comparator function for AStarPlannerNode pointers. Compares by f-value */
	static bool comp(const AStarPlannerNode * n1, const AStarPlannerNode * n2) {
		// return true;
		return n1->f < n2->f;
	}
	

	/* Custom stupid class for sorted lists.
	 * Sorts every time an item is pushed.
	 */
	class STEERLIB_API MyDumbAssClass{
		public:
			std::deque<AStarPlannerNode*> list;
			void push(AStarPlannerNode * node)
			{
				this->list.push_back(node);
				std::sort(this->list.begin(), this->list.end(), comp);
				// [](const AStarPlannerNode * n1, const AStarPlannerNode * n2) { return n1->f < n2->f; }
			}
			void pop()
			{
				this->list.pop_front();
			}
			AStarPlannerNode * top()
			{
				return this->list.front();
			}
			bool contains(AStarPlannerNode* node)
			{
				std::find(this->list.begin(), this->list.end(), node) != this->list.end();
			}
			void remove(AStarPlannerNode* node)
			{
				this->list.erase(std::remove(this->list.begin(), this->list.end(), node), this->list.end());
			}
	};

	

	

	class STEERLIB_API AStarPlanner{
		public:
			AStarPlanner();
			~AStarPlanner();
			// NOTE: There are four indices that need to be disambiguated
			// -- Util::Points in 3D space(with Y=0)
			// -- (double X, double Z) Points with the X and Z coordinates of the actual points
			// -- (int X_GRID, int Z_GRID) Points with the row and column coordinates of the GridDatabase2D. The Grid database can start from any physical point(say -100,-100). So X_GRID and X need not match
			// -- int GridIndex  is the index of the GRID data structure. This is an unique id mapping to every cell.
			// When navigating the space or the Grid, do not mix the above up

			/*
				@function canBeTraversed checkes for a OBSTACLE_CLEARANCE area around the node index id for the presence of obstacles.
				The function finds the grid coordinates for the cell index  as (X_GRID, Z_GRID)
				and checks cells in bounding box area
				[[X_GRID-OBSTACLE_CLEARANCE, X_GRID+OBSTACLE_CLEARANCE],
				[Z_GRID-OBSTACLE_CLEARANCE, Z_GRID+OBSTACLE_CLEARANCE]]
				This function also contains the griddatabase call that gets traversal costs.
			*/
			bool canBeTraversed ( int id );
			/*
				@function getPointFromGridIndex accepts the grid index as input and returns an Util::Point corresponding to the center of that cell.
			*/
			Util::Point getPointFromGridIndex(int id);


			/*
				@function computePath
				DO NOT CHANGE THE DEFINITION OF THIS FUNCTION
				This function executes an A* query
				@parameters
				agent_path : The solution path that is populated by the A* search
				start : The start point
				goal : The goal point
				_gSpatialDatabase : The pointer to the GridDatabase2D from the agent
				append_to_path : An optional argument to append to agent_path instead of overwriting it.
			*/
			bool computePath(std::vector<Util::Point>& agent_path, Util::Point start, Util::Point goal, SteerLib::SpatialDataBaseInterface * _gSpatialDatabase, bool append_to_path = false);
		private:
			SteerLib::SpatialDataBaseInterface * gSpatialDatabase;
			MyDumbAssClass openSet; // MyDumbAssClass is a custom class for a sorted list
			MyDumbAssClass closedSet;
			std::map<int, AStarPlannerNode> gridIndex_gValuedNodes_map;
			

			// sets are minimum priority queues. Because std::priority_queue is a max-priority by default, need to use std::greater as comparator.
			// std::priority_queue< AStarPlannerNode, std::vector<AStarPlannerNode>, std::greater<AStarPlannerNode> > openSet, inconsistentSet;
			// std::vector<AStarPlannerNode> inconsistentSet;
			

			/*
				@function getNeighborGridIndices returns a vector of grid indices, corresponding
				to cells in the grid structure of the spatial database that are neighbors of the
				node n.
			*/
			std::vector<int> getNeighborGridIndices(AStarPlannerNode * n);

			/*
			*/
			AStarPlannerNode * getNodeFromGridIndex(int gridIndex);

			/*
				@function ARAstar_improvePath is a helper function for ARAstar.
				Runs A* search with a certain epsilon value
			*/
			void ARAStar_improvePath(double epsilon, AStarPlannerNode goal);

			/* 
				@function ARAstar runs ARA* search algorithm.
				Modifies or appends found path to agent_path (depending on value of append_to_path).
			*/
			bool ARAStar(std::vector<Util::Point>& agent_path, Util::Point startPoint, Util::Point goalPoint, bool append_to_path);


			/*
				@function AStar runs A* search algorithm.
			*/
			bool WeightedAStar(std::vector<Util::Point>& agent_path, Util::Point startPoint, Util::Point goalPoint, bool append_to_path);

	};


}


#endif
