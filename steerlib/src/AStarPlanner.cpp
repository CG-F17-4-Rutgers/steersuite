//
// Copyright (c) 2009-2015 Glen Berseth, Mubbasir Kapadia, Shawn Singh, Petros Faloutsos, Glenn Reinman, Rahul Shome
// See license.txt for complete license.
//


#include <vector>
#include <stack>
#include <set>
#include <map>
#include <iostream>
#include <algorithm> 
#include <functional>
#include <queue>
#include <math.h>
#include "planning/AStarPlanner.h"
#include <limits> // added -Taichi
#include <queue> // added -Taichi
#include <map> // added -Taichi
#include <unordered_map> // added -Taichi


#define COLLISION_COST  1000
#define GRID_STEP  1
#define OBSTACLE_CLEARANCE 1
#define MIN(X,Y) ((X) < (Y) ? (X) : (Y))
#define MAX(X,Y) ((X) > (Y) ? (X) : (Y))

namespace SteerLib
{

	// typedef struct {
	// 	Util::Point point;
	// 	double gValue;
	// } Node;

	AStarPlanner::AStarPlanner(){}

	AStarPlanner::~AStarPlanner(){}

	bool AStarPlanner::canBeTraversed ( int id ) 
	{
		double traversal_cost = 0;
		int current_id = id;
		unsigned int x,z;
		gSpatialDatabase->getGridCoordinatesFromIndex(current_id, x, z);
		int x_range_min, x_range_max, z_range_min, z_range_max;

		x_range_min = MAX(x-OBSTACLE_CLEARANCE, 0);
		x_range_max = MIN(x+OBSTACLE_CLEARANCE, gSpatialDatabase->getNumCellsX());

		z_range_min = MAX(z-OBSTACLE_CLEARANCE, 0);
		z_range_max = MIN(z+OBSTACLE_CLEARANCE, gSpatialDatabase->getNumCellsZ());


		for (int i = x_range_min; i<=x_range_max; i+=GRID_STEP)
		{
			for (int j = z_range_min; j<=z_range_max; j+=GRID_STEP)
			{
				int index = gSpatialDatabase->getCellIndexFromGridCoords( i, j );
				traversal_cost += gSpatialDatabase->getTraversalCost ( index );
				
			}
		}

		if ( traversal_cost > COLLISION_COST ) 
			return false;
		return true;
	}

	Util::Point AStarPlanner::getPointFromGridIndex(int id)
	{
		Util::Point p;
		gSpatialDatabase->getLocationFromIndex(id, p);
		return p;
	}




	/* Calculates f-value of node n with euclidean distance heuristic
	 * Taichi */
	float fValue(AStarPlannerNode * n, double epsilon, Util::Point goalPoint) {
		return n->g + epsilon * Util::distanceBetween(n->point, goalPoint);
	}

	/* Given an AStarPlannerNode n, returns the grid-indices of grid-cells neighboring n
	 * Taichi */
	std::vector<int> AStarPlanner::getNeighborGridIndices(AStarPlannerNode * n)
	{
		int nGridIndex = gSpatialDatabase->getCellIndexFromLocation(n->point.x, n->point.z);
		std::vector<int> neighborGridIndices;
		for (int i = -1; i <= 1; i+=GRID_STEP) {
			for (int j = -1; j <= 1; j+=GRID_STEP) {
				int neighborGridIndex = gSpatialDatabase->getCellIndexFromLocation(n->point.x + i, n->point.z + j);
				if (neighborGridIndex != nGridIndex) // add neighbor if not self
					neighborGridIndices.push_back(neighborGridIndex);
			}
		}
		return neighborGridIndices;
	}

	/* Computes cost between node and its neighbor. Simple Euclidean distance. Higher cost for diagonal neighbors. 
	 * Taichi */
	double computeCost(AStarPlannerNode * n, AStarPlannerNode * neighbor)
	{
		return Util::distanceBetween(n->point, neighbor->point);
	}

	/* Returns node corresponding to grid index in spatial database
	 * If a node has not been created for the given grid index,
	 * a new node with default values is created and returned.
	 * Taichi */
	AStarPlannerNode * AStarPlanner::getNodeFromGridIndex(int gridIndex)
	{
		if (gridIndex_gValuedNodes_map.find(gridIndex) == gridIndex_gValuedNodes_map.end())
		{
			std::cout << "\nCREATING NEW NODE FOR GRIDINDEX " << gridIndex << std::endl;
			// AStarPlannerNode dummy;
			gridIndex_gValuedNodes_map.emplace(gridIndex, AStarPlannerNode(getPointFromGridIndex(gridIndex), DBL_MAX, DBL_MAX, gridIndex, nullptr));
		}
		return &gridIndex_gValuedNodes_map[gridIndex];
	}

	/* Takes the start and goal node and reconstructs a path by
	 * backtracking through parents.
	 * NEEDS WORK FIGURE OUT POINTER / REFERENCE DISTINCTION - Taichi */
	std::vector<Util::Point> reconstructPath(const AStarPlannerNode* goal)
	{
		// std::deque<Util::Point> temp; // temporary deque to reverse the order
		// AStarPlannerNode n = goal;
		
		// // Backtrack through parents until we can't anymore (start node reached)
		// while (n != nullptr) {
		// 	std::cout << n.point << std::endl;
		// 	temp.push_front(n.point);
		// 	n = n.parent;
		// }

		// temp.push_front(n.point);

		// // Copy contents of deque to path output.
		// std::vector<Util::Point> path;
		// for (Util::Point p : temp)
		// {
		// 	path.push_back(p);
		// }
		// return path;
	}



	/* A* Search
	 * Remember: CALL BY REFERENCE is required to changes the parameters of arguments
	 * e.g. void swap(int &x, int &y) { ... x = y; ... }
	 * More info: https://www.tutorialspoint.com/cplusplus/cpp_function_call_by_reference.htm
	 * Taichi */
	bool AStarPlanner::WeightedAStar(std::vector<Util::Point>& agent_path, Util::Point startPoint, Util::Point goalPoint, bool append_to_path)
	{
		// Set epsilon parameter (weight)
		double epsilon = 1.0;

		// Initialize start node
		int startGridIndex = gSpatialDatabase->getCellIndexFromLocation(startPoint.x, startPoint.z);
		AStarPlannerNode start = AStarPlannerNode(startPoint, 0, DBL_MAX, startGridIndex, nullptr);
		start.f = fValue(&start, epsilon, goalPoint);
		gridIndex_gValuedNodes_map.emplace(startGridIndex, start);

		// Get grid index of goal point
		int goalGridIndex = gSpatialDatabase->getCellIndexFromLocation(goalPoint.x, goalPoint.z);
		std::cout << "GOAL GRID INDEX: " << goalGridIndex << std::endl;

		// Initialize OPEN Set
		openSet.push(&start); // add start to open set

		// OUTER MEMORY: Current taken from the top of the open set and added to the closed set.
		// AStarPlannerNode& current = openSet.top();

		// Repeat search loop until the min in open set is the goal
		int counter = 0;
		while (openSet.top()->gridIndex != goalGridIndex && counter++ < 100)
		{
			// // Current taken from the top of the open set and added to the closed set.
			AStarPlannerNode * current = openSet.top();
			// OUTER MEMORY: current = openSet.top();
			openSet.pop();
			closedSet.push(current);
			std::cout << "\nCURRENT - Address: " << &current << ", Point: " << current->point << ", g: " << current->g << ", f: " << current->f << std::endl;

			// Get neighbors of current
			std::vector<int> neighborGridIndices = getNeighborGridIndices(current);

			// Search neighbors of current
			for (int neighborGridIndex : neighborGridIndices)
			{
				AStarPlannerNode * neighbor = getNodeFromGridIndex(neighborGridIndex);

				// calculate cost of start ~~> current -> neighbor
				double cost = current->g + computeCost(current, neighbor);

				if (openSet.contains(neighbor) && cost < neighbor->g) {
					openSet.remove(neighbor); // remove duplicate to update cost
				}
				if (closedSet.contains(neighbor) && cost < neighbor->g) {
					closedSet.remove(neighbor); // remove duplicate to update cost
				}
				if (!openSet.contains(neighbor) && !closedSet.contains(neighbor)) {
					neighbor->g = cost;
					neighbor->f = fValue(neighbor, epsilon, goalPoint);
					neighbor->parent = current;
					openSet.push(neighbor);

					std::cout << "\nNEIGHBOR - Address: " << &neighbor << ", Point: " << neighbor->point << " COST: " << neighbor->g << std::endl;
					std::cout << "New parent: " << neighbor->parent << std::endl;
				}

			}
		}

		// std::cout << "\nGOAL NODE (outside loop): " << goal->g << " POINT: " << goal->point << " PARENT: " << goal->parent << std::endl;

		// Display everything remaining in open set
		// std::cout << "\nOPEN SET: " << std::endl;
		// for (AStarPlannerNode n : openSet.list)
		// {
		// 	std::cout << n.f << std::endl;
		// }

		// std::cout << "CURRENT (at the end) - Address: " << &current << std::endl;
		std::cout << "\nTOP OF OPENSET (at the end) - Address: " << openSet.top() << std::endl;

		AStarPlannerNode * goal = openSet.top();
		std::cout << "\nGOAL - Address: " << &goal << ", Cost: " << goal->g << std::endl;

		std::cout << "\nGOAL'S PARENT - Address: " << goal->parent << std::endl;

		// AStarPlannerNode& prev = * goal.parent;
		// std::cout << &prev << std::endl;
		// std::cout << &prev << std::endl;
		// std::cout << "GOAL - Address: " << &goal << ", Cost: " << goal.g << ", Parent: " << goal.parent << std::endl;
		// AStarPlannerNode* parent = goal->parent;
		// std::cout << parent->g << std::endl;
		// std::cout << "PREV - Address: " << prev << std::endl;
		// const AStarPlannerNode* prev = goal->parent;
		// std::cout << "PREVIOUS - Address: " << prev << ", Cost: " << prev->g << ", Parent: " << &prev->parent << std::endl;

		return true;
		// Print path (if found)
		// std::cout << "\n PRINTING PATH" << std::endl;
		// if (&goal->parent == nullptr)
		// {
		// 	std::cout << "NO PATH CREATED. GOAL HAS NO PARENT" << std::endl;
		// 	return false; // goal has no parent, so it was never reached
		// }
		// else
		// {
		// 	std::vector<Util::Point> path = reconstructPath(goal);
		// 	if (append_to_path)
		// 		agent_path.insert(agent_path.end(), path.begin(), path.end());
		// 	else 
		// 		agent_path = path;
		// 	return true;
		// }
	}


	bool AStarPlanner::computePath(std::vector<Util::Point>& agent_path, Util::Point start, Util::Point goal, SteerLib::SpatialDataBaseInterface * _gSpatialDatabase, bool append_to_path)
	{
		gSpatialDatabase = _gSpatialDatabase;

		// A* IMPLEMENTATION - Taichi
		std::cout << "WeightedA*" << std::endl;
		bool result = WeightedAStar(agent_path, start, goal, append_to_path);
		std::cout << "result: " << result << std::endl;
		for (Util::Point p : agent_path)
		{
			std::cout << p << std::endl;
		}
		return result;
	}


}