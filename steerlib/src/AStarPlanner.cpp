//
// Copyright (c) 2009-2015 Glen Berseth, Mubbasir Kapadia, Shawn Singh, Petros Faloutsos, Glenn Reinman, Rahul Shome
// See license.txt for complete license.
//


#define NOMINMAX

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
	AStarPlannerNode * startNode;
	AStarPlannerNode * goalNode;
	double eps;

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
	double fValue(AStarPlannerNode * n, double epsilon, Util::Point goalPoint) {
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
		// if there is no node corresponding to the grid index, create a new one in the map
		if (gridIndex_gValuedNodes_map.find(gridIndex) == gridIndex_gValuedNodes_map.end())
		{
			// std::cout << "\nCREATING NEW NODE FOR GRIDINDEX " << gridIndex << std::endl;
			gridIndex_gValuedNodes_map.emplace(gridIndex, AStarPlannerNode(getPointFromGridIndex(gridIndex), DBL_MAX, DBL_MAX, gridIndex, nullptr));
		}

		return &gridIndex_gValuedNodes_map[gridIndex];
	}

	/* Takes the start and goal node and reconstructs a path by
	 * backtracking through parents.
	 * NEEDS WORK FIGURE OUT POINTER / REFERENCE DISTINCTION - Taichi */
	std::vector<Util::Point> reconstructPath(AStarPlannerNode * goal)
	{
		std::deque<Util::Point> temp; // temporary deque to reverse the order
		AStarPlannerNode * n = goal;
		
		// Backtrack through parents until we can't anymore (start node reached)
		while (n != nullptr) {
			temp.push_front(n->point);
			n = n->parent;
			// if(n == nullptr) {
			// 	std::cout << "n is null" << std::endl;
			// }
		}

		// Copy contents of deque to path output.
		std::vector<Util::Point> path;
		for (Util::Point p : temp)
		{
			path.push_back(p);
			std::cout << p << std::endl; // publish path
		}
		return path;
	}



	/* A* Search
	 * Remember: CALL BY REFERENCE is required to change the parameters of arguments
	 * e.g. void swap(int &x, int &y) { ... x = y; ... }
	 * More info: https://www.tutorialspoint.com/cplusplus/cpp_function_call_by_reference.htm
	 * Taichi */
	bool AStarPlanner::WeightedAStar(std::vector<Util::Point>& agent_path, Util::Point startPoint, Util::Point goalPoint, bool append_to_path)
	{
		// Set epsilon parameter (weight)
		double epsilon = 1.0;

		// Initialize start node
		int startGridIndex = gSpatialDatabase->getCellIndexFromLocation(startPoint.x, startPoint.z);
		AStarPlannerNode start = AStarPlannerNode(getPointFromGridIndex(startGridIndex), 0, DBL_MAX, startGridIndex, nullptr);
		start.f = fValue(&start, epsilon, goalPoint);
		gridIndex_gValuedNodes_map.emplace(startGridIndex, start);

		// Get grid index of goal point
		int goalGridIndex = gSpatialDatabase->getCellIndexFromLocation(goalPoint.x, goalPoint.z);
		std::cout << "GOAL GRID INDEX: " << goalGridIndex << std::endl;

		// Initialize OPEN Set
		openSet.push(&start); // add start to open set

		// Repeat search loop until the min in open set is the goal
		// int counter = 0;
		while (openSet.top()->gridIndex != goalGridIndex)
		{
			// // Current taken from the top of the open set and added to the closed set.
			AStarPlannerNode * current = openSet.top();
			// OUTER MEMORY: current = openSet.top();
			openSet.pop();
			closedSet.push(current);
			std::cout << "\nCURRENT - Address: " << current << ", Point: " << current->point << ", g: " << current->g << ", f: " << current->f << std::endl;

			// Get neighbors of current
			std::vector<int> neighborGridIndices = getNeighborGridIndices(current);

			// Search neighbors of current
			for (int neighborGridIndex : neighborGridIndices)
			{
				if (canBeTraversed(neighborGridIndex)) {
					AStarPlannerNode * neighbor = getNodeFromGridIndex(neighborGridIndex);

					// calculate cost of start ~~> current -> neighbor
					double cost = current->g + computeCost(current, neighbor);

					if (openSet.contains(neighbor) && cost < neighbor->g) {
						openSet.remove(neighbor); // remove duplicate to update cost
					}
					if (closedSet.contains(neighbor) && cost < neighbor->g) {
						closedSet.remove(neighbor); // remove duplicate to update cost
					}
					if (!openSet.contains(neighbor) && !closedSet.contains(neighbor) && cost < neighbor->g) {
						neighbor->g = cost;
						neighbor->f = fValue(neighbor, epsilon, goalPoint);
						neighbor->parent = current;
						openSet.push(neighbor);

						std::cout << "\nNEIGHBOR - Address: " << neighbor << ", Point: " << neighbor->point << " COST: " << neighbor->g << " f: " << neighbor->f << std::endl;
						std::cout << "New parent: " << neighbor->parent << std::endl;
					}
				}
			}
		}

		AStarPlannerNode * goal = openSet.top();

		// std::cout << "CURRENT (at the end) - Address: " << &current << std::endl;
		// std::cout << "\nTOP OF OPENSET (at the end) - Address: " << openSet.top() << std::endl;
		// std::cout << "\nGOAL - Address: " << &goal << ", Cost: " << goal->g << std::endl;
		// std::cout << "\nGOAL'S PARENT - Address: " << goal->parent << std::endl;

		// Print path (if found)
		std::cout << "\n PRINTING PATH" << std::endl;
		if (goal->parent == nullptr)
		{
			std::cout << "NO PATH CREATED. GOAL HAS NO PARENT" << std::endl;
			return false; // goal has no parent, so it was never reached
		}
		else
		{
			std::vector<Util::Point> path = reconstructPath(goal);
			if (append_to_path)
				agent_path.insert(agent_path.end(), path.begin(), path.end());
			else 
				agent_path = path;
			return true;
		}
	}


	/* Helper function for ARA* implementation */
	bool AStarPlanner::ARAStar_improvePath(double epsilon, AStarPlannerNode * goal)
	{	
		int counter = 0; // limit to 10000 iterations. if goal isn't reached before this, return false for failure
		
		while (openSet.list.size() > 0 && counter++ < 10000 && fValue(goal, epsilon, goal->point) > fValue(openSet.top(), epsilon, goal->point))
		{
			// std::cout << "\nfValue of goal: " << fValue(goal, epsilon, goal->point) << ", fValue of min(openSet): " << fValue(openSet.top(), epsilon, goal->point) << std::endl;
			
			AStarPlannerNode * current = openSet.top(); // get node from open set with smallest f value
			openSet.pop();
			closedSet.push(current);
			// std::cout << "CURRENT - address: " << current << ", Point: " << current->point << ", g: " << current->g << ", f: " << current->f << std::endl;

			std::vector<int> neighborGridIndices = getNeighborGridIndices(current); // get grid indices of neighboring cells
			
			for (int neighborGridIndex : neighborGridIndices) {
				if (canBeTraversed(neighborGridIndex)) {

					AStarPlannerNode * neighbor = getNodeFromGridIndex(neighborGridIndex);

					// calculate cost of start ~~> current -> neighbor
					double cost = current->g + computeCost(current, neighbor);

					assert(computeCost(current, neighbor) < 1.5); // make sure cost doesn't exceed a diagonal distance

					// relax neighbor if better path is found
					if (cost < neighbor->g)
					{
						neighbor->g = cost;
						neighbor->parent = current;
						// std::cout << "NEIGHBOR - Address: " << neighbor << ", Point: " << neighbor->point << " COST: " << neighbor->g << " f: " << neighbor->f << std::endl;
						// std::cout << "New parent: " << neighbor->parent << std::endl;
						
						if (closedSet.contains(neighbor))
						{
							inconsistentSet.push(neighbor);
						}
						else
						{
							// std::cout << "Adding/updating neighbor..." << std::endl;
							if (openSet.contains(neighbor))
								openSet.remove(neighbor); // remove duplicate to update cost
							neighbor->f = fValue(neighbor, epsilon, goal->point);
							openSet.push(neighbor);
						}
					}
				}
			}
		}
		return counter < 10000;
		// std::cout << "GOAL REACHED" << std::endl;
	}


	/* ARA* Main Method */
	bool AStarPlanner::ARAStar(std::vector<Util::Point>& agent_path, Util::Point startPoint, Util::Point goalPoint, bool append_to_path)
	{
		double epsilon = 2.5; // 2.5;

		// Initialize start node
		int startGridIndex = gSpatialDatabase->getCellIndexFromLocation(startPoint.x, startPoint.z);
		AStarPlannerNode start = AStarPlannerNode(getPointFromGridIndex(startGridIndex), 0, DBL_MAX, startGridIndex, nullptr);
		start.f = fValue(&start, epsilon, goalPoint);
		gridIndex_gValuedNodes_map.emplace(startGridIndex, start);

		// Get grid index of goal point
		int goalGridIndex = gSpatialDatabase->getCellIndexFromLocation(goalPoint.x, goalPoint.z);
		AStarPlannerNode * goal = getNodeFromGridIndex(goalGridIndex);
		std::cout << "GOAL GRID INDEX: " << goalGridIndex << std::endl;

		// Initialize OPEN Set
		openSet.push(&start); // add start to open set

		bool result = ARAStar_improvePath(epsilon, goal);

		double suboptimality_bound = std::min(epsilon, (goal->g / std::min(fValue(openSet.top(), 1.0, goal->point), (inconsistentSet.list.size() > 0 ? fValue(inconsistentSet.top(), 1.0, goal->point) : DBL_MAX))));

		if (result)
		{
			// std::cout << &suboptimality_bound << std::endl;
			// std::cout << "Inconsistent Set - Size: " << inconsistentSet.list.size() << std::endl;
			std::cout << "Path found with suboptimality bound " << suboptimality_bound << std::endl;
			agent_path = reconstructPath(goal);
		}

		while (result && suboptimality_bound > 1)
		{
			// decrease epsilon by 1 (minimum = 1)
			epsilon = MAX(epsilon - 1.0, 1.0);

			// move nodes from inconsistentSet to openSet
			while (inconsistentSet.list.size() > 0)
			{
				openSet.push(inconsistentSet.top());
				inconsistentSet.pop();
			}

			// update f-values for each node and re-sort
			for (AStarPlannerNode * n : openSet.list)
			{
				n->f = fValue(n, epsilon, goalPoint);
			}
			openSet.sort();

			// clear closedSet
			closedSet.list.clear();

			result = ARAStar_improvePath(epsilon, goal);

			suboptimality_bound = std::min(epsilon, (goal->g / std::min(fValue(openSet.top(), 1.0, goal->point), (inconsistentSet.list.size() > 0 ? fValue(inconsistentSet.top(), 1.0, goal->point) : DBL_MAX))));

			if (result)
			{
				std::cout << "Path found with suboptimality bound " << suboptimality_bound << std::endl;
				agent_path = reconstructPath(goal);
			}

		}
		

		return result;
	}

	bool AStarPlanner::ADStar(std::vector<Util::Point>& agent_path, Util::Point startPoint, Util::Point goalPoint, bool append_to_path)
	{
		eps = 2.5;

		// Initialize start node
		int startGridIndex = gSpatialDatabase->getCellIndexFromLocation(startPoint.x, startPoint.z);
		startNode = &AStarPlannerNode(startPoint, DBL_MAX, DBL_MAX, startGridIndex, nullptr, DBL_MAX, DBL_MAX, DBL_MAX);
		gridIndex_gValuedNodes_map.emplace(startGridIndex, startNode);

		//Initialize goal node
		int goalGridIndex = gSpatialDatabase->getCellIndexFromLocation(goalPoint.x, goalPoint.z);
		goalNode = &AStarPlannerNode(goalPoint, DBL_MAX, -1, goalGridIndex, nullptr, 0, 0, 0);
		key(goalNode);

		openSet.push(goalNode);

		ComputeorImprovePath();


		
		
		
		return true;
	}

	void AStarPlanner::ComputeorImprovePath() {
		while (*openSet.top() < *startNode || startNode->rhs != startNode->g) {

		}
	}

	double AStarPlanner::g(AStarPlannerNode * s) {
		

		return 0.0;
	}

	double AStarPlanner::rhs(AStarPlannerNode * s) {


		return 0.0;
	}

	double AStarPlanner::h(AStarPlannerNode * s1, AStarPlannerNode * s2) {


		return 0.0;
	}

	void AStarPlanner::key(AStarPlannerNode * s) {
		if (s->g > s->rhs) {
			s->k1 = s->rhs + eps*h(startNode, s);
			s->k2 = s->rhs;
		}
		else {
			s->k1 = s->g + h(startNode, s);
			s->k2 = s->g;
		}
	}

	




	bool AStarPlanner::computePath(std::vector<Util::Point>& agent_path, Util::Point start, Util::Point goal, SteerLib::SpatialDataBaseInterface * _gSpatialDatabase, bool append_to_path)
	{
		gSpatialDatabase = _gSpatialDatabase;

		// A* IMPLEMENTATION
		// std::cout << "WeightedA*" << std::endl;
		// bool result = WeightedAStar(agent_path, start, goal, append_to_path);

		// ARA* IMPLEMENTATION
		std::cout << "ARA*" << std::endl;
		bool result = WeightedAStar(agent_path, start, goal, append_to_path);
		return result;
	}


}