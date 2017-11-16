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
		// if there is no node corresponding to the grid index, create a new one in the map
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
	std::vector<Util::Point> reconstructPath(AStarPlannerNode * goal)
	{
		std::deque<Util::Point> temp; // temporary deque to reverse the order
		AStarPlannerNode * n = goal;
		
		// Backtrack through parents until we can't anymore (start node reached)
		while (n != nullptr) {
			std::cout << "HE" <<  n->point << std::endl;
			temp.push_front(n->point);
			n = n->parent;
			if(n == nullptr) {
				std::cout << "n is null" << std::endl;
			}
		}

		// Copy contents of deque to path output.
		std::vector<Util::Point> path;
		for (Util::Point p : temp)
		{
			path.push_back(p);
		}
		return path;
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


	void publish(std::vector<Util::Point>& agent_path, AStarPlannerNode * goal, double suboptimality_bound)
	{
		// print the path, update agent_path, and print suboptimality bound??
		std::vector<Util::Point> blah = reconstructPath(goal);
		agent_path = blah;
	}


	/* Helper function for ARA* implementation */
	void AStarPlanner::ARAStar_improvePath(double epsilon, AStarPlannerNode * goal)
	{	

		while (openSet.list.size() > 0 && fValue(goal, epsilon, goal->point) > fValue(openSet.top(), epsilon, goal->point))
		{
			std::cout << "Size: " << openSet.list.size() << std::endl;
			AStarPlannerNode * current = openSet.top(); // get node from open set with smallest f value
			// std::cout << n.f << std::endl;
			openSet.pop();
			closedSet.push(current);

			std::vector<int> neighborGridIndices = getNeighborGridIndices(current); // get grid indices of neighboring cells
			
			for (int neighborGridIndex : neighborGridIndices) {
				if (canBeTraversed(neighborGridIndex)) {

					AStarPlannerNode * neighbor = getNodeFromGridIndex(neighborGridIndex);

					// calculate cost of start ~~> current -> neighbor
					double cost = current->g + computeCost(current, neighbor);
					std::cout << cost << std::endl;

					assert(computeCost(current, neighbor) < 1.5); // make sure cost doesn't exceed a diagonal distance

					// relax neighbor if better value is found
					if (cost < neighbor->g) {
						std::cout << "HELY" << std::endl;
						neighbor->g = cost;
						neighbor->parent = current;
						std::cout << "Closedset Size: " << closedSet.list.size() << std::endl;

						std::cout << "Top item in closed set: " << closedSet.top() << std::endl;
						std::cout << "Neighbor: " << neighbor << std::endl;
						std::cout << "Does closed set contain neighbor? " << closedSet.contains(neighbor) << std::endl;

						if (!(closedSet.contains(neighbor)))
						{
							std::cout << "please get here" << std::endl;
							neighbor->f = fValue(neighbor, epsilon, goal->point);
							openSet.push(neighbor);
						}
						else
						{
							inconsistentSet.push(neighbor);
						}
					}
				}
			}
		}
	}


	/* ARA* Main Method */
	bool AStarPlanner::ARAStar(std::vector<Util::Point>& agent_path, Util::Point startPoint, Util::Point goalPoint, bool append_to_path)
	{
		double epsilon = 2.5;

		// Initialize start node
		int startGridIndex = gSpatialDatabase->getCellIndexFromLocation(startPoint.x, startPoint.z);
		AStarPlannerNode start = AStarPlannerNode(startPoint, 0, DBL_MAX, startGridIndex, nullptr);
		start.f = fValue(&start, epsilon, goalPoint);
		gridIndex_gValuedNodes_map.emplace(startGridIndex, start);

		// Get grid index of goal point
		int goalGridIndex = gSpatialDatabase->getCellIndexFromLocation(goalPoint.x, goalPoint.z);
		AStarPlannerNode goalNode = AStarPlannerNode(goalPoint, DBL_MAX, DBL_MAX, goalGridIndex, nullptr);
		gridIndex_gValuedNodes_map.emplace(goalGridIndex, goalNode);
		std::cout << "GOAL GRID INDEX: " << goalGridIndex << std::endl;
		AStarPlannerNode * goal = &goalNode;

		// Initialize OPEN Set
		openSet.push(&start); // add start to open set

		ARAStar_improvePath(epsilon, goal);

		double suboptimality_bound = MIN(epsilon, (goal->g / MIN(fValue(openSet.top(), 1.0, goal->point), fValue(inconsistentSet.top(), 1.0, goal->point))));

		//publish(agent_path, goal, suboptimality_bound); // TODO

		return true;

		// while (suboptimality_bound > 1.0)
		// {
		// 	epsilon = MAX(epsilon - 1.0, 1.0); // decrease epsilon by 1

		// }





		// AStarPlannerNode goal = AStarPlannerNode(goalPoint, DBL_MAX, DBL_MAX, 0);
		// int goalGridIndex = gSpatialDatabase->getCellIndexFromLocation(goal.point.x, goal.point.z);
		// gridIndex_gValuedNodes_map.emplace(goalGridIndex, goal);
		// AStarPlannerNode start = AStarPlannerNode(startPoint, 0, DBL_MAX, 0);
		// int startGridIndex = gSpatialDatabase->getCellIndexFromLocation(start.point.x, start.point.z);
		// gridIndex_gValuedNodes_map.emplace(startGridIndex, start);

		// // Populate spatial database points
		// start.f = calcFValueEuclidean(start, epsilon, goal);
		// openSet.push(start);

		// ARAStar_improvePath(epsilon, goal);

		// double suboptimality_bound = MIN(epsilon, (goal.g / MIN(calcFValueEuclidean(openSet.top(), 1.0, goal), calcFValueEuclidean(inconsistentSet.top(), 1.0, goal))));

		// while (suboptimality_bound > 1.0)
		// {
		// 	epsilon = MAX(epsilon - 1.0, 1.0); // decrease epsilon by 1
		// 	std::priority_queue< AStarPlannerNode, std::vector<AStarPlannerNode>, std::greater<AStarPlannerNode> > newOpenSet;
		// 	for (unsigned int i = 0; i < inconsistentSet.size(); i++)
		// 	{
		// 		AStarPlannerNode n = inconsistentSet.top();
		// 		inconsistentSet.pop();
		// 		n.f = calcFValueEuclidean(n, epsilon, goal);
		// 		newOpenSet.push(n);
		// 	}
		// 	for (unsigned int i = 0; i < openSet.size(); i++)
		// 	{
		// 		AStarPlannerNode n = openSet.top();
		// 		openSet.pop();
		// 		n.f = calcFValueEuclidean(n, epsilon, goal);
		// 		newOpenSet.push(n);
		// 	}
		// 	openSet = newOpenSet;
		// 	closedSet.clear(); // does this work, who knows.

		// 	ARAStar_improvePath(epsilon, goal);

		// 	suboptimality_bound = MIN(epsilon, (goal.g / MIN(calcFValueEuclidean(openSet.top(), 1.0, goal), calcFValueEuclidean(inconsistentSet.top(), 1.0, goal))));

		// 	std::cout << "SUBOPTIMALITY BOUND: " << suboptimality_bound << std::endl;
		// }

		// if (goal.parent == NULL)
		// {
		// 	return false; // goal has no parent, so it was never reached
		// }
		// else
		// {
		// 	std::deque<AStarPlannerNode*> path;
		// 	AStarPlannerNode* n = &goal;
		// 	while (n != NULL)
		// 	{
		// 		path.push_front(n);
		// 		n = n->parent;
		// 	}
		// 	std::vector<Util::Point> pathPoints;
		// 	for (AStarPlannerNode* n : path)
		// 		pathPoints.push_back(n->point);
		// 	if (append_to_path)
		// 		agent_path.insert(agent_path.end(), pathPoints.begin(), pathPoints.end());
		// 	else 
		// 		agent_path = pathPoints;
		// 	return true;
		// }
	}



	bool AStarPlanner::computePath(std::vector<Util::Point>& agent_path, Util::Point start, Util::Point goal, SteerLib::SpatialDataBaseInterface * _gSpatialDatabase, bool append_to_path)
	{
		gSpatialDatabase = _gSpatialDatabase;

		// A* IMPLEMENTATION
		// std::cout << "WeightedA*" << std::endl;
		// bool result = WeightedAStar(agent_path, start, goal, append_to_path);
		// std::cout << "result: " << result << std::endl;
		// for (Util::Point p : agent_path)
		// {
		// 	std::cout << p << std::endl;
		// }

		// ARA* IMPLEMENTATION
		std::cout << "ARA*" << std::endl;
		bool result = ARAStar(agent_path, start, goal, append_to_path);
		std::cout << "result: " << result << std::endl;
		for (Util::Point p : agent_path)
		{
			std::cout << p << std::endl;
		}
		return result;
	}


}