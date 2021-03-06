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
	AStarPlannerNode * startNode = NULL;
	AStarPlannerNode * goalNode = NULL;
	double eps;
	int ARAStar_time_counter;
	int ARAStar_time_limit;

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
		int nGridIndex = n->gridIndex;
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
			// //std::cout << "\nCREATING NEW NODE FOR GRIDINDEX " << gridIndex << std::endl;
			gridIndex_gValuedNodes_map.emplace(gridIndex, AStarPlannerNode(getPointFromGridIndex(gridIndex), DBL_MAX, DBL_MAX, gridIndex, NULL));
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
		while (n != NULL) {
			temp.push_front(n->point);
			n = n->parent;
			// if(n == NULL) {
			// 	//std::cout << "n is null" << std::endl;
			// }
		}

		// Copy contents of deque to path output.
		std::vector<Util::Point> path;
		for (Util::Point p : temp)
		{
			path.push_back(p);
			//std::cout << p << std::endl; // publish path
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
		AStarPlannerNode start = AStarPlannerNode(getPointFromGridIndex(startGridIndex), 0, DBL_MAX, startGridIndex, NULL);
		start.f = fValue(&start, epsilon, goalPoint);
		gridIndex_gValuedNodes_map.emplace(startGridIndex, start);

		// Get grid index of goal point
		int goalGridIndex = gSpatialDatabase->getCellIndexFromLocation(goalPoint.x, goalPoint.z);
		//std::cout << "GOAL GRID INDEX: " << goalGridIndex << std::endl;

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
			//std::cout << "\nCURRENT - Address: " << current << ", Point: " << current->point << ", g: " << current->g << ", f: " << current->f << std::endl;

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

						//std::cout << "\nNEIGHBOR - Address: " << neighbor << ", Point: " << neighbor->point << " COST: " << neighbor->g << " f: " << neighbor->f << std::endl;
						//std::cout << "New parent: " << neighbor->parent << std::endl;
					}
				}
			}
		}

		AStarPlannerNode * goal = openSet.top();

		// //std::cout << "CURRENT (at the end) - Address: " << &current << std::endl;
		// //std::cout << "\nTOP OF OPENSET (at the end) - Address: " << openSet.top() << std::endl;
		// //std::cout << "\nGOAL - Address: " << &goal << ", Cost: " << goal->g << std::endl;
		// //std::cout << "\nGOAL'S PARENT - Address: " << goal->parent << std::endl;

		// Print path (if found)
		//std::cout << "\n PRINTING PATH" << std::endl;
		if (goal->parent == NULL)
		{
			//std::cout << "NO PATH CREATED. GOAL HAS NO PARENT" << std::endl;
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
		// std::cout << "NEW IMPROVE PATH CALL " << std::endl;
		// std::cout << "fValue of goal: " << fValue(goal, epsilon, goal->point) << ", fValue of top: " << fValue(openSet.top(), epsilon, goal->point) << std::endl;
		// std::cout << "Time counter: " << ARAStar_time_counter << ", Time limit: " << ARAStar_time_limit << std::endl;
		// std::cout << "Open set, size: " << openSet.list.size() << std::endl;

		while (openSet.list.size() > 0 && (ARAStar_time_counter++ < ARAStar_time_limit) && (fValue(goal, epsilon, goal->point) > fValue(openSet.top(), epsilon, goal->point)))
		{
			// std::cout << "\nIteration: " << ARAStar_time_counter << ", fValue of goal: " << fValue(goal, epsilon, goal->point) << ", fValue of min(openSet): " << fValue(openSet.top(), epsilon, goal->point) << std::endl;
			
			AStarPlannerNode * current = openSet.top(); // get node from open set with smallest f value
			openSet.pop();
			closedSet.push(current);
			// //std::cout << "CURRENT - address: " << current << ", Point: " << current->point << ", g: " << current->g << ", f: " << current->f << std::endl;

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
						// //std::cout << "NEIGHBOR - Address: " << neighbor << ", Point: " << neighbor->point << " COST: " << neighbor->g << " f: " << neighbor->f << std::endl;
						// //std::cout << "New parent: " << neighbor->parent << std::endl;
						
						if (closedSet.contains(neighbor))
						{
							// neighbor->f = fValue(neighbor, epsilon, goal->point);
							inconsistentSet.push(neighbor);
						}
						else
						{
							// //std::cout << "Adding/updating neighbor..." << std::endl;
							if (openSet.contains(neighbor))
								openSet.remove(neighbor); // remove duplicate to update cost
							neighbor->f = fValue(neighbor, epsilon, goal->point);
							openSet.push(neighbor);
						}
					}
				}
			}
		}
		
		return ARAStar_time_counter < ARAStar_time_limit;
	}


	/* ARA* Main Method */
	bool AStarPlanner::ARAStar(std::vector<Util::Point>& agent_path, Util::Point startPoint, Util::Point goalPoint, bool append_to_path)
	{
		double epsilon = 10;
		double epsilon_decrement = 2.0;
		ARAStar_time_counter = 0; // number of total search iterations allowed in ARAStar_improvePath helper method
		ARAStar_time_limit = 250;

		// Get grid index of goal point
		int goalGridIndex = gSpatialDatabase->getCellIndexFromLocation(goalPoint.x, goalPoint.z);
		AStarPlannerNode * goal = getNodeFromGridIndex(goalGridIndex);
		//std::cout << "GOAL GRID INDEX: " << goalGridIndex << std::endl;

		// Initialize start node
		int startGridIndex = gSpatialDatabase->getCellIndexFromLocation(startPoint.x, startPoint.z);
		AStarPlannerNode start = AStarPlannerNode(getPointFromGridIndex(startGridIndex), 0, DBL_MAX, startGridIndex, NULL);
		start.f = fValue(&start, epsilon, goal->point);
		gridIndex_gValuedNodes_map.emplace(startGridIndex, start);

		// Initialize OPEN Set
		openSet.push(&start); // add start to open set

		bool pathFound = ARAStar_improvePath(epsilon, goal);

		// double suboptimality_bound = std::min(epsilon, (goal->g / std::min(fValue(openSet.top(), 1.0, goal->point), (inconsistentSet.list.size() > 0 ? fValue(inconsistentSet.top(), 1.0, goal->point) : DBL_MAX))));

		if (pathFound)
		{
			std::cout << epsilon << "-suboptimal path found." << std::endl;
			agent_path = reconstructPath(goal);
		}

		bool betterPathFound = true;

		while (betterPathFound && epsilon > 1)
		{
			// decrease epsilon by 1 (minimum = 1)
			epsilon = MAX(epsilon - epsilon_decrement, 1.0);

			// move nodes from inconsistentSet to openSet
			while (inconsistentSet.list.size() > 0)
			{
				openSet.push(inconsistentSet.top());
				inconsistentSet.pop();
			}

			// std::cout << "F Values after sort " << std::endl;
			// update f-values for each node and re-sort
			for (AStarPlannerNode * n : openSet.list)
			{
				n->f = fValue(n, epsilon, goal->point);
				// std::cout << "F Value: " << n->f << ", Point: " << n->point << std::endl;
			}
			openSet.sort();
			// std::cout << "Open set, size: " << openSet.list.size() << std::endl;

			// clear closedSet
			// std::cout << "Closed set, size: " << closedSet.list.size() << std::endl;
			closedSet.list.clear();

			betterPathFound = ARAStar_improvePath(epsilon, goal);

			if (betterPathFound)
			{
				// suboptimality_bound = std::min(epsilon, (goal->g / std::min(fValue(openSet.top(), 1.0, goal->point))));
				std::cout << epsilon << "-suboptimal path found." << std::endl;
				agent_path = reconstructPath(goal);
			}

		}

		if (epsilon <= 1.0)
			std::cout << "Optimal path found." << std::endl;
		else
			std::cout << "Time ran out before optimal path was found." << std::endl;
		
		return pathFound;
	}

	bool AStarPlanner::ADStar(std::vector<Util::Point>& agent_path, Util::Point startPoint, Util::Point goalPoint, bool append_to_path)
	{
		eps = 2.5;
		//std::cout << "AD* Start" << startPoint << std::endl;
		// Initialize start node
		int startGridIndex = gSpatialDatabase->getCellIndexFromLocation(startPoint.x, startPoint.z);
		AStarPlannerNode start = AStarPlannerNode(startPoint, DBL_MAX, -1, startGridIndex, NULL, DBL_MAX, DBL_MAX, DBL_MAX);
		gridIndex_gValuedNodes_map.emplace(startGridIndex, start);
		startNode = &start;
		//std::cout << "startNode = " << startNode->point << std::endl;
		//Initialize goal node
		int goalGridIndex = gSpatialDatabase->getCellIndexFromLocation(goalPoint.x, goalPoint.z);
		AStarPlannerNode goal = AStarPlannerNode(goalPoint, DBL_MAX, -1, goalGridIndex, NULL, 0, 0, 0);
		goalNode = &goal;
		key(goalNode);
		gridIndex_gValuedNodes_map.emplace(goalGridIndex, goal);
		//std::cout << "startNode = " << startNode->point << std::endl;
		//std::cout << "Start/Goal created" << std::endl;

		openSet.push(goalNode);

		//std::cout << "pre-computeorimprovepath" << std::endl;
		ComputeorImprovePath();
		//std::cout << "postcomputeorimprovepath" << std::endl;

		//std::cout << startNode->parent << std::endl;

		AStarPlannerNode * temp = startNode;
		while (temp != NULL) {
			agent_path.push_back(temp->point);
			std::cout << temp->point << ": g=" << temp->g << " rhs=" << temp->rhs << std::endl;
			temp = temp->parent;
		}
		
		return true;
	}

	void AStarPlanner::ComputeorImprovePath()
	{
		int i = 0;
		while (comp(openSet.top(), startNode) || startNode->rhs != startNode->g) {
			//std::cout << i++ << std::endl;
			//std::cout << "Front of compute/ip loop" << std::endl;
			AStarPlannerNode * current = openSet.top();
			openSet.pop();
			std::cout << current->point << ": g=" << current->g << " rhs=" << current->rhs << " index=" << current->gridIndex << std::ends;
			if (current->parent != NULL) {
				std::cout << " parent=" << current->parent->point << std::endl;
			}
			else {
				std::cout << std::endl;
			}
			//std::cout << "before looking at neighbors" << std::endl;
			if (current->g > current->rhs) {
				current->g = current->rhs;
				closedSet.push(current);

				// Get neighbors of current
				std::vector<int> neighborGridIndices = getNeighborGridIndices(current);

				for (int nGridIndex : neighborGridIndices) {
					if (canBeTraversed(nGridIndex)) {

						AStarPlannerNode * neighbor = getNodeFromGridIndex2(nGridIndex);
						neighbor->succ.push(current);
						UpdateState(neighbor);
						
					}
				}
			}
			else {
				current->g = DBL_MAX;

				std::vector<int> neighborGridIndices = getNeighborGridIndices(current);
				neighborGridIndices.push_back(current->gridIndex);

				for (int nGridIndex : neighborGridIndices) {
					if (canBeTraversed(nGridIndex)) {

						AStarPlannerNode * neighbor = getNodeFromGridIndex2(nGridIndex);
						if (nGridIndex != current->gridIndex) {
							neighbor->succ.push(current);
						}
						UpdateState(neighbor);
					}
				}
			}

		}
	}

	void AStarPlanner::UpdateState(AStarPlannerNode * s) {
		if (s != goalNode) {
			//std::cout << "calculating rhs" << std::endl;

			s->rhs = Calculaterhs(s);
		}
		if (openSet.contains(s)) {
			//std::cout << "if in open set..." << std::endl;
			openSet.remove(s);
		}
		//std::cout << s->point << ": g=" << s->g << " rhs=" << s->rhs << std::endl;

		if (s->g != s->rhs) {
			//std::cout << "g and rhs n/e" << std::endl;

			if (!closedSet.contains(s)) {
				//std::cout << "Pushing to open set" << std::endl;
				key(s);
				openSet.push(s);
			}
			else {
				//std::cout << "pushing to incons" << std::endl;

				key(s);
				inconsistentSet.push(s);
			}
		}
	}

	double AStarPlanner::Calculaterhs(AStarPlannerNode * s) {
		double minrhs = DBL_MAX;
		for (AStarPlannerNode * s2 : s->succ.list) {
			//std::cout << h(s2, s) << " " << s2->g << std::endl;
			double temp = h(s2, s) + s2->g;
			if (temp < minrhs) {
				minrhs = temp;
				if (s2->gridIndex != s->gridIndex) {
					s->parent = s2;
				}

			}
		}
		return minrhs;
	}

	double AStarPlanner::h(AStarPlannerNode * s1, AStarPlannerNode * s2) {
		return Util::distanceBetween(s1->point, s2->point);
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

	/* Returns node corresponding to grid index in spatial database
	* If a node has not been created for the given grid index,
	* a new node with default values is created and returned.
	* modified from Taichi's version to work for AD*
	* Justin */
	AStarPlannerNode * AStarPlanner::getNodeFromGridIndex2(int gridIndex)
	{
		// if there is no node corresponding to the grid index, create a new one in the map
		if (gridIndex_gValuedNodes_map.find(gridIndex) == gridIndex_gValuedNodes_map.end())
		{
			 //std::cout << "\nCREATING NEW NODE FOR GRIDINDEX " << gridIndex << std::endl;
			AStarPlannerNode temp = AStarPlannerNode(getPointFromGridIndex(gridIndex), DBL_MAX, -1, gridIndex, NULL, DBL_MAX, DBL_MAX, DBL_MAX);
			if (h(&temp, startNode) < 1) {
				return startNode;
			}
			else if (h(&temp, goalNode) < 1) {
				return goalNode;
			}
			gridIndex_gValuedNodes_map.emplace(gridIndex, temp);
		}

		return &gridIndex_gValuedNodes_map[gridIndex];
	}
	




	bool AStarPlanner::computePath(std::vector<Util::Point>& agent_path, Util::Point start, Util::Point goal, SteerLib::SpatialDataBaseInterface * _gSpatialDatabase, bool append_to_path)
	{
		gSpatialDatabase = _gSpatialDatabase;

		// A* IMPLEMENTATION
		// //std::cout << "WeightedA*" << std::endl;
		// bool result = WeightedAStar(agent_path, start, goal, append_to_path);

		// ARA* IMPLEMENTATION
		std::cout << "ARA*" << std::endl;
		bool result = ARAStar(agent_path, start, goal, append_to_path);

		// AD* IMPLEMENTATION
		//std::cout << "AD*" << std::endl;
		// bool result = ARAStar(agent_path, start, goal, append_to_path);
		return result;
	}


}