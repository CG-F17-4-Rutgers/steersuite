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

	// Calculates the Manhattan distance between two points.
	int calcManhattanDistance(Util::Point start, Util::Point goal) {
		return std::abs(start.x - goal.x) + std::abs(start.z - goal.z);
	}

	// Calculates the f-value of a point n
	int calcFValue(Util::Point start, Util::Point goal, Util::Point n, int epsilon) {
		// TODO: Add g value 
		return epsilon * calcManhattanDistance(n, goal);
	}





	/* Implemented f-value based on euclidean distance heuristic
	 * UNTESTED - Taichi */
	float calcEuclideanDistance(Util::Point p1, Util::Point p2) {
		return Util::distanceBetween(p1, p2);
	}
	float calcFValueEuclidean(AStarPlannerNode n, double epsilon, AStarPlannerNode goal) {
		return n.g + epsilon * calcEuclideanDistance(n.point, goal.point);
	}

	/* Given an AStarPlannerNode n, returns the grid-indices of grid-cells neighboring that of n
	 * UNTESTED - Taichi */
	std::vector<int> AStarPlanner::getNeighborGridIndices(AStarPlannerNode n)
	{
		std::vector<int> neighborGridIndices;
		for (int i = -1; i <= 1; i++) {
			for (int j = -1; j <= 1; j++) {
				int temp = gSpatialDatabase->getCellIndexFromLocation(n.point.x + i, n.point.z + j);
				neighborGridIndices.push_back(temp);
			}
		}
		return neighborGridIndices;
	}

	/* Computes cost between node and its neighbor. Simple Euclidean distance. Higher cost for diagonal neighbors. 
	 * UNTESTED - Taichi */
	double computeCost(AStarPlannerNode n, AStarPlannerNode neighbor)
	{
		return Util::distanceBetween(n.point, neighbor.point);
	}


	/* Helper function for ARA* implementation */
	void AStarPlanner::ARAStar_improvePath(double epsilon, AStarPlannerNode goal)
	{
		while (openSet.size() > 0 && goal.f > openSet.top().f)
		{
			AStarPlannerNode n = openSet.top(); // get node from open set with smallest f value
			openSet.pop();
			closedSet.push_back(n);

			std::vector<int> neighborGridIndices = getNeighborGridIndices(n); // get grid indices of neighboring cells
			
			for (int neighborGridIndex : neighborGridIndices) {
				if (canBeTraversed(neighborGridIndex)) {
					AStarPlannerNode neighbor;
					
					// if there is no g value for this neighbor, create a new AStarPlannerNode and store it.
					try {
						neighbor = gridIndex_gValuedNodes_map.at(neighborGridIndex);
					}
					catch (const std::out_of_range& oor) {
						neighbor = AStarPlannerNode(getPointFromGridIndex(neighborGridIndex), DBL_MAX, DBL_MAX, 0);
						gridIndex_gValuedNodes_map.emplace(neighborGridIndex, neighbor);
					}

					// compute cost from n to neighbor (diagonal costs more)
					double totalValue = n.g + computeCost(n, neighbor);

					// relax neighbor if better value is found
					if (totalValue < neighbor.g) {
						neighbor.g = totalValue;
						neighbor.parent = &n;
						if (std::find(closedSet.begin(), closedSet.end(), neighbor) == closedSet.end()) // if closedSet doesn't contain neighbor
						{
							neighbor.f = calcFValueEuclidean(neighbor, epsilon, goal);
							openSet.push(neighbor);
						} else { // if closedSet contains neighbor
							inconsistentSet.push(neighbor);
						}
					}
				}
			}
		}
	}

	bool AStarPlanner::ARAStar(std::vector<Util::Point>& agent_path, Util::Point startPoint, Util::Point goalPoint, bool append_to_path)
	{

		double epsilon = 2.5;
		AStarPlannerNode goal = AStarPlannerNode(goalPoint, DBL_MAX, DBL_MAX, 0);
		AStarPlannerNode start = AStarPlannerNode(startPoint, 0, DBL_MAX, 0);

		// Populate spatial database points
		start.f = calcFValueEuclidean(start, epsilon, goal);
		openSet.push(start);

		ARAStar_improvePath(epsilon, goal);

		double suboptimality_bound = MIN(epsilon, (goal.g / MIN(calcFValueEuclidean(openSet.top(), 1.0, goal), calcFValueEuclidean(inconsistentSet.top(), 1.0, goal))));

		while (suboptimality_bound > 1.0)
		{
			epsilon = MAX(epsilon - 1.0, 1.0); // decrease epsilon by 1
			std::priority_queue< AStarPlannerNode, std::vector<AStarPlannerNode>, std::greater<AStarPlannerNode> > newOpenSet;
			for (unsigned int i = 0; i < inconsistentSet.size(); i++)
			{
				AStarPlannerNode n = inconsistentSet.top();
				inconsistentSet.pop();
				n.f = calcFValueEuclidean(n, epsilon, goal);
				newOpenSet.push(n);
			}
			for (unsigned int i = 0; i < openSet.size(); i++)
			{
				AStarPlannerNode n = openSet.top();
				openSet.pop();
				n.f = calcFValueEuclidean(n, epsilon, goal);
				newOpenSet.push(n);
			}
			openSet = newOpenSet;
			closedSet.clear(); // does this work, who knows.

			ARAStar_improvePath(epsilon, goal);

			suboptimality_bound = MIN(epsilon, (goal.g / MIN(calcFValueEuclidean(openSet.top(), 1.0, goal), calcFValueEuclidean(inconsistentSet.top(), 1.0, goal))));
		}

		if (goal.parent == NULL)
		{
			return false; // goal has no parent, so it was never reached
		}
		else
		{
			std::deque<AStarPlannerNode*> path;
			AStarPlannerNode* n = &goal;
			while (n != NULL)
			{
				path.push_front(n);
				n = n->parent;
			}
			std::vector<Util::Point> pathPoints;
			for (AStarPlannerNode* n : path)
				pathPoints.push_back(n->point);
			if (append_to_path)
				agent_path.insert(agent_path.end(), pathPoints.begin(), pathPoints.end());
			else 
				agent_path = pathPoints;
			return true;
		}
	}


	bool AStarPlanner::computePath(std::vector<Util::Point>& agent_path, Util::Point start, Util::Point goal, SteerLib::SpatialDataBaseInterface * _gSpatialDatabase, bool append_to_path)
	{
		gSpatialDatabase = _gSpatialDatabase;

		// //TODO
		// std::cout<<"\nIn A*";

		std::cout << "how the fuck do i get a print message to show up god fucking damn it." << std::endl;

		// std::vector<Util::Point> open;
		// std::vector<Util::Point> closed;
		// int epsilon = 1;
		// int g = 0;
		// //int f = 

		// open.push_back(start);

		return ARAStar(agent_path, start, goal, append_to_path);

		// return false;

		// ARA* IMPLEMENTATION - Taichi
		// std::cout << "ARA*" << std::endl;

		// for (Util::Point p : agent_path)
		// {
		// 	std::cout << p << std::endl;
		// }
		// return true;
		// return ARAStar(agent_path, start, goal, append_to_path);
	}


}