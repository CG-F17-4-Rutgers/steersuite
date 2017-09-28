//
// Copyright (c) 2009-2015 Glen Berseth, Mubbasir Kapadia, Shawn Singh, Petros Faloutsos, Glenn Reinman
// See license.txt for complete license.
// Copyright (c) 2015 Mahyar Khayatkhoei
//

#include <algorithm>
#include <vector>
#include <math.h>
#include <cmath>
#include <util/Geometry.h>
#include <util/Curve.h>
#include <util/Color.h>
#include <util/DrawLib.h>
#include "Globals.h"

using namespace Util;

Curve::Curve(const CurvePoint& startPoint, int curveType) : type(curveType)
{
	controlPoints.push_back(startPoint);
}

Curve::Curve(const std::vector<CurvePoint>& inputPoints, int curveType) : type(curveType)
{
	controlPoints = inputPoints;
	sortControlPoints();
}

// Add one control point to the vector controlPoints
void Curve::addControlPoint(const CurvePoint& inputPoint)
{
	controlPoints.push_back(inputPoint);
	sortControlPoints();
}

// Add a vector of control points to the vector controlPoints
void Curve::addControlPoints(const std::vector<CurvePoint>& inputPoints)
{
	for (int i = 0; i < inputPoints.size(); i++)
		controlPoints.push_back(inputPoints[i]);
	sortControlPoints();
}

// Draw the curve shape on screen, usign window as step size (bigger window: less accurate shape)
void Curve::drawCurve(Color curveColor, float curveThickness, int window)
{
#ifdef ENABLE_GUI

	// Robustness: make sure there is at least two control point: start and end points
	// Move on the curve from t=0 to t=finalPoint, using window as step size, and linearly interpolate the curve points
	// Note that you must draw the whole curve at each frame, that means connecting line segments between each two points on the curve
	if (!checkRobust())
		return;

	float t_i = controlPoints[0].time + (float)window;
	const float t_final = controlPoints[controlPoints.size() - 1].time;

	Point p_i_minus_1 = controlPoints[0].position;
	Point p_i;

	while (t_i <= t_final)
	{
		calculatePoint(p_i, t_i);
		DrawLib::drawLine(p_i_minus_1, p_i, curveColor, curveThickness);
		p_i_minus_1 = p_i;
		t_i = t_i + window;
	}
	

	return;
#endif
}

// Compare two control points and return boolean indicating if point A is an earlier time than point B
bool compareControlPoints(const CurvePoint& pointA, const CurvePoint& pointB)
{
	return (pointA.time < pointB.time);
}

// Sort controlPoints vector in ascending order: min-first
void Curve::sortControlPoints()
{
	
	// FOR TESTING -- before sort
	
	/*for (CurvePoint pt : controlPoints)
		std::cout << pt.time << " ";
	std::cout << std::endl;*/
	

	// Sort vector of control points
	std::sort(controlPoints.begin(), controlPoints.end(), compareControlPoints);

	// FOR TESTING -- after sort
	/*for (CurvePoint pt : controlPoints)
	{
		std::cout << pt.time << " ";
		std::cout << pt.position << " ";
		std::cout << pt.tangent << " ";
		std::cout << std::endl;
	}*/
	
}

// Calculate the position on curve corresponding to the given time, outputPoint is the resulting position
// Note that this function should return false if the end of the curve is reached, or no next point can be found
bool Curve::calculatePoint(Point& outputPoint, float time)
{
	// Robustness: make sure there is at least two control point: start and end points
	if (!checkRobust())
		return false;

	// Define temporary parameters for calculation
	unsigned int nextPoint;

	// Find the current interval in time, supposing that controlPoints is sorted (sorting is done whenever control points are added)
	// Note that nextPoint is an integer containing the index of the next control point
	if (!findTimeInterval(nextPoint, time))
		return false;

	// Calculate position at t = time on curve given the next control point (nextPoint)
	if (type == hermiteCurve)
	{
		outputPoint = useHermiteCurve(nextPoint, time);
	}
	else if (type == catmullCurve)
	{
		outputPoint = useCatmullCurve(nextPoint, time);
	}

	// Return
	return true;
}

// Check Roboustness
bool Curve::checkRobust()
{
	//================DELETE THIS PART AND THEN START CODING===================
	/*static bool flag = false;
	if (!flag)
	{
		std::cerr << "ERROR>>>>Member function checkRobust is not implemented!" << std::endl;
		flag = true;
	}*/
	//=========================================================================

	return controlPoints.size() >= 2;

	// return true;
}

// Find the current time interval (i.e. index of the next control point to follow according to current time)
bool Curve::findTimeInterval(unsigned int& nextPoint, float time)
{
	// assign nextPoint the index of the next control point. return false if there is no next point
	// using binary search: terminate if exact time is found. otherwise, let it rock and the resulting lo will be the index of the next point. 
	// std::cout << "Looking for next point after time " << time << std::endl;
	unsigned int lo = 0, mid, hi = controlPoints.size() - 1;
	while (lo <= hi)
	{
		mid = (lo + hi) / 2;
		if (controlPoints[mid].time < time)
			lo = mid + 1;
		else if (controlPoints[mid].time > time)
			hi = mid - 1;
		else
		{
			nextPoint = mid + 1; // found point at exact time, so return next point
			// std::cout << "BS Next point found. Its index is: " << nextPoint << std::endl;
			return true;
		}
	}

	if (lo >= controlPoints.size())
	{
		// std::cout << "No next point found." << std::endl;
		return false; // no next point
	}
	else
	{
		nextPoint = lo;
		// std::cout << "Next point found. Its index is: " << nextPoint << std::endl;
	}

	return true;
}

// Implement Hermite curve
Point Curve::useHermiteCurve(const unsigned int nextPoint, const float time)
{
	Point newPosition;
	float normalTime, intervalTime;

	const unsigned int currentPoint = nextPoint - 1;
	assert(nextPoint >= 1 && currentPoint < controlPoints.size());

	// Convert time to t | 0<=t<=1
	intervalTime = controlPoints[nextPoint].time - controlPoints[currentPoint].time;
	normalTime = (time - controlPoints[currentPoint].time) / intervalTime;
	// std::cout << time << " " << currentPoint << " " << controlPoints[currentPoint].time << std::endl;
	// std::cout << "normalized time: " << normalTime << std::endl;

	// Calculate position at t = time on Hermite curve
	// assert(time >= 0 && time <= 1); // the following depends on this to be true. if it's not, set const float t = (time - controlPoints[currentPoint].time) / (controlPoints[nextPoint].time - controlPoints[currentPoint].time);
	const float t = normalTime;
	const float t3 = std::pow(t, 3);
	const float t2 = std::pow(t, 2);

	// Using hermite curve formula
	newPosition = (2 * t3 - 3 * t2 + 1) * controlPoints[currentPoint].position
		+ (-2 * t3 + 3 * t2) * controlPoints[nextPoint].position
		+ (t3 - 2 * t2 + t) * controlPoints[currentPoint].tangent
		+ (t3 - t2) * controlPoints[nextPoint].tangent;

	// Return result
	return newPosition;
}

// Implement Catmull-Rom curve
Point Curve::useCatmullCurve(const unsigned int nextPoint, const float time)
{
	// Check if we've been given a well-formed call
	const unsigned int n = controlPoints.size();
	assert(nextPoint >= 1 && nextPoint < n);

	// Generation of Catmull-Rom point requires four neighboring control points (two ahead and two behind) (http://algorithmist.net/docs/catmullrom.pdf)

	// Get immediate neighbors
	Point p[4];
	p[1] = controlPoints[nextPoint - 1].position;
	p[2] = controlPoints[nextPoint].position;

	// Get farther neighbors
	// If we are next to an endpoint, we'll have to create an auxiliary control point beyond that endpoint.
	// For the auxiliary endpoint, we'll simply use the negative of the tangent stored in the endpoint.
	if ((int) nextPoint - 2 < 0)
		p[0] = Point(-1 * controlPoints[0].tangent.x, -1 * controlPoints[0].tangent.y, -1 * controlPoints[0].tangent.z);
	else
		p[0] = controlPoints[nextPoint - 2].position;
	if (nextPoint + 1 >= n)
		p[3] = Point(-1 * controlPoints[n - 1].tangent.x, -1 * controlPoints[n - 1].tangent.y, -1 * controlPoints[n - 1].tangent.z);
	else
		p[3] = controlPoints[nextPoint + 1].position;


	// Convert time to t | 0<=t<=1
	float intervalTime = controlPoints[nextPoint].time - controlPoints[nextPoint - 1].time;
	float normalTime = (time - controlPoints[nextPoint - 1].time) / intervalTime;

	// Calculate position at t = time on Catmull-Rom curve
	const float t = normalTime;
	const float t2 = std::pow(t, 2);
	const float t3 = std::pow(t, 3);


	Point newPosition = 0.5 * (
		(2 * p[1])
		+ ((-1 * p[0]) + p[2]) * t
		+ ((2 * p[0]) + (-5 * p[1]) + (4 * p[2]) + (-1 * p[3])) * t2
		+ ((-1 * p[0]) + (3 * p[1]) + (-3 * p[2]) + p[3]) * t3 );
	
	// Return result
	return newPosition;
}