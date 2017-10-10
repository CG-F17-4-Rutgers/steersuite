#include "obstacles/GJK_EPA.h"
#include <iostream>
#include <cmath>
#include <algorithm>


SteerLib::GJK_EPA::GJK_EPA()
{
}

// 2D cross product of OA and OB vectors, i.e. y-component of their 3D cross product.
// Returns a positive value, if OAB makes a counter-clockwise turn,
// negative for clockwise turn, and zero if the points are collinear.
float cross2D(const Util::Vector &O, const Util::Vector &A, const Util::Vector &B)
{
    return (A.x - O.x) * (B.z - O.z) - (A.z - O.z) * (B.x - O.x);
}

// Compare two vectors lexicographically. First compares x coordinates, and in the case of a tie, compares z coordinates.
// Returns a boolean indicating if point A is "less" than point B
bool compareVector2D(const Util::Vector &pointA, const Util::Vector &pointB)
{
    return pointA.x < pointB.x || (pointA.x == pointB.x && pointA.z <= pointB.z);
}


// Returns the convex hull of a list of points as a new list of points in counter-clockwise order
// The last point in the list is the same as the first one
// Algorithm taken from: https://en.wikibooks.org/wiki/Algorithm_Implementation/Geometry/Convex_hull/Monotone_chain#C.2B.2B
std::vector<Util::Vector> getConvexHull(std::vector<Util::Vector> shape)
{
    // Run getConvexHull
    int n = shape.size(), k = 0;
    if (n == 1) return shape;
    std::vector<Util::Vector> hull(2*n);

    // Sort points lexicographically
    std::sort(shape.begin(), shape.end(), compareVector2D);

    // Build lower hull
    for (int i = 0; i < n; ++i) {
        while (k >= 2 && cross2D(hull[k-2], hull[k-1], shape[i]) <= 0) k--;
        hull[k++] = shape[i];
    }

    // Build upper hull
    for (int i = n-2, t = k+1; i >= 0; i--) {
        while (k >= t && cross2D(hull[k-2], hull[k-1], shape[i]) <= 0) k--;
        hull[k++] = shape[i];
    }

    hull.resize(k-1);
    return hull;
}

// Test to see if a point is inside a given shape
bool isPointInsideShape(Util::Vector point, std::vector<Util::Vector> shape)
{
    std::vector<Util::Vector> hull = getConvexHull(shape); // get the convex hull of the shape
    unsigned int n = hull.size();                                   // store the number of points in the hull
    hull.push_back (point);                                // add the query point to the hull
    hull = getConvexHull(hull);                            // re-evaluate the convex hull
    return (hull.size() == n);                               // if the number of points stayed the same, the point is inside
}


float getXMax(std::vector<Util::Vector> shape) {
    float max = shape[0].x;
    for(Util::Vector v: shape) {
        if(v.x > max) {
            max = v.x;
        }
    }

    return max;
}

bool containsOrigin(std::vector<Util::Vector> shape) {

    //Util::Ray originRay;
    //originRay.initWithLengthInterval(Util::Point(), Util::Vector(getXMax(shape), 0.0f, 0.0f));
    Util::Point intersectionPoint;
    int intersections = 0;
    shape.push_back(shape[0]);
    for(int i = 0; i < shape.size() - 1; i++) {
        if(Util::intersect2Lines2D(Util::Point(), Util::Point(getXMax(shape), 0.0f, 0.0f), 
        Util::Point(shape[i].x, shape[i].y, shape[i].z), Util::Point(shape[i+1].x, shape[i+1].y, shape[i+1].z), intersectionPoint)) {
            intersections++;
        }
    }

    return intersections % 2 == 1 ? true : false;
}

Util::Vector getSupport(std::vector<Util::Vector> shape, int count, Util::Vector d) {
    float highest = std::numeric_limits<float>::max() * -1;
    Util::Vector support;

    for(int i = 0; i < count; ++i) {
        Util::Vector v = shape[i];
        float dot = v.x * d.x + v.z * d.z;

        if(dot > highest) {
            highest = dot;
            support = v;
        }
    }

    return support;
}

std::vector<Util::Vector> calculateMinkowskiDifference(std::vector<Util::Vector> shapeA, std::vector<Util::Vector> shapeB) {
    std::vector<Util::Vector> minkowskiDifference;

    for(Util::Vector pointA: shapeA) {
        for(Util::Vector pointB: shapeB) {
            minkowskiDifference.push_back(pointA - pointB);
        }
    }

    /*
    std::cout << "shapeA: ";
    for(Util::Vector pointA: shapeA) {
        std::cout << "(" << pointA.x << ", " << pointA.y << ", " << pointA.z << ")" << std::endl;
    }

    std::cout << "shapeB: ";
    for(Util::Vector pointB: shapeB) {
        std::cout << "(" << pointB.x << ", " << pointB.y << ", " << pointB.z << ")" << std::endl;
    }

    std::cout << "Minkowski Difference: ";

    for(Util::Vector v: minkowskiDifference) {
        std::cout << "(" << v.x << ", " << v.y << ", " << v.z << ")" << std::endl;
    } 
    */
    return minkowskiDifference;
}

// Removes the point that is furthest from the origin
void removeFarthestPoint(std::vector<Util::Vector>& simplex) {
    float furthestDistance = 0;
    int furthestDistanceIndex = 0;

    for(int i = 0; i < simplex.size(); i++) {
        float distanceFromOrigin = Util::distanceBetween(Util::Point(), Util::Point(simplex[i].x, simplex[i].y, simplex[i].z));
        if(distanceFromOrigin >= furthestDistance) {
            furthestDistance = distanceFromOrigin;
            furthestDistanceIndex = i;
        }
    }

    simplex.erase(simplex.begin() + furthestDistanceIndex);
}

//Subtract 2 vectors
Util::Vector subtract(Util::Vector a, Util::Vector b) { a.x -= b.x; a.y -= b.y; return a; }

Util::Vector tripleProduct(Util::Vector a, Util::Vector b, Util::Vector c) {

	Util::Vector r;

	float ac = a.x * c.x + a.y * c.y; // perform a.dot(c)
	float bc = b.x * c.x + b.y * c.y; // perform b.dot(c)

	// perform b * a.dot(c) - a * b.dot(c)
	r.x = b.x * ac - a.x * bc;
	r.y = b.y * ac - a.y * bc;
	return r;
}

bool gjk(std::vector<Util::Vector> shapeA, std::vector<Util::Vector> shapeB, std::vector<Util::Vector>& simplex) {
    
	int iteration = 0;
    std::vector<Util::Vector> minkowskiDifference = calculateMinkowskiDifference(shapeA, shapeB);
	Util::Vector d = minkowskiDifference[0];
	Util::Vector a, b, ab;
    Util::Vector supportA = getSupport(shapeA, shapeA.size(), d );
    Util::Vector supportB = getSupport(shapeB, shapeB.size(), -d);
    Util::Vector w = supportA - supportB;
    std::cout << "md[0]: (" << minkowskiDifference[0].x << ", " << w.y << ", " << minkowskiDifference[0].z << ")" << std::endl;
    std::cout << "w0: (" << w.x << ", " << w.y << ", " << w.z << ")" << std::endl;
	simplex.push_back(w);
	d = -w;
    supportA = getSupport(shapeA, shapeA.size(), d);
    supportB = getSupport(shapeB, shapeB.size(), -d);
    w = supportA - supportB;
    std::cout << "w1: (" << w.x << ", " << w.y << ", " << w.z << ")" << std::endl;
	simplex.push_back(w);

	a = simplex[0];
	b = simplex[1];
	ab = subtract(a, b);
	d = tripleProduct(ab, -a, ab);

    supportA = getSupport(shapeA, shapeA.size(), d);
    supportB = getSupport(shapeB, shapeB.size(), -d);
    w = supportA - supportB;
    std::cout << "w2: (" << w.x << ", " << w.y << ", " << w.z << ")" << std::endl;
	simplex.push_back(w);

	if (isPointInsideShape(Util::Vector(0, 0, 0), simplex)) {
		for (Util::Vector pointA : simplex) {
			std::cout << "Simplex: (" << pointA.x << ", " << pointA.z << ")" << std::endl;
		}
		std::cout << "First 3" << std::endl;
		return true;
	}

	while (iteration < 10) {

		std::cout << "Iteration: " << iteration << std::endl;

		simplex[0] = simplex[1];
		simplex[1] = simplex[2];

		a = simplex[0];
		b = simplex[1];
		ab = subtract(a, b);
		d = tripleProduct(ab, -a, ab);
		supportA = getSupport(shapeA, shapeA.size(), d);
		supportB = getSupport(shapeB, shapeB.size(), -d);
		w = supportA - supportB;
		simplex.pop_back();
		simplex.push_back(w);

		if (isPointInsideShape(Util::Vector(0, 0, 0), simplex)) {
			return true;
		}

		iteration++;
	}

	return false;



    /*for (Util::Vector pointA : shapeA) {
        std::cout << "Shape A: (" << pointA.x << ", " << pointA.z << ")" << std::endl;
    }
    for (Util::Vector pointB : shapeB) {
        std::cout << "Shape B: (" << pointB.x << ", " << pointB.z << ")" << std::endl;
    }
    std::cout << "sA: (" << supportA.x << ", " << w.y << ", " << supportA.z << ")" << std::endl;
    std::cout << "sB: (" << supportB.x << ", " << w.y << ", " << supportB.z << ")" << std::endl;
    std::cout << "w: (" << w.x << ", " << w.y << ", " << w.z << ")" << std::endl;
    std::cout << "md[0]: (" << minkowskiDifference[0].x << ", " << w.y << ", " << minkowskiDifference[0].z << ")" << std::endl;
    */
    return false;
}

//Look at the GJK_EPA.h header file for documentation and instructions
bool SteerLib::GJK_EPA::intersect(float& return_penetration_depth, Util::Vector& return_penetration_vector, const std::vector<Util::Vector>& _shapeA, const std::vector<Util::Vector>& _shapeB)
{
    // test Convex Hull
    std::cout << "Testing Convex Hull. Remove Tests before submitting assignment!" << std::endl;
    std::vector<Util::Vector> frankenshape;
    for (Util::Vector point : _shapeA)
        frankenshape.push_back(point);
    for (Util::Vector point : _shapeB)
        frankenshape.push_back(point);
    frankenshape = getConvexHull(frankenshape);
    for (Util::Vector point : frankenshape)
        std::cout << point;
    std::cout << std::endl;
    std::cout << "Is (2.8, 0.5) inside the convex hull?" << isPointInsideShape(Util::Vector(2.8, 0, 0.5), frankenshape) << std::endl; // should be false
    std::cout << "Is (0.8, 2.3) inside the convex hull?" << isPointInsideShape(Util::Vector(0.8, 0, 2.3), frankenshape) << std::endl; // should be true

    std::cout << "Testing GJK. Remove Tests before submitting assignment!" << std::endl;


	std::vector<Util::Vector> simplex;
	return_penetration_depth = 0;
	return_penetration_vector = Util::Vector(0, 0, 0);

	return gjk(_shapeA, _shapeB, simplex);
	//return false; // There is no collision
}

