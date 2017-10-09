#include "obstacles/GJK_EPA.h"
#include <iostream>
#include <cmath>


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

// Returns the convex hull of a list of points as a new list of points in counter-clockwise order
// The last point in the list is the same as the first one
// Algorithm taken from: https://en.wikibooks.org/wiki/Algorithm_Implementation/Geometry/Convex_hull/Monotone_chain#C.2B.2B
std::vector<Util::Vector> getConvexHull(std::vector<Util::Vector> shape)
{
    int n = shape.size(), k = 0;
    if (n == 1) return shape;
    std::vector<Util::Vector> hull(2*n);

    // Sort points lexicographically
    sort(shape.begin(), shape.end());

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
    int n = hull.size();                                   // store the number of points in the hull
    hull.push_back (point);                                // add the query point to the hull
    hull = getConvexHull(hull);                            // re-evaluate the convex hull
    return (hull.size == n);                               // if the number of points stayed the same, the point is inside
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
    float highest = std::numeric_limits<float>::min();
    Util::Vector support;

    for(int i = 0; i < count; ++i) {
        Util::Vector v = shape[i];
        float dot = v.x * d.x+ v.y * d.y;

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

//Look at the GJK_EPA.h header file for documentation and instructions
bool SteerLib::GJK_EPA::intersect(float& return_penetration_depth, Util::Vector& return_penetration_vector, const std::vector<Util::Vector>& _shapeA, const std::vector<Util::Vector>& _shapeB)
{
    std::vector<Util::Vector> minkowskiDifference = calculateMinkowskiDifference(_shapeA, _shapeB);
    Util::Vector supportA = getSupport(_shapeA, _shapeA.size(), minkowskiDifference[0] * -1);
    Util::Vector supportB = getSupport(_shapeB, _shapeB.size(), minkowskiDifference[0]);
    Util::Vector w = supportA - supportB;

    if(w.norm() == minkowskiDifference[0].norm()) {
        // Shapes intersect
        return_penetration_depth = minkowskiDifference[0].norm();
        return true;
    }

    std::vector<Util::Vector> simplex;
    simplex.push_back(w);

    if(containsOrigin(simplex)) {
        // Shapes intersect
        return true;
    }

	return false; // There is no collision
}

