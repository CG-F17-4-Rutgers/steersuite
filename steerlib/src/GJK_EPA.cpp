#include "obstacles/GJK_EPA.h"
#include <iostream>
#include <cmath>
#include <algorithm>


SteerLib::GJK_EPA::GJK_EPA()
{
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

    return minkowskiDifference;
}

//Subtract 2 vectors
Util::Vector subtract(Util::Vector a, Util::Vector b) { a.x -= b.x; a.z -= b.z; return a; }

//Gives the perpendicular vector
Util::Vector perpendicular(Util::Vector v) { Util::Vector p = { v.z, 0, -v.x }; return p; }

//Squares the vector
float lengthSquared(Util::Vector v) { return v.x * v.x + v.z * v.z; }

//DotProduct of 2 vectors
float dotProduct(Util::Vector a, Util::Vector b) { return a.x * b.x + a.z * b.z; }

//Cross product of 3 vectors
Util::Vector tripleProduct(Util::Vector a, Util::Vector b, Util::Vector c) {

	Util::Vector r;

	float ac = a.x * c.x + a.z * c.z; // perform a.dot(c)
	float bc = b.x * c.x + b.z * c.z; // perform b.dot(c)

	// perform b * a.dot(c) - a * b.dot(c)
	r.x = b.x * ac - a.x * bc;
	r.z = b.z * ac - a.z * bc;
	return r;
}

bool gjk(std::vector<Util::Vector> shapeA, std::vector<Util::Vector> shapeB, std::vector<Util::Vector>& simplex) {
    
	int iteration = 0;
    std::vector<Util::Vector> minkowskiDifference = calculateMinkowskiDifference(shapeA, shapeB);
	Util::Vector d = minkowskiDifference[0];
	Util::Vector a, b, c, ab, ac, acperp, abperp;
    Util::Vector supportA = getSupport(shapeA, shapeA.size(), d );
    Util::Vector supportB = getSupport(shapeB, shapeB.size(), -d);
	a = supportA - supportB;
	//Sets the first point in the simplex
	simplex.push_back(a);

	d = -a;

	while(iteration < 10) {
		iteration++;

		supportA = getSupport(shapeA, shapeA.size(), d);
		supportB = getSupport(shapeB, shapeB.size(), -d);
		a = supportA - supportB;
		simplex.push_back(a);

		if (dotProduct(a, d) <= 0) {
			return false; // no collision
		}

		// simplex has 2 points (a line segment, not a triangle yet)
		if (simplex.size() < 3) {
			b = simplex[0];
			ab = subtract(b, a); // from point A to B
			d = tripleProduct(ab, -a, ab); // normal to AB towards Origin
			if (lengthSquared(d) == 0) {
				d = perpendicular(ab);
			}
			continue; // skip to next iteration
		}

		b = simplex[1];
		c = simplex[0];
		ab = subtract(b, a); // from point A to B
		ac = subtract(c, a); // from point A to C

		acperp = tripleProduct(ab, ac, ac);

		if (dotProduct(acperp, -a) >= 0) {

			d = acperp; // new direction is normal to AC towards Origin

		}
		else {

			abperp = tripleProduct(ac, ab, ab);

			if (dotProduct(abperp, -a) < 0) {
				return true; // collision
			}
			simplex[0] = simplex[1]; // swap first element (point C)

			d = abperp; // new direction is normal to AB towards Origin
		}

		simplex[1] = simplex[2]; // swap element in the middle (point B)

		simplex.pop_back();

	}

	return false;
}

struct Edge
{
    Util::Vector p1;     // point 1
    Util::Vector p2;     // point 2
    float distance;      // distance from origin
    Util::Vector normal; // normal
    unsigned int index;  // index of point 2 in shape
};

// Returns the closest edge of a simplex s to the origin (as a vector of two points)
Edge findClosestEdge(std::vector<Util::Vector> s)
{
    Edge closest;
    // prime the distance of the edge to the max
    closest.distance = std::numeric_limits<float>::max();
    // float distance = std::numeric_limits<float>::max();
    // float normal;
    // unsigned int closestIndex;
    // s is the passed in simplex
    for (unsigned int i = 0; i < s.size(); i++) {
        // compute the next points index
        unsigned int j = i + 1 == s.size() ? 0 : i + 1;
        // get the current point and the next one
        Util::Vector a = s[i];
        Util::Vector b = s[j];
        // create the edge vector
        Util::Vector e = b - a; // or a.to(b);
        // get the vector from the origin to a
        Util::Vector oa = a; // or a - ORIGIN
        // get the vector from the edge towards the origin
        Util::Vector n = tripleProduct(e, oa, e);
        // normalize the vector
        n = Util::normalize(n);
        // calculate the distance from the origin to the edge
        float d = dotProduct(n, a); // could use b or a here
        // check the distance against the other distances
        if (d < closest.distance) {
            // if this edge is closer then use it
            closest.distance = d;
            closest.normal = n;
            closest.index = j;
        }
    }
// return the closest edge we found
return closest;
}

float TOLERANCE = 0.001;

void epa(float& return_penetration_depth, Util::Vector& return_penetration_vector, std::vector<Util::Vector>& simplex, const std::vector<Util::Vector>& shapeA, const std::vector<Util::Vector>& shapeB)
{
    int iteration = 0;

    // loop to find the collision information
    while (iteration < 10) {
        iteration++;
        // obtain the feature (edge for 2D) closest to the 
        // origin on the Minkowski Difference
        Edge e = findClosestEdge(simplex);
        std::cout << "TEST DEPTH" << e.distance << std::endl;
        // obtain a new support point in the direction of the edge normal
        // Vector p = support(shapeA, shapeB, e.normal);
        Util::Vector support = getSupport(shapeA, shapeA.size(), e.normal) - getSupport(shapeB, shapeB.size(), -e.normal);
        // check the distance from the origin to the edge against the
        // distance p is along e.normal
        float d = dotProduct(support, e.normal);
        std::cout << "D VALUE" << d << std::endl;
        if (d - e.distance < TOLERANCE) {
            // the tolerance should be something positive close to zero (ex. 0.00001)

            // if the difference is less than the tolerance then we can
            // assume that we cannot expand the simplex any further and
            // we have our solution
            return_penetration_vector = e.normal;
            return_penetration_depth = d;
            return;
        }
        else
        {
            // we haven't reached the edge of the Minkowski Difference
            // so continue expanding by adding the new point to the simplex
            // in between the points that made the closest edge
            simplex.insert(simplex.begin() + e.index, support);
        }
    }
    return;
}

//Look at the GJK_EPA.h header file for documentation and instructions
bool SteerLib::GJK_EPA::intersect(float& return_penetration_depth, Util::Vector& return_penetration_vector, const std::vector<Util::Vector>& _shapeA, const std::vector<Util::Vector>& _shapeB)
{

	std::vector<Util::Vector> simplex;

	if (gjk(_shapeA, _shapeB, simplex)) {
		for (Util::Vector pointA : simplex) {
			std::cout << "Simplex: (" << pointA.x << ", " << pointA.z << ")" << std::endl;
		}
        epa(return_penetration_depth, return_penetration_vector, simplex, _shapeA, _shapeB);
		return true;
	}
	else {
		for (Util::Vector pointA : simplex) {
			std::cout << "Simplex: (" << pointA.x << ", " << pointA.z << ")" << std::endl;
		}
		return false;
	}
	//return false; // There is no collision
}

