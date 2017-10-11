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

//Look at the GJK_EPA.h header file for documentation and instructions
bool SteerLib::GJK_EPA::intersect(float& return_penetration_depth, Util::Vector& return_penetration_vector, const std::vector<Util::Vector>& _shapeA, const std::vector<Util::Vector>& _shapeB)
{

	std::vector<Util::Vector> simplex;

	if (gjk(_shapeA, _shapeB, simplex)) {
		for (Util::Vector pointA : simplex) {
			std::cout << "Simplex: (" << pointA.x << ", " << pointA.z << ")" << std::endl;
		}
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

