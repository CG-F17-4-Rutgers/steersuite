#include "obstacles/GJK_EPA.h"
#include <iostream>
#include <cmath>
#include <algorithm>

SteerLib::GJK_EPA::GJK_EPA()
{
}

#include <cstdint>
#include <list>
#include <vector>
#include <algorithm>

#include <util/DrawLib.h>
#include <util/Color.h>

/* Polygon triangle decomposition package from https://gist.github.com/Shaptic/6526805 */
class Triangulation {
    // This code uses C++11 features like "auto" and initializer lists.

public:
    // Compare two floating point numbers accurately.
    static bool compf(float a, float b, float threshold=0.00001f)
    {
        return (a + threshold > b && a - threshold < b);
    }

    // A point in space. 
    struct vector_t 
    {
        float x, y;
        inline bool operator==(const vector_t& B)
        { return compf(B.x, x) && compf(B.y, y); }
    };

    // Determine orientation of a triangle.
    // TRUE if ccw, FALSE if cw or not a triangle.
    static bool orientation(const std::vector<vector_t>& p)
    {
        return (p[1].x - p[0].x) * (p[2].y - p[0].y) - 
               (p[2].x - p[0].x) * (p[1].y - p[0].y) > 0;
    }

    // Barycentric coordinate calculation.
    static bool in_triangle(const vector_t& V, const vector_t& A,
                     const vector_t& B, const vector_t& C)
    {
        float denom = ((B.y - C.y) * (A.x - C.x) + (C.x - B.x) * (A.y - C.y));
        if(compf(denom, 0.0)) return true;
        denom = 1 / denom;
        
        float alpha = denom * ((B.y - C.y) * (V.x - C.x) + (C.x - B.x) * (V.y - C.y));
        if(alpha < 0) return false;
     
        float beta  = denom * ((C.y - A.y) * (V.x - C.x) + (A.x - C.x) * (V.y - C.y));
        if(beta < 0) return false;
        
        return alpha + beta >= 1;
    }

    static std::vector<vector_t> triangulate(std::vector<vector_t> Polygon)
    {
        std::vector<uint16_t> reflex;
        std::vector<vector_t> triangles;    
        
        if(Polygon.size() < 3) return Polygon;
        
        // Polygon orientation
        vector_t left = Polygon[0];
        size_t index = 0;
        
        for(size_t i = 0; i < Polygon.size(); ++i)
        {
            if(Polygon[i].x < left.x ||
              (compf(Polygon[i].x, left.x) && Polygon[i].y < left.y))
            {
                index = i;
                left = Polygon[i];
            }
        }
        
        // C++11 initializer list (not on <= MSVC11)
        std::vector<vector_t> tri {
            Polygon[(index > 0) ? index - 1 : Polygon.size() - 1],
            Polygon[index],
            Polygon[(index < Polygon.size()) ? index + 1 : 0]
        };
        bool ccw = orientation(tri);
        
        // We know there will be vertex_count - 2 triangles made.
        triangles.reserve(Polygon.size() - 2);

        if(Polygon.size() == 3) return Polygon;
        while(Polygon.size() >= 3)
        {
            reflex.clear();
            int16_t eartip = -1, index = -1;
            for(auto& i : Polygon)
            {
                ++index;
                if(eartip >= 0) break;
                
                uint16_t p = (index > 0) ? index - 1 : Polygon.size() - 1;
                uint16_t n = (index < Polygon.size()) ? index + 1 : 0;
                
                std::vector<vector_t> tri { Polygon[p], i, Polygon[n] };
                if(orientation(tri) != ccw)
                {
                    reflex.emplace_back(index);
                    continue;
                }
                
                bool ear = true;
                for(auto& j : reflex)
                {
                    if(j == p || j == n) continue;
                    if(in_triangle(Polygon[j], Polygon[p], i, Polygon[n]))
                    {
                        ear = false;
                        break;
                    }
                }
                
                if(ear)
                {
                    auto j = Polygon.begin() + index + 1,
                         k = Polygon.end();
                     
                    for( ; j != k; ++j)
                    {
                        auto& v = *j;

                        if(&v == &Polygon[p] ||
                           &v == &Polygon[n] ||
                           &v == &Polygon[index]) continue;

                        if(in_triangle(v, Polygon[p], i, Polygon[n]))
                        {
                            ear = false;
                            break;
                        }
                    }
                }
                
                if(ear) eartip = index;
            }
            
            if(eartip < 0) break;
            
            uint16_t p = (eartip > 0) ? eartip - 1 : Polygon.size() - 1;
            uint16_t n = (eartip < Polygon.size()) ? eartip + 1 : 0;
            vector_t* parts[3] = { 
                &Polygon[p], &Polygon[eartip], &Polygon[n]
            };
            
            // Create the triangulated piece.
            for(const auto& i : parts) triangles.push_back(*i);
            
            // Clip the ear from the polygon.
            Polygon.erase(std::find(Polygon.begin(), Polygon.end(), *parts[1]));
        }
        
        return triangles;
    }
};


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
        //std::cout << "TEST DEPTH" << e.distance << std::endl;

        // obtain a new support point in the direction of the edge normal
        // Vector p = support(shapeA, shapeB, e.normal);
        Util::Vector support = getSupport(shapeA, shapeA.size(), e.normal) - getSupport(shapeB, shapeB.size(), -e.normal);
        // check the distance from the origin to the edge against the
        // distance p is along e.normal
        float d = dotProduct(support, e.normal);
        
        // std::cout << "D VALUE" << d << std::endl;
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


struct Triangle
{
    std::vector<Util::Vector> points;
};

std::vector<Triangle> getTriangles(std::vector<Util::Vector> shape)
{
    std::vector<Triangulation::vector_t> shape_as_vector_t;
    for (Util::Vector v: shape)
    {
        Triangulation::vector_t point;
        point.x = v.x;
        point.y = v.z;

        shape_as_vector_t.push_back(point);
    }
    std::vector<Triangulation::vector_t> triangles_as_vector_t = Triangulation::triangulate(shape_as_vector_t);
    std::vector<Triangle> triangles;
    for (unsigned int i = 0; i < triangles_as_vector_t.size(); i += 3)
    {
        Triangle t;
        t.points.push_back(Util::Vector(triangles_as_vector_t[i].x, 0, triangles_as_vector_t[i].y));
        t.points.push_back(Util::Vector(triangles_as_vector_t[i+1].x, 0, triangles_as_vector_t[i+1].y));
        t.points.push_back(Util::Vector(triangles_as_vector_t[i+2].x, 0, triangles_as_vector_t[i+2].y));
        triangles.push_back(t);
    }
    return triangles;
}

// Doesn't work for some reason
void drawTriangle(Triangle t)
{
    Util::Point a = Util::Point(t.points[0].x, 0, t.points[0].z);
    Util::Point b = Util::Point(t.points[1].x, 0, t.points[1].z);
    Util::Point c = Util::Point(t.points[2].x, 0, t.points[2].z);
    Util::DrawLib::drawLine(a,b,Util::gRed,2.0f);
    Util::DrawLib::drawLine(b,c,Util::gRed,2.0f);
    Util::DrawLib::drawLine(c,a,Util::gRed,2.0f);
}


// Performs a triangle decomposition on input shapes to detect collisions for concave shapes
bool gjk_concave(std::vector<Util::Vector> shapeA, std::vector<Util::Vector> shapeB)
{
    std::vector<Triangle> trianglesA = getTriangles(shapeA);
    std::vector<Triangle> trianglesB = getTriangles(shapeB);

    std::cout << "Triangles for shape A" << std::endl;
    std::cout << trianglesA.size() << std::endl;

    for (int i = 0; i < trianglesA.size(); i++)
    {
        std::cout << trianglesA[i].points[0] << " " << trianglesA[i].points[1] << " " << trianglesA[i].points[2] << std::endl;
    }

    std::cout << "Triangles for shape B" << std::endl;
    std::cout << trianglesB.size() << std::endl;
    for (int i = 0; i < trianglesB.size(); i++)
    {
        std::cout << trianglesB[i].points[0] << " " << trianglesB[i].points[1] << " " << trianglesB[i].points[2] << std::endl;
    }

    for (Triangle tA : trianglesA)
    {
        for (Triangle tB : trianglesB)
        {
            std::vector<Util::Vector> simplex;
            if (gjk(tA.points, tB.points, simplex))
            {
                return true;
            }
        }
    }
    return false;
}


//Look at the GJK_EPA.h header file for documentation and instructions
bool SteerLib::GJK_EPA::intersect(float& return_penetration_depth, Util::Vector& return_penetration_vector, const std::vector<Util::Vector>& _shapeA, const std::vector<Util::Vector>& _shapeB)
{   

    bool USE_GJK_CONCAVE = false;

    if (USE_GJK_CONCAVE)
    {
        gjk_concave(_shapeA, _shapeB);
    }
    else
    {
        std::vector<Util::Vector> simplex;

        if (gjk(_shapeA, _shapeB, simplex)) {
            // FOR TESTING: CHECK SIMPLEX
            // for (Util::Vector pointA : simplex) {
            //    std::cout << "Simplex: (" << pointA.x << ", " << pointA.z << ")" << std::endl;
            // }
            epa(return_penetration_depth, return_penetration_vector, simplex, _shapeA, _shapeB);
            return true;
        }
        else {
            // FOR TESTING: CHECK SIMPLEX
            // for (Util::Vector pointA : simplex) {
            //    std::cout << "Simplex: (" << pointA.x << ", " << pointA.z << ")" << std::endl;
            // }
            return false; // no collision
        }
    }
}

