#include <cstdint>
#include <list>
#include <vector>
#include <algorithm>

// This code uses C++11 features like "auto" and initializer lists.

// Compare two floating point numbers accurately.
bool compf(float a, float b, float threshold=0.00001f)
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
bool orientation(const std::vector<vector_t>& p)
{
    return (p[1].x - p[0].x) * (p[2].y - p[0].y) - 
           (p[2].x - p[0].x) * (p[1].y - p[0].y) > 0;
}

// Barycentric coordinate calculation.
bool in_triangle(const vector_t& V, const vector_t& A,
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

std::vector<vector_t> triangulate(std::vector<vector_t> Polygon)
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
