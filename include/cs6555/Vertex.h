/*-----------------------THE GEORGE WASHINGTON UNIVERSITY-----------------------
James Taylor                                                         jrt@gwu.edu

Vertex class definition

------------------------------------------------------------------------------*/

#ifndef _VERTEX_H_
#define _VERTEX_H_

//------------------------------------------------------------------------------

#include <vector>
#include <cs6555/Math/Vector3.h>
#include <cs6555/Math/Vector4.h>
#include <cs6555/Color.h>
#include <cs6555/Polygon.h>
#include <cs6555/Material.h>

//------------------------------------------------------------------------------

typedef std::vector<Polygon*> PolyList;

//------------------------------------------------------------------------------

class Vertex
{
public:
    // Constructors
    Vertex( void );
    Vertex( const Vertex& vertex );
    Vertex( const Vector3& position );
    Vertex( const Vector3& position, const Vector3& normal );
    Vertex( const float& px, const float& py, const float& pz );
    Vertex( const float& px, const float& py, const float& pz, const float& nx, const float& ny, const float& nz );

    // Destructor
    ~Vertex( void );

    // Polygon Insertion and Queries
    void appendPolygon( Polygon* poly );

    // Utilities
    void interpolateNormal( void );

    // Member Data
    Vector3 world;
    Vector4 position;
    Vector3 normal;
    //Vector3 uvw;

    Color color;
    Material* material;

protected:
    PolyList m_polygons;
};

//------------------------------------------------------------------------------

#endif // _VERTEX_H_
