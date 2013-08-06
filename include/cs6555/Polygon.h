/*-----------------------THE GEORGE WASHINGTON UNIVERSITY-----------------------
James Taylor                                                         jrt@gwu.edu

Polygon class definition

------------------------------------------------------------------------------*/

#ifndef _POLYGON_H_
#define _POLYGON_H_

//------------------------------------------------------------------------------

#include <vector>

#include <cs6555/Math/Vector3.h>
#include <cs6555/Material.h>

//------------------------------------------------------------------------------

class Polygon {
public:
    // Constructor
    Polygon( void );

    // Copy Constructor
    Polygon( Polygon& polygon );

    // Destructor
    virtual ~Polygon( void );

    // Print the Polygon data to the console for debugging
    void echo( void );

    // Add a Vertex id into the Polygon
    void addVertex( const unsigned int& vertex );

    // Return a Vertex id from the polygon based on the relative index in the poly
    unsigned int& getVertex( const unsigned int& index );

    // Return the number of vertices that make up the polygon
    unsigned int numVertices( );

    // Add a Texture Vertex into the Polygon
    void addTextureVertex( const unsigned int& tv );

    // Return a Texture Vertex from the polygon based on the relative index in the poly
    unsigned int& getTextureVertex( const unsigned int& index );

    // Return the number of texture vertices that make up the polygon
    unsigned int numTextureVertices( );

    bool backface;

    Vector3 normal;

    Material* material;

    void flip( void );

protected:
    // Internal count of the number of vertices in the polygon.  Prevents repeated
    // traversal of the linked list that stores the vertex data when the number of
    // vertices is requested.
    unsigned int m_vertexCount;

    // The linked list of vertex ids stored in the polygon
    std::vector<unsigned int> m_vertices;

    unsigned int m_texturevertexCount;

    std::vector<unsigned int> m_texturevertices;

};

//------------------------------------------------------------------------------

#endif  // _POLYGON_H_
