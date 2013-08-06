#ifndef _TRIGON_H_
#define _TRIGON_H_

//------------------------------------------------------------------------------

#include <vector>

#include <cs6555/Math/Vector3.h>
#include <cs6555/Material.h>

#include <cs6555/Polygon.h>

//------------------------------------------------------------------------------

class Trigon : public Polygon {
public:
    // Constructor
    Trigon( void );

    // Copy Constructor
    Trigon( Trigon& trigon );

    // Destructor
    virtual ~Trigon( void );

    void addVertex( const unsigned int& vertexId );
};
/*
class Trigon {
public:
    // Constructor
    Trigon( void );

    // Copy Constructor
    Trigon( Trigon& trigon );

    // Destructor
    virtual ~Trigon( void );

    // Print the Trigon data to the console for debugging
    void echo( void );

    // Add a Vertex id into the Trigon
    void addVertex( const unsigned int& vertex );

    // Return a Vertex id from the trigon based on the relative index in the trigon
    unsigned int& getVertex( const unsigned int& index );

    // Return the number of vertices that make up the trigon.  Should be 3
    unsigned int numVertices( );

    // Add a Texture Vertex into the Trigon
    void addTextureVertex( const unsigned int& tv );

    // Return a Texture Vertex from the trigon based on the relative index in the trigon
    unsigned int& getTextureVertex( const unsigned int& index );

    // Return the number of texture vertices that make up the trigon
    unsigned int numTextureVertices( );

    bool backface;

    Vector3 normal;

    Material* material;

    void flip( void );

protected:
    // Internal count of the number of vertices in the trigon.  Prevents repeated
    // traversal of the linked list that stores the vertex data when the number of
    // vertices is requested.
    unsigned int m_vertexCount;

    // The linked list of vertex ids stored in the trigon
    std::vector<unsigned int> m_vertices;

    unsigned int m_texturevertexCount;

    std::vector<unsigned int> m_texturevertices;

};
*/
//------------------------------------------------------------------------------


#endif // _TRIGON_H_
