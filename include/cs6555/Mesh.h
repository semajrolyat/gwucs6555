/*-----------------------THE GEORGE WASHINGTON UNIVERSITY-----------------------
James Taylor                                                         jrt@gwu.edu

Mesh class definition

A mesh is a type of geometry.  Datastructure for rigid bodies.  Renderable
------------------------------------------------------------------------------*/

#ifndef _MESH_H_
#define _MESH_H_

//------------------------------------------------------------------------------

#include <vector>

#include <cs6555/Geometry.h>

#include <cs6555/Vertex.h>
#include <cs6555/Math/Matrix4.h>
#include <cs6555/Polygon.h>
#include <cs6555/Material.h>

//------------------------------------------------------------------------------

class Mesh : public Geometry {

public:
    // Constructor
    Mesh( void );

    // Destructor
    virtual ~Mesh( void );

    // [Base Class] Geometry::Type Queries
    virtual EGeometryType geometry_type( void );

    // Print the data structure to the console for debugging
    void echo( void );

    // Vertex Insertion and Queries
    unsigned int vertexCount( void );
    int appendVertex( Vertex* v );
    Vertex* vertex( const unsigned int& i );

    // Texture Vertex Insertion and Queries
    unsigned int textureVertexCount( void );
    int appendTextureVertex( Vector3* v );
    Vector3* texturevertex( const unsigned int& i );

    // Polygon Insertion and Queries
    int appendPolygon( Polygon* p );
    unsigned int polygonCount( void );
    Polygon* polygon( const unsigned int& i );

    // Material Insertion and Queries
    int appendMaterial( Material* m );
    unsigned int materialCount( void );
    Material* material( const unsigned int& i );

    // Material Operations
    void setMaterial( Material* material );

    // Utilities
    void calculate_normals( void );
    void calculate_polygon_normals( void );
    void calculate_vertex_normals( void );

    void flip_polys( void );

    // Member Data
protected:

    unsigned int m_vertexCount;         // Number of vertices in the linked list
    unsigned int m_polygonCount;        // Number of polygons in the linked list
    unsigned int m_texturevertexCount;  // Number of texture vertices in the linked list

    unsigned int m_materialCount;

    std::vector<Vertex*> m_vertices;            // The linked list of vertices
    std::vector<Polygon*> m_polygons;           // The linked list of polygons
    std::vector<Vector3*> m_texturevertices;    // The linked list of texture vertices

    std::vector<Material*> m_materials;
};

//------------------------------------------------------------------------------

#endif // _MESH_H_
