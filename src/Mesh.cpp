/*-----------------------THE GEORGE WASHINGTON UNIVERSITY-----------------------
James Taylor                                                         jrt@gwu.edu

Mesh class implementation

------------------------------------------------------------------------------*/

#include <cs6555/Mesh.h>

#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include <assert.h>
#include <math.h>

#include <cs6555/Constants.h>

//------------------------------------------------------------------------------
// Constructors
//------------------------------------------------------------------------------
/// Default Constructor
Mesh::Mesh( void ) :
    m_vertexCount( 0 ),
    m_polygonCount( 0 )
{
    //material = NULL;
    m_texturevertexCount = 0;
    m_materialCount = 0;
}

//------------------------------------------------------------------------------
// Destructor
//------------------------------------------------------------------------------
/// Destructor
Mesh::~Mesh( void ) {
    Material* m;
    while( !m_materials.empty( ) ) {
        m = *m_materials.end( );
        m_materials.pop_back( );
        //if( m != NULL ) delete m;
        delete m;
    }

    Polygon* p;
    while( !m_polygons.empty( ) ) {
        p = *m_polygons.end( );
        m_polygons.pop_back( );
        //if( p != NULL ) delete p;
        delete p;
    }
    Vertex* v;
    while( !m_vertices.empty( ) ) {
        v = *m_vertices.end( );
        m_vertices.pop_back( );
        //if( v != NULL ) delete v;
        delete v;
    }
    Vector3* t;
    while( !m_texturevertices.empty( ) ) {
        t = *m_texturevertices.end( );
        m_texturevertices.pop_back( );
        //if( t != NULL ) delete t;
        delete t;
    }
    //if( material != NULL )
    //    delete material;
}

//------------------------------------------------------------------------------
// [Base Class] Geometry::Queries
//------------------------------------------------------------------------------
/// Geometry Type Query.  Implemented by inheritor.
EGeometryType Mesh::geometry_type( void ) {
    return GEOMETRY_TYPE_MESH;
}

//------------------------------------------------------------------------------
// Vertex Insertion and Queries
//------------------------------------------------------------------------------
/// Query for number of vertices
unsigned int Mesh::vertexCount( void ) {
    return m_vertexCount;
}

//------------------------------------------------------------------------------
/// Insert a pointer to a vertex into the linked list of vertices
int Mesh::appendVertex( Vertex* v ) {
    m_vertices.push_back( v );
    m_vertexCount++;

    return 0;
}

//------------------------------------------------------------------------------
/// Query for a vertex by id where id the position in the linked list
Vertex* Mesh::vertex( const unsigned int& i ) {
    assert( i < m_vertexCount );
    return m_vertices.at( i );
}

//------------------------------------------------------------------------------
// Texture Vertex Insertion and Queries
//------------------------------------------------------------------------------
/// Query for number of texture vertices
unsigned int Mesh::textureVertexCount( void ) {
    return m_texturevertexCount;
}

//------------------------------------------------------------------------------
/// Insert a pointer to a texture vertex
int Mesh::appendTextureVertex( Vector3* v ) {
    m_texturevertices.push_back( v );
    m_texturevertexCount++;

    return 0;
}

//------------------------------------------------------------------------------
/// Query for a texture vertex based on index in the list of texture vertices
Vector3* Mesh::texturevertex( const unsigned int& i ) {
    assert( i < m_texturevertexCount );
    return m_texturevertices.at( i );
}

//------------------------------------------------------------------------------
// Polygon Insertion and Queries
//------------------------------------------------------------------------------
/// Query for number of polygons
unsigned int Mesh::polygonCount( void ) {
    return m_polygonCount;
}

//------------------------------------------------------------------------------
/// Insert a pointer to a polygon into the linked list of polygons
int Mesh::appendPolygon( Polygon* p ) {
    m_polygons.push_back( p );
    m_polygonCount++;

    return 0;
}

//------------------------------------------------------------------------------
/// Query for a polygon by id where id the position in the linked list
Polygon* Mesh::polygon( const unsigned int& i ) {
    assert( i < m_polygonCount );
    return m_polygons.at( i );
}

//------------------------------------------------------------------------------
// Material Insertion and Queries
//------------------------------------------------------------------------------
/// Query for number of materials
unsigned int Mesh::materialCount( void ) {
    return m_materialCount;
}
//------------------------------------------------------------------------------
/// Insert a pointer to a material into the linked list of polygons
int Mesh::appendMaterial( Material* m ) {
    m_materials.push_back( m );
    m_materialCount++;

    return 0;
}
//------------------------------------------------------------------------------
/// Query for a material by id where id the position in the linked list
Material* Mesh::material( const unsigned int& i ) {
    assert( i < m_materialCount );
    return m_materials.at( i );
}

//------------------------------------------------------------------------------
// Utilities
//------------------------------------------------------------------------------
/// Print the data structure to the console for debugging
void Mesh::echo( void ) {

    printf( "Verts: %d\n", m_vertexCount );
    printf( "Polys: %d\n", m_polygonCount );

    for( unsigned int i = 0; i < m_vertexCount; i++ ) {
        Vertex* v = m_vertices.at( i );
        printf( "Vertex %d: %f, %f, %f\n", i, v->position.x(), v->position.y(), v->position.z() );
    }

    for( unsigned int i = 0; i < m_polygonCount; i++ ) {
        Polygon* p = m_polygons.at( i );
        printf( "Poly %d:", i );
        for( unsigned int j = 0; j < p->numVertices( ); j++ ) {
            if( j > 0 )
                printf( "," );
            printf( " %d", p->getVertex( j ) );
        }
        printf( "\n" );
    }
}
/*
void Mesh::setMaterial( Material* material ) {

    this->material = material;

    for( unsigned int i = 0; i < vertexCount( ); i++ ) {
        vertex( i )->material = this->material;
    }
    for( unsigned int i = 0; i < polygonCount( ); i++ ) {
        polygon( i )->material = this->material;
    }
}
*/

//------------------------------------------------------------------------------
// Utility Functions
//------------------------------------------------------------------------------
/// calculates polygon normals and vertex normals
void Mesh::calculate_normals( void ) {
    // first calculate the poly normals
    calculate_polygon_normals();

    // second vertex normals because they are based on poly normals
    calculate_vertex_normals();
}

//------------------------------------------------------------------------------
/// Calculates polygon normals using cross product of vertex positions
void Mesh::calculate_polygon_normals( void ) {

    for( unsigned int poly_id = 0; poly_id < polygonCount( ); poly_id++ ) {

        // Select the current polygon
        Polygon* poly = polygon( poly_id );

        Vertex *v0, *v1, *v2;

        unsigned int verts = poly->numVertices( );
        if( verts < 3 ) continue;   // sanity check -> malformed poly & bad juju

        v0 = vertex( poly->getVertex( 0 ) );
        v1 = vertex( poly->getVertex( 1 ) );
        v2 = vertex( poly->getVertex( 2 ) );

//        v0 = vertex( poly->getVertex( poly->numVertices( ) -  1 ) );
//        v1 = vertex( poly->getVertex( 0 ) );
//        v2 = vertex( poly->getVertex( 1 ) );

        Vector3 edge1, edge2;

        Vector3 v3_0, v3_1, v3_2;

        v3_0 = Vector3( v0->position.x(), v0->position.y(), v0->position.z() );
        v3_1 = Vector3( v1->position.x(), v1->position.y(), v1->position.z() );
        v3_2 = Vector3( v2->position.x(), v2->position.y(), v2->position.z() );

        edge1 = v3_1 - v3_0;
        edge2 = v3_2 - v3_1;

        Vector3 cp = Vector3::cross( edge1, edge2 );
        poly->normal = cp;
    }

}
//------------------------------------------------------------------------------
/// Calculates vertex normals by interpolating from polygon normals shared by vertex
/// also registers all polygons shared by vertex
void Mesh::calculate_vertex_normals( void ) {
    for( unsigned int i = 0; i < polygonCount(); i++ ) {
        Polygon* poly = polygon( i );
        for( unsigned int j = 0; j < poly->numVertices(); j++ ) {
            unsigned int v = poly->getVertex( j );
            Vertex* vert = vertex( v );
            vert->appendPolygon( poly );
        }
    }
    for( unsigned int i = 0; i < vertexCount(); i++ ) {
        Vertex* vert = vertex( i );
        vert->interpolateNormal();
    }
}

//------------------------------------------------------------------------------
/// Iterates over all polygons in the mesh and flips each polygon
void Mesh::flip_polys( void ) {
    for( unsigned int i = 0; i < polygonCount(); i++ )
        polygon( i )->flip();
}

//------------------------------------------------------------------------------
