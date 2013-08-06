/*-----------------------THE GEORGE WASHINGTON UNIVERSITY-----------------------
James Taylor                                                         jrt@gwu.edu

Polygon class implementation

------------------------------------------------------------------------------*/

#include <cs6555/Polygon.h>

#include <assert.h>
#include <stdio.h>

//------------------------------------------------------------------------------
// Constructors
//------------------------------------------------------------------------------
/// Default Constructor
Polygon::Polygon( void )
{
    m_vertexCount = 0;
    m_texturevertexCount = 0;
    backface = false;
    material = NULL;
    normal = Vector3( 0.0f, 0.0f, 0.0f );
}

//------------------------------------------------------------------------------
/// Copy Constructor
Polygon::Polygon( Polygon& polygon )
{
    m_vertexCount = 0;
    m_texturevertexCount = 0;
    backface = polygon.backface;

    material = polygon.material;

    for( unsigned int i = 0; i < polygon.numVertices( ); i++ ) {
        addVertex( polygon.getVertex( i ) );
    }
    for( unsigned int i = 0; i < polygon.numTextureVertices( ); i++ ) {
        addTextureVertex( polygon.getTextureVertex( i ) );
    }
}

//------------------------------------------------------------------------------
// Destructor
//------------------------------------------------------------------------------
/// Destructor
Polygon::~Polygon( void ) {
    material = NULL;
}

//------------------------------------------------------------------------------
// Vertex Insertion and Queries
//------------------------------------------------------------------------------
/// Return a Vertex id from the polygon based on the relative index in the poly
unsigned int& Polygon::getVertex( const unsigned int &index ) {
    assert( index < m_vertexCount );
    return m_vertices.at( index );
}

//------------------------------------------------------------------------------
/// Add a Vertex id into the Polygon
void Polygon::addVertex( const unsigned int& vertexId ) {
    m_vertices.push_back(vertexId);
    m_vertexCount++;
}

//------------------------------------------------------------------------------
/// Return the number of vertices that make up the polygon
unsigned int Polygon::numVertices( ) {
    return m_vertexCount;
}

//------------------------------------------------------------------------------
// Utilities
//------------------------------------------------------------------------------
/// Print the Polygon data to the console for debugging
void Polygon::echo( void ) {
    printf( "Verts %d: ", numVertices( ) );
    for( unsigned int j = 0; j < numVertices( ); j++ ) {
        if( j > 0 )
            printf( "," );
        printf( " %d", getVertex( j ) );
    }
    printf( "\n" );
}

//------------------------------------------------------------------------------
void Polygon::flip( void ) {
    std::vector<int> tempvertices;
    for( unsigned int i = 0; i < m_vertexCount; i++ ) {
        tempvertices.push_back( m_vertices.at( i ) );
    }
    m_vertices.erase( m_vertices.begin( ),  m_vertices.end() );
    for( unsigned int i = m_vertexCount; i > 0; i-- ) {
        m_vertices.push_back( tempvertices.at( i - 1 ) );
    }
}

//------------------------------------------------------------------------------
// Texture Vertex Insertion and Queries
//------------------------------------------------------------------------------
/// Return a Texture Vertex from the polygon based on the relative index in the poly
unsigned int& Polygon::getTextureVertex( const unsigned int& index ) {
    assert( index < m_texturevertexCount );
    return m_texturevertices.at( index );
}

//------------------------------------------------------------------------------
/// Add a Texture Vertex into the Polygon
void Polygon::addTextureVertex( const unsigned int& tv ) {
    m_texturevertices.push_back( tv );
    m_texturevertexCount++;
}

//------------------------------------------------------------------------------
/// Return the number of texture vertices that make up the polygon
unsigned int Polygon::numTextureVertices( ) {
    return m_texturevertexCount;
}

//------------------------------------------------------------------------------
