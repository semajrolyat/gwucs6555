/*-----------------------THE GEORGE WASHINGTON UNIVERSITY-----------------------
James Taylor                                                         jrt@gwu.edu

Trigon class implementation

------------------------------------------------------------------------------*/

#include <cs6555/Trigon.h>

#include <assert.h>
#include <stdio.h>

//------------------------------------------------------------------------------
// Constructors
//------------------------------------------------------------------------------
/// Default Constructor
Trigon::Trigon( void )
{
    m_vertexCount = 0;
    m_texturevertexCount = 0;
    backface = false;
    material = NULL;
    normal = Vector3( 0.0f, 0.0f, 0.0f );
}

//------------------------------------------------------------------------------
/// Copy Constructor
Trigon::Trigon( Trigon& trigon ) : Polygon( trigon )
{
    /*
    m_vertexCount = 0;
    m_texturevertexCount = 0;
    backface = trigon.backface;

    material = trigon.material;

    for( unsigned int i = 0; i < trigon.numVertices( ); i++ ) {
        addVertex( trigon.getVertex( i ) );
    }
    for( unsigned int i = 0; i < trigon.numTextureVertices( ); i++ ) {
        addTextureVertex( trigon.getTextureVertex( i ) );
    }
    */
}

//------------------------------------------------------------------------------
// Destructor
//------------------------------------------------------------------------------
/// Destructor
Trigon::~Trigon( void ) {
    material = NULL;
}

//------------------------------------------------------------------------------
// Vertex Insertion and Queries
//------------------------------------------------------------------------------
/// Add a Vertex id into the Trigon
void Trigon::addVertex( const unsigned int& vertexId ) {
    assert( m_vertexCount < 3 );
    Polygon::addVertex( vertexId );
    //m_vertices.push_back(vertexId);
    //m_vertexCount++;
}

/*
//------------------------------------------------------------------------------
// Vertex Insertion and Queries
//------------------------------------------------------------------------------
/// Return a Vertex id from the trigon based on the relative index in the trigon
unsigned int& Trigon::getVertex( const unsigned int &index ) {
    assert( index < m_vertexCount );
    return m_vertices.at( index );
}

//------------------------------------------------------------------------------
/// Add a Vertex id into the Trigon
void Trigon::addVertex( const unsigned int& vertexId ) {
    m_vertices.push_back(vertexId);
    m_vertexCount++;
}

//------------------------------------------------------------------------------
/// Return the number of vertices that make up the trigon
unsigned int Trigon::numVertices( ) {
    return m_vertexCount;
}

//------------------------------------------------------------------------------
// Utilities
//------------------------------------------------------------------------------
/// Print the Trigon data to the console for debugging
void Trigon::echo( void ) {
    printf( "Verts %d: ", numVertices( ) );
    for( unsigned int j = 0; j < numVertices( ); j++ ) {
        if( j > 0 )
            printf( "," );
        printf( " %d", getVertex( j ) );
    }
    printf( "\n" );
}

//------------------------------------------------------------------------------
void Trigon::flip( void ) {
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
/// Return a Texture Vertex from the trigon based on the relative index in the trigon
unsigned int& Trigon::getTextureVertex( const unsigned int& index ) {
    assert( index < m_texturevertexCount );
    return m_texturevertices.at( index );
}

//------------------------------------------------------------------------------
/// Add a Texture Vertex into the Trigon
void Trigon::addTextureVertex( const unsigned int& tv ) {
    m_texturevertices.push_back( tv );
    m_texturevertexCount++;
}

//------------------------------------------------------------------------------
/// Return the number of texture vertices that make up the trigon
unsigned int Trigon::numTextureVertices( ) {
    return m_texturevertexCount;
}
*/
//------------------------------------------------------------------------------

