/*-----------------------THE GEORGE WASHINGTON UNIVERSITY-----------------------
James Taylor                                                         jrt@gwu.edu

Vertex class implementation

------------------------------------------------------------------------------*/

#include <cs6555/Vertex.h>

//------------------------------------------------------------------------------
// Constructors
//------------------------------------------------------------------------------
/// Default Constructor
Vertex::Vertex( void ) :
    position( 0.0f, 0.0f, 0.0f, 1.0f ),
    normal( 0.0f, 0.0f, 0.0f )
{
    //uvw = Vector3();
}

//------------------------------------------------------------------------------
/// Copy Constructor
Vertex::Vertex( const Vertex& vertex )
{
    position = Vector4( vertex.position );
    normal = Vector3( vertex.normal );
    world = Vector3( vertex.world );
    //uvw = Vector3( vertex.uvw );

    for( unsigned int i = 0; i < vertex.m_polygons.size( ); i ++ )
        m_polygons.push_back( vertex.m_polygons.at(i) );
    material = vertex.material;
}

//------------------------------------------------------------------------------
/// Position only Constructor
Vertex::Vertex( const Vector3& position ) :
    position( position ),
    normal( 0.0f, 0.0f, 0.0f )
{
    //uvw = Vector3();
}

//------------------------------------------------------------------------------
/// Position and Normal Constructor
Vertex::Vertex( const Vector3& position, const Vector3& normal ) :
    position( position ),
    normal( normal )
{
    //uvw = Vector3();
}

//------------------------------------------------------------------------------
/// Positional Component Constructor
Vertex::Vertex( const float& px, const float& py, const float& pz ) :
    position( px, py, pz ),
    normal( 0.0f, 0.0f, 0.0f )
{
    //uvw = Vector3();
}

//------------------------------------------------------------------------------
/// Positional and Normal Component Constructor
Vertex::Vertex( const float& px, const float& py, const float& pz, const float& nx, const float& ny, const float& nz ) :
    position( px, py, pz ),
    normal( nx, ny, nz )
{
    //uvw = Vector3();
}

//------------------------------------------------------------------------------
// Destructor
//------------------------------------------------------------------------------
/// Destructor
Vertex::~Vertex( void )
{
    for( unsigned int i = 0; i < m_polygons.size( ); i ++ )
        m_polygons.pop_back( );
    material = NULL;
}

//------------------------------------------------------------------------------
// Polygon Insertion and Queries
//------------------------------------------------------------------------------
/// Appends a Polygon at the end of the list of polygons, e.g. users of this vertex
void Vertex::appendPolygon( Polygon* poly ) {
    m_polygons.push_back( poly );
}

//------------------------------------------------------------------------------
// Utilities
//------------------------------------------------------------------------------
/// Interpolates a normal from the polygon list
void Vertex::interpolateNormal( void ) {
    normal = Vector3( 0, 0, 0 );
    double den = 1.0 / (double) m_polygons.size();
    for( PolyList::iterator it = m_polygons.begin( ); it != m_polygons.end( ); it++ ) {
        Polygon* poly = *it;
        // allows for weighted norm calculation
        normal += poly->normal * den;
        //normal += poly->normal;
    }
    normal.normalize( );
    normal = -normal;
}

//------------------------------------------------------------------------------
