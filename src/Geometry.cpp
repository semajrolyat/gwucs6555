//------------------------------------------------------------------------------

#include <cs6555/Geometry.h>

#include <assert.h>

//------------------------------------------------------------------------------
// Constructors
//------------------------------------------------------------------------------
/// Default Constructor
Geometry::Geometry( void ) :
    m_childCount( 0 )
{
    m_bv = NULL;
    m_shallow_copy = false;
    scale = Vector3( 1.0, 1.0, 1.0 );
}

//------------------------------------------------------------------------------
// Destructor
//------------------------------------------------------------------------------
/// Destructor
Geometry::~Geometry( void ) {
    if( !m_shallow_copy && m_bv != NULL )
        delete m_bv;
}

void Geometry::shallow_copy( Geometry* g ) {

}

//------------------------------------------------------------------------------
// Hierarchy Operations
//------------------------------------------------------------------------------
/// Insert a child geometry into this instance
void Geometry::insert( Geometry* geometry ) {
    assert( geometry != NULL );
    m_children.push_back( geometry );
    m_childCount++;
}

//------------------------------------------------------------------------------
/// Retrieve a child geometry from this instance
Geometry* Geometry::geometry( unsigned int index ) {
    assert( index < m_childCount );
    return m_children.at( index );
}

//------------------------------------------------------------------------------
/// Query for the number of child geometries under this instance
unsigned int Geometry::geometryCount( void ) {
    return m_childCount;
}

//------------------------------------------------------------------------------
// Type Queries
//------------------------------------------------------------------------------
/// Geometry Type Query.  Implemented by inheritor.
EGeometryType Geometry::geometry_type( void ) {
    return GEOMETRY_TYPE_UNKNOWN;
}

//------------------------------------------------------------------------------
// Bounding Volume Operations
//------------------------------------------------------------------------------
/// Get a pointer to the bounding volume
BoundingVolume* Geometry::bv( void ) {
    return m_bv;
}
//------------------------------------------------------------------------------
/// Set the bounding volume
void Geometry::bv( BoundingVolume* v ) {
    assert( v != NULL );
    m_bv = v;
}
//------------------------------------------------------------------------------
