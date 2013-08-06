/*-----------------------THE GEORGE WASHINGTON UNIVERSITY-----------------------
James Taylor                                                         jrt@gwu.edu

DeformableBody class implementation

------------------------------------------------------------------------------*/

#include <cs6555/DeformableBody.h>

#include <assert.h>

//------------------------------------------------------------------------------
// Constructors
//------------------------------------------------------------------------------
/// Default Constructor
DeformableBody::DeformableBody( void ) :
    m_nodeCount( 0 ),
    m_edgeCount( 0 ),
    m_tetrahedronCount( 0 )
{
    mesh = NULL;
}

//------------------------------------------------------------------------------
// Destructor
//------------------------------------------------------------------------------
/// Destructor
DeformableBody::~DeformableBody( void ) {

    Tetrahedron* t;
    while( !m_tetrahedrons.empty( ) ) {
        t = *m_tetrahedrons.end( );
        m_tetrahedrons.pop_back( );
        //if( t != NULL ) delete t;
        delete t;
    }
    Edge* e;
    while( !m_edges.empty( ) ) {
        e = *m_edges.end( );
        m_edges.pop_back( );
        //if( e != NULL ) delete e;
        delete e;
    }
    Node* n;
    while( !m_nodes.empty( ) ) {
        n = *m_nodes.end( );
        m_nodes.pop_back( );
        //if( n != NULL ) delete n;
        delete n;
    }
}

/*
//------------------------------------------------------------------------------
// [Base Class] Geometry::Queries
//------------------------------------------------------------------------------
/// Geometry Type Query.  Implemented by inheritor.
EGeometryType DeformableBody::geometry_type( void ) {
    return GEOMETRY_TYPE_UNKNOWN;
}
*/
//------------------------------------------------------------------------------
// [Base Class] Body::Queries
//------------------------------------------------------------------------------
/// Body Type Query.  Implemented by inheritor.
EBodyType DeformableBody::body_type( void ) {
    return BODY_TYPE_DEFORMABLE;
}

//------------------------------------------------------------------------------
// Node Insertion and Queries
//------------------------------------------------------------------------------
/// Query for number of nodes
unsigned int DeformableBody::nodeCount( void ) {
    return m_nodeCount;
}

//------------------------------------------------------------------------------
/// Insert a pointer to a node into the linked list of nodes
int DeformableBody::appendNode( Node* n ) {
    m_nodes.push_back( n );
    m_nodeCount++;

    return 0;
}

//------------------------------------------------------------------------------
/// Query for a node by id where id the position in the linked list
Node* DeformableBody::node( const unsigned int& i ) {
    assert( i < m_nodeCount );
    return m_nodes.at( i );
}

//------------------------------------------------------------------------------
// Edge Insertion and Queries
//------------------------------------------------------------------------------
/// Query for number of edges
unsigned int DeformableBody::edgeCount( void ) {
    return m_edgeCount;
}

//------------------------------------------------------------------------------
/// Insert a pointer to a edge into the linked list of edges
int DeformableBody::appendEdge( Edge* e ) {
    m_edges.push_back( e );
    m_edgeCount++;

    return 0;
}

//------------------------------------------------------------------------------
/// Query for a edge by id where id the position in the linked list
Edge* DeformableBody::edge( const unsigned int& i ) {
    assert( i < m_edgeCount );
    return m_edges.at( i );
}


//------------------------------------------------------------------------------
// Tetrahedron Insertion and Queries
//------------------------------------------------------------------------------
/// Query for number of tetrahdrons
unsigned int DeformableBody::tetrahedronCount( void ) {
    return m_tetrahedronCount;
}

//------------------------------------------------------------------------------
/// Insert a pointer to a tetrahedron into the linked list of tetrahedrons
int DeformableBody::appendTetrahedron( Tetrahedron* t ) {
    m_tetrahedrons.push_back( t );
    m_tetrahedronCount++;

    return 0;
}

//------------------------------------------------------------------------------
/// Query for a tetrahedron by id where id the position in the linked list
Tetrahedron* DeformableBody::tetrahedron( const unsigned int& i ) {
    assert( i < m_tetrahedronCount );
    return m_tetrahedrons.at( i );
}

//------------------------------------------------------------------------------

