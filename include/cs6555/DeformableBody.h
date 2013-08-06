/*-----------------------THE GEORGE WASHINGTON UNIVERSITY-----------------------
James Taylor                                                         jrt@gwu.edu

DeformableBody class definition

------------------------------------------------------------------------------*/

#ifndef _DEFORMABLEBODY_H_
#define _DEFORMABLEBODY_H_

//------------------------------------------------------------------------------

#include <cs6555/Body.h>
#include <cs6555/Mesh.h>


#include <cs6555/Node.h>
#include <cs6555/Edge.h>
#include <cs6555/Geometry.h>
#include <cs6555/Tetrahedron.h>

//------------------------------------------------------------------------------
// Enums
//------------------------------------------------------------------------------
typedef enum {
    DEFORMABLE_TYPE_UNKNOWN,
    DEFORMABLE_TYPE_BODY,
    DEFORMABLE_TYPE_SPACE
} EDeformableType;
//------------------------------------------------------------------------------

class DeformableBody : public Geometry {
public:
    DeformableBody( void );
    virtual ~DeformableBody( void );

    // [Base Class] Body::Queries
    virtual EBodyType body_type( void );

    virtual EDeformableType deformable_type( void ) { return DEFORMABLE_TYPE_BODY; }

    Mesh* mesh;

    unsigned int nodeCount( void );
    int appendNode( Node* n );
    Node* node( const unsigned int& i );

    unsigned int edgeCount( void );
    int appendEdge( Edge* e );
    Edge* edge( const unsigned int& i );

    int appendTetrahedron( Tetrahedron* t );
    unsigned int tetrahedronCount( void );
    Tetrahedron* tetrahedron( const unsigned int& i );

private:
    unsigned int m_nodeCount;
    unsigned int m_edgeCount;
    unsigned int m_tetrahedronCount;

    std::vector<Node*> m_nodes;
    std::vector<Edge*> m_edges;
    std::vector<Tetrahedron*> m_tetrahedrons;
};

/*
//------------------------------------------------------------------------------
// Enums
//------------------------------------------------------------------------------
typedef enum {
    DEFORMABLE_TYPE_UNKNOWN,
    DEFORMABLE_TYPE_BODY,
    DEFORMABLE_TYPE_SPACE
} EDeformableType;
//------------------------------------------------------------------------------

class DeformableBody : public Body {
public:
    DeformableBody( void );
    virtual ~DeformableBody( void );

    // [Base Class] Body::Queries
    virtual EBodyType body_type( void );

    virtual EDeformableType deformable_type( void ) { return DEFORMABLE_TYPE_BODY; }
};
*/
//------------------------------------------------------------------------------

#endif // _DEFORMABLEBODY_H_


