/*-----------------------THE GEORGE WASHINGTON UNIVERSITY-----------------------
James Taylor                                                         jrt@gwu.edu

Geometry class definition

geometry is the fundamental class uniting all forms of geometrical data.
geometrical data should be renderable.  It is also a hierarchy where the root
defines a frame and every child under the root is transformed relative to
the root's pose.
------------------------------------------------------------------------------*/

#ifndef _GEOMETRY_H_
#define _GEOMETRY_H_

//------------------------------------------------------------------------------

#include <vector>

#include <cs6555/Pose.h>
#include <cs6555/Material.h>
#include <cs6555/BoundingVolume.h>
#include <cs6555/Point3.h>

//------------------------------------------------------------------------------

// Forward Declaration
class Geometry;

// The definition of list of Geometry for hierarchy management
typedef std::vector<Geometry*> GeometryList;

//------------------------------------------------------------------------------

typedef enum {
    GEOMETRY_TYPE_UNKNOWN,
    GEOMETRY_TYPE_MESH,
    GEOMETRY_TYPE_TRAJECTORY,
    GEOMETRY_TYPE_POINTCLOUD,
    GEOMETRY_TYPE_PATCH,
    GEOMETRY_TYPE_BODY,
    GEOMETRY_TYPE_PARTICLE
} EGeometryType;
//------------------------------------------------------------------------------

class Geometry : public Point3 {
public:
    // Constructors
    Geometry( void );

    // Destructor
    virtual ~Geometry( void );

    virtual void shallow_copy( Geometry* g );

    // Hierarchy Insertion and Queries
    void insert( Geometry* geometry );
    Geometry* geometry( unsigned int index );
    unsigned int geometryCount( void );

    // Type Queries
    virtual EGeometryType geometry_type( void );

    // Bounding Volume Operations
    BoundingVolume* bv( void );
    void bv( BoundingVolume* v );

    // Member Data
    Pose pose;                  // the 6-dimensional pose defining this frame
    Material material;          // the material associated with this geometry

    Vector3 scale;

protected:

    GeometryList m_children;    // The edges to children under the root
    unsigned int m_childCount;  // The number of children attached to the root

    GeometryList* m_ref_children;       // reference to a child list.  Set only in shallow copy

    BoundingVolume *m_bv;       // bounding volume

    bool m_shallow_copy;
    Geometry* m_origin;
};

//------------------------------------------------------------------------------

#endif // _GEOMETRY_H_
