/*-----------------------THE GEORGE WASHINGTON UNIVERSITY-----------------------
James Taylor                                                         jrt@gwu.edu

BoundingVolume class definition

------------------------------------------------------------------------------*/

#ifndef _BOUNDINGVOLUME_H_
#define _BOUNDINGVOLUME_H_

//------------------------------------------------------------------------------

#include <cs6555/Math/Vector3.h>
#include <cs6555/Polygon.h>

//------------------------------------------------------------------------------

typedef enum {
    BV_TYPE_UNKNOWN,
    BV_TYPE_SPHERE,
    BV_TYPE_AABB,
    BV_TYPE_OBB
} EBoundingVolumeType;

//------------------------------------------------------------------------------
// Forward Declarations
class BoundingSphere;
class AABB;
class OBB;

//------------------------------------------------------------------------------

class BoundingVolume {
public:
    // Constructors
    BoundingVolume( void );

    // Destructor
    virtual ~BoundingVolume( void );

    // Type Queries
    virtual EBoundingVolumeType bounding_volume_type( void );

    // Intersection Queries
    static bool intersects( BoundingVolume* a, BoundingVolume* b );

    static bool intersects( const BoundingSphere& a, const BoundingSphere& b );
    static bool intersects( BoundingSphere& a, AABB& b );
    static bool intersects( const BoundingSphere& a, const OBB& b );

    static bool intersects( AABB& a, BoundingSphere& b );
    static bool intersects( const AABB& a, const AABB& b );
    static bool intersects( const AABB& a, const OBB& b );

    static bool intersects( const OBB& a, const BoundingSphere& b );
    static bool intersects( const OBB& a, const AABB& b );
    static bool intersects( OBB& a, OBB& b );

    // Member Data

    static double SqDistPointSegment( Vector3& a, Vector3& b, Vector3& p );
    static double SqDistPointAABB( Vector3& point, AABB& aabb );
    static Vector3 ClosestPtPointTriangle( Vector3& p, Vector3& a, Vector3& b, Vector3& c );
    static void ClosestPtPointSphere( Vector3& p, BoundingSphere& b, Vector3& q );
    static void ClosestPtPointAABB( Vector3& p, AABB& b, Vector3& q );
    static bool TestSphereTriangle( BoundingSphere& s, Vector3& a, Vector3& b, Vector3& c, Vector3& p );
    static bool TestSpherePolygon( BoundingSphere& s, Polygon& p );
    static bool TestSphereAABB( BoundingSphere& s, AABB& b, Vector3& p );
};

//------------------------------------------------------------------------------

#endif // _BOUNDINGVOLUME_H_
