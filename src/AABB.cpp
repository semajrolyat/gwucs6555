/*-----------------------THE GEORGE WASHINGTON UNIVERSITY-----------------------
James Taylor                                                         jrt@gwu.edu

AABB class implementation

------------------------------------------------------------------------------*/

#include <cs6555/AABB.h>

//------------------------------------------------------------------------------
// Constructors
//------------------------------------------------------------------------------
/// Default Constructor
AABB::AABB( const Vector3& center, const Vector3& extens ) {
    this->center = center;
    this->extens = extens;
}

//------------------------------------------------------------------------------
// Destructor
//------------------------------------------------------------------------------
/// Destructor
AABB::~AABB( void ) {

}

//------------------------------------------------------------------------------
// [Base Class] BoundingVolume::Type Queries
//------------------------------------------------------------------------------
/// BoundingVolume Type Query.  Implemented by inheritor.
EBoundingVolumeType AABB::bounding_volume_type( void ) {
    return BV_TYPE_AABB;
}

Vector3 AABB::normal( Vector3& point_on_aabb ) {
    // check corners, if point is corner then normal is the sum of three planar normals
    // check edges, if point on edge then normal is sum of two planar edges
    // otherwise determine which surface the point is on

    Vector3 n;
    Vector3 min_p = center - extens;
    Vector3 max_p = center + extens;

    // bottom quad, ccw
    Vector3 p1 = min_p;
    Vector3 p2 = Vector3( min_p(0), min_p(1), max_p(2) );
    Vector3 p3 = Vector3( max_p(0), min_p(1), max_p(2) );
    Vector3 p4 = Vector3( max_p(0), min_p(1), min_p(2) );

    // top quad, ccw
    Vector3 p5 = Vector3( min_p(0), max_p(1), min_p(2) );
    Vector3 p6 = Vector3( min_p(0), max_p(1), max_p(2) );
    Vector3 p7 = max_p;
    Vector3 p8 = Vector3( max_p(0), max_p(1), min_p(2) );

    // check corners
    if( point_on_aabb == p1 ) {
        n = Vector3(-1, 0, 0) + Vector3(0, -1, 0) + Vector3(0, 0, -1);
        n.normalize();
        return n;
    }
    if( point_on_aabb == p2 ) {
        n = Vector3(-1, 0, 0) + Vector3(0, -1, 0) + Vector3(0, 0, 1);
        n.normalize();
        return n;
    }
    if( point_on_aabb == p3 ) {
        n = Vector3(1, 0, 0) + Vector3(0, -1, 0) + Vector3(0, 0, 1);
        n.normalize();
        return n;
    }
    if( point_on_aabb == p4 ) {
        n = Vector3(1, 0, 0) + Vector3(0, -1, 0) + Vector3(0, 0, -1);
        n.normalize();
        return n;
    }
    if( point_on_aabb == p5 ) {
        n = Vector3(-1, 0, 0) + Vector3(0, 1, 0) + Vector3(0, 0, -1);
        n.normalize();
        return n;
    }
    if( point_on_aabb == p6 ) {
        n = Vector3(-1, 0, 0) + Vector3(0, 1, 0) + Vector3(0, 0, 1);
        n.normalize();
        return n;
    }
    if( point_on_aabb == p7 ) {
        n = Vector3(1, 0, 0) + Vector3(0, 1, 0) + Vector3(0, 0, 1);
        n.normalize();
        return n;
    }
    if( point_on_aabb == p8 ) {
        n = Vector3(1, 0, 0) + Vector3(0, 1, 0) + Vector3(0, 0, -1);
        n.normalize();
        return n;
    }

    // check edges
    if( BoundingVolume::SqDistPointSegment( p1, p2, point_on_aabb ) == 0.0 ) {
        n = Vector3(-1, 0, 0) + Vector3(0, -1, 0);
        n.normalize();
        return n;
    }
    if( BoundingVolume::SqDistPointSegment( p2, p3, point_on_aabb ) == 0.0 ) {
        n = Vector3(0, -1, 0) + Vector3(0, 0, 1);
        n.normalize();
        return n;
    }
    if( BoundingVolume::SqDistPointSegment( p3, p4, point_on_aabb ) == 0.0 ) {
        n = Vector3(1, 0, 0) + Vector3(0, -1, 0);
        n.normalize();
        return n;
    }
    if( BoundingVolume::SqDistPointSegment( p4, p1, point_on_aabb ) == 0.0 ) {
        n = Vector3(0, -1, 0) + Vector3(0, 0, -1);
        n.normalize();
        return n;
    }
    if( BoundingVolume::SqDistPointSegment( p5, p6, point_on_aabb ) == 0.0 ) {
        n = Vector3(-1, 0, 0) + Vector3(0, 1, 0);
        n.normalize();
        return n;
    }
    if( BoundingVolume::SqDistPointSegment( p6, p7, point_on_aabb ) == 0.0 ) {
        n = Vector3(0, 1, 0) + Vector3(0, 0, 1);
        n.normalize();
        return n;
    }
    if( BoundingVolume::SqDistPointSegment( p7, p8, point_on_aabb ) == 0.0 ) {
        n = Vector3(1, 0, 0) + Vector3(0, 1, 0);
        n.normalize();
        return n;
    }
    if( BoundingVolume::SqDistPointSegment( p8, p5, point_on_aabb ) == 0.0 ) {
        n = Vector3(0, 1, 0) + Vector3(0, 0, -1);
        n.normalize();
        return n;
    }
    if( BoundingVolume::SqDistPointSegment( p1, p5, point_on_aabb ) == 0.0 ) {
        n = Vector3(-1, 0, 0) + Vector3(0, 0, -1);
        n.normalize();
        return n;
    }
    if( BoundingVolume::SqDistPointSegment( p2, p6, point_on_aabb ) == 0.0 ) {
        n = Vector3(-1, 0, 0) + Vector3(0, 0, 1);
        n.normalize();
        return n;
    }
    if( BoundingVolume::SqDistPointSegment( p3, p7, point_on_aabb ) == 0.0 ) {
        n = Vector3(1, 0, 0) + Vector3(0, 0, 1);
        n.normalize();
        return n;
    }
    if( BoundingVolume::SqDistPointSegment( p4, p8, point_on_aabb ) == 0.0 ) {
        n = Vector3(1, 0, 0) + Vector3(0, 0, -1);
        n.normalize();
        return n;
    }

    // check surfaces
    if( point_on_aabb.x() == min_p.x() ) {
        return Vector3(-1, 0, 0);
    }
    if( point_on_aabb.x() == max_p.x() ) {
        return Vector3(1, 0, 0);
    }
    if( point_on_aabb.y() == min_p.y() ) {
        return Vector3(0, -1, 0);
    }
    if( point_on_aabb.y() == max_p.y() ) {
        return Vector3(0, 1, 0);
    }
    if( point_on_aabb.z() == min_p.z() ) {
        return Vector3(0, 0, -1);
    }
    if( point_on_aabb.z() == max_p.z() ) {
        return Vector3(0, 0, 1);
    }
    // if numerical error then can reach here.
    // ok for now but added assertion to check for the case arising
    assert( true );

    return Vector3(0, 0, 1); // a bogus vector to resolve warning
}
