/*-----------------------THE GEORGE WASHINGTON UNIVERSITY-----------------------
James Taylor                                                         jrt@gwu.edu

BoundingVolume class implementation

References:
    #1 Real-Time Collision Detection - Christer Ericson - Morgan Kaufman - 2005
------------------------------------------------------------------------------*/

#include <cs6555/BoundingVolume.h>

#include <cs6555/BoundingSphere.h>
#include <cs6555/AABB.h>
#include <cs6555/OBB.h>

#include <math.h>

//------------------------------------------------------------------------------
// Constructors
//------------------------------------------------------------------------------
/// Default Constructor
BoundingVolume::BoundingVolume( void ) {

}

//------------------------------------------------------------------------------
// Destructor
//------------------------------------------------------------------------------
/// Destructor
BoundingVolume::~BoundingVolume( void ) {

}

//------------------------------------------------------------------------------
// Type Queries
//------------------------------------------------------------------------------
/// BoundingVolume Type Query.  Implemented by inheritor.
EBoundingVolumeType BoundingVolume::bounding_volume_type( void ) {
    return BV_TYPE_UNKNOWN;
}

//------------------------------------------------------------------------------
// Intersection Tests
//------------------------------------------------------------------------------
bool BoundingVolume::intersects( BoundingVolume* a, BoundingVolume* b ) {
    if( a->bounding_volume_type() == BV_TYPE_SPHERE && b->bounding_volume_type() == BV_TYPE_SPHERE )
        return intersects( *static_cast<BoundingSphere*>( a ), *static_cast<BoundingSphere*>( b ) );

    if( a->bounding_volume_type() == BV_TYPE_AABB && b->bounding_volume_type() == BV_TYPE_AABB )
        return intersects( *static_cast<AABB*>( a ), *static_cast<AABB*>( b ) );

    if( a->bounding_volume_type() == BV_TYPE_SPHERE && b->bounding_volume_type() == BV_TYPE_AABB )
        return intersects( *static_cast<BoundingSphere*>( a ), *static_cast<AABB*>( b ) );

    if( a->bounding_volume_type() == BV_TYPE_AABB && b->bounding_volume_type() == BV_TYPE_SPHERE )
        return intersects( *static_cast<AABB*>( a ), *static_cast<BoundingSphere*>( b ) );
    /*
        BV_TYPE_SPHERE,
        BV_TYPE_AABB,
        BV_TYPE_OBB
        */
    return false;
}
//------------------------------------------------------------------------------
// Ref #1 pp 88
bool BoundingVolume::intersects( const BoundingSphere& a, const BoundingSphere& b ) {
    // Calculate the squared distance between spheres
    Vector3 d = a.center - b.center;
    double dist2 = Vector3::dot( d, d );
    // Spheres intersect if squared distance is less than square sum of radii
    double radsum = a.radius + b.radius;
    double radsum2 = radsum * radsum;
    return dist2 <= radsum2;
}

//------------------------------------------------------------------------------
// Ref #1 pp 165
bool BoundingVolume::intersects( BoundingSphere& a, AABB& b ) {
    // Compute squared distance between sphere center and AABB
    double sqDist = BoundingVolume::SqDistPointAABB( a.center, b );
    // Sphere and AABB intersect if the (squared) distance
    // between them is less than the (squared) sphere radius
    return sqDist <= a.radius * a.radius;
}

//------------------------------------------------------------------------------

bool BoundingVolume::intersects( const BoundingSphere& a, const OBB& b ) {
    return false;  // bogus
}

//------------------------------------------------------------------------------

bool BoundingVolume::intersects( AABB& a, BoundingSphere& b ) {
    return intersects( b, a );
}

//------------------------------------------------------------------------------
// Ref #1 pp 80
bool BoundingVolume::intersects( const AABB& a, const AABB& b ) {
    if( fabs( a.center.x() - b.center.x() ) > ( a.extens.x() + b.extens.x() ) ) return false;
    if( fabs( a.center.y() - b.center.y() ) > ( a.extens.y() + b.extens.y() ) ) return false;
    if( fabs( a.center.z() - b.center.z() ) > ( a.extens.z() + b.extens.z() ) ) return false;
    return true;
}

//------------------------------------------------------------------------------

bool BoundingVolume::intersects( const AABB& a, const OBB& b ) {
    return false;  // bogus
}

//------------------------------------------------------------------------------

bool BoundingVolume::intersects( const OBB& a, const BoundingSphere& b ) {
    return intersects( b, a );
}

//------------------------------------------------------------------------------

bool BoundingVolume::intersects( const OBB& a, const AABB& b ) {
    return intersects( b, a );
}

//------------------------------------------------------------------------------
// Ref #1 pp 103
bool BoundingVolume::intersects( OBB& a, OBB& b ) {
    double ra,rb;
    Matrix3 R, AbsR;
    const double EPSILON = 1e-5;

    // Compute rotation matrix expressing b in a's coordinate frame
    for( unsigned int i = 0; i < 3; i++ )
        for( unsigned int j = 0; j < 3; j++ )
            R( i, j ) = Vector3::dot( a.axes.column( i ), b.axes.column( i ) );

    // Compute translation vector t
    Vector3 t = b.position - a.position;
    // Bring translation into a's coordinate frame
    t = Vector3( Vector3::dot( t, a.axes.column( 0 ) ), Vector3::dot( t, a.axes.column( 1 ) ), Vector3::dot( t, a.axes.column( 2 ) ) );

    // Compute common subexpressions.  Add in an epsilon term to
    // counteract arithmetic errors when two edges are parallel and
    // their cross product is (near) null.
    for( unsigned int i = 0; i < 3; i++ )
        for( unsigned int j = 0; j < 3; j++ )
            AbsR( i, j ) = fabs( R( i, j ) ) + EPSILON;

    // Test axes L = A0, L = A1, L = A2
    for( unsigned int i = 0; i < 3; i++ ) {
        ra = a.extens( i );
        rb = b.extens( 0 ) * AbsR( i, 0 ) + b.extens( 1 ) * AbsR( i, 1 ) + b.extens( 2 ) * AbsR( i, 2 );
        if( fabs(t(i)) > ra + rb ) return false;
    }

    // Test axes L = B0, L = B1, L = B2
    for( unsigned int i = 0; i < 3; i++ ) {
        ra = a.extens( 0 ) * AbsR( 0, i ) + a.extens( 1 ) * AbsR( 1, i ) + a.extens( 2 ) * AbsR( 2, i );
        rb = b.extens( i );
        if( fabs(t(0) * R(0, i) + t(1) * R(1, i) + t(2) * R(2, i)) > ra + rb ) return false;
    }

    // Test axis L = A0 x B0
    ra = a.extens(1) * AbsR(2,0) + a.extens(2) * AbsR(1,0);
    rb = b.extens(1) * AbsR(0,2) + b.extens(2) * AbsR(0,1);
    if( fabs(t(2) * R(1,0) - t(1) * R(2,0)) > ra + rb ) return false;

    // Test axis L = A0 x B1
    ra = a.extens(1) * AbsR(2,1) + a.extens(2) * AbsR(1,1);
    rb = b.extens(0) * AbsR(0,2) + b.extens(2) * AbsR(0,0);
    if( fabs(t(2) * R(1,1) - t(1) * R(2,1)) > ra + rb ) return false;

    // Test axis L = A0 x B2
    ra = a.extens(1) * AbsR(2,2) + a.extens(2) * AbsR(1,2);
    rb = b.extens(0) * AbsR(0,1) + b.extens(1) * AbsR(0,0);
    if( fabs(t(2) * R(1,2) - t(1) * R(2,2)) > ra + rb ) return false;

    // Test axis L = A1 x B0
    ra = a.extens(0) * AbsR(2,0) + a.extens(2) * AbsR(0,0);
    rb = b.extens(1) * AbsR(1,2) + b.extens(2) * AbsR(1,1);
    if( fabs(t(0) * R(2,0) - t(2) * R(0,0)) > ra + rb ) return false;

    // Test axis L = A1 x B1
    ra = a.extens(0) * AbsR(2,1) + a.extens(2) * AbsR(0,1);
    rb = b.extens(0) * AbsR(1,2) + b.extens(2) * AbsR(1,0);
    if( fabs(t(0) * R(2,1) - t(2) * R(0,1)) > ra + rb ) return false;

    // Test axis L = A1 x B2
    ra = a.extens(0) * AbsR(2,2) + a.extens(2) * AbsR(0,2);
    rb = b.extens(0) * AbsR(1,1) + b.extens(1) * AbsR(1,0);
    if( fabs(t(0) * R(2,2) - t(2) * R(0,2)) > ra + rb ) return false;

    // Test axis L = A2 x B0
    ra = a.extens(0) * AbsR(1,0) + a.extens(1) * AbsR(0,0);
    rb = b.extens(1) * AbsR(2,2) + b.extens(2) * AbsR(2,1);
    if( fabs(t(1) * R(0,0) - t(0) * R(1,0)) > ra + rb ) return false;

    // Test axis L = A2 x B1
    ra = a.extens(0) * AbsR(1,1) + a.extens(1) * AbsR(0,1);
    rb = b.extens(0) * AbsR(2,2) + b.extens(2) * AbsR(2,0);
    if( fabs(t(1) * R(0,1) - t(0) * R(1,1)) > ra + rb ) return false;

    // Test axis L = A2 x B2
    ra = a.extens(0) * AbsR(1,2) + a.extens(1) * AbsR(0,2);
    rb = b.extens(0) * AbsR(2,1) + b.extens(1) * AbsR(2,0);
    if( fabs(t(1) * R(0,2) - t(0) * R(1,2)) > ra + rb ) return false;

    // Since no seperating axis is found, the OBBs must be intersecting
    return true;
}

//------------------------------------------------------------------------------
// Ref #1 pp 130
// Returns the squared distance between point p and segment ab
double BoundingVolume::SqDistPointSegment( Vector3& a, Vector3& b, Vector3& p ) {
    Vector3 ab = b - a, ap = p - a, bp = p - b;
    double e = Vector3::dot( ap, ab );
    // Handle cases where p projects outside ab
    if( e <= 0.0 ) return Vector3::dot( ap, ap );
    double f = Vector3::dot( ab, ab );
    if( e >= f ) return Vector3::dot( bp, bp );
    // Handle cases where p projects onto ab
    return Vector3::dot( ap, ap ) - e * e / f;
}

//------------------------------------------------------------------------------
// Ref #1 pp 131
double BoundingVolume::SqDistPointAABB( Vector3& point, AABB& aabb ) {
    double sqDist = 0.0;
    for( unsigned int i = 0; i < 3; i++ ) {
        // For each axis count any excess distance outside box extents
        double v = point(i);
        double min_i = aabb.center(i) - aabb.extens(i);
        double max_i = aabb.center(i) + aabb.extens(i);
        if( v < min_i ) sqDist += ( min_i - v ) * ( min_i - v );
        if( v > max_i ) sqDist += ( v - max_i ) * ( v - max_i );
    }
    return sqDist;
}

//------------------------------------------------------------------------------
// Ref #1 pp 141
Vector3 BoundingVolume::ClosestPtPointTriangle( Vector3& p, Vector3& a, Vector3& b, Vector3& c ) {
    // Check if P in vertex region outside A
    Vector3 ab = b - a;
    Vector3 ac = c - a;
    Vector3 ap = p - a;
    double d1 = Vector3::dot( ab, ap );
    double d2 = Vector3::dot( ac, ap );
    if( d1 <= 0.0 && d2 <= 0.0 ) return a;  // barycentric coordinates (1,0,0)

    // Check if P in vertex region outside B
    Vector3 bp = p - b;
    double d3 = Vector3::dot( ab, bp );
    double d4 = Vector3::dot( ac, bp );
    if( d3 >= 0.0 && d4 <= 0.0 ) return b;  // barycentric coordinates (0,1,0)

    double vc = d1*d4 - d3*d2;
    if( vc <= 0.0 && d1 >= 0.0 && d3 <= 0.0 ) {
        double v = d1 / (d1 - d3);
        return a + v * ab;                  // barycentric coordinates (1-v,v,0)
    }

    // Check if P in vertex region outside C
    Vector3 cp = p - c;
    double d5 = Vector3::dot( ab, cp );
    double d6 = Vector3::dot( ac, cp );
    if( d6 >= 0.0 && d5 <= d6 ) return c;  // barycentric coordinates (0,0,1)

    // Check if P in edge region of AC, if so return projection of P onto AC
    double vb = d5*d2 - d1*d6;
    if( vb <= 0.0 && d2 >= 0.0 && d6 <= 0.0 ) {
        double w = d2 / (d2 - d6);
        return a + w * ac;
    }

    // Check if P in edge region of BC, if so return projection of P onto BC
    double va = d3*d6 - d5*d4;
    if( va <= 0.0 && (d4 - d3) >= 0.0 && (d5 - d6) >= 0.0 ) {
        double w = (d4 - d3) / ((d4 - d3) + (d5 - d6));
        return b + w * (c - b);             // barycentric coordinates (0,1-w,w)
    }

    // P inside face region.  Conpute Q through its barysentric coordinates (u,v,w)
    double denom = 1.0 / (va + vb + vc);
    double v = vb * denom;
    double w = vc * denom;
    return a + ab * v + ac * w;             // = u*a + v*b + w*c, u = va * denom = 1.0 - v - w
}

//------------------------------------------------------------------------------
void BoundingVolume::ClosestPtPointSphere( Vector3& p, BoundingSphere& b, Vector3& q ) {
    assert( p != b.center );
    q = b.normal( p ) * b.radius + b.center;
}

//------------------------------------------------------------------------------
// Ref #1 pp 167
// Given point p, return the point q on or in AABB b that is closest to p
void BoundingVolume::ClosestPtPointAABB( Vector3& p, AABB& b, Vector3& q ) {
    // For each coordinate axis, if the point coordinate value is
    // outside box, clamp it to the box, else keep it as is

    for( unsigned int i = 0; i < 3; i++ ) {
        double v = p(i);
        double min_i = b.center(i) - b.extens(i);
        double max_i = b.center(i) + b.extens(i);
        if( v < min_i ) v = min_i;      // v = max( v, min_i )
        if( v > max_i ) v = max_i;      // v = min( v, max_i )
        q(i) = v;
    }
}

//------------------------------------------------------------------------------
// Ref #1 pp 167
// Returns true if sphere s intersects triangle ABC, false otherwise.
// The point p on abx closest to the sphere ceneter is also returned
bool BoundingVolume::TestSphereTriangle( BoundingSphere& s, Vector3& a, Vector3& b, Vector3& c, Vector3& p ) {
    // Find point P on triangle ABC closest to sphere center
    p = ClosestPtPointTriangle( s.center, a, b, c );
    // Sphere and triangle intersect if the (squared) distance from sphere
    // center to point p is less than the (squared) sphere radius
    Vector3 v = p - s.center;
    return Vector3::dot( v,v ) <= s.radius * s.radius;
}

//------------------------------------------------------------------------------
// Ref #1 pp 168
// Test whether sphere s intersects polygon p
// ** Note : need to add significant number of structures to support this function **
bool BoundingVolume::TestSpherePolygon( BoundingSphere& s, Polygon& p ) {
    /*
    // Compute normal for the plane of the polygon
    Vector3 n = p.normal;   //Vector3::cross(  );
    n.normalize();
    */

    return false; // bogus
}

//------------------------------------------------------------------------------
// Ref #1 pp 166
// Returns true if sphere s intersects AABB b, false otherwise.
// The point p on the AABB closest to the sphere center is also returned
bool BoundingVolume::TestSphereAABB( BoundingSphere& s, AABB& b, Vector3& p ) {
    // Find point p on AABB closest to sphere center
    ClosestPtPointAABB( s.center, b, p );

    // Sphere and AABB intersect if the (squared) distance from sphere
    // center to point p is less than the (squared) sphere radius
    Vector3 v = p - s.center;
    return Vector3::dot( v, v ) <= s.radius * s.radius;
}

//------------------------------------------------------------------------------
