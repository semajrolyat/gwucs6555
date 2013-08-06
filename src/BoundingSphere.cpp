/*-----------------------THE GEORGE WASHINGTON UNIVERSITY-----------------------
James Taylor                                                         jrt@gwu.edu

BoundingSphere class implementation

------------------------------------------------------------------------------*/

#include <cs6555/BoundingSphere.h>
#include <assert.h>

//------------------------------------------------------------------------------
// Constructors
//------------------------------------------------------------------------------
/// Component Constructor
BoundingSphere::BoundingSphere( const double& radius ) {
    assert( radius > 0 );
    this->radius = radius;
}

//------------------------------------------------------------------------------
// Destructor
//------------------------------------------------------------------------------
/// Destructor
BoundingSphere::~BoundingSphere( void ) {

}

//------------------------------------------------------------------------------
// [Base Class] BoundingVolume::Type Queries
//------------------------------------------------------------------------------
/// BoundingVolume Type Query.  Implemented by inheritor.
EBoundingVolumeType BoundingSphere::bounding_volume_type( void ) {
    return BV_TYPE_SPHERE;
}

//------------------------------------------------------------------------------
Vector3 BoundingSphere::normal( Vector3& point_in_space ) {
    assert( point_in_space != center );

    Vector3 n = point_in_space - center;
    n.normalize();
    return n;
}
