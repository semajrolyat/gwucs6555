/*-----------------------THE GEORGE WASHINGTON UNIVERSITY-----------------------
James Taylor                                                         jrt@gwu.edu

AABB (Axis-Aligned Bounding Box) class definition

Uses a center/half-width extens representation as it is easy to convert to
bounding sphere
------------------------------------------------------------------------------*/

#ifndef _AABB_H_
#define _AABB_H_

//------------------------------------------------------------------------------

#include <cs6555/BoundingVolume.h>
#include <cs6555/Math/Vector3.h>

//------------------------------------------------------------------------------

class AABB : public BoundingVolume {
public:
    // Constructors
    AABB( const Vector3& center, const Vector3& extens );

    // Destructor
    virtual ~AABB( void );

    // [Base Class] BoundingVolume::Type Queries
    virtual EBoundingVolumeType bounding_volume_type( void );

    // Member Data
    Vector3 center;
    Vector3 extens;

    Vector3 normal( Vector3& point_on_aabb );
};

//------------------------------------------------------------------------------

#endif // _AABB_H_
