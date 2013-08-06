/*-----------------------THE GEORGE WASHINGTON UNIVERSITY-----------------------
James Taylor                                                         jrt@gwu.edu

BoundingSphere class definition

------------------------------------------------------------------------------*/

#ifndef _BOUNDINGSPHERE_H_
#define _BOUNDINGSPHERE_H_

//------------------------------------------------------------------------------

#include <cs6555/BoundingVolume.h>
#include <cs6555/Math/Vector3.h>

//------------------------------------------------------------------------------

class BoundingSphere : public BoundingVolume {
public:
    // Constructors
    BoundingSphere( const double& radius );

    // Destructor
    virtual ~BoundingSphere( void );

    // [Base Class] BoundingVolume::Type Queries
    virtual EBoundingVolumeType bounding_volume_type( void );

    Vector3 normal( Vector3& point_in_space );

    // Member Data
    Vector3 center;
    double radius;
};

//------------------------------------------------------------------------------

#endif // _BOUNDINGSPHERE_H_
