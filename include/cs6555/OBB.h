/*-----------------------THE GEORGE WASHINGTON UNIVERSITY-----------------------
James Taylor                                                         jrt@gwu.edu

OBB (Oriented Bounding Box) class definition

------------------------------------------------------------------------------*/

#ifndef _OBB_H_
#define _OBB_H_

//------------------------------------------------------------------------------

#include <cs6555/BoundingVolume.h>
#include <cs6555/Math/Vector3.h>
#include <cs6555/Math/Matrix3.h>

//------------------------------------------------------------------------------

class OBB : public BoundingVolume {
public:
    // Constructors
    OBB( void );

    // Destructor
    virtual ~OBB( void );

    // [Base Class] BoundingVolume::Type Queries
    virtual EBoundingVolumeType bounding_volume_type( void );

    // Member Data
    Vector3 position;
    Vector3 extens;
    Matrix3 axes;
};

//------------------------------------------------------------------------------

#endif // _OBB_H_
