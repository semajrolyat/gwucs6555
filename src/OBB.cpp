/*-----------------------THE GEORGE WASHINGTON UNIVERSITY-----------------------
James Taylor                                                         jrt@gwu.edu

OBB class implementation

------------------------------------------------------------------------------*/

#include <cs6555/OBB.h>

//------------------------------------------------------------------------------
// Constructors
//------------------------------------------------------------------------------
/// Default Constructor
OBB::OBB( void ) {

}

//------------------------------------------------------------------------------
// Destructor
//------------------------------------------------------------------------------
/// Destructor
OBB::~OBB( void ) {

}

//------------------------------------------------------------------------------
// [Base Class] BoundingVolume::Type Queries
//------------------------------------------------------------------------------
/// BoundingVolume Type Query.  Implemented by inheritor.
EBoundingVolumeType OBB::bounding_volume_type( void ) {
    return BV_TYPE_OBB;
}
