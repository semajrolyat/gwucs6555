/*-----------------------THE GEORGE WASHINGTON UNIVERSITY-----------------------
James Taylor                                                         jrt@gwu.edu

BVH (Bounding Volume Hierarchy) class definition

------------------------------------------------------------------------------*/

#ifndef _BVH_H_
#define _BVH_H_

//------------------------------------------------------------------------------

#include <cs6555/BoundingVolume.h>
#include <vector>

//------------------------------------------------------------------------------

class BVH {
public:
    // Constructors
    BVH( void );

    // Destructor
    virtual ~BVH( void );

    // Member Data
private:
    std::vector<BoundingVolume*> m_child_bounding_volumes;
    int m_child_bounding_volume_count;
};

//------------------------------------------------------------------------------

#endif // _BVH_H_
