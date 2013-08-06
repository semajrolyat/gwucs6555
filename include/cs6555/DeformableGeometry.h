/*-----------------------THE GEORGE WASHINGTON UNIVERSITY-----------------------
James Taylor                                                         jrt@gwu.edu

DeformableGeometry class definition

------------------------------------------------------------------------------*/
#ifndef _DEFORMABLEGEOMETRY_H_
#define _DEFORMABLEGEOMETRY_H_

//------------------------------------------------------------------------------

#include <cs6555/Geometry.h>

//------------------------------------------------------------------------------
// Enums
//------------------------------------------------------------------------------
typedef enum {
    DEFORMABLE_TYPE_UNKNOWN,
    DEFORMABLE_TYPE_BODY,
    DEFORMABLE_TYPE_SPACE
} EDeformableGeometryType;
//------------------------------------------------------------------------------

class DeformableGeometry : public Geometry {
public:
    DeformableGeometry( void ) { }
    virtual ~DeformableGeometry( void ) { }

    virtual EGeometryType geometry_type( void ) { return GEOMETRY_TYPE_BODY; }

    virtual EDeformableGeometryType deformable_type( void ) { return DEFORMABLE_TYPE_BODY; }
};

//------------------------------------------------------------------------------

#endif // _DEFORMABLEGEOMETRY_H_
