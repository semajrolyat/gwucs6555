/*-----------------------THE GEORGE WASHINGTON UNIVERSITY-----------------------
James Taylor                                                         jrt@gwu.edu

Patch class definition

------------------------------------------------------------------------------*/

#ifndef _PATCH_H_
#define _PATCH_H_

//------------------------------------------------------------------------------

#include <cs6555/Geometry.h>
#include <cs6555/ControlPoint.h>

//------------------------------------------------------------------------------

typedef enum {
    PATCH_BASIS_UNKNOWN,
    PATCH_BASIS_SPLINE,
    PATCH_BASIS_NURBS
} EPatchBasis;
//------------------------------------------------------------------------------

class Patch : public Geometry {
public:
    // Constructor
    Patch( void );

    // Destructor
    virtual ~Patch( void );

    // [Base Class] Geometry::Type Queries
    virtual EGeometryType geometry_type( void ) { return GEOMETRY_TYPE_PATCH; }

    // TODO: add graph (grid) of control points
protected:
    ControlPoint m_cp[4][4];
};

//------------------------------------------------------------------------------

#endif // _PATCH_H_


