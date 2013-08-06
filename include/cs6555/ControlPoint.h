/*-----------------------THE GEORGE WASHINGTON UNIVERSITY-----------------------
James Taylor                                                         jrt@gwu.edu

ControlPoint class definition and implementation

controlpoints are the datastructures that contain information to support spline
implementaions.

------------------------------------------------------------------------------*/

#ifndef CONTROLPOINT_H
#define CONTROLPOINT_H

//------------------------------------------------------------------------------

#include <cs6555/Point3.h>

//------------------------------------------------------------------------------
/// Discrete parameter describing whether a control point is interim within a
/// spline or happens to be a terminus at the end.
typedef enum {
    CONTROL_POINT_END,
    CONTROL_POINT_MIDDLE
} EControlPointType;

//------------------------------------------------------------------------------

class ControlPoint : public Point3 {
public:
    // Constructors
    /// Default Constructor
    ControlPoint( void ) : Point3( ) {
        type = CONTROL_POINT_END;
    }
    /// Copy Constructor
    ControlPoint( const ControlPoint& point ) : Point3( point ) {
        type = point.type;
    }
    /// Component Constructor
    ControlPoint( const Vector3& position, const EControlPointType& type ) : Point3( position ) {
        this->type = type;
    }

    // Destructor
    /// Destructor
    virtual ~ControlPoint( void ) { }

    // Member Data
    EControlPointType type;         // type of the control point either end or middle

};

//------------------------------------------------------------------------------

#endif // CONTROLPOINT_H
