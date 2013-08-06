#ifndef _BODY_H_
#define _BODY_H_

//------------------------------------------------------------------------------

#include <cs6555/Geometry.h>
#include <cs6555/Trajectory.h>

//------------------------------------------------------------------------------

typedef enum {
    BODY_TYPE_UNDEFINED,
    BODY_TYPE_POINTMASS,
    BODY_TYPE_RIGID,
    BODY_TYPE_ARTICULATED,
    BODY_TYPE_DEFORMABLE
} EBodyType;

//------------------------------------------------------------------------------

class Body : public Geometry {
public:
    Body( void ) { }
    virtual ~Body( void ) { }

    virtual EGeometryType geometry_type( void ) { return GEOMETRY_TYPE_BODY; }
    virtual EBodyType body_type( void ) { return BODY_TYPE_UNDEFINED; }

    Trajectory trajectory;      // Most links don't have trajectory.  Artifact of development to be refactored.  Trajectory should really be a function of the body.  Added so root_link can have trajectory, but should instead by for overall body.
    double distance_travelled;  // how far has this link travelled.  Artifact of development.  Still here b/c some unit tests assumed this requirement.
    double trajectory_position; // where the link over its trajectory.  Artifact of development.  See above.

    double      mass;

    Matrix3     body_frame_inertial_tensor;
    Matrix3     body_frame_inertial_tensor_inverse;

    Vector3     position;
    Matrix3     rotation;
    Quaternion  q_rotation;
    Vector3     linear_momentum;
    Vector3     angular_momentum;

    Matrix3     inertial_tensor_inverse;
    Vector3     linear_velocity;
    Vector3     angular_velocity;

    Vector3     force;
    Vector3     torque;

    double      coefficient_of_restitution;

};

//------------------------------------------------------------------------------

#endif // _BODY_H_
