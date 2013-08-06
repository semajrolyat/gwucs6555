/*-----------------------THE GEORGE WASHINGTON UNIVERSITY-----------------------
James Taylor                                                         jrt@gwu.edu

Joint class definition

Joints are a part of the articulated body hierarchy and define the transformation
from inboard (parent) frame to outboard (child) frame.  Additionally, joints
may articulate and are therefore dynamic features such that the transformation
from frame to frame needs to be recalculated between movements in time.  Joints
may have a trajectory through which they travel and should be constrained by
limits imposed on the overall amount of travel they may have in a given
rotational degree of freedom.
------------------------------------------------------------------------------*/

#ifndef _JOINT_H_
#define _JOINT_H_

//------------------------------------------------------------------------------

#include <vector>

#include <cs6555/Math/Vector3.h>
#include <cs6555/Math/Matrix4.h>
#include <cs6555/Trajectory.h>
#include <string>

//------------------------------------------------------------------------------

// only supporting revolute at this point
/// Discrete set of Joint Types
typedef enum {
    JOINT_TYPE_UNDEFINED,
    JOINT_TYPE_PRISMATIC,
    JOINT_TYPE_PLANAR,
    JOINT_TYPE_REVOLUTE,
    JOINT_TYPE_CYLINDRICAL,
    JOINT_TYPE_SPHERICAL,
    JOINT_TYPE_SCREW
} EJointTypes;

/// Discrete set of primary Joint Axes
typedef enum {
    JOINT_AXIS_NONE = 0x00,
    JOINT_AXIS_X = 0x01,
    JOINT_AXIS_Y = 0x02,
    JOINT_AXIS_Z = 0x04
} EJointAxes;

//------------------------------------------------------------------------------

// forward declaration
class Link;

//------------------------------------------------------------------------------

class Joint {
public:
    // Constructors
    Joint( void );
    Joint( const Joint& joint );
    Joint( const EJointTypes& type,
           Link* inboard_link,
           const Vector3& inboard_displacement,
           Link* outboard_link,
           const Vector3& outboard_displacement
    );
    Joint( const std::string& name );

    // Destructor
    virtual ~Joint( void );

    void interpolate( const unsigned int& frame_number );

    // Member Data
    Vector3 angle;                  // current joint angle
    Vector3 median_angle;           // median angle of a joint trajectory.  Not computed.  Only used in rendering joint trajectories.

    EJointTypes type;               // the type of this joint

    Link* inboard_link;             // points to the parent link
    Vector3 inboard_displacement;   // distance from the parent link COM to the joint

    Link* outboard_link;            // points to the child link
    Vector3 outboard_displacement;  // distance from the joint to the child link COM

    Matrix4 frame_transformation;   // joint frame transformation from inboard frame to outboard frame.  orientation of the joint.  Permutation matrix for orthogonal joints

    std::string name;               // name of the joint for searches

    Trajectory trajectory;          // the joint trajectory.  Note position of the pose definition is only significant if wanting to visualize the trajectory

    /*
      // To be applied later when physical simulations are implemented
        // joint constraints
        Vector3 max_rotation;
        Vector3 min_rotation;
        // won't be used at this time (for prismatic/cylindrical/screw joints)
        Vector3 max_translation;
        Vector3 min_translation;
    */

};

//------------------------------------------------------------------------------

#endif // _JOINT_H_
