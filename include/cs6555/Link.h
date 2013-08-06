/*-----------------------THE GEORGE WASHINGTON UNIVERSITY-----------------------
James Taylor                                                         jrt@gwu.edu

Link class definition

Links are a part of the articulated body hierarchy and define the offset and
the geometry for a given edge in the hierarchy.
------------------------------------------------------------------------------*/

#ifndef _LINK_H_
#define _LINK_H_

//------------------------------------------------------------------------------

#include <vector>
#include <cs6555/Math/Vector3.h>
#include <cs6555/Mesh.h>
#include <cs6555/Pose.h>
#include <cs6555/Trajectory.h>
#include <cs6555/Geometry.h>
#include <cs6555/RigidBody.h>

#include <string>
#include <stack>

//------------------------------------------------------------------------------

// Forward Declaration of a Joint
class Joint;

//------------------------------------------------------------------------------

// Note: a link is geometry BUT it may have both bone and skin geometries
// so it is actually a root of a geometric hierarchy.  At this point the
// bone link is the root, but it would be possible to push other geometries
// into this hierarchy

class Link : public RigidBody {
public:
    // Constructors
    Link( void );
    Link( const std::string& name );

    // Destructor
    virtual ~Link( void );

    virtual EGeometryType geometry_type( void );

    // Joint Insertion and Queries
    Joint* child_joint( const unsigned int& index );
    unsigned int child_joints( void );
    void append_child_joint( Joint* j );

    void transform( std::stack<Matrix4>& matrixstack );

    // Member Data
    Vector3 center_of_mass;     // center of the link or COM

    Joint* parent_joint;        // reference to the parent joint.  Allows traversal back up the hierarchy

    std::string name;           // name of the link for searches

private:
    unsigned int m_number_child_joints;     // total number of joints attached to this link
    std::vector<Joint*> m_child_joints;     // the reference buffer for all joints in this hierarchy
};

//------------------------------------------------------------------------------

#endif // _LINK_H_
