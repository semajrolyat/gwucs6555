/*-----------------------THE GEORGE WASHINGTON UNIVERSITY-----------------------
James Taylor                                                         jrt@gwu.edu

ArticulatedBody class definition

The articulated body defines the hierarchy of links and joints where links are
nodes and joints are edges.  Therefore, a link may have many joints attached to
its reference frame, but a joint has only a one-to-one correspondence between
parent link and child link.
------------------------------------------------------------------------------*/

#ifndef _ARTICULATEDBODY_H_
#define _ARTICULATEDBODY_H_

//------------------------------------------------------------------------------

#include <cs6555/Link.h>
#include <cs6555/Joint.h>

#include <vector>
#include <stack>

#include <cs6555/Body.h>

//------------------------------------------------------------------------------

class ArticulatedBody : public Body {
public:
    // Constructor
    ArticulatedBody( void );

    // Destructor
    virtual ~ArticulatedBody( void );

    // Link Insertion and Queries
    Link* link( const unsigned int& index );
    unsigned int links( void );
    void append_link( Link* l );

    // Joint Insertion and Queries
    Joint* joint( const unsigned int& index );
    Joint* joint( const std::string& name );
    unsigned int joints( void );
    void append_joint( Joint* j );

    // Predetermined Hierarchies
    void init_humanoid( void );
    void init_biped( void );
    void init_boid( const double& scale = 1.0, const unsigned int& period_in_frames = 16 );

    virtual EBodyType body_type( void );

    void construct_calf_swing_trajectory( Trajectory* trajectory, const unsigned int& period_in_frames );
    void construct_forearm_swing_trajectory( Trajectory* trajectory, const unsigned int& period_in_frames );
    void construct_pendulum_trajectory( Trajectory* trajectory, const unsigned int& period_in_frames );

    void construct_wing_trajectory( Trajectory* trajectory, const unsigned int& period_in_frames );

    void interpolate( const unsigned int& frame_number );
    void transform( std::stack<Matrix4>& matrixstack );

    // Member Data
    Link* root_link;                // the root of the hierarchy

private:
    unsigned int m_link_count;      // the total number of links in the hierarchy
    std::vector<Link*> m_links;     // the reference buffer for all links in the hierarchy

    unsigned int m_joint_count;      // the total number of joints in the hierarchy
    std::vector<Joint*> m_joints;    // the reference buffer for all joints in the hierarchy
};

//------------------------------------------------------------------------------

#endif // _ARTICULATEDBODY_H_
