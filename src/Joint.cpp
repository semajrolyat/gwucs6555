//------------------------------------------------------------------------------

#include <cs6555/Joint.h>

#include <cs6555/Constants.h>

#include <cs6555/Link.h>

//------------------------------------------------------------------------------
// Constructors
//------------------------------------------------------------------------------
/// Default Constructor
Joint::Joint( void ) {
    angle = Vector3( 0.0, 0.0, 0.0 );
    inboard_link = NULL;
    inboard_displacement = Vector3( 0.0, 0.0, 0.0 );
    outboard_link = NULL;
    outboard_displacement = Vector3( 0.0, 0.0, 0.0 );

    type = JOINT_TYPE_UNDEFINED;

    name = "";
}

//------------------------------------------------------------------------------
/// Copy Constructor
Joint::Joint( const Joint& joint ) {
    angle = joint.angle;
    type = joint.type;
    inboard_link = joint.inboard_link;
    inboard_displacement = joint.inboard_displacement;
    outboard_link = joint.outboard_link;
    outboard_displacement = joint.outboard_displacement;

    name = joint.name;
}

//------------------------------------------------------------------------------
/// Component Constructor
Joint::Joint( const EJointTypes& type,
       Link* inboard_link,
       const Vector3& inboard_displacement,
       Link* outboard_link,
       const Vector3& outboard_displacement
) {
    angle = Vector3( 0.0, 0.0, 0.0 );
    this->type = type;
    this->inboard_link = inboard_link;
    this->inboard_displacement = Vector3( inboard_displacement);
    this->outboard_link = outboard_link;
    this->outboard_displacement = Vector3( outboard_displacement );

    name = "";
}
//------------------------------------------------------------------------------
/// Name Constructor
Joint::Joint( const std::string& name ) {
    angle = Vector3( 0.0, 0.0, 0.0 );
    inboard_link = NULL;
    inboard_displacement = Vector3( 0.0, 0.0, 0.0 );

    outboard_link = NULL;
    outboard_displacement = Vector3( 0.0, 0.0, 0.0 );

    type = JOINT_TYPE_UNDEFINED;

    this->name = name;
}

//------------------------------------------------------------------------------
// Destructor
//------------------------------------------------------------------------------
/// Destructor
Joint::~Joint( void ) {

}

//------------------------------------------------------------------------------
/// Interpolate trajectories defined for the joint and descend the hierarchy to
/// interpolate any joints down the kinematic chain
void Joint::interpolate( const unsigned int& frame_number ) {

    // if this joint has keyframes interpolate
    if( trajectory.keyframes() ) {
        // if the joint is moving through the trajectory backward and it is a reversible trajectory
        // then decrement the frame tic; otherwise, increment it
        if( !trajectory.forward && trajectory.reversible ) {
            trajectory.frame_tic--;
        } else {
            trajectory.frame_tic++;
        }

        // updating the keyframe ids
        if( trajectory.reversible &&
            ( trajectory.frame_tic == trajectory.keyframe( trajectory.keyframes() - 1 ).frame ||
              ( trajectory.frame_tic == trajectory.keyframe( 0 ).frame && frame_number > trajectory.keyframe( 0 ).frame ) ) ) {
            // if the joint is reversible and has reached the end of its trajectory, swap the keyframes
                int temp = trajectory.prev_keyframe_id;
                trajectory.prev_keyframe_id = trajectory.next_keyframe_id;
                trajectory.next_keyframe_id = temp;
                trajectory.forward = !trajectory.forward;
        } else if( trajectory.cyclic && trajectory.frame_tic == trajectory.keyframe( trajectory.keyframes() - 1 ).frame ) {
            // if cyclic but not reversible and it has reached the end of the cycle, restart it
                trajectory.prev_keyframe_id = 0;
                trajectory.next_keyframe_id = 1;
                trajectory.frame_tic = 1;
        } else if( trajectory.forward ) {
          // check for a transition point over the keyframes.
            // if its a forward trajectory and the tic has reached a keyframe, then increment the identifiers
            if( trajectory.frame_tic == trajectory.keyframe( trajectory.next_keyframe_id ).frame ) {
                trajectory.prev_keyframe_id++;
                trajectory.next_keyframe_id++;
            }
        } else {
            // if its a backward trajectory and the tic has reached a keyframe, then decrement the identifiers
            if( trajectory.frame_tic == trajectory.keyframe( trajectory.next_keyframe_id ).frame ) {
                trajectory.prev_keyframe_id--;
                trajectory.next_keyframe_id--;
            }
        }

        // Now, do the interpolation

        // get the current keyframes
        Keyframe keyframe0 = trajectory.keyframe( trajectory.prev_keyframe_id );
        Keyframe keyframe1 = trajectory.keyframe( trajectory.next_keyframe_id );

        // blending matrix for the spline
        Eigen::MatrixXd C;

        // calculate 'time' in terms of frames
        double frame0 = (double) keyframe0.frame;
        double frame1 = (double) keyframe1.frame;
        double dframe = frame1 - frame0;
        double dt = ((double)trajectory.frame_tic - frame0) / dframe;

        // calculate the blending matrix C
        if( trajectory.forward )
            C = CubicSpline::blend( trajectory.spline_basis, trajectory.controlpoints, trajectory.next_keyframe_id + 1 );
        else
            C = CubicSpline::blend( trajectory.spline_basis, trajectory.controlpoints, trajectory.prev_keyframe_id + 1 );

        // euler integrate the joint angle
        double drotxdt = keyframe1.pose.orientation.x() - keyframe0.pose.orientation.x();
        double drotydt = keyframe1.pose.orientation.y() - keyframe0.pose.orientation.y();
        double drotzdt = keyframe1.pose.orientation.z() - keyframe0.pose.orientation.z();

        angle.x( keyframe0.pose.orientation.x() + drotxdt * dt );
        angle.y( keyframe0.pose.orientation.y() + drotydt * dt );
        angle.z( keyframe0.pose.orientation.z() + drotzdt * dt );
    }
    // descend the hierarchy via recursion
    for( unsigned int i = 0; i < outboard_link->child_joints(); i++ ) {
        Joint* child_joint = outboard_link->child_joint( i );
        child_joint->interpolate( frame_number );
    }
}
