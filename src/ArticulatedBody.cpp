/*-----------------------THE GEORGE WASHINGTON UNIVERSITY-----------------------
James Taylor                                                         jrt@gwu.edu

ArticulatedBody class implementation

------------------------------------------------------------------------------*/

#include <cs6555/ArticulatedBody.h>

#include <assert.h>

#include <cs6555/MeshLoader.h>

//------------------------------------------------------------------------------
// Constructors
//------------------------------------------------------------------------------
/// Default Constructor
ArticulatedBody::ArticulatedBody( void ) {
    m_link_count = 0;
    m_joint_count = 0;
    distance_travelled = 0.0;
    trajectory_position = 0.0;
}

//------------------------------------------------------------------------------
// Destructor
//------------------------------------------------------------------------------
/// Destructor
ArticulatedBody::~ArticulatedBody( void ) {
    root_link = NULL;

    Link* l;
    while( !m_links.empty( ) ) {
        l = *m_links.end( );
        m_links.pop_back( );
        //if( l != NULL ) delete l;
        delete l;
    }
    Joint* j;
    while( !m_joints.empty( ) ) {
        j = *m_joints.end( );
        m_joints.pop_back( );
        //if( j != NULL ) delete j;
        delete j;
    }
}

/*
//------------------------------------------------------------------------------
// [Base Class] Geometry::Queries
//------------------------------------------------------------------------------
/// Geometry Type Query.  Implemented by inheritor.
EGeometryType ArticulatedBody::geometry_type( void ) {
    return GEOMETRY_TYPE_UNKNOWN;
}
*/
//------------------------------------------------------------------------------
// [Base Class] Body::Queries
//------------------------------------------------------------------------------
/// Body Type Query.  Implemented by inheritor.
EBodyType ArticulatedBody::body_type( void ) {
    return BODY_TYPE_ARTICULATED;
}

//------------------------------------------------------------------------------
//
//------------------------------------------------------------------------------
/// Interpolate the articulated body trajectory.
void ArticulatedBody::interpolate( const unsigned int& frame_number ) {
    trajectory.frame_tic++;

    bool transition = false;

    // handle special transitions such as the cycle or intermediate transitions from keyframe to keyframe
    if( trajectory.frame_tic == trajectory.keyframe( trajectory.keyframes() - 1 ).frame ) {
        trajectory_position = 0.0;
        trajectory.prev_keyframe_id = 0;
        trajectory.next_keyframe_id = 1;
        trajectory.frame_tic = 1;
    } else if( trajectory.frame_tic == trajectory.keyframe( trajectory.next_keyframe_id ).frame ) {
        trajectory.prev_keyframe_id++;
        trajectory.next_keyframe_id++;
        transition = true;
    }

    Keyframe keyframe0 = trajectory.keyframe( trajectory.prev_keyframe_id );
    Keyframe keyframe1 = trajectory.keyframe( trajectory.next_keyframe_id );

    double frame0 = (double) keyframe0.frame;
    double frame1 = (double) keyframe1.frame;
    double dframe = frame1 - frame0;
    double u = 0.0;
    Eigen::MatrixXd C;

    SdS sds_frame0 = trajectory.spline_arclength_map.at( trajectory.prev_keyframe_id + 1 );
    SdS sds_frame1 = trajectory.spline_arclength_map.at( trajectory.next_keyframe_id + 1 );

    double dsdframe = sds_frame1.second / dframe;
    double dt = ((double)trajectory.frame_tic - frame0) / dframe;

    C = CubicSpline::blend( trajectory.spline_basis, trajectory.controlpoints, trajectory.next_keyframe_id + 1 );

    // integrate rotation
    double drotxdt = keyframe1.pose.orientation.x() - keyframe0.pose.orientation.x();
    double drotydt = keyframe1.pose.orientation.y() - keyframe0.pose.orientation.y();
    double drotzdt = keyframe1.pose.orientation.z() - keyframe0.pose.orientation.z();

    pose.orientation.x( keyframe0.pose.orientation.x() + drotxdt * dt );
    pose.orientation.y( keyframe0.pose.orientation.y() + drotydt * dt );
    pose.orientation.z( keyframe0.pose.orientation.z() + drotzdt * dt );

    // integrate position
    double length_of_current_spline = sds_frame1.second;
    double position_on_current_spline = trajectory_position - sds_frame0.first + dsdframe;

    u = position_on_current_spline / length_of_current_spline;
    trajectory_position += dsdframe;

    distance_travelled += dsdframe;
    pose.position = CubicSpline::position( C, u );
    pose.transformByQuaternion();

    // now descend the hierarchy and interpolate all joints
    for( unsigned int i = 0; i < root_link->child_joints(); i++ ) {
        Joint* joint = root_link->child_joint( i );
        joint->interpolate( frame_number );
    }
}
//------------------------------------------------------------------------------
/// Entry point for transformation of an articulated body hierarchy.
void ArticulatedBody::transform( std::stack<Matrix4>& matrixstack ) {
    // push the root link's transform
    matrixstack.push( pose.transform );

    root_link->pose = pose;
    root_link->pose.transform = pose.transform;

    // descend the heirarchy and update the poses of all children
    root_link->transform( matrixstack );

    // pop the root link's transform
    matrixstack.pop();
}

//------------------------------------------------------------------------------
// Link Insertion and Queries
//------------------------------------------------------------------------------
/// Get a link
Link* ArticulatedBody::link( const unsigned int& index ) {
    assert( index < m_link_count );
    return m_links.at( index );
}

//------------------------------------------------------------------------------
/// Get the number of links
unsigned int ArticulatedBody::links( void ) {
    return m_link_count;
}

//------------------------------------------------------------------------------
/// Append a new link into the structure
void ArticulatedBody::append_link( Link* l ) {
    m_links.push_back( l );
    m_link_count++;
}

//------------------------------------------------------------------------------
// Joint Insertion and Queries
//------------------------------------------------------------------------------
/// Get a joint by index
Joint* ArticulatedBody::joint( const unsigned int& index ) {
    assert( index < m_joint_count );
    return m_joints.at( index );
}

//------------------------------------------------------------------------------
/// Get a joint by name
Joint* ArticulatedBody::joint( const std::string& name ) {
    for( std::vector<Joint*>::iterator it = m_joints.begin(); it != m_joints.end(); it++ ) {
        if( (*it)->name == name )
            return (*it);
    }
    return NULL;
}

//------------------------------------------------------------------------------
/// Get the number of joints
unsigned int ArticulatedBody::joints( void ) {
    return m_joint_count;
}

//------------------------------------------------------------------------------
/// Append a new joint into the structure
void ArticulatedBody::append_joint( Joint* j ) {
    m_joints.push_back( j );
    m_joint_count++;
}

//------------------------------------------------------------------------------
// Predetermined Hierarchies
//------------------------------------------------------------------------------
/// Initialize a bipedal Articulated Body
/// Note, this was actually abandoned and instead implemented directly in the
/// Unit Test.
void ArticulatedBody::init_biped( void ) {
    Material material = Material(
                Color( 1.0, 1.0, 1.0, 1.0 ),
                Color( 1.0, 1.0, 1.0, 1.0 ),
                Color( 1.0, 1.0, 1.0, 1.0 ),
                Color( 0.0, 0.0, 0.0, 1.0 ),
                10                           );

    unsigned int period_in_frames = 20;

    Link* pelvis = new Link( "pelvis" );
    Joint* left_hip = new Joint( "left_hip" );
    Joint* right_hip = new Joint( "right_hip" );
    Link* left_leg = new Link( "left_leg" );
    Link* right_leg = new Link( "right_leg" );

    append_link( pelvis );
    append_joint( left_hip );
    append_joint( right_hip );
    append_link( left_leg );
    append_link( right_leg );

    trajectory.construct_cyclic_circle_trajectory( 80, true, Vector3(-PI/2, 0.0, 0.0), 2.0, Vector3(0.0, 0.0, 0.0) );
    //trajectory.construct_cyclic_circle_trajectory( 160, true, Vector3(-PI/2, 0.0, 0.0), 2.0, Vector3(0.0, 0.0, 0.0) );
    trajectory.spline_basis = CubicSpline::basisUniformNonrationalBSpline();
    trajectory.spline_arclength_map = CubicSpline::map_spline( CUBIC_SPLINE_B, trajectory.controlpoints );

    root_link = pelvis;

    pelvis->pose.position = Vector3( 0.0, 0.0, 0.0 );
    pelvis->insert( MeshLoader::box( 1.0, 0.5, 0.5 ) );
    pelvis->material = material;
    pelvis->pose.transform.identity();

    pelvis->append_child_joint( left_hip );
    left_hip->inboard_link = pelvis;
    left_hip->inboard_displacement = Vector3( 0.75, 0.0, 0.0 );
    left_hip->outboard_link = left_leg;
    left_hip->outboard_displacement = Vector3( 0.0, 1.0, 0.0 );
    construct_pendulum_trajectory( &left_hip->trajectory, period_in_frames );
    left_hip->trajectory.spline_basis = CubicSpline::basisUniformNonrationalBSpline();
    left_hip->trajectory.spline_arclength_map = CubicSpline::map_spline( CUBIC_SPLINE_B, left_hip->trajectory.controlpoints );
    left_hip->trajectory.prev_keyframe_id = 1;
    left_hip->trajectory.next_keyframe_id = 2;
    left_hip->trajectory.frame_tic = period_in_frames/2 + 1;
    left_hip->frame_transformation = Matrix4( 0, 0, 1, 0,
                                             -1, 0, 0, 0,
                                              0, 1, 0, 0,
                                              0, 0, 0, 1 );
    left_hip->median_angle = Vector3( 0.0, 0.0, PI/2 );

    pelvis->append_child_joint( right_hip );
    right_hip->inboard_link = pelvis;
    right_hip->inboard_displacement = Vector3( -0.75, 0.0, 0.0 );
    right_hip->outboard_link = right_leg;
    right_hip->outboard_displacement = Vector3( 0.0, 1.0, 0.0 );
    construct_pendulum_trajectory( &right_hip->trajectory, period_in_frames );
    right_hip->trajectory.spline_basis = CubicSpline::basisUniformNonrationalBSpline();
    right_hip->trajectory.spline_arclength_map = CubicSpline::map_spline( CUBIC_SPLINE_B, left_hip->trajectory.controlpoints );
    right_hip->trajectory.prev_keyframe_id = 2;
    right_hip->trajectory.next_keyframe_id = 1;
    right_hip->trajectory.frame_tic = period_in_frames/2 + 1;
    right_hip->trajectory.forward = false;
    right_hip->frame_transformation = Matrix4( 0, 0, 1, 0,
                                              -1, 0, 0, 0,
                                               0, 1, 0, 0,
                                               0, 0, 0, 1 );
    right_hip->median_angle = Vector3( 0.0, 0.0, PI/2 );

    left_leg->insert( MeshLoader::box( 0.5, 2.5, 0.5 ) );
    left_leg->parent_joint = left_hip;
    left_leg->material = material;
    left_leg->pose.transform.identity();

    right_leg->insert( MeshLoader::box( 0.5, 2.5, 0.5 ) );
    right_leg->parent_joint = right_hip;
    right_leg->material = material;
    right_leg->pose.transform.identity();

    /*
    Material material = Material(
                Color( 1.0, 1.0, 1.0, 1.0 ),
                Color( 1.0, 1.0, 1.0, 1.0 ),
                Color( 1.0, 1.0, 1.0, 1.0 ),
                Color( 0.0, 0.0, 0.0, 1.0 ),
                10                           );

    Link* pelvis = new Link( "pelvis" );
    Joint* left_hip = new Joint( "left_hip" );
    Joint* right_hip = new Joint( "right_hip" );
    Link* left_leg = new Link( "left_leg" );
    Link* right_leg = new Link( "right_leg" );

    append_link( pelvis );
    append_joint( left_hip );
    append_joint( right_hip );
    append_link( left_leg );
    append_link( right_leg );

    trajectory.construct_cyclic_circle_trajectory( 800, true, Vector3(-PI/2, 0.0, 0.0), 2.0, Vector3(0.0, 0.0, 0.0) );
    trajectory.spline_basis = CubicSpline::basisUniformNonrationalBSpline();
    trajectory.spline_arclength_map = CubicSpline::map_spline( CUBIC_SPLINE_B, pelvis->trajectory.controlpoints );

    root_link = pelvis;

    pelvis->pose.position = Vector3( 0.0, 0.0, 0.0 );
    pelvis->insert( MeshLoader::box( 2.0, 0.5, 0.5 ) );
    pelvis->material = material;

    //Vector3 hip_trajectory_orientation = Vector3( -PI/2, -PI/2, 0.0 );
    Vector3 hip_trajectory_orientation = Vector3( 0.0, 0.0, 0.0 );

    pelvis->append_child_joint( left_hip );
    left_hip->inboard_link = pelvis;
    left_hip->inboard_displacement = Vector3( 1.0, 0.0, 0.0 );
    left_hip->outboard_link = left_leg;
    left_hip->outboard_displacement = Vector3( 0.0, 1.0, 0.0 );
    //left_hip->axis_of_rotation = JOINT_AXIS_X;
    left_hip->trajectory.construct_pendulum_trajectory( 200, true, hip_trajectory_orientation );
    //left_hip->trajectory.construct_pendulum_trajectory( 200, true, hip_trajectory_orientation, PI, 2.0 );
    left_hip->trajectory.spline_basis = CubicSpline::basisUniformNonrationalBSpline();
    left_hip->trajectory.spline_arclength_map = CubicSpline::map_spline( CUBIC_SPLINE_B, left_hip->trajectory.controlpoints );

    pelvis->append_child_joint( right_hip );
    right_hip->inboard_link = pelvis;
    right_hip->inboard_displacement = Vector3( -1.0, 0.0, 0.0 );
    right_hip->outboard_link = right_leg;
    right_hip->outboard_displacement = Vector3( 0.0, 1.0, 0.0 );
    //right_hip->axis_of_rotation = JOINT_AXIS_X;
    right_hip->trajectory.construct_pendulum_trajectory( 200, false, hip_trajectory_orientation );
    //right_hip->trajectory.construct_pendulum_trajectory( 200, false, hip_trajectory_orientation, PI, 2.0 );
    right_hip->trajectory.spline_basis = CubicSpline::basisUniformNonrationalBSpline();
    right_hip->trajectory.spline_arclength_map = CubicSpline::map_spline( CUBIC_SPLINE_B, right_hip->trajectory.controlpoints );

    //Vector3 leg_trajectory_orientation = Vector3( -PI/2, -PI/2, 0.0 );
    //Vector3 leg_trajectory_orientation = Vector3( 0.0, 0.0, 0.0 );

    left_leg->insert( MeshLoader::box( 0.5, 2.0, 0.5 ) );
    left_leg->parent_joint = left_hip;
//    left_leg->trajectory.construct_pendulum_trajectory( 200, true, leg_trajectory_orientation );
//    //left_leg->trajectory.construct_pendulum_trajectory( 200, true, leg_trajectory_orientation, PI, 2.0 );
//    left_leg->trajectory.spline_basis = CubicSpline::basisUniformNonrationalBSpline();
//    left_leg->trajectory.spline_arclength_map = CubicSpline::map_spline( CUBIC_SPLINE_B, left_leg->trajectory.controlpoints );
    left_leg->material = material;

    right_leg->insert( MeshLoader::box( 0.5, 2.0, 0.5 ) );
    right_leg->parent_joint = right_hip;
//    right_leg->trajectory.construct_pendulum_trajectory( 200, false, leg_trajectory_orientation );
//    //right_leg->trajectory.construct_pendulum_trajectory( 200, false, leg_trajectory_orientation, PI, 2.0 );
//    right_leg->trajectory.spline_basis = CubicSpline::basisUniformNonrationalBSpline();
//    right_leg->trajectory.spline_arclength_map = CubicSpline::map_spline( CUBIC_SPLINE_B, right_leg->trajectory.controlpoints );
    right_leg->material = material;
    */
}

//------------------------------------------------------------------------------
/// Initialize a humanoid Articulated Body
/// Note, this was actually abandoned and instead implemented directly in the
/// Unit Test and App.
void ArticulatedBody::init_humanoid( void ) {
    Material material = Material(
                Color( 1.0, 1.0, 1.0, 1.0 ),
                Color( 1.0, 1.0, 1.0, 1.0 ),
                Color( 1.0, 1.0, 1.0, 1.0 ),
                Color( 0.0, 0.0, 0.0, 1.0 ),
                10                           );

    unsigned int period_in_frames = 20;
    //unsigned int period_in_frames = 400;

    Link* pelvis = new Link( "pelvis" );
    Joint* left_hip = new Joint( "left_hip" );
    Joint* right_hip = new Joint( "right_hip" );
    Link* left_thigh = new Link( "left_thigh" );
    Link* right_thigh = new Link( "right_thigh" );
    Joint* left_knee = new Joint( "left_knee" );
    Joint* right_knee = new Joint( "right_knee" );
    Link* left_calf = new Link( "left_calf" );
    Link* right_calf = new Link( "right_calf" );

    Joint* sacrum = new Joint( "sacrum" );
    Link* spine = new Link( "spine" );

    Joint* thoracic = new Joint( "thoracic" );
    Link* shoulders = new Link( "shoulders" );
    Joint* left_shoulder = new Joint( "left_shoulder" );
    Joint* right_shoulder = new Joint( "right_shoulder" );
    Link* left_arm = new Link( "left_arm" );
    Link* right_arm = new Link( "right_arm" );
    Joint* left_elbow = new Joint( "left_elbow" );
    Joint* right_elbow = new Joint( "right_elbow" );
    Link* left_forearm = new Link( "left_forearm" );
    Link* right_forearm = new Link( "right_forearm" );

    Joint* cervical = new Joint( "cervical" );
    Link* neck = new Link( "neck" );
    Joint* condyloid = new Joint( "condyloid" );
    Link* head = new Link( "head" );

    append_link( pelvis );
    append_joint( left_hip );
    append_joint( right_hip );
    append_link( left_thigh );
    append_link( right_thigh );
    append_joint( left_knee );
    append_joint( right_knee );
    append_link( left_calf );
    append_link( right_calf );

    append_joint( sacrum );
    append_link( spine );
    append_joint( thoracic );

    append_link( shoulders );
    append_joint( left_shoulder );
    append_joint( right_shoulder );
    append_link( left_arm );
    append_link( right_arm );
    append_joint( left_elbow );
    append_joint( right_elbow );
    append_link( left_forearm );
    append_link( right_forearm );

    append_joint( cervical );
    append_link( neck );
    append_joint( condyloid );
    append_link( head );

    trajectory.construct_cyclic_circle_trajectory( period_in_frames * 4, true, Vector3(-PI/2, 0.0, 0.0), 7.0, Vector3(0.0, 0.0, 0.0) );
    //trajectory.construct_cyclic_circle_trajectory( 80, true, Vector3(-PI/2, 0.0, 0.0), 5.0, Vector3(0.0, 0.0, 0.0) );
    //trajectory.construct_cyclic_circle_trajectory( 160, true, Vector3(-PI/2, 0.0, 0.0), 2.0, Vector3(0.0, 0.0, 0.0) );
    trajectory.spline_basis = CubicSpline::basisUniformNonrationalBSpline();
    trajectory.spline_arclength_map = CubicSpline::map_spline( CUBIC_SPLINE_B, trajectory.controlpoints );

    root_link = pelvis;

    pelvis->pose.position = Vector3( 0.0, 0.0, 0.0 );
    pelvis->insert( MeshLoader::box( 1.0, 0.5, 0.5 ) );
    pelvis->material = material;
    pelvis->pose.transform.identity();

    pelvis->append_child_joint( left_hip );
    left_hip->inboard_link = pelvis;
    left_hip->inboard_displacement = Vector3( 0.75, 0.0, 0.0 );
    left_hip->outboard_link = left_thigh;
    left_hip->outboard_displacement = Vector3( 0.0, 1.0, 0.0 );
    construct_pendulum_trajectory( &left_hip->trajectory, period_in_frames );
    left_hip->trajectory.spline_basis = CubicSpline::basisUniformNonrationalBSpline();
    left_hip->trajectory.spline_arclength_map = CubicSpline::map_spline( CUBIC_SPLINE_B, left_hip->trajectory.controlpoints );
    left_hip->trajectory.prev_keyframe_id = 1;
    left_hip->trajectory.next_keyframe_id = 2;
    left_hip->trajectory.frame_tic = period_in_frames/2 + 1;
    left_hip->frame_transformation = Matrix4( 0, 0, 1, 0,
                                             -1, 0, 0, 0,
                                              0, 1, 0, 0,
                                              0, 0, 0, 1 );
    left_hip->median_angle = Vector3( 0.0, 0.0, PI/2 );

    pelvis->append_child_joint( right_hip );
    right_hip->inboard_link = pelvis;
    right_hip->inboard_displacement = Vector3( -0.75, 0.0, 0.0 );
    right_hip->outboard_link = right_thigh;
    right_hip->outboard_displacement = Vector3( 0.0, 1.0, 0.0 );
    construct_pendulum_trajectory( &right_hip->trajectory, period_in_frames );
    right_hip->trajectory.spline_basis = CubicSpline::basisUniformNonrationalBSpline();
    right_hip->trajectory.spline_arclength_map = CubicSpline::map_spline( CUBIC_SPLINE_B, right_hip->trajectory.controlpoints );
    right_hip->trajectory.prev_keyframe_id = 2;
    right_hip->trajectory.next_keyframe_id = 1;
    right_hip->trajectory.frame_tic = period_in_frames/2 + 1;
    right_hip->trajectory.forward = false;
    right_hip->frame_transformation = Matrix4( 0, 0, 1, 0,
                                              -1, 0, 0, 0,
                                               0, 1, 0, 0,
                                               0, 0, 0, 1 );
    right_hip->median_angle = Vector3( 0.0, 0.0, PI/2 );

    left_thigh->insert( MeshLoader::box( 0.5, 2.5, 0.5 ) );
    left_thigh->parent_joint = left_hip;
    left_thigh->material = material;
    left_thigh->pose.transform.identity();
    left_thigh->append_child_joint( left_knee );

    right_thigh->insert( MeshLoader::box( 0.5, 2.5, 0.5 ) );
    right_thigh->parent_joint = right_hip;
    right_thigh->material = material;
    right_thigh->pose.transform.identity();
    right_thigh->append_child_joint( right_knee );

    left_knee->inboard_link = left_thigh;
    left_knee->inboard_displacement = Vector3( 0.0, -1.35, 0.0 );
    left_knee->outboard_link = left_calf;
    left_knee->outboard_displacement = Vector3( 0.0, 1.1, 0.0 );
    construct_calf_swing_trajectory( &left_knee->trajectory, period_in_frames );
    left_knee->trajectory.spline_basis = CubicSpline::basisUniformNonrationalBSpline();
    left_knee->trajectory.spline_arclength_map = CubicSpline::map_spline( CUBIC_SPLINE_B, left_knee->trajectory.controlpoints );
    left_knee->trajectory.prev_keyframe_id = 0;
    left_knee->trajectory.next_keyframe_id = 1;
    left_knee->trajectory.frame_tic = 1;
    left_knee->frame_transformation = Matrix4( 1, 0, 0, 0,
                                              0, 1, 0, 0,
                                              0, 0, 1, 0,
                                              0, 0, 0, 1 );
    left_knee->median_angle = Vector3( 0.0, 0.0, PI/8 );

    right_knee->inboard_link = right_thigh;
    right_knee->inboard_displacement = Vector3( 0.0, -1.35, 0.0 );
    right_knee->outboard_link = right_calf;
    right_knee->outboard_displacement = Vector3( 0.0, 1.1, 0.0 );
    construct_calf_swing_trajectory( &right_knee->trajectory, period_in_frames );
    right_knee->trajectory.spline_basis = CubicSpline::basisUniformNonrationalBSpline();
    right_knee->trajectory.spline_arclength_map = CubicSpline::map_spline( CUBIC_SPLINE_B, right_knee->trajectory.controlpoints );
    right_knee->trajectory.prev_keyframe_id = 1;
    right_knee->trajectory.next_keyframe_id = 2;
    right_knee->trajectory.frame_tic = period_in_frames/2 + 1;
    right_knee->median_angle = Vector3( 0.0, 0.0, PI/8 );

    left_calf->insert( MeshLoader::box( 0.5, 2.0, 0.5 ) );
    left_calf->parent_joint = left_knee;
    left_calf->material = material;

    right_calf->insert( MeshLoader::box( 0.5, 2.0, 0.5 ) );
    right_calf->parent_joint = right_knee;
    right_calf->material = material;

    pelvis->append_child_joint( sacrum );
    sacrum->inboard_link = pelvis;
    sacrum->inboard_displacement = Vector3( 0.0, 0.35, 0.0 );
    sacrum->outboard_link = spine;
    sacrum->outboard_displacement = Vector3( 0.0, -1.10, 0.0 );

    spine->insert( MeshLoader::box( 0.5, 2.0, 0.5 ) );
    spine->parent_joint = sacrum;
    spine->append_child_joint( thoracic );
    spine->material = material;

    thoracic->inboard_link = spine;
    thoracic->inboard_displacement = Vector3( 0.0, 1.10, 0.0 );
    thoracic->outboard_link = shoulders;
    thoracic->outboard_displacement = Vector3( 0.0, -0.6, 0.0 );

    shoulders->insert( MeshLoader::box( 2.5, 1.0, 0.5 ) );
    shoulders->parent_joint = thoracic;
    shoulders->material = material;

    shoulders->append_child_joint( left_shoulder );
    left_shoulder->inboard_link = shoulders;
    left_shoulder->inboard_displacement = Vector3( 1.5, 0.5, 0.0 );
    left_shoulder->outboard_link = left_arm;
    left_shoulder->outboard_displacement = Vector3( 0.0, 1.0, 0.0 );
    construct_pendulum_trajectory( &left_shoulder->trajectory, period_in_frames );
    left_shoulder->trajectory.spline_basis = CubicSpline::basisUniformNonrationalBSpline();
    left_shoulder->trajectory.spline_arclength_map = CubicSpline::map_spline( CUBIC_SPLINE_B, left_shoulder->trajectory.controlpoints );
    left_shoulder->trajectory.prev_keyframe_id = 2;
    left_shoulder->trajectory.next_keyframe_id = 1;
    left_shoulder->trajectory.frame_tic = period_in_frames/2 + 1;
    left_shoulder->trajectory.forward = false;
    left_shoulder->frame_transformation = Matrix4( 0, 0, 1, 0,
                                              -1, 0, 0, 0,
                                               0, 1, 0, 0,
                                               0, 0, 0, 1 );
    left_shoulder->median_angle = Vector3( 0.0, 0.0, PI/2 );

    shoulders->append_child_joint( right_shoulder );
    right_shoulder->inboard_link = shoulders;
    right_shoulder->inboard_displacement = Vector3( -1.5, 0.5, 0.0 );
    right_shoulder->outboard_link = right_arm;
    right_shoulder->outboard_displacement = Vector3( 0.0, 1.0, 0.0 );
    construct_pendulum_trajectory( &right_shoulder->trajectory, period_in_frames );
    right_shoulder->trajectory.spline_basis = CubicSpline::basisUniformNonrationalBSpline();
    right_shoulder->trajectory.spline_arclength_map = CubicSpline::map_spline( CUBIC_SPLINE_B, right_shoulder->trajectory.controlpoints );
    right_shoulder->trajectory.prev_keyframe_id = 1;
    right_shoulder->trajectory.next_keyframe_id = 2;
    right_shoulder->trajectory.frame_tic = period_in_frames/2 + 1;
    right_shoulder->frame_transformation = Matrix4( 0, 0, 1, 0,
                                             -1, 0, 0, 0,
                                              0, 1, 0, 0,
                                              0, 0, 0, 1 );
    right_shoulder->median_angle = Vector3( 0.0, 0.0, PI/2 );

    left_arm->insert( MeshLoader::box( 0.5, 2.0, 0.5 ) );
    left_arm->parent_joint = left_shoulder;
    left_arm->append_child_joint( left_elbow );
    left_arm->material = material;

    right_arm->insert( MeshLoader::box( 0.5, 2.0, 0.5 ) );
    right_arm->parent_joint = right_shoulder;
    right_arm->append_child_joint( right_elbow );
    right_arm->material = material;

    left_elbow->inboard_link = left_arm;
    left_elbow->inboard_displacement = Vector3( 0.0, -1.1, 0.0 );
    left_elbow->outboard_link = left_forearm;
    left_elbow->outboard_displacement = Vector3( 0.0, 0.95, 0.0 );
    construct_forearm_swing_trajectory( &left_elbow->trajectory, period_in_frames );
    left_elbow->trajectory.spline_basis = CubicSpline::basisUniformNonrationalBSpline();
    left_elbow->trajectory.spline_arclength_map = CubicSpline::map_spline( CUBIC_SPLINE_B, left_elbow->trajectory.controlpoints );
    left_elbow->trajectory.prev_keyframe_id = 0;
    left_elbow->trajectory.next_keyframe_id = 1;
    left_elbow->trajectory.frame_tic = 1;
    left_elbow->median_angle = Vector3( 0.0, 0.0, PI/8 );

    right_elbow->inboard_link = right_arm;
    right_elbow->inboard_displacement = Vector3( 0.0, -1.1, 0.0 );
    right_elbow->outboard_link = right_forearm;
    right_elbow->outboard_displacement = Vector3( 0.0, 0.95, 0.0 );
    construct_forearm_swing_trajectory( &right_elbow->trajectory, period_in_frames );
    right_elbow->trajectory.spline_basis = CubicSpline::basisUniformNonrationalBSpline();
    right_elbow->trajectory.spline_arclength_map = CubicSpline::map_spline( CUBIC_SPLINE_B, right_elbow->trajectory.controlpoints );
    right_elbow->trajectory.prev_keyframe_id = 1;
    right_elbow->trajectory.next_keyframe_id = 2;
    right_elbow->trajectory.frame_tic = period_in_frames/2 + 1;
    right_elbow->median_angle = Vector3( 0.0, 0.0, PI/8 );

    left_forearm->insert( MeshLoader::box( 0.5, 1.75, 0.5 ) );
    left_forearm->parent_joint = left_elbow;
    left_forearm->material = material;

    right_forearm->insert( MeshLoader::box( 0.5, 1.75, 0.5 ) );
    right_forearm->parent_joint = right_elbow;
    right_forearm->material = material;

    shoulders->append_child_joint( cervical );
    cervical->inboard_link = shoulders;
    cervical->inboard_displacement = Vector3( 0.0, 0.60, 0.0 );
    cervical->outboard_link = neck;
    cervical->outboard_displacement = Vector3( 0.0, -0.10, 0.0 );

    neck->insert( MeshLoader::box( 0.5, 0.2, 0.5 ) );
    neck->parent_joint = cervical;
    neck->append_child_joint( condyloid );
    neck->material = material;

    condyloid->inboard_link = neck;
    condyloid->inboard_displacement = Vector3( 0.0, 0.10, 0.0 );
    condyloid->outboard_link = head;
    condyloid->outboard_displacement = Vector3( 0.0, -0.6, 0.0 );

    head->insert( MeshLoader::box( 1.0, 1.0, 1.0 ) );
    head->parent_joint = condyloid;
    head->material = material;
}
//------------------------------------------------------------------------------

/// Trajectory for the knee joints.  Defines the swing of the calves.
void ArticulatedBody::construct_calf_swing_trajectory( Trajectory* trajectory, const unsigned int& period_in_frames ) {
    assert( period_in_frames % 2 == 0 );

    unsigned int frames_over_spline = period_in_frames / 2;

    trajectory->forward = true;

    Pose pose;
    ControlPoint cp;
    Vector3 pt, rot;
    Matrix3 A;

    // dummy end control point
    pt = A * Vector3( 0.0, 0.0, 0.0 );
    rot = Vector3( 0.0, 0.0, PI/4 );
    pose = Pose( pt, rot );
    cp = ControlPoint( pose.position, CONTROL_POINT_END );
    trajectory->controlpoints.push_back( cp );

    // begin visible spline control points
    pt = A * Vector3( 0.0, 0.0, 0.0 );
    rot = Vector3( 0.0, 0.0, 0.0 );
    pose = Pose( pt, rot );
    trajectory->poses.push_back( pose );
    trajectory->insert_keyframe( 1, pose );
    cp = ControlPoint( pose.position, CONTROL_POINT_MIDDLE );
    trajectory->controlpoints.push_back( cp );

    pt = A * Vector3( 0.0, 0.0, 0.0 );
    rot = Vector3( 0.0, 0.0, 0.0 );
    pose = Pose( pt, rot );
    trajectory->poses.push_back( pose );
    trajectory->insert_keyframe( frames_over_spline * 1 + 1, pose );
    cp = ControlPoint( pose.position, CONTROL_POINT_MIDDLE );
    trajectory->controlpoints.push_back( cp );

    pt = A * Vector3( 0.0, 0.0, 0.0 );
    rot = Vector3( 0.0, 0.0, -PI/4 );
    pose = Pose( pt, rot );
    trajectory->poses.push_back( pose );
    trajectory->insert_keyframe( frames_over_spline * 2 + 1, pose );
    cp = ControlPoint( pose.position, CONTROL_POINT_MIDDLE );
    trajectory->controlpoints.push_back( cp );
    // end visible spline control points

    // dummy end control point
    pt = A * Vector3( 0.0, 0.0, 0.0 );
    rot = Vector3( 0.0, 0.0, -PI/2 );
    pose = Pose( pt, rot );
    cp = ControlPoint( pose.position, CONTROL_POINT_END );
    trajectory->controlpoints.push_back( cp );

    trajectory->cyclic = true;
    trajectory->reversible = true;
}

//------------------------------------------------------------------------------
/// Trajectory for the elbow joints.  Defines the swing of the forearms.
void ArticulatedBody::construct_forearm_swing_trajectory( Trajectory* trajectory, const unsigned int& period_in_frames ) {
    assert( period_in_frames % 2 == 0 );

    unsigned int frames_over_spline = period_in_frames / 2;

    trajectory->forward = true;

    Pose pose;
    ControlPoint cp;
    Vector3 pt, rot;
    Matrix3 A;

    // dummy end control point
    pt = A * Vector3( 0.0, 0.0, 0.0 );
    rot = Vector3( 0.0, 0.0, -PI/4 );
    pose = Pose( pt, rot );
    cp = ControlPoint( pose.position, CONTROL_POINT_END );
    trajectory->controlpoints.push_back( cp );

    // begin visible spline control points
    pt = A * Vector3( 0.0, 0.0, 0.0 );
    rot = Vector3( 0.0, 0.0, 0.0 );
    pose = Pose( pt, rot );
    trajectory->poses.push_back( pose );
    trajectory->insert_keyframe( 1, pose );
    cp = ControlPoint( pose.position, CONTROL_POINT_MIDDLE );
    trajectory->controlpoints.push_back( cp );

    pt = A * Vector3( 0.0, 0.0, 0.0 );
    rot = Vector3( 0.0, 0.0, 0.0 );
    pose = Pose( pt, rot );
    trajectory->poses.push_back( pose );
    trajectory->insert_keyframe( frames_over_spline * 1 + 1, pose );
    cp = ControlPoint( pose.position, CONTROL_POINT_MIDDLE );
    trajectory->controlpoints.push_back( cp );

    pt = A * Vector3( 0.0, 0.0, 0.0 );
    rot = Vector3( 0.0, 0.0, PI/4 );
    pose = Pose( pt, rot );
    trajectory->poses.push_back( pose );
    trajectory->insert_keyframe( frames_over_spline * 2 + 1, pose );
    cp = ControlPoint( pose.position, CONTROL_POINT_MIDDLE );
    trajectory->controlpoints.push_back( cp );
    // end visible spline control points

    // dummy end control point
    pt = A * Vector3( 0.0, 0.0, 0.0 );
    rot = Vector3( 0.0, 0.0, PI/2 );
    pose = Pose( pt, rot );
    cp = ControlPoint( pose.position, CONTROL_POINT_END );
    trajectory->controlpoints.push_back( cp );

    trajectory->cyclic = true;
    trajectory->reversible = true;
}

//------------------------------------------------------------------------------
/// Trajectory for the hip and shoulder joints.  Defines the swing of the thighs and upper arms.
void ArticulatedBody::construct_pendulum_trajectory( Trajectory* trajectory, const unsigned int& period_in_frames ) {
    assert( period_in_frames % 2 == 0 );

    unsigned int frames_over_spline = period_in_frames / 2;

    trajectory->forward = true;

    Pose pose;
    ControlPoint cp;
    Vector3 pt, rot;
    Matrix3 A;

    // dummy end control point
    // 9 o'clock
    pt = A * Vector3( -1.0, 0.0, 0.0 );
    rot = Vector3( 0.0, 0.0, 0.0 );
    pose = Pose( pt, rot );
    cp = ControlPoint( pose.position, CONTROL_POINT_END );
    trajectory->controlpoints.push_back( cp );

    // begin visible spline control points
    // 7:30 o'clock
    pt = A * Vector3( -cos(PI/4), -sin(PI/4), 0.0 );
    rot = Vector3( 0.0, 0.0, PI/4 );
    pose = Pose( pt, rot );
    trajectory->poses.push_back( pose );
    trajectory->insert_keyframe( 1, pose );
    cp = ControlPoint( pose.position, CONTROL_POINT_MIDDLE );
    trajectory->controlpoints.push_back( cp );

    // 6 o'clock
    pt = A * Vector3( 0.0, -1.0, 0.0 );
    rot = Vector3( 0.0, 0.0, PI/2 );
    pose = Pose( pt, rot );
    trajectory->poses.push_back( pose );
    trajectory->insert_keyframe( frames_over_spline * 1 + 1, pose );
    cp = ControlPoint( pose.position, CONTROL_POINT_MIDDLE );
    trajectory->controlpoints.push_back( cp );

    // 4:30 o'clock
    pt = A * Vector3( cos(PI/4), -sin(PI/4), 0.0 );
    rot = Vector3( 0.0, 0.0, PI*3/4 );
    pose = Pose( pt, rot );
    trajectory->poses.push_back( pose );
    trajectory->insert_keyframe( frames_over_spline * 2 + 1, pose );
    cp = ControlPoint( pose.position, CONTROL_POINT_MIDDLE );
    trajectory->controlpoints.push_back( cp );
    // end visible spline control points

    // dummy end control point
    // 3 o'clock
    pt = A * Vector3( 1.0, 0.0, 0.0 );
    rot = Vector3( 0.0, 0.0, PI );
    pose = Pose( pt, rot );
    cp = ControlPoint( pose.position, CONTROL_POINT_END );
    trajectory->controlpoints.push_back( cp );

    trajectory->cyclic = true;
    trajectory->reversible = true;
}

void ArticulatedBody::init_boid( const double& scale, const unsigned int& period_in_frames ) {
    assert( scale > 0.0 );

    Material material = Material(
                Color( 1.0, 1.0, 1.0, 1.0 ),
                Color( 1.0, 1.0, 1.0, 1.0 ),
                Color( 1.0, 1.0, 1.0, 1.0 ),
                Color( 0.0, 0.0, 0.0, 1.0 ),
                10                           );

    //unsigned int wing_period = (unsigned int) ((double) period_in_frames * scale);

    Link* body = new Link( "body" );
    Joint* left_shoulder = new Joint( "left_shoulder" );
    Joint* right_shoulder = new Joint( "right_shoulder" );
    Link* left_humerus = new Link( "left_humerus" );
    Link* right_humerus = new Link( "right_humerus" );

    Joint* left_elbow = new Joint( "left_elbow" );
    Joint* right_elbow = new Joint( "right_elbow" );
    Link* left_ulna = new Link( "left_ulna" );
    Link* right_ulna = new Link( "right_ulna" );

    Joint* pubis = new Joint( "pubis" );
    Link* tail = new Link( "tail" );

    Joint* neck = new Joint( "neck" );
    Link* head = new Link( "head" );

    //body->pose.orientation_type = POSE_BY_QUATERNION;
    //pose.orientation_type = POSE_BY_QUATERNION;
//    pose.position = Vector3( 0.0, 0.0, 0.0 );
//    pose.orientation = Vector3( 0.0, 0.0, 0.0 );
//    pose.orientation_q = Quaternion( 0.0, 0.0, 0.0, 0.0 );
//    pose.transform.identity();

    append_link( body );
    append_joint( left_shoulder );
    append_joint( right_shoulder );
    append_link( left_humerus );
    append_link( right_humerus );

    append_joint( left_elbow );
    append_joint( right_elbow );
    append_link( left_ulna );
    append_link( right_ulna );

    append_joint( pubis );
    append_link( tail );

    append_joint( neck );
    append_link( head );

    root_link = body;

    body->pose.position = Vector3( 0.0, 0.0, 0.0 );
    body->insert( MeshLoader::box( 0.15 * scale, 0.15 * scale, 0.6 * scale ) );
    body->material = material;
    body->pose.transform.identity();

    body->append_child_joint( left_shoulder );
    left_shoulder->inboard_link = body;
    left_shoulder->inboard_displacement = Vector3( 0.075 * scale, 0.0, 0.0 );
    left_shoulder->outboard_link = left_humerus;
    left_shoulder->outboard_displacement = Vector3( 0.0, 0.0, 0.15 * scale );
    construct_wing_trajectory( &left_shoulder->trajectory, period_in_frames );
    //construct_wing_trajectory( &left_shoulder->trajectory, wing_period );
    left_shoulder->trajectory.spline_basis = CubicSpline::basisUniformNonrationalBSpline();
    left_shoulder->trajectory.spline_arclength_map = CubicSpline::map_spline( CUBIC_SPLINE_B, left_shoulder->trajectory.controlpoints );
    left_shoulder->trajectory.prev_keyframe_id = 2;
    left_shoulder->trajectory.next_keyframe_id = 1;
    left_shoulder->trajectory.forward = false;
    left_shoulder->trajectory.frame_tic = period_in_frames/2 + 1;
    left_shoulder->frame_transformation = Matrix4( 0, 0, 1, 0,
                                              -1, 0, 0, 0,
                                               0, 1, 0, 0,
                                               0, 0, 0, 1 );
    left_shoulder->median_angle = Vector3( 0.0, 0.0, PI/2 );

    body->append_child_joint( right_shoulder );
    right_shoulder->inboard_link = body;
    right_shoulder->inboard_displacement = Vector3( -0.075 * scale, 0.0, 0.0 );
    right_shoulder->outboard_link = right_humerus;
    right_shoulder->outboard_displacement = Vector3( 0.0, 0.0, -0.15 * scale );
    construct_wing_trajectory( &right_shoulder->trajectory, period_in_frames );
    //construct_wing_trajectory( &right_shoulder->trajectory, wing_period );
    right_shoulder->trajectory.spline_basis = CubicSpline::basisUniformNonrationalBSpline();
    right_shoulder->trajectory.spline_arclength_map = CubicSpline::map_spline( CUBIC_SPLINE_B, right_shoulder->trajectory.controlpoints );
    right_shoulder->trajectory.prev_keyframe_id = 1;
    right_shoulder->trajectory.next_keyframe_id = 2;
    right_shoulder->trajectory.frame_tic = period_in_frames/2 + 1;
    right_shoulder->frame_transformation = Matrix4( 0, 0, 1, 0,
                                             -1, 0, 0, 0,
                                              0, 1, 0, 0,
                                              0, 0, 0, 1 );
    right_shoulder->median_angle = Vector3( 0.0, 0.0, PI/2 );

    left_humerus->insert( MeshLoader::box( 0.30 * scale, 0.075 * scale, 0.30 * scale ) );
    left_humerus->parent_joint = left_shoulder;
    left_humerus->material = material;
    left_humerus->pose.transform.identity();

    right_humerus->insert( MeshLoader::box( 0.30 * scale, 0.075 * scale, 0.30 * scale ) );
    right_humerus->parent_joint = right_shoulder;
    right_humerus->material = material;
    right_humerus->pose.transform.identity();

    left_humerus->append_child_joint( left_elbow );
    left_elbow->inboard_link = left_humerus;
    left_elbow->inboard_displacement = Vector3( 0.0, 0.0, -0.15 * scale );
    left_elbow->outboard_link = left_ulna;
    left_elbow->outboard_displacement = Vector3( 0.15 * scale, 0.0, 0.0 );
    construct_pendulum_trajectory( &left_elbow->trajectory, period_in_frames );
    //construct_pendulum_trajectory( &left_elbow->trajectory, wing_period );
    left_elbow->trajectory.spline_basis = CubicSpline::basisUniformNonrationalBSpline();
    left_elbow->trajectory.spline_arclength_map = CubicSpline::map_spline( CUBIC_SPLINE_B, left_elbow->trajectory.controlpoints );
    left_elbow->trajectory.prev_keyframe_id = 2;
    left_elbow->trajectory.next_keyframe_id = 1;
    left_elbow->trajectory.forward = false;
    left_elbow->trajectory.frame_tic = period_in_frames/2 + 1;
    left_elbow->frame_transformation = Matrix4( 0, 0, 1, 0,
                                               1, 0, 0, 0,
                                               0, 1, 0, 0,
                                               0, 0, 0, 1 );
    left_elbow->median_angle = Vector3( 0.0, 0.0, PI/2 );

    right_humerus->append_child_joint( right_elbow );
    right_elbow->inboard_link = right_humerus;
    right_elbow->inboard_displacement = Vector3( 0.0, 0.0, 0.15 * scale );
    right_elbow->outboard_link = right_ulna;
    right_elbow->outboard_displacement = Vector3( -0.15 * scale, 0.0, 0.0 );
    construct_pendulum_trajectory( &right_elbow->trajectory, period_in_frames );
    //construct_pendulum_trajectory( &right_elbow->trajectory, wing_period );
    right_elbow->trajectory.spline_basis = CubicSpline::basisUniformNonrationalBSpline();
    right_elbow->trajectory.spline_arclength_map = CubicSpline::map_spline( CUBIC_SPLINE_B, right_elbow->trajectory.controlpoints );
    right_elbow->trajectory.prev_keyframe_id = 1;
    right_elbow->trajectory.next_keyframe_id = 2;
    right_elbow->trajectory.forward = true;
    right_elbow->trajectory.frame_tic = period_in_frames/2 + 1;
    right_elbow->frame_transformation = Matrix4( 0, 0, 1, 0,
                                               1, 0, 0, 0,
                                               0, 1, 0, 0,
                                               0, 0, 0, 1 );
    right_elbow->median_angle = Vector3( 0.0, 0.0, PI/2 );

    left_ulna->insert( MeshLoader::box( 0.30 * scale, 0.075 * scale, 0.30 * scale ) );
    left_ulna->parent_joint = left_elbow;
    left_ulna->material = material;
    left_ulna->pose.transform.identity();

    right_ulna->insert( MeshLoader::box( 0.30 * scale, 0.075 * scale, 0.30 * scale ) );
    right_ulna->parent_joint = right_elbow;
    right_ulna->material = material;
    right_ulna->pose.transform.identity();

    body->append_child_joint( pubis );
    pubis->inboard_link = body;
    pubis->inboard_displacement = Vector3( 0.0, 0.075 * scale, -0.30 * scale );
    pubis->outboard_link = tail;
    pubis->outboard_displacement = Vector3( 0.0, 0.0, 0.115 * scale );

    tail->insert( MeshLoader::box( 0.15 * scale, 0.03 * scale, 0.225 * scale ) );
    tail->parent_joint = pubis;
    tail->material = material;

    body->append_child_joint( neck );
    neck->inboard_link = body;
    neck->inboard_displacement = Vector3( 0.0, 0.0, 0.30 * scale );
    neck->outboard_link = head;
    neck->outboard_displacement = Vector3( -0.0, 0.0, -0.075 * scale );

    head->parent_joint = neck;
    head->material = material;
    head->insert( MeshLoader::sphere( 0.115 * scale, 6, 3, &head->material, ML_MATERIAL_ASSIGN ) );
    head->insert( MeshLoader::cone( 0.115 * scale, 0.225 * scale, 4, &head->material, ML_MATERIAL_ASSIGN ) );

}

//------------------------------------------------------------------------------
/// Trajectory for the hip and shoulder joints.  Defines the swing of the thighs and upper arms.
void ArticulatedBody::construct_wing_trajectory( Trajectory* trajectory, const unsigned int& period_in_frames ) {
    assert( period_in_frames % 2 == 0 );

    unsigned int frames_over_spline = period_in_frames / 2;

    trajectory->forward = true;

    Pose pose;
    ControlPoint cp;
    Vector3 pt, rot;
    Matrix3 A;

    // dummy end control point
    // 9 o'clock
    pt = A * Vector3( -1.0, 0.0, 0.0 );
    rot = Vector3( 0.0, PI/2.0, PI/2.0 );
    pose = Pose( pt, rot );
    cp = ControlPoint( pose.position, CONTROL_POINT_END );
    trajectory->controlpoints.push_back( cp );

    // begin visible spline control points
    // 7:30 o'clock
    pt = A * Vector3( -cos(PI/4), -sin(PI/4), 0.0 );
    rot = Vector3( 0.0, PI*3.0/4.0, PI/2.0 );
    pose = Pose( pt, rot );
    trajectory->poses.push_back( pose );
    trajectory->insert_keyframe( 1, pose );
    cp = ControlPoint( pose.position, CONTROL_POINT_MIDDLE );
    trajectory->controlpoints.push_back( cp );

    // 6 o'clock
    pt = A * Vector3( 0.0, -1.0, 0.0 );
    rot = Vector3( 0.0, PI, PI/2.0 );
    pose = Pose( pt, rot );
    trajectory->poses.push_back( pose );
    trajectory->insert_keyframe( frames_over_spline * 1 + 1, pose );
    cp = ControlPoint( pose.position, CONTROL_POINT_MIDDLE );
    trajectory->controlpoints.push_back( cp );

    // 4:30 o'clock
    pt = A * Vector3( cos(PI/4), -sin(PI/4), 0.0 );
    rot = Vector3( 0.0, PI*5.0/4.0, PI/2.0 );
    pose = Pose( pt, rot );
    trajectory->poses.push_back( pose );
    trajectory->insert_keyframe( frames_over_spline * 2 + 1, pose );
    cp = ControlPoint( pose.position, CONTROL_POINT_MIDDLE );
    trajectory->controlpoints.push_back( cp );
    // end visible spline control points

    // dummy end control point
    // 3 o'clock
    pt = A * Vector3( 1.0, 0.0, 0.0 );
    rot = Vector3( 0.0, PI*3.0/2.0, PI/2.0 );
    pose = Pose( pt, rot );
    cp = ControlPoint( pose.position, CONTROL_POINT_END );
    trajectory->controlpoints.push_back( cp );

    trajectory->cyclic = true;
    trajectory->reversible = true;
}
