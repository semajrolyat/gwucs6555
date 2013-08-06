/*-----------------------THE GEORGE WASHINGTON UNIVERSITY-----------------------
James Taylor                                                         jrt@gwu.edu

Lab 2 implementation

------------------------------------------------------------------------------*/

#include <iostream>

#include <cs6555/Math/CubicSpline.h>
#include <cs6555/Constants.h>

#include <GL/glut.h>
#include <stdio.h>

#include <string>

#include <cs6555/Math/Vector3.h>
#include <cs6555/Math/Matrix4.h>

#include <cs6555/Camera.h>
#include <cs6555/Trajectory.h>

#include <cs6555/Math/EulerAngle.h>
#include <cs6555/Keyframe.h>

#include <cs6555/ArticulatedBody.h>
#include <stack>

#include <cs6555/Mesh.h>
#include <cs6555/MeshLoader.h>

#include <stdexcept>

#include <cs6555/Color.h>
#include <cs6555/Material.h>

#include <cs6555/Utilities.h>

//------------------------------------------------------------------------------
// Hardcoded Defines
//------------------------------------------------------------------------------

#define GENERATE_MOVIE  0               // No, do not generate a movie
//#define GENERATE_MOVIE  1               // Yes, do generate a movie

//------------------------------------------------------------------------------
// Toggle Options
//------------------------------------------------------------------------------

bool opt_draw_actor = true;                 // toggle by pressing 1
//bool opt_draw_trajectories = true;
bool opt_draw_trajectories = false;         // toggle by pressing 2
bool sim_pause = false;                     // toggle by pressing p

//------------------------------------------------------------------------------

static unsigned int frame_id = 0;            // current frame.  Counter
int Width;                                  // Width of OpenGL Viewport
int Height;                                 // Height of OpenGL Viewport
unsigned char *Pixels;
std::string title;                          // Window Title
std::string filetitle;                      // Movie file title prefix
char filename[128];                         // Buffer for filename of a frame render to file

double line_width = 8.0;                    // Width of a spline when rendering trajectories

//------------------------------------------------------------------------------
// Because cannot use the OpenGL matrix stack, define own.
// Only used during idle to precalculate all transforms in hierarchy
static std::stack<Matrix4> MatrixStack;
//------------------------------------------------------------------------------

Camera camera;                              // A camera actor object.  Generates projection matrix
ArticulatedBody articulatedbody;            // The actor for this program

//------------------------------------------------------------------------------
// Camera Functions
//------------------------------------------------------------------------------
void initCamera( void ) {
    ///*
    camera.position = Vector3( 10.0, 5.0, 10.0 );
    camera.viewpoint = Vector3( 0.0, 0.0, 0.0 );
    camera.up = Vector3( 0.0, 1.0, 0.0 );
    //*/
/*
    camera.position = Vector3( 0.0, 5.0, 0.0 );
    camera.viewpoint = Vector3( 0.0, 0.0, 0.0 );
    camera.up = Vector3( 0.0, 0.0, 1.0 );
*/
    camera.project();
}

//------------------------------------------------------------------------------
// ArticulatedBody Functions
//------------------------------------------------------------------------------
/// Trajectory for the knee joints.  Defines the swing of the calves.
void construct_calf_swing_trajectory( Trajectory* trajectory, const unsigned int& period_in_frames ) {
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
void construct_forearm_swing_trajectory( Trajectory* trajectory, const unsigned int& period_in_frames ) {
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
void construct_pendulum_trajectory( Trajectory* trajectory, const unsigned int& period_in_frames ) {
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

//------------------------------------------------------------------------------
// Pointers for the joints that have trajectories to eliminate any searches
// Will only be used if drawing trajectories
Joint* body_left_hip;
Joint* body_right_hip;
Joint* body_left_knee;
Joint* body_right_knee;
Joint* body_left_shoulder;
Joint* body_right_shoulder;
Joint* body_left_elbow;
Joint* body_right_elbow;

//------------------------------------------------------------------------------
/// Initialize the actor for this program.  Builds the hierarchy of the articulated body
/// and adds any trajectories to the necessary joints
void initBody( void ) {
    title = "Lab 2";
    filetitle = "lab2";

    articulatedbody.init_humanoid();
    /*
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

    body_left_hip = left_hip;
    body_right_hip = right_hip;
    body_left_knee = left_knee;
    body_right_knee = right_knee;
    body_left_shoulder = left_shoulder;
    body_right_shoulder = right_shoulder;
    body_left_elbow = left_elbow;
    body_right_elbow = right_elbow;

    articulatedbody.append_link( pelvis );
    articulatedbody.append_joint( left_hip );
    articulatedbody.append_joint( right_hip );
    articulatedbody.append_link( left_thigh );
    articulatedbody.append_link( right_thigh );
    articulatedbody.append_joint( left_knee );
    articulatedbody.append_joint( right_knee );
    articulatedbody.append_link( left_calf );
    articulatedbody.append_link( right_calf );

    articulatedbody.append_joint( sacrum );
    articulatedbody.append_link( spine );
    articulatedbody.append_joint( thoracic );

    articulatedbody.append_link( shoulders );
    articulatedbody.append_joint( left_shoulder );
    articulatedbody.append_joint( right_shoulder );
    articulatedbody.append_link( left_arm );
    articulatedbody.append_link( right_arm );
    articulatedbody.append_joint( left_elbow );
    articulatedbody.append_joint( right_elbow );
    articulatedbody.append_link( left_forearm );
    articulatedbody.append_link( right_forearm );

    articulatedbody.append_joint( cervical );
    articulatedbody.append_link( neck );
    articulatedbody.append_joint( condyloid );
    articulatedbody.append_link( head );


    articulatedbody.root_link = pelvis;

    pelvis->pose.position = Vector3( 0.0, 0.0, 0.0 );
    pelvis->insert( MeshLoader::box( 1.0, 0.5, 0.5 ) );
    pelvis->trajectory.construct_cyclic_circle_trajectory( period_in_frames * 4, true, Vector3(-PI/2, 0.0, 0.0), 7.0, Vector3(0.0, 0.0, 0.0) );
    //pelvis->trajectory.construct_cyclic_circle_trajectory( 80, true, Vector3(-PI/2, 0.0, 0.0), 5.0, Vector3(0.0, 0.0, 0.0) );
    //pelvis->trajectory.construct_cyclic_circle_trajectory( 160, true, Vector3(-PI/2, 0.0, 0.0), 2.0, Vector3(0.0, 0.0, 0.0) );
    pelvis->trajectory.spline_basis = CubicSpline::basisUniformNonrationalBSpline();
    pelvis->trajectory.spline_arclength_map = CubicSpline::map_spline( CUBIC_SPLINE_B, pelvis->trajectory.controlpoints );
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
    */
}

//------------------------------------------------------------------------------
// Drawing Operations
//------------------------------------------------------------------------------
/// Draws a mesh
void draw_mesh( Mesh* mesh ) {
    glBegin( GL_TRIANGLES );
    for( unsigned int poly_id = 0; poly_id < mesh->polygonCount( ); poly_id++ ) {
        // Select the current polygon
        Polygon* poly = mesh->polygon( poly_id );

        Vertex *v0, *v1, *v2;

        unsigned int verts = poly->numVertices( );
        if( verts < 3 ) continue;   // sanity check -> malformed poly & bad juju

        // the model is not tessellated, so have to tessellate for OpenGL
        // If poly is non-convex this won't work, but assume convex.

        // Select the first vertex as the root of all triangles in the poly
        v0 = mesh->vertex( poly->getVertex( 0 ) );

        // Iterate over the rest of the vertices in the polygon up to the n-1 vert
        for( unsigned int poly_vert = 1; poly_vert < verts - 1; poly_vert++ ) {

            // select the current vertex
            v1 = mesh->vertex( poly->getVertex( poly_vert ) );
            // and the next vertex (for n-1 case this will be n so closes the poly)
            v2 = mesh->vertex( poly->getVertex( poly_vert + 1 ) );

            glVertex3d( v0->position.x(), v0->position.y(), v0->position.z() );
            glVertex3d( v1->position.x(), v1->position.y(), v1->position.z() );
            glVertex3d( v2->position.x(), v2->position.y(), v2->position.z() );
        }
    }
    glEnd();
}

//------------------------------------------------------------------------------
/// Draws a link and descends the hierarchy of child joints and draws any
/// links attached down the kinematic chain
/// Note: No Push or Pop Matrix
void draw_link( Link* link ) {
    GLfloat* material_Ka = (GLfloat*)link->material.ambient.arrayOpenGL();
    GLfloat* material_Kd = (GLfloat*)link->material.diffuse.arrayOpenGL();
    GLfloat* material_Ks = (GLfloat*)link->material.specular.arrayOpenGL();
    GLfloat* material_Ke = (GLfloat*)link->material.emissive.arrayOpenGL();
    GLfloat material_Se = (GLfloat)link->material.shininess;

    glMaterialfv(GL_FRONT, GL_AMBIENT, material_Ka);
    glMaterialfv(GL_FRONT, GL_DIFFUSE, material_Kd);
    glMaterialfv(GL_FRONT, GL_SPECULAR, material_Ks);
    glMaterialfv(GL_FRONT, GL_EMISSION, material_Ke);
    glMaterialf(GL_FRONT, GL_SHININESS, material_Se);

    glLoadMatrixd( link->pose.transform.arrayOpenGL() );

    for( unsigned int geometry_id = 0; geometry_id < link->geometryCount( ); geometry_id++ ) {
        Mesh* mesh = static_cast<Mesh*>( link->geometry( geometry_id ) );
        draw_mesh( mesh );
    }

    for( unsigned int i = 0; i < link->child_joints(); i++ ) {
        Joint* joint = link->child_joint( i );
        draw_link( joint->outboard_link );
    }

}

//------------------------------------------------------------------------------
/// Draw a trajectory/spline.  Can be toggled.
void draw_trajectory( const Trajectory& trajectory, Matrix4* transform) {

    Eigen::MatrixXd M;
    GLfloat *material_Ka, *material_Kd, *material_Ks, *material_Ke, material_Se;

    material_Ka = (GLfloat*)trajectory.material.ambient.arrayOpenGL();
    material_Kd = (GLfloat*)trajectory.material.diffuse.arrayOpenGL();
    material_Ks = (GLfloat*)trajectory.material.specular.arrayOpenGL();
    material_Ke = (GLfloat*)trajectory.material.emissive.arrayOpenGL();
    material_Se = (GLfloat)trajectory.material.shininess;

    M = trajectory.spline_basis;

    glMaterialfv(GL_FRONT, GL_AMBIENT, material_Ka);
    glMaterialfv(GL_FRONT, GL_DIFFUSE, material_Kd);
    glMaterialfv(GL_FRONT, GL_SPECULAR, material_Ks);
    glMaterialfv(GL_FRONT, GL_EMISSION, material_Ke);
    glMaterialf(GL_FRONT, GL_SHININESS, material_Se);

    Vector3 vi, vi1;
    unsigned int n = trajectory.controlpoints.size();

    glLineWidth( line_width );

    glPushMatrix();

    if( transform != NULL )
        glLoadMatrixd( transform->arrayOpenGL() );

    glBegin(GL_LINES);

    for( unsigned int i = 2; i < n-1; i++ ) {
        Eigen::MatrixXd C = CubicSpline::blend( M, trajectory.controlpoints, i );

        std::vector<Vector3> points;
        for( double u = 0.0; u < 1.0; u += 0.1 ) {
            points.push_back( CubicSpline::position( C, u ) );
        }
        points.push_back( CubicSpline::position( C, 1.0 ) );

        for( std::vector<Vector3>::iterator it = points.begin(); it != points.end(); it++ ) {
            if( it == points.begin() ) {
                vi = *it;
            } else {
                vi1 = *it;
                glVertex3d( vi.x(), vi.y(), vi.z() );
                glVertex3d( vi1.x(), vi1.y(), vi1.z() );
                vi = *it;
            }
        }
    }

    glEnd();

    glPopMatrix();
}

//------------------------------------------------------------------------------
/// Draw the animated actor.  Starts at the root link and descends the hierarchy
/// to draw each link
/// Note: This is the only pair of Push and Pop Matrix associated with the
/// articulated body hierarchy
void draw_articulatedbody( const ArticulatedBody& body ) {

    // By requirements can only have one push and pop of OpenGL's matrix stack
    // Therefore do that here then descend the hierarchy
    glPushMatrix();

    // recursive descent beginning with the root
    draw_link( body.root_link );

    glPopMatrix();

}

//------------------------------------------------------------------------------
// Transformation Operations
//------------------------------------------------------------------------------
/// Transform a link and recursively descend to transform all children in the
/// kinematic chain.  Uses the custom matrix stack to keep the current
/// transform readily available.  Executed during the idle calculation
void transform_link( Link* link ) {

    // grab the current transform off the top of the matrixstack
    Matrix4 current_transform = MatrixStack.top();
    for( unsigned int i = 0; i < link->child_joints(); i++ ) {
        Joint* joint = link->child_joint( i );

        Matrix4 F1, X1, J1, X2, F2;
        // where F1 is Frame1 transformation, X1 is inboard displacement from F1 COM to joint
        // J1 is the joint orientation and rotation, X2 is outboard displacement from joint to F2 COM
        // and F2 is the resultant Frame2 transformation

        F1 = current_transform;
        X1 = Matrix4::translationMatrix( joint->inboard_displacement );

        // probably a more elegant way to transform from the union of coordinate frames defined at the joint
        // but for now the frame_transformation is provided by the implementor
        J1 = joint->frame_transformation * EulerAngle::matrix4( EULER_ANGLE_XYZ, EULER_HANDED_RIGHT, joint->angle );
        X2 = Matrix4::translationMatrix( Vector3::invert(joint->outboard_displacement) );

        F2 = F1 * X1 * J1 * X2;

        joint->outboard_link->pose.transform = F2;

        // push this link's transform
        MatrixStack.push( joint->outboard_link->pose.transform );

        // recurse to descend the hierarchy
        transform_link( joint->outboard_link );

        // pop this link's transform
        MatrixStack.pop();
    }
}

//------------------------------------------------------------------------------
/// Entry point for transformation.  Called by idle.
void transform_hierarchy( void ) {

    articulatedbody.root_link->pose = articulatedbody.pose;
    articulatedbody.root_link->pose.transform = articulatedbody.pose.transform;

    // push the root link's transform
    MatrixStack.push( articulatedbody.root_link->pose.transform );

    // descend the heirarchy and update the poses of all children
    transform_link( articulatedbody.root_link );

    // pop the root link's transform
    MatrixStack.pop();
}

//------------------------------------------------------------------------------
/// Interpolate the articulated body trajectory.
void interpolate( ArticulatedBody* body ) {

    body->trajectory.frame_tic++;

    bool transition = false;

    // handle special transitions such as the cycle or intermediate transitions from keyframe to keyframe
    if( body->trajectory.frame_tic == body->trajectory.keyframe( body->trajectory.keyframes() - 1 ).frame ) {
        body->trajectory_position = 0.0;
        body->trajectory.prev_keyframe_id = 0;
        body->trajectory.next_keyframe_id = 1;
        body->trajectory.frame_tic = 1;
    } else if( body->trajectory.frame_tic == body->trajectory.keyframe( body->trajectory.next_keyframe_id ).frame ) {
        body->trajectory.prev_keyframe_id++;
        body->trajectory.next_keyframe_id++;
        transition = true;
    }

    Keyframe keyframe0 = body->trajectory.keyframe( body->trajectory.prev_keyframe_id );
    Keyframe keyframe1 = body->trajectory.keyframe( body->trajectory.next_keyframe_id );

    double frame0 = (double) keyframe0.frame - 1;
    double frame1 = (double) keyframe1.frame - 1;
    double dframe = frame1 - frame0;
    double u = 0.0;
    Eigen::MatrixXd C;

    SdS sds_frame0 = body->trajectory.spline_arclength_map.at( body->trajectory.prev_keyframe_id + 1 );
    SdS sds_frame1 = body->trajectory.spline_arclength_map.at( body->trajectory.next_keyframe_id + 1 );

    double dsdframe = sds_frame1.second / dframe;
    double dt = ((double)body->trajectory.frame_tic - frame0) / dframe;

    C = CubicSpline::blend( body->trajectory.spline_basis, body->trajectory.controlpoints, body->trajectory.next_keyframe_id + 1 );

    // integrate rotation
    double drotxdt = keyframe1.pose.orientation.x() - keyframe0.pose.orientation.x();
    double drotydt = keyframe1.pose.orientation.y() - keyframe0.pose.orientation.y();
    double drotzdt = keyframe1.pose.orientation.z() - keyframe0.pose.orientation.z();

    body->pose.orientation.x( keyframe0.pose.orientation.x() + drotxdt * dt );
    body->pose.orientation.y( keyframe0.pose.orientation.y() + drotydt * dt );
    body->pose.orientation.z( keyframe0.pose.orientation.z() + drotzdt * dt );

    // integrate position
    double length_of_current_spline = sds_frame1.second;
    double position_on_current_spline = body->trajectory_position - sds_frame0.first + dsdframe;

    u = position_on_current_spline / length_of_current_spline;
    body->trajectory_position += dsdframe;

    body->distance_travelled += dsdframe;
    body->pose.position = CubicSpline::position( C, u );
    body->pose.transformByQuaternion();
}

//------------------------------------------------------------------------------
/// The heart of interpolation.  Interpolate trajectories defined in the joints.
/// Done throughout the kinematic chain by recursion
void interpolate( Joint* joint ) {

    // if this joint has keyframes interpolate
    if( joint->trajectory.keyframes() ) {
        // if the joint is moving through the trajectory backward and it is a reversible trajectory
        // then decrement the frame tic; otherwise, increment it
        if( !joint->trajectory.forward && joint->trajectory.reversible ) {
            joint->trajectory.frame_tic--;
        } else {
            joint->trajectory.frame_tic++;
        }

        // updating the keyframe ids
        if( joint->trajectory.reversible &&
            ( joint->trajectory.frame_tic == joint->trajectory.keyframe( joint->trajectory.keyframes() - 1 ).frame ||
              ( joint->trajectory.frame_tic == joint->trajectory.keyframe( 0 ).frame && frame_id > joint->trajectory.keyframe( 0 ).frame ) ) ) {
            // if the joint is reversible and has reached the end of its trajectory, swap the keyframes
                int temp = joint->trajectory.prev_keyframe_id;
                joint->trajectory.prev_keyframe_id = joint->trajectory.next_keyframe_id;
                joint->trajectory.next_keyframe_id = temp;
                joint->trajectory.forward = !joint->trajectory.forward;
        } else if( joint->trajectory.cyclic && joint->trajectory.frame_tic == joint->trajectory.keyframe( joint->trajectory.keyframes() - 1 ).frame ) {
            // if cyclic but not reversible and it has reached the end of the cycle, restart it
                joint->trajectory.prev_keyframe_id = 0;
                joint->trajectory.next_keyframe_id = 1;
                joint->trajectory.frame_tic = 1;
        } else if( joint->trajectory.forward ) {
          // check for a transition point over the keyframes.
            // if its a forward trajectory and the tic has reached a keyframe, then increment the identifiers
            if( joint->trajectory.frame_tic == joint->trajectory.keyframe( joint->trajectory.next_keyframe_id ).frame ) {
                joint->trajectory.prev_keyframe_id++;
                joint->trajectory.next_keyframe_id++;
            }
        } else {
            // if its a backward trajectory and the tic has reached a keyframe, then decrement the identifiers
            if( joint->trajectory.frame_tic == joint->trajectory.keyframe( joint->trajectory.next_keyframe_id ).frame ) {
                joint->trajectory.prev_keyframe_id--;
                joint->trajectory.next_keyframe_id--;
            }
        }

        // Now, do the interpolation

        // get the current keyframes
        Keyframe keyframe0 = joint->trajectory.keyframe( joint->trajectory.prev_keyframe_id );
        Keyframe keyframe1 = joint->trajectory.keyframe( joint->trajectory.next_keyframe_id );

        // blending matrix for the spline
        Eigen::MatrixXd C;

        // calculate 'time' in terms of frames
        double frame0 = (double) keyframe0.frame;
        double frame1 = (double) keyframe1.frame;
        double dframe = frame1 - frame0;
        double dt = ((double)joint->trajectory.frame_tic - frame0) / dframe;

        // calculate the blending matrix C
        if( joint->trajectory.forward )
            C = CubicSpline::blend( joint->trajectory.spline_basis, joint->trajectory.controlpoints, joint->trajectory.next_keyframe_id + 1 );
        else
            C = CubicSpline::blend( joint->trajectory.spline_basis, joint->trajectory.controlpoints, joint->trajectory.prev_keyframe_id + 1 );

        // euler integrate the joint angle
        double drotxdt = keyframe1.pose.orientation.x() - keyframe0.pose.orientation.x();
        double drotydt = keyframe1.pose.orientation.y() - keyframe0.pose.orientation.y();
        double drotzdt = keyframe1.pose.orientation.z() - keyframe0.pose.orientation.z();

        joint->angle.x( keyframe0.pose.orientation.x() + drotxdt * dt );
        joint->angle.y( keyframe0.pose.orientation.y() + drotydt * dt );
        joint->angle.z( keyframe0.pose.orientation.z() + drotzdt * dt );
    }
    // descend the hierarchy via recursion
    for( unsigned int i = 0; i < joint->outboard_link->child_joints(); i++ ) {
        Joint* child_joint = joint->outboard_link->child_joint( i );
        interpolate( child_joint );
    }
}

//------------------------------------------------------------------------------
/// Interpolate the articulated body actor.
void interpolate_hierarchy( void ) {
    interpolate( &articulatedbody );

    for( unsigned int i = 0; i < articulatedbody.root_link->child_joints(); i++ ) {
        Joint* joint = articulatedbody.root_link->child_joint( i );
        interpolate( joint );
    }
}

//------------------------------------------------------------------------------
// Modified SimpleGLUT
//------------------------------------------------------------------------------
/// Initialization of OpenGL
void init(void)
{
   glClearColor (0.0, 0.0, 0.0, 0.0);
   glClearDepth (1.0);
   glShadeModel (GL_SMOOTH);
   glEnable( GL_LINE_SMOOTH );
}

//------------------------------------------------------------------------------
/// Registered OpenGL Display Callback Function
void display(void)
{
    // recompute the camera projection in case it was moved
    camera.project();
    glMatrixMode( GL_PROJECTION );
    glLoadIdentity( );
    glLoadMatrixd( camera.projection.arrayOpenGL() );

    // now process the scene
    glMatrixMode( GL_MODELVIEW );
    glLoadIdentity( );

    glClear (GL_COLOR_BUFFER_BIT);
    glEnable(GL_LIGHTING);
    glEnable(GL_DEPTH_TEST);

    GLfloat LightAmbient[] =  { 0.4f, 0.4f, 0.4f, 1.0f};
    GLfloat LightDiffuse[] =  { 0.3f, 0.3f, 0.3f, 1.0f};
    GLfloat LightSpecular[] = { 0.4f, 0.4f, 0.4f, 1.0f};
    //GLfloat LightSpecular[] = { 0.0f, 0.0f, 0.0f, 1.0f};
    //GLfloat LightPosition[] = { 5.0f, 5.0f, 5.0f, 1.0f};
    GLfloat LightPosition[] = { 0.0f, 5.0f, 0.0f, 1.0f};

    glClearColor( 0.25, 0.25, 0.25, 0.0 );
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

    glLightfv(GL_LIGHT0, GL_AMBIENT, LightAmbient);
    glLightfv(GL_LIGHT0, GL_DIFFUSE, LightDiffuse);
    glLightfv(GL_LIGHT0, GL_SPECULAR, LightSpecular);
    glLightfv(GL_LIGHT0, GL_POSITION, LightPosition);
    glEnable(GL_LIGHT0);
    glShadeModel(GL_SMOOTH);

    if( opt_draw_actor )
        draw_articulatedbody( articulatedbody );

    if( opt_draw_trajectories ) {
        draw_trajectory( articulatedbody.trajectory, NULL );

        Matrix4 F1, X1, J1, F2;

        F1 = body_left_hip->inboard_link->pose.transform;
        X1 = Matrix4::translationMatrix( body_left_hip->inboard_displacement );
        J1 = body_left_hip->frame_transformation * EulerAngle::matrix4( EULER_ANGLE_XYZ, EULER_HANDED_RIGHT, body_left_hip->median_angle );
        F2 = F1 * X1 * J1;
        draw_trajectory( body_left_hip->trajectory, &F2 );

        F1 = body_right_hip->inboard_link->pose.transform;
        X1 = Matrix4::translationMatrix( body_right_hip->inboard_displacement );
        J1 = body_right_hip->frame_transformation * EulerAngle::matrix4( EULER_ANGLE_XYZ, EULER_HANDED_RIGHT, body_right_hip->median_angle );
        F2 = F1 * X1 * J1;
        draw_trajectory( body_right_hip->trajectory, &F2 );

        F1 = body_left_knee->inboard_link->pose.transform;
        X1 = Matrix4::translationMatrix( body_left_knee->inboard_displacement );
        J1 = body_left_knee->frame_transformation * EulerAngle::matrix4( EULER_ANGLE_XYZ, EULER_HANDED_RIGHT, body_left_knee->median_angle );
        F2 = F1 * X1 * J1;
        draw_trajectory( body_left_knee->trajectory, &F2 );

        F1 = body_right_knee->inboard_link->pose.transform;
        X1 = Matrix4::translationMatrix( body_right_knee->inboard_displacement );
        J1 = body_right_knee->frame_transformation * EulerAngle::matrix4( EULER_ANGLE_XYZ, EULER_HANDED_RIGHT, body_right_knee->median_angle );
        F2 = F1 * X1 * J1;
        draw_trajectory( body_right_knee->trajectory, &F2 );

        F1 = body_left_shoulder->inboard_link->pose.transform;
        X1 = Matrix4::translationMatrix( body_left_shoulder->inboard_displacement );
        J1 = body_left_shoulder->frame_transformation * EulerAngle::matrix4( EULER_ANGLE_XYZ, EULER_HANDED_RIGHT, body_left_shoulder->median_angle );
        F2 = F1 * X1 * J1;
        draw_trajectory( body_left_shoulder->trajectory, &F2 );

        F1 = body_right_shoulder->inboard_link->pose.transform;
        X1 = Matrix4::translationMatrix( body_right_shoulder->inboard_displacement );
        J1 = body_right_shoulder->frame_transformation * EulerAngle::matrix4( EULER_ANGLE_XYZ, EULER_HANDED_RIGHT, body_right_shoulder->median_angle );
        F2 = F1 * X1 * J1;
        draw_trajectory( body_right_shoulder->trajectory, &F2 );

        F1 = body_left_elbow->inboard_link->pose.transform;
        X1 = Matrix4::translationMatrix( body_left_elbow->inboard_displacement );
        J1 = body_left_elbow->frame_transformation * EulerAngle::matrix4( EULER_ANGLE_XYZ, EULER_HANDED_RIGHT, body_left_elbow->median_angle );
        F2 = F1 * X1 * J1;
        draw_trajectory( body_left_elbow->trajectory, &F2 );

        F1 = body_right_elbow->inboard_link->pose.transform;
        X1 = Matrix4::translationMatrix( body_right_elbow->inboard_displacement );
        J1 = body_right_elbow->frame_transformation * EulerAngle::matrix4( EULER_ANGLE_XYZ, EULER_HANDED_RIGHT, body_right_elbow->median_angle );
        F2 = F1 * X1 * J1;
        draw_trajectory( body_right_elbow->trajectory, &F2 );
    }

    glutSwapBuffers();

    if( GENERATE_MOVIE ) {
        sprintf( filename, "%s_%.04d.tif",filetitle.c_str(), frame_id );
        printf( "%s\n", filename );
        Utilities::writetiff( filename, "movie", 0, 0, Width, Height, COMPRESSION_NONE );
    }
}
//------------------------------------------------------------------------------
/// Registered OpenGL Reshape Callback Function
void reshape( int w, int h )
{
    glViewport( 0, 0, (GLsizei) w, (GLsizei) h );
    glMatrixMode( GL_PROJECTION );
    glLoadIdentity( );
    glLoadMatrixd( camera.projection.arrayOpenGL() );

    glMatrixMode( GL_MODELVIEW );
    glLoadIdentity( );

    Width = w;
    Height = h;

}

//------------------------------------------------------------------------------
/// Registered OpenGL (GLUT) Keyboard Callback Function
void keyboard( unsigned char key, int x, int y )
{
    switch( key ) {
    case 27:
        exit( 0 );
        break;
    case 49:
        opt_draw_actor = !opt_draw_actor;
        break;
    case 50:
        opt_draw_trajectories = !opt_draw_trajectories;
        break;
    case 97:    // a
        camera.track_left( PI/16 );     // not functional at the moment
        break;
    case 100:    // d
        camera.track_right( PI/16 );    // not functional at the moment
        break;
    case 115:    // s
        camera.dolly_out( 0.5 );        // not functional at the moment
        break;
    case 119:    // w
        camera.dolly_in( 0.5 );         // not functional at the moment
        break;
    case 112:
        sim_pause = !sim_pause;
        break;
    default:
        std::cout << "key: " << key << std::endl;
        break;
   }
}
//------------------------------------------------------------------------------
/// Registered OpenGL Idle Callback Function
/// Computes and Updates all component transformations
void idle( void )
{
    if( !sim_pause ) {
        frame_id++;
        interpolate_hierarchy( );
        transform_hierarchy( );
    }
    glutPostRedisplay( );
}

//------------------------------------------------------------------------------
/// Program entry point
/// Initialize then start rendering
int main( int argc, char** argv )
{
    initBody( );
    initCamera( );

    // initialize OpenGL
    glutInit( &argc, argv );
    glutInitDisplayMode( GLUT_DOUBLE | GLUT_RGB |GLUT_DEPTH );
    glutInitWindowSize( 600, 600 );
    glutInitWindowPosition( 100, 100 );
    glutCreateWindow( title.c_str() );

    init( );

    // Register OpenGL callback functions
    glutDisplayFunc( display );
    glutReshapeFunc( reshape );
    glutKeyboardFunc( keyboard );
    glutIdleFunc( idle );

    // Start the simulation
    glutMainLoop( );
    return 0;
}
//------------------------------------------------------------------------------

