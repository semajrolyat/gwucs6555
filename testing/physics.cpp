/*-----------------------THE GEORGE WASHINGTON UNIVERSITY-----------------------
James Taylor                                                         jrt@gwu.edu

Physics unit test

------------------------------------------------------------------------------*/
#include <GL/glut.h>
#include <stdio.h>
#include <iostream>
#include <string>
#include <stack>
#include <stdexcept>

#include <cs6555/Math/CubicSpline.h>
#include <cs6555/Constants.h>
#include <cs6555/Math/Vector3.h>
#include <cs6555/Math/Matrix4.h>
#include <cs6555/Math/EulerAngle.h>

#include <cs6555/Mesh.h>
#include <cs6555/MeshLoader.h>
#include <cs6555/Color.h>
#include <cs6555/Material.h>
#include <cs6555/Utilities.h>

#include <cs6555/ArticulatedBody.h>
#include <cs6555/RigidBody.h>
#include <cs6555/DeformableBody.h>
#include <cs6555/Patch.h>

#include <cs6555/BoundingVolume.h>
#include <cs6555/BoundingSphere.h>
#include <cs6555/AABB.h>
#include <cs6555/OBB.h>

#include <cs6555/Physics.h>

#include <cs6555/Event.h>
#include <cs6555/ContactEvent.h>

#include <cs6555/Scene.h>

//------------------------------------------------------------------------------
// Hardcoded Defines
//------------------------------------------------------------------------------

#define GENERATE_MOVIE  0               // No, do not generate a movie
//#define GENERATE_MOVIE  1               // Yes, do generate a movie

//------------------------------------------------------------------------------
// Toggle Options
//------------------------------------------------------------------------------

bool opt_draw_actor = true;                 // toggle by pressing 1
bool opt_draw_trajectories = false;         // toggle by pressing 2
bool sim_pause = false;                     // toggle by pressing p

//------------------------------------------------------------------------------

static int frame_id = 0;                    // current frame.  Counter
int Width = 600;                            // Width of OpenGL Viewport
int Height = 600;                           // Height of OpenGL Viewport
unsigned char *Pixels;
std::string title;                          // Window Title
std::string filetitle;                      // Movie file title prefix
char filename[128];                         // Buffer for filename of a frame render to file

double line_width = 8.0;                    // Width of a spline when rendering trajectories

double sim_time = 0.0;
double sim_time_step = 1.0/30.0;
double sim_min_time_step = sim_time_step / 10.0;

//------------------------------------------------------------------------------
// Because cannot use the OpenGL matrix stack, define own.
// Only used during idle to precalculate all transforms in hierarchy
static std::stack<Matrix4> matrixstack;
//------------------------------------------------------------------------------

#include <stdlib.h>

Scene scene;

//------------------------------------------------------------------------------
// Scene Functions
//------------------------------------------------------------------------------

Material ball_material;
Material ground_material;

Material ball1_material;
Material ball2_material;
Material ball3_material;

void initFallingBodies( void ) {
    scene.camera.position = Vector3( 0.0, 0.0, 5000.0 );
    scene.camera.viewpoint = Vector3( 0.0, 0.0, 4999.0 );
    scene.camera.far = 10000;
    scene.camera.up = Vector3( 0.0, 1.0, 0.0 );
    scene.camera.project();

    const double radius = 500.0;
    const double mass1 = 1.0;
    const double mass2 = 100.0;

    RigidBody* rb;
    AABB* bv;

    rb = new RigidBody();
    rb->pose = Pose( POSE_BY_QUATERNION, Vector3( 0.0, 0.0, 0.0 ), Vector3( 0.0, 0.0, 0.0 ) );
    rb->pose.position = Vector3( -1000.0, 3000.0, 0.0 );
    rb->force = Vector3( 0.0, 0.0, 0.0 );
    rb->torque = Vector3( 0.0, 0.0, 0.0 );
    rb->cube( mass1, radius, radius, radius );
    bv = new AABB( Vector3( 0.0, 0.0, 0.0 ), Vector3( radius/2.0, radius/2.0, radius/2.0 ) );
    rb->bv( bv );
    scene.insert( rb );

    rb = new RigidBody();
    rb->pose = Pose( POSE_BY_QUATERNION, Vector3( 0.0, 0.0, 0.0 ), Vector3( 0.0, 0.0, 0.0 ) );
    rb->pose.position = Vector3( 1000.0, 3000.0, 0.0 );
    rb->force = Vector3( 0.0, 0.0, 0.0 );
    rb->torque = Vector3( 0.0, 0.0, 0.0 );
    rb->cube( mass2, radius, radius, radius );
    bv = new AABB( Vector3( 0.0, 0.0, 0.0 ), Vector3( radius/2.0, radius/2.0, radius/2.0 ) );
    rb->bv( bv );
    scene.insert( rb );
}

void initBallisticRotation( void ) {
    scene.camera.position = Vector3( 0.0, 0.0, 5000.0 );
    scene.camera.viewpoint = Vector3( 0.0, 0.0, 4999.0 );
    scene.camera.far = 10000;
    scene.camera.up = Vector3( 0.0, 1.0, 0.0 );
    scene.camera.project();

    const double radius = 500.0;
    const double mass = 1.0;

    RigidBody* rb1 = new RigidBody();
    rb1->pose = Pose( POSE_BY_QUATERNION, Vector3( 0.0, 0.0, 0.0 ), Vector3( 0.0, 0.0, 0.0 ) );
    rb1->pose.position = Vector3( -3000.0, 0.0, 0.0 );
    rb1->force = Vector3( 3000.0, 8000.0, 0.0 );
    rb1->torque = Vector3( 0.0, 0.0, 100000.0 );
    rb1->cube( mass, radius, radius, radius );
    BoundingSphere* bv1 = new BoundingSphere( radius );
    rb1->bv( bv1 );
    scene.insert( rb1 );
}

void initBallisticCollisionAABB( void ) {
    scene.camera.position = Vector3( 0.0, 0.0, 5000.0 );
    scene.camera.viewpoint = Vector3( 0.0, 0.0, 4999.0 );
    scene.camera.far = 10000;
    scene.camera.up = Vector3( 0.0, 1.0, 0.0 );
    scene.camera.project();

    const double radius = 500.0;
    const double mass = 1.0;

    RigidBody* box = new RigidBody();
    box->pose = Pose( POSE_BY_QUATERNION, Vector3( 0.0, 0.0, 0.0 ), Vector3( 0.0, 0.0, 0.0 ) );
    box->pose.position = Vector3( 0.0, 3000.0, 0.0 );
    box->force = Vector3( 0.0, 0.0, 0.0 );
    box->torque = Vector3( 0.0, 0.0, 0.0 );
    box->cube( mass, radius, radius, radius );
    AABB* bvbox = new AABB( Vector3( 0.0, 0.0, 0.0 ), Vector3( radius/2.0, radius/2.0, radius/2.0 ) );
    box->bv( bvbox );
    scene.insert( box );

    RigidBody* ground = new RigidBody( RIGIDBODY_STATIC );
    ground->pose = Pose( POSE_BY_QUATERNION, Vector3( 0.0, 0.0, 0.0 ), Vector3( 0.0, 0.0, 0.0 ) );
    ground->pose.position = Vector3( 0.0, -3000.0, 0.0 );
    ground->force = Vector3( 0.0, 0.0, 0.0 );
    ground->torque = Vector3( 0.0, 0.0, 0.0 );
    double width = 1000000.0;
    double height = 100.0;
    double depth = 1000000.0;
    ground->cube( 1000000.0, width, height, depth );
    AABB* bvground = new AABB( Vector3( 0.0, 0.0, 0.0 ), Vector3( width/2.0, height/2.0, depth/2.0 ) );
    ground->bv( bvground );
    ground->pose.transformByQuaternion();
    scene.insert( ground );
}

void initBallisticCollisionAABBandSphere( void ) {
    scene.camera.position = Vector3( 0.0, 0.0, 500.0 );
    scene.camera.viewpoint = Vector3( 0.0, 0.0, 499.0 );
    scene.camera.far = 10000;
    scene.camera.up = Vector3( 0.0, 1.0, 0.0 );
    scene.camera.project();

    const double radius = 50.0;
    const double mass = 1.0;

    ball_material = Material( Color( 0.8, 0.0, 0.0, 1.0 ), Color( 0.6, 0.0, 0.0, 1.0 ), Color( 0.4, 0.0, 0.0, 1.0 ), Color( 0.0, 0.0, 0.0, 1.0 ), 10.0 );

    RigidBody* ball = new RigidBody();
    ball->pose = Pose( POSE_BY_QUATERNION, Vector3( 0.0, 0.0, 0.0 ), Vector3( 0.0, 0.0, 0.0 ) );
    ball->pose.position = Vector3( 0.0, 250.0, 0.0 );
    ball->force = Vector3( 0.0, 0.0, 0.0 );
    ball->torque = Vector3( 0.0, 0.0, 0.0 );
    ball->sphere( mass, radius, 12, 5, &ball_material );
    BoundingSphere* bvball = new BoundingSphere( radius );
    ball->bv( bvball );
    scene.insert( ball );

    ground_material = Material( Color( 0.6, 0.6, 0.3, 1.0 ), Color( 0.4, 0.4, 0.2, 1.0 ), Color( 0.2, 0.2, 0.1, 1.0 ), Color( 0.0, 0.0, 0.0, 1.0 ), 10.0 );

    RigidBody* ground = new RigidBody( RIGIDBODY_STATIC );
    ground->pose = Pose( POSE_BY_QUATERNION, Vector3( 0.0, 0.0, 0.0 ), Vector3( 0.0, 0.0, 0.0 ) );
    ground->pose.position = Vector3( 0.0, -250.0, 0.0 );
    ground->force = Vector3( 0.0, 0.0, 0.0 );
    ground->torque = Vector3( 0.0, 0.0, 0.0 );
    double width = 1000000.0;
    double height = 100.0;
    double depth = 1000000.0;
    ground->cube( 1000000.0, width, height, depth, &ground_material );
    AABB* bvground = new AABB( Vector3( 0.0, 0.0, 0.0 ), Vector3( width/2.0, height/2.0, depth/2.0 ) );
    ground->bv( bvground );
    ground->pose.transformByQuaternion();
    scene.insert( ground );
}

void initBallisticCollisionAABBandSphere2( void ) {
    scene.camera.position = Vector3( 0.0, 0.0, 500.0 );
    scene.camera.viewpoint = Vector3( 0.0, 0.0, 499.0 );
    scene.camera.far = 10000;
    scene.camera.up = Vector3( 0.0, 1.0, 0.0 );
    scene.camera.project();

    const double radius = 50.0;
    const double mass = 1.0;

    ball_material = Material( Color( 0.8, 0.0, 0.0, 1.0 ), Color( 0.6, 0.0, 0.0, 1.0 ), Color( 0.4, 0.0, 0.0, 1.0 ), Color( 0.0, 0.0, 0.0, 1.0 ), 10.0 );

    RigidBody* ball = new RigidBody();
    ball->pose = Pose( POSE_BY_QUATERNION, Vector3( 0.0, 0.0, 0.0 ), Vector3( 0.0, 0.0, 0.0 ) );
    ball->pose.position = Vector3( 300.0, 250.0, 0.0 );
    ball->force = Vector3( -200, 0.0, 0.0 );
    ball->torque = Vector3( 0.0, 0.0, 0.0 );
    ball->sphere( mass, radius, 12, 5, &ball_material );
    ball->coefficient_of_restitution = 0.83;        // basketball
    BoundingSphere* bvball = new BoundingSphere( radius );
    ball->bv( bvball );
    scene.insert( ball );

    ground_material = Material( Color( 0.6, 0.6, 0.3, 1.0 ), Color( 0.4, 0.4, 0.2, 1.0 ), Color( 0.2, 0.2, 0.1, 1.0 ), Color( 0.0, 0.0, 0.0, 1.0 ), 10.0 );

    RigidBody* ground = new RigidBody( RIGIDBODY_STATIC );
    ground->pose = Pose( POSE_BY_QUATERNION, Vector3( 0.0, 0.0, 0.0 ), Vector3( 0.0, 0.0, 0.0 ) );
    ground->pose.position = Vector3( 0.0, -250.0, 0.0 );
    ground->force = Vector3( 0.0, 0.0, 0.0 );
    ground->torque = Vector3( 0.0, 0.0, 0.0 );
    double width = 1000000.0;
    double height = 100.0;
    double depth = 1000000.0;
    ground->cube( 1000000.0, width, height, depth, &ground_material );
    AABB* bvground = new AABB( Vector3( 0.0, 0.0, 0.0 ), Vector3( width/2.0, height/2.0, depth/2.0 ) );
    ground->bv( bvground );
    ground->pose.transformByQuaternion();
    scene.insert( ground );
}

void initBallisticCollisionAABBandSphere2WithTorque( void ) {
    scene.camera.position = Vector3( 0.0, 0.0, 500.0 );
    scene.camera.viewpoint = Vector3( 0.0, 0.0, 499.0 );
    scene.camera.far = 10000;
    scene.camera.up = Vector3( 0.0, 1.0, 0.0 );
    scene.camera.project();

    const double radius = 50.0;
    const double mass = 1.0;

    //ball_material = Material( Color( 0.8, 0.0, 0.0, 1.0 ), Color( 0.6, 0.0, 0.0, 1.0 ), Color( 0.4, 0.0, 0.0, 1.0 ), Color( 0.0, 0.0, 0.0, 1.0 ), 10.0 );

    RigidBody* ball = new RigidBody();
    ball->pose = Pose( POSE_BY_QUATERNION, Vector3( 0.0, 0.0, 0.0 ), Vector3( 0.0, 0.0, 0.0 ) );
    ball->pose.position = Vector3( 250.0, 250.0, 0.0 );
    ball->force = Vector3( -250, 0.0, 0.0 );
    ball->torque = Vector3( 0.0, 0.0, 10000.0 );
    //ball->sphere( mass, radius, 12, 5, &ball_material );
    ball->sphere( mass, radius, 12, 5 );
    ball->coefficient_of_restitution = 0.83;        // basketball
    BoundingSphere* bvball = new BoundingSphere( radius );
    ball->bv( bvball );
    scene.insert( ball );

    ground_material = Material( Color( 0.6, 0.6, 0.3, 1.0 ), Color( 0.4, 0.4, 0.2, 1.0 ), Color( 0.2, 0.2, 0.1, 1.0 ), Color( 0.0, 0.0, 0.0, 1.0 ), 10.0 );

    RigidBody* ground = new RigidBody( RIGIDBODY_STATIC );
    ground->pose = Pose( POSE_BY_QUATERNION, Vector3( 0.0, 0.0, 0.0 ), Vector3( 0.0, 0.0, 0.0 ) );
    ground->pose.position = Vector3( 0.0, -250.0, 0.0 );
    ground->force = Vector3( 0.0, 0.0, 0.0 );
    ground->torque = Vector3( 0.0, 0.0, 0.0 );
    double width = 1000000.0;
    double height = 100.0;
    double depth = 1000000.0;
    ground->cube( 1000000.0, width, height, depth, &ground_material );
    AABB* bvground = new AABB( Vector3( 0.0, 0.0, 0.0 ), Vector3( width/2.0, height/2.0, depth/2.0 ) );
    ground->bv( bvground );
    ground->pose.transformByQuaternion();
    scene.insert( ground );
}

void initBallisticCollisionRestingContact( void ) {
    scene.camera.position = Vector3( 0.0, 0.0, 500.0 );
    scene.camera.viewpoint = Vector3( 0.0, 0.0, 499.0 );
    scene.camera.far = 10000;
    scene.camera.up = Vector3( 0.0, 1.0, 0.0 );
    scene.camera.project();

    const double radius = 50.0;
    const double mass = 1.0;

    ball_material = Material( Color( 0.8, 0.0, 0.0, 1.0 ), Color( 0.6, 0.0, 0.0, 1.0 ), Color( 0.4, 0.0, 0.0, 1.0 ), Color( 0.0, 0.0, 0.0, 1.0 ), 10.0 );

    RigidBody* ball = new RigidBody();
    ball->pose = Pose( POSE_BY_QUATERNION, Vector3( 0.0, 0.0, 0.0 ), Vector3( 0.0, 0.0, 0.0 ) );
    ball->pose.position = Vector3( 0.0, -150.0, 0.0 );
    ball->force = Vector3( 0.0, 0.0, 0.0 );
    ball->torque = Vector3( 0.0, 0.0, 0.0 );
    ball->sphere( mass, radius, 12, 5, &ball_material );
    BoundingSphere* bvball = new BoundingSphere( radius );
    ball->bv( bvball );
    scene.insert( ball );

    ground_material = Material( Color( 0.6, 0.6, 0.3, 1.0 ), Color( 0.4, 0.4, 0.2, 1.0 ), Color( 0.2, 0.2, 0.1, 1.0 ), Color( 0.0, 0.0, 0.0, 1.0 ), 10.0 );

    RigidBody* ground = new RigidBody( RIGIDBODY_STATIC );
    ground->pose = Pose( POSE_BY_QUATERNION, Vector3( 0.0, 0.0, 0.0 ), Vector3( 0.0, 0.0, 0.0 ) );
    ground->pose.position = Vector3( 0.0, -250.0, 0.0 );
    ground->force = Vector3( 0.0, 0.0, 0.0 );
    ground->torque = Vector3( 0.0, 0.0, 0.0 );
    double width = 1000000.0;
    double height = 100.0;
    double depth = 1000000.0;
    ground->cube( 1000000.0, width, height, depth, &ground_material );
    AABB* bvground = new AABB( Vector3( 0.0, 0.0, 0.0 ), Vector3( width/2.0, height/2.0, depth/2.0 ) );
    ground->bv( bvground );
    ground->pose.transformByQuaternion();
    scene.insert( ground );
}

/*
void initBallisticCollisionRollingContact( void ) {
    scene.camera.position = Vector3( 0.0, 0.0, 500.0 );
    scene.camera.viewpoint = Vector3( 0.0, 0.0, 499.0 );
    scene.camera.far = 10000;
    scene.camera.up = Vector3( 0.0, 1.0, 0.0 );
    scene.camera.project();

    const double radius = 50.0;
    const double mass = 1.0;

    ball_material = Material( Color( 0.8, 0.0, 0.0, 1.0 ), Color( 0.6, 0.0, 0.0, 1.0 ), Color( 0.4, 0.0, 0.0, 1.0 ), Color( 0.0, 0.0, 0.0, 1.0 ), 10.0 );

    RigidBody* ball = new RigidBody();
    ball->pose = Pose( POSE_BY_QUATERNION, Vector3( 0.0, 0.0, 0.0 ), Vector3( 0.0, 0.0, 0.0 ) );
    ball->pose.position = Vector3( 0.0, -150.0, 0.0 );
    ball->force = Vector3( 0.0, 0.0, 0.0 );
    ball->torque = Vector3( 0.0, 0.0, 0.0 );
    ball->sphere( mass, radius, 12, 5, &ball_material );
    BoundingSphere* bvball = new BoundingSphere( radius );
    ball->bv( bvball );
    scene.insert( ball );

    ground_material = Material( Color( 0.6, 0.6, 0.3, 1.0 ), Color( 0.4, 0.4, 0.2, 1.0 ), Color( 0.2, 0.2, 0.1, 1.0 ), Color( 0.0, 0.0, 0.0, 1.0 ), 10.0 );

    RigidBody* ground = new RigidBody( RIGIDBODY_STATIC );
    ground->pose = Pose( POSE_BY_QUATERNION, Vector3( 0.0, 0.0, 0.0 ), Vector3( 0.0, 0.0, 0.0 ) );
    ground->pose.position = Vector3( 0.0, -250.0, 0.0 );
    ground->force = Vector3( 0.0, 0.0, 0.0 );
    ground->torque = Vector3( 0.0, 0.0, 0.0 );
    double width = 1000000.0;
    double height = 100.0;
    double depth = 1000000.0;
    ground->cube( 1000000.0, width, height, depth, &ground_material );
    AABB* bvground = new AABB( Vector3( 0.0, 0.0, 0.0 ), Vector3( width/2.0, height/2.0, depth/2.0 ) );
    ground->bv( bvground );
    ground->pose.transformByQuaternion();
    scene.insert( ground );
}
*/
void initBallisticCollisionMultiball( void ) {
    scene.camera.position = Vector3( 0.0, 0.0, 500.0 );
    scene.camera.viewpoint = Vector3( 0.0, 0.0, 499.0 );
    scene.camera.far = 10000;
    scene.camera.up = Vector3( 0.0, 1.0, 0.0 );
    scene.camera.project();

    const double radius = 50.0;
    const double mass1 = 100.0;
    const double mass2 = 100.0;

    ball1_material = Material( Color( 0.8, 0.0, 0.0, 1.0 ), Color( 0.6, 0.0, 0.0, 1.0 ), Color( 0.4, 0.0, 0.0, 1.0 ), Color( 0.0, 0.0, 0.0, 1.0 ), 10.0 );

    RigidBody* ball1 = new RigidBody();
    ball1->pose = Pose( POSE_BY_QUATERNION, Vector3( 0.0, 0.0, 0.0 ), Vector3( 0.0, 0.0, 0.0 ) );
    ball1->pose.position = Vector3( -300.0, 300.0, 0.0 );
    ball1->force = Vector3( 500.0 * mass1, 0.0, 0.0 );
    ball1->torque = Vector3( 0.0, 0.0, 0.0 );
    ball1->sphere( mass1, radius, 12, 5, &ball1_material );
    BoundingSphere* bvball1 = new BoundingSphere( radius );
    ball1->bv( bvball1 );
    scene.insert( ball1 );

    ball2_material = Material( Color( 0.0, 0.8, 0.8, 1.0 ), Color( 0.0, 0.8, 0.8, 1.0 ), Color( 0.0, 0.4, 4.0, 1.0 ), Color( 0.0, 0.0, 0.0, 1.0 ), 10.0 );

    RigidBody* ball2 = new RigidBody();
    ball2->pose = Pose( POSE_BY_QUATERNION, Vector3( 0.0, 0.0, 0.0 ), Vector3( 0.0, 0.0, 0.0 ) );
    ball2->pose.position = Vector3( 300.0, 300.0, 0.0 );
    ball2->force = Vector3( -500.0 * mass2, 0.0, 0.0 );
    ball2->torque = Vector3( 0.0, 0.0, 0.0 );
    ball2->sphere( mass2, radius, 12, 5, &ball2_material );
    BoundingSphere* bvball2 = new BoundingSphere( radius );
    ball2->bv( bvball2 );
    scene.insert( ball2 );

    ground_material = Material( Color( 0.6, 0.6, 0.3, 1.0 ), Color( 0.4, 0.4, 0.2, 1.0 ), Color( 0.2, 0.2, 0.1, 1.0 ), Color( 0.0, 0.0, 0.0, 1.0 ), 10.0 );

    RigidBody* ground = new RigidBody( RIGIDBODY_STATIC );
    ground->pose = Pose( POSE_BY_QUATERNION, Vector3( 0.0, 0.0, 0.0 ), Vector3( 0.0, 0.0, 0.0 ) );
    ground->pose.position = Vector3( 0.0, -250.0, 0.0 );
    ground->force = Vector3( 0.0, 0.0, 0.0 );
    ground->torque = Vector3( 0.0, 0.0, 0.0 );
    double width = 1000000.0;
    double height = 100.0;
    double depth = 1000000.0;
    ground->cube( 1000000.0, width, height, depth, &ground_material );
    AABB* bvground = new AABB( Vector3( 0.0, 0.0, 0.0 ), Vector3( width/2.0, height/2.0, depth/2.0 ) );
    ground->bv( bvground );
    ground->pose.transformByQuaternion();
    scene.insert( ground );
}

void initBallisticCollisionMultiball2( void ) {
    scene.camera.position = Vector3( 0.0, 0.0, 500.0 );
    scene.camera.viewpoint = Vector3( 0.0, 0.0, 499.0 );
    scene.camera.far = 10000;
    scene.camera.up = Vector3( 0.0, 1.0, 0.0 );
    scene.camera.project();

    const double radius = 50.0;
    const double mass1 = 1.0;
    const double mass2 = 1.0;

    ball1_material = Material( Color( 0.8, 0.0, 0.0, 1.0 ), Color( 0.6, 0.0, 0.0, 1.0 ), Color( 0.4, 0.0, 0.0, 1.0 ), Color( 0.0, 0.0, 0.0, 1.0 ), 10.0 );

    RigidBody* ball1 = new RigidBody();
    ball1->pose = Pose( POSE_BY_QUATERNION, Vector3( 0.0, 0.0, 0.0 ), Vector3( 0.0, 0.0, 0.0 ) );
    //ball1->pose.position = Vector3( 0.0, 400.0, 0.0 );
    ball1->pose.position = Vector3( 0.0, 500.0, 0.0 );
    ball1->force = Vector3( 0.0, 0.0, 0.0 );
    ball1->torque = Vector3( 0.0, 0.0, 0.0 );
    ball1->sphere( mass1, radius, 12, 5, &ball1_material );
    BoundingSphere* bvball1 = new BoundingSphere( radius );
    ball1->bv( bvball1 );
    scene.insert( ball1 );

    ball2_material = Material( Color( 0.0, 0.8, 0.8, 1.0 ), Color( 0.0, 0.8, 0.8, 1.0 ), Color( 0.0, 0.4, 4.0, 1.0 ), Color( 0.0, 0.0, 0.0, 1.0 ), 10.0 );

    RigidBody* ball2 = new RigidBody();
    ball2->pose = Pose( POSE_BY_QUATERNION, Vector3( 0.0, 0.0, 0.0 ), Vector3( 0.0, 0.0, 0.0 ) );
    ball2->pose.position = Vector3( 0.0, 200.0, 0.0 );
    ball2->force = Vector3( 0.0, 0.0, 0.0 );
    ball2->torque = Vector3( 0.0, 0.0, 0.0 );
    ball2->sphere( mass2, radius, 12, 5, &ball2_material );
    BoundingSphere* bvball2 = new BoundingSphere( radius );
    ball2->bv( bvball2 );
    scene.insert( ball2 );

    ground_material = Material( Color( 0.6, 0.6, 0.3, 1.0 ), Color( 0.4, 0.4, 0.2, 1.0 ), Color( 0.2, 0.2, 0.1, 1.0 ), Color( 0.0, 0.0, 0.0, 1.0 ), 10.0 );

    RigidBody* ground = new RigidBody( RIGIDBODY_STATIC );
    ground->pose = Pose( POSE_BY_QUATERNION, Vector3( 0.0, 0.0, 0.0 ), Vector3( 0.0, 0.0, 0.0 ) );
    ground->pose.position = Vector3( 0.0, -250.0, 0.0 );
    ground->force = Vector3( 0.0, 0.0, 0.0 );
    ground->torque = Vector3( 0.0, 0.0, 0.0 );
    double width = 1000000.0;
    double height = 100.0;
    double depth = 1000000.0;
    ground->cube( 1000000.0, width, height, depth, &ground_material );
    AABB* bvground = new AABB( Vector3( 0.0, 0.0, 0.0 ), Vector3( width/2.0, height/2.0, depth/2.0 ) );
    ground->bv( bvground );
    ground->pose.transformByQuaternion();
    scene.insert( ground );
}

void initBallisticCollisionMultiball3( void ) {
    scene.camera.position = Vector3( 0.0, 0.0, 500.0 );
    scene.camera.viewpoint = Vector3( 0.0, 0.0, 499.0 );
    scene.camera.far = 10000;
    scene.camera.up = Vector3( 0.0, 1.0, 0.0 );
    scene.camera.project();

    const double radius = 50.0;
    const double mass1 = 1.0;
    const double mass2 = 1.0;

    ball1_material = Material( Color( 0.8, 0.0, 0.0, 1.0 ), Color( 0.6, 0.0, 0.0, 1.0 ), Color( 0.4, 0.0, 0.0, 1.0 ), Color( 0.0, 0.0, 0.0, 1.0 ), 10.0 );

    RigidBody* ball1 = new RigidBody();
    ball1->pose = Pose( POSE_BY_QUATERNION, Vector3( 0.0, 0.0, 0.0 ), Vector3( 0.0, 0.0, 0.0 ) );
    //ball1->pose.position = Vector3( 0.0, 400.0, 0.0 );
    ball1->pose.position = Vector3( 25.0, 500.0, 0.0 );
    ball1->force = Vector3( 0.0, 0.0, 0.0 );
    ball1->torque = Vector3( 0.0, 0.0, 0.0 );
    ball1->sphere( mass1, radius, 12, 5, &ball1_material );
    BoundingSphere* bvball1 = new BoundingSphere( radius );
    ball1->bv( bvball1 );
    scene.insert( ball1 );

    ball2_material = Material( Color( 0.0, 0.8, 0.8, 1.0 ), Color( 0.0, 0.8, 0.8, 1.0 ), Color( 0.0, 0.4, 4.0, 1.0 ), Color( 0.0, 0.0, 0.0, 1.0 ), 10.0 );

    RigidBody* ball2 = new RigidBody();
    ball2->pose = Pose( POSE_BY_QUATERNION, Vector3( 0.0, 0.0, 0.0 ), Vector3( 0.0, 0.0, 0.0 ) );
    ball2->pose.position = Vector3( -25.0, 200.0, 0.0 );
    ball2->force = Vector3( 0.0, 0.0, 0.0 );
    ball2->torque = Vector3( 0.0, 0.0, 0.0 );
    ball2->sphere( mass2, radius, 12, 5, &ball2_material );
    BoundingSphere* bvball2 = new BoundingSphere( radius );
    ball2->bv( bvball2 );
    scene.insert( ball2 );

    ground_material = Material( Color( 0.6, 0.6, 0.3, 1.0 ), Color( 0.4, 0.4, 0.2, 1.0 ), Color( 0.2, 0.2, 0.1, 1.0 ), Color( 0.0, 0.0, 0.0, 1.0 ), 10.0 );

    RigidBody* ground = new RigidBody( RIGIDBODY_STATIC );
    ground->pose = Pose( POSE_BY_QUATERNION, Vector3( 0.0, 0.0, 0.0 ), Vector3( 0.0, 0.0, 0.0 ) );
    ground->pose.position = Vector3( 0.0, -250.0, 0.0 );
    ground->force = Vector3( 0.0, 0.0, 0.0 );
    ground->torque = Vector3( 0.0, 0.0, 0.0 );
    double width = 1000000.0;
    double height = 100.0;
    double depth = 1000000.0;
    ground->cube( 1000000.0, width, height, depth, &ground_material );
    AABB* bvground = new AABB( Vector3( 0.0, 0.0, 0.0 ), Vector3( width/2.0, height/2.0, depth/2.0 ) );
    ground->bv( bvground );
    ground->pose.transformByQuaternion();
    scene.insert( ground );
}

void initBallisticCollisionMultiball4( void ) {
    scene.camera.position = Vector3( 0.0, 0.0, 500.0 );
    scene.camera.viewpoint = Vector3( 0.0, 0.0, 499.0 );
    scene.camera.far = 10000;
    scene.camera.up = Vector3( 0.0, 1.0, 0.0 );
    scene.camera.project();

    const double radius = 50.0;
    const double mass1 = 1.0;
    const double mass2 = 1.0;
    const double mass3 = 1.0;

    ball1_material = Material( Color( 0.8, 0.0, 0.0, 1.0 ), Color( 0.6, 0.0, 0.0, 1.0 ), Color( 0.4, 0.0, 0.0, 1.0 ), Color( 0.0, 0.0, 0.0, 1.0 ), 10.0 );

    RigidBody* ball1 = new RigidBody();
    ball1->pose = Pose( POSE_BY_QUATERNION, Vector3( 0.0, 0.0, 0.0 ), Vector3( 0.0, 0.0, 0.0 ) );
    //ball1->pose.position = Vector3( 0.0, 400.0, 0.0 );
    ball1->pose.position = Vector3( 25.0, 500.0, 0.0 );
    ball1->force = Vector3( 0.0, 0.0, 0.0 );
    ball1->torque = Vector3( 0.0, 0.0, 0.0 );
    ball1->sphere( mass1, radius, 12, 5, &ball1_material );
    //ball1->sphere( mass1, radius, 12, 5 );
    BoundingSphere* bvball1 = new BoundingSphere( radius );
    ball1->bv( bvball1 );
    scene.insert( ball1 );

    ball2_material = Material( Color( 0.0, 0.8, 0.8, 1.0 ), Color( 0.0, 0.8, 0.8, 1.0 ), Color( 0.0, 0.4, 4.0, 1.0 ), Color( 0.0, 0.0, 0.0, 1.0 ), 10.0 );

    RigidBody* ball2 = new RigidBody();
    ball2->pose = Pose( POSE_BY_QUATERNION, Vector3( 0.0, 0.0, 0.0 ), Vector3( 0.0, 0.0, 0.0 ) );
    ball2->pose.position = Vector3( -25.0, 200.0, 0.0 );
    ball2->force = Vector3( 0.0, 0.0, 0.0 );
    ball2->torque = Vector3( 0.0, 0.0, 0.0 );
    ball2->sphere( mass2, radius, 12, 5, &ball2_material );
    //ball2->sphere( mass2, radius, 12, 5 );
    BoundingSphere* bvball2 = new BoundingSphere( radius );
    ball2->bv( bvball2 );
    scene.insert( ball2 );

    ball3_material = Material( Color( 0.8, 0.8, 0.0, 1.0 ), Color( 0.8, 0.8, 0.0, 1.0 ), Color( 0.4, 4.0, 0.0, 1.0 ), Color( 0.0, 0.0, 0.0, 1.0 ), 10.0 );

    RigidBody* ball3 = new RigidBody();
    ball3->pose = Pose( POSE_BY_QUATERNION, Vector3( 0.0, 0.0, 0.0 ), Vector3( 0.0, 0.0, 0.0 ) );
    ball3->pose.position = Vector3( -200.0, 500.0, 0.0 );
    ball3->force = Vector3( 0.0, 0.0, 0.0 );
    ball3->torque = Vector3( 0.0, 0.0, 0.0 );
    ball3->sphere( mass3, radius, 12, 5, &ball3_material );
    //ball3->sphere( mass3, radius, 12, 5 );
    BoundingSphere* bvball3 = new BoundingSphere( radius );
    ball3->bv( bvball3 );
    scene.insert( ball3 );

    ground_material = Material( Color( 0.6, 0.6, 0.3, 1.0 ), Color( 0.4, 0.4, 0.2, 1.0 ), Color( 0.2, 0.2, 0.1, 1.0 ), Color( 0.0, 0.0, 0.0, 1.0 ), 10.0 );

    RigidBody* ground = new RigidBody( RIGIDBODY_STATIC );
    ground->pose = Pose( POSE_BY_QUATERNION, Vector3( 0.0, 0.0, 0.0 ), Vector3( 0.0, 0.0, 0.0 ) );
    ground->pose.position = Vector3( 0.0, -250.0, 0.0 );
    ground->force = Vector3( 0.0, 0.0, 0.0 );
    ground->torque = Vector3( 0.0, 0.0, 0.0 );
    double width = 1000000.0;
    double height = 100.0;
    double depth = 1000000.0;
    ground->cube( 1000000.0, width, height, depth, &ground_material );
    AABB* bvground = new AABB( Vector3( 0.0, 0.0, 0.0 ), Vector3( width/2.0, height/2.0, depth/2.0 ) );
    ground->bv( bvground );
    ground->pose.transformByQuaternion();
    scene.insert( ground );
}

void initBallPit( void ) {
    //scene.camera.position = Vector3( 0.0, 0.0, 2000.0 );
    //scene.camera.viewpoint = Vector3( 0.0, 0.0, 1999.0 );
    scene.camera.position = Vector3( 0.0, 0.0, 500.0 );
    scene.camera.viewpoint = Vector3( 0.0, 0.0, 0.0 );
    scene.camera.far = 10000;
    scene.camera.up = Vector3( 0.0, 1.0, 0.0 );
    scene.camera.project();

    const double radius = 40.0;
    const double mass1 = 1.0;
//    const double mass2 = 1.0;
//    const double mass3 = 1.0;

    double width = 1000.0;
    double height = 100.0;
    double depth = 1000.0;

    double edge_width = 10.0;
    double edge_height = 600.0;

    RigidBody* ball;
    Material* material;

    double x, y, z;

    // falling balls
    for( unsigned int i = 0; i < 6; i++ ) {
        x = (double) rand() / (double) RAND_MAX * (double) width - width/2.0;
        y = (double) rand() / (double) RAND_MAX * (double) 1000;
        z = (double) rand() / (double) RAND_MAX * (double) depth/8.0 - depth/16.0;

        ball = new RigidBody();
        ball->pose = Pose( POSE_BY_QUATERNION, Vector3( 0.0, 0.0, 0.0 ), Vector3( 0.0, 0.0, 0.0 ) );
        ball->pose.position = Vector3( x, y, 0.0 );
        ball->force = Vector3( 0.0, 0.0, 0.0 );
        ball->torque = Vector3( 0.0, 0.0, 0.0 );
        ball->coefficient_of_restitution = 0.9;
        material = new Material( true );
        ball->sphere( mass1, radius, 12, 5, material );
        ball->bv( new BoundingSphere( radius ) );
        scene.appendMaterial( material );
        scene.insert( ball );
    }

    x = (double) rand() / (double) RAND_MAX * (double) width - width/2.0;
    y = (double) rand() / (double) RAND_MAX * (double) 500;
    z = (double) rand() / (double) RAND_MAX * (double) depth - depth/2.0;

    // preturber ball
    ball = new RigidBody();
    ball->pose = Pose( POSE_BY_QUATERNION, Vector3( 0.0, 0.0, 0.0 ), Vector3( 0.0, 0.0, 0.0 ) );
    ball->pose.position = Vector3( 0.0, 0.0, 0.0 );
    //ball->force = Vector3( -2000.0, 0.0, 0.0 );
    ball->force = Vector3( -1500.0, 0.0, -1000.0 );
    ball->torque = Vector3( 0.0, 0.0, 0.0 );
    ball->coefficient_of_restitution = 0.9;
    material = new Material( true );
    ball->sphere( mass1, radius, 12, 5, material );
    ball->bv( new BoundingSphere( radius ) );
    scene.appendMaterial( material );
    scene.insert( ball );

    /*
    ball1_material = Material( Color( 0.8, 0.0, 0.0, 1.0 ), Color( 0.6, 0.0, 0.0, 1.0 ), Color( 0.4, 0.0, 0.0, 1.0 ), Color( 0.0, 0.0, 0.0, 1.0 ), 10.0 );

    RigidBody* ball1 = new RigidBody();
    ball1->pose = Pose( POSE_BY_QUATERNION, Vector3( 0.0, 0.0, 0.0 ), Vector3( 0.0, 0.0, 0.0 ) );
    //ball1->pose.position = Vector3( 0.0, 400.0, 0.0 );
    ball1->pose.position = Vector3( 25.0, 500.0, 0.0 );
    ball1->force = Vector3( 0.0, 0.0, 0.0 );
    ball1->torque = Vector3( 0.0, 0.0, 0.0 );
    ball1->sphere( mass1, radius, 12, 5, &ball1_material );
    //ball1->sphere( mass1, radius, 12, 5 );
    BoundingSphere* bvball1 = new BoundingSphere( radius );
    ball1->bv( bvball1 );
    scene.insert( ball1 );

    ball2_material = Material( Color( 0.0, 0.8, 0.8, 1.0 ), Color( 0.0, 0.8, 0.8, 1.0 ), Color( 0.0, 0.4, 4.0, 1.0 ), Color( 0.0, 0.0, 0.0, 1.0 ), 10.0 );

    RigidBody* ball2 = new RigidBody();
    ball2->pose = Pose( POSE_BY_QUATERNION, Vector3( 0.0, 0.0, 0.0 ), Vector3( 0.0, 0.0, 0.0 ) );
    ball2->pose.position = Vector3( -25.0, 200.0, 0.0 );
    ball2->force = Vector3( 0.0, 0.0, 0.0 );
    ball2->torque = Vector3( 0.0, 0.0, 0.0 );
    ball2->sphere( mass2, radius, 12, 5, &ball2_material );
    //ball2->sphere( mass2, radius, 12, 5 );
    BoundingSphere* bvball2 = new BoundingSphere( radius );
    ball2->bv( bvball2 );
    scene.insert( ball2 );

    ball3_material = Material( Color( 0.8, 0.8, 0.0, 1.0 ), Color( 0.8, 0.8, 0.0, 1.0 ), Color( 0.4, 4.0, 0.0, 1.0 ), Color( 0.0, 0.0, 0.0, 1.0 ), 10.0 );

    RigidBody* ball3 = new RigidBody();
    ball3->pose = Pose( POSE_BY_QUATERNION, Vector3( 0.0, 0.0, 0.0 ), Vector3( 0.0, 0.0, 0.0 ) );
    ball3->pose.position = Vector3( -200.0, 500.0, 0.0 );
    ball3->force = Vector3( 0.0, 0.0, 0.0 );
    ball3->torque = Vector3( 0.0, 0.0, 0.0 );
    ball3->sphere( mass3, radius, 12, 5, &ball3_material );
    //ball3->sphere( mass3, radius, 12, 5 );
    BoundingSphere* bvball3 = new BoundingSphere( radius );
    ball3->bv( bvball3 );
    scene.insert( ball3 );
*/
    ground_material = Material( Color( 0.6, 0.6, 0.3, 1.0 ), Color( 0.4, 0.4, 0.2, 1.0 ), Color( 0.2, 0.2, 0.1, 1.0 ), Color( 0.0, 0.0, 0.0, 1.0 ), 10.0 );

    RigidBody* ground = new RigidBody( RIGIDBODY_STATIC );
    ground->pose = Pose( POSE_BY_QUATERNION, Vector3( 0.0, 0.0, 0.0 ), Vector3( 0.0, 0.0, 0.0 ) );
    ground->pose.position = Vector3( 0.0, -250.0, 0.0 );
    ground->force = Vector3( 0.0, 0.0, 0.0 );
    ground->torque = Vector3( 0.0, 0.0, 0.0 );
    ground->cube( 1000000.0, width, height, depth, &ground_material );
    AABB* bvground = new AABB( Vector3( 0.0, 0.0, 0.0 ), Vector3( width/2.0, height/2.0, depth/2.0 ) );
    ground->bv( bvground );
    ground->pose.transformByQuaternion();
    scene.insert( ground );

    RigidBody* edge;
    edge = new RigidBody( RIGIDBODY_STATIC );
    edge->pose = Pose( POSE_BY_QUATERNION, Vector3( 0.0, 0.0, 0.0 ), Vector3( 0.0, 0.0, 0.0 ) );
    edge->pose.position = Vector3( width/2.0 + edge_width/2.0, edge_height/2.0 - 250, 0.0 );
    edge->force = Vector3( 0.0, 0.0, 0.0 );
    edge->torque = Vector3( 0.0, 0.0, 0.0 );
    edge->cube( 1000000.0, edge_width, edge_height, depth );
    edge->bv( new AABB( edge->pose.position, Vector3( edge_width/2.0, edge_height/2.0, depth/2.0 ) ) );
    edge->pose.transformByQuaternion();
    scene.insert( edge );

    edge = new RigidBody( RIGIDBODY_STATIC );
    edge->pose = Pose( POSE_BY_QUATERNION, Vector3( 0.0, 0.0, 0.0 ), Vector3( 0.0, 0.0, 0.0 ) );
    edge->pose.position = Vector3( -(width/2.0 + edge_width/2.0), edge_height/2.0 - 250, 0.0 );
    edge->force = Vector3( 0.0, 0.0, 0.0 );
    edge->torque = Vector3( 0.0, 0.0, 0.0 );
    edge->cube( 1000000.0, edge_width, edge_height, depth );
    edge->bv( new AABB( edge->pose.position, Vector3( edge_width/2.0, edge_height/2.0, depth/2.0 ) ) );
    edge->pose.transformByQuaternion();
    scene.insert( edge );

    edge = new RigidBody( RIGIDBODY_STATIC );
    edge->pose = Pose( POSE_BY_QUATERNION, Vector3( 0.0, 0.0, 0.0 ), Vector3( 0.0, 0.0, 0.0 ) );
    edge->pose.position = Vector3( 0.0, edge_height/2.0 - 250, -(width/2.0 + edge_width/2.0) );
    edge->force = Vector3( 0.0, 0.0, 0.0 );
    edge->torque = Vector3( 0.0, 0.0, 0.0 );
    edge->cube( 1000000.0, depth, edge_height, edge_width );
    edge->bv( new AABB( edge->pose.position, Vector3( depth/2.0, edge_height/2.0, edge_width/2.0 ) ) );
    edge->pose.transformByQuaternion();
    scene.insert( edge );

    edge = new RigidBody( RIGIDBODY_STATIC );
    edge->pose = Pose( POSE_BY_QUATERNION, Vector3( 0.0, 0.0, 0.0 ), Vector3( 0.0, 0.0, 0.0 ) );
    edge->pose.position = Vector3( 0.0, edge_height/2.0 - 250, width/2.0 + edge_width/2.0 );
    edge->force = Vector3( 0.0, 0.0, 0.0 );
    edge->torque = Vector3( 0.0, 0.0, 0.0 );
    edge->cube( 1000000.0, depth, edge_height, edge_width );
    edge->bv( new AABB( edge->pose.position, Vector3( depth/2.0, edge_height/2.0, edge_width/2.0 ) ) );
    edge->pose.transformByQuaternion();
    scene.insert( edge );
/*
    RigidBody* block;

    block = new RigidBody( RIGIDBODY_STATIC );
    block->pose = Pose( POSE_BY_QUATERNION, Vector3( 0.0, 0.0, 0.0 ), Vector3( 0.0, 0.0, 0.0 ) );
    block->pose.position = Vector3( 0.0, -200, 0.0 );
    block->force = Vector3( 0.0, 0.0, 0.0 );
    block->torque = Vector3( 0.0, 0.0, 0.0 );
    block->cube( 1000000.0, 400.0, 50.0, 400.0 );
    block->bv( new AABB( edge->pose.position, Vector3( 400.0/2.0, 50.0/2.0, 400.0/2.0 ) ) );
    block->pose.transformByQuaternion();
    scene.insert( block );

    block = new RigidBody( RIGIDBODY_STATIC );
    block->pose = Pose( POSE_BY_QUATERNION, Vector3( 0.0, 0.0, 0.0 ), Vector3( 0.0, 0.0, 0.0 ) );
    block->pose.position = Vector3( 0.0, -150, 0.0 );
    block->force = Vector3( 0.0, 0.0, 0.0 );
    block->torque = Vector3( 0.0, 0.0, 0.0 );
    block->cube( 1000000.0, 100.0, 50.0, 100.0 );
    block->bv( new AABB( edge->pose.position, Vector3( 100.0/2.0, 50.0/2.0, 100.0/2.0 ) ) );
    block->pose.transformByQuaternion();
    scene.insert( block );
*/
}

void initScene( void ) {
    title = "Lab3";
    filetitle = "lab3";

    //initFallingBodies();
    //initBallisticRotation();
    //initBallisticCollisionAABB();
    //initBallisticCollisionAABBandSphere();
    //initBallisticCollisionAABBandSphere2();
    //initBallisticCollisionAABBandSphere2WithTorque();
    //initBallisticCollisionRestingContact();
    //initBallisticCollisionMultiball();
    //initBallisticCollisionMultiball2();
    //initBallisticCollisionMultiball3();
    //initBallisticCollisionMultiball4();
    initBallPit();

    // preprocess the scene
    for( unsigned int i = 0; i < scene.geometryCount(); i++ ) {
        Geometry* geometry = scene.geometry( i );
        if( geometry->geometry_type() == GEOMETRY_TYPE_BODY ) {
            Body* body = static_cast<Body*>( geometry );
            if( body->body_type() == BODY_TYPE_ARTICULATED ) {
//                ArticulatedBody* abody = static_cast<ArticulatedBody*>( body );
            } else if( body->body_type() == BODY_TYPE_RIGID ) {
                RigidBody* rbody = static_cast<RigidBody*>( body );
                if( rbody->rigidbody_type == RIGIDBODY_DYNAMIC )
                    Physics::ode( sim_time, sim_time + sim_time_step, rbody );
            } else if( body->body_type() == BODY_TYPE_DEFORMABLE) {
                //DeformableBody* dbody = static_cast<DeformableBody*>( body );
            }
        } else if( geometry->geometry_type() == GEOMETRY_TYPE_MESH ) {
//            Mesh* mesh = static_cast<Mesh*>( geometry );
        } else if( geometry->geometry_type() == GEOMETRY_TYPE_PATCH ) {
//            Patch* patch = static_cast<Patch*>( geometry );
        }
    }
}

//------------------------------------------------------------------------------
// Drawing Operations
//------------------------------------------------------------------------------
/// Draws a mesh
void draw( Mesh* mesh ) {
    glBegin( GL_TRIANGLES );
    for( unsigned int poly_id = 0; poly_id < mesh->polygonCount( ); poly_id++ ) {

        // Select the current polygon
        Polygon* poly = mesh->polygon( poly_id );

        GLfloat *material_Ka, *material_Kd, *material_Ks, *material_Ke, material_Se;


        if( poly->material != NULL ) {
            material_Ka = (GLfloat*)poly->material->ambient.arrayOpenGL();
            material_Kd = (GLfloat*)poly->material->diffuse.arrayOpenGL();
            material_Ks = (GLfloat*)poly->material->specular.arrayOpenGL();
            material_Ke = (GLfloat*)poly->material->emissive.arrayOpenGL();
            material_Se = (GLfloat)poly->material->shininess;

            glMaterialfv(GL_FRONT, GL_AMBIENT, material_Ka);
            glMaterialfv(GL_FRONT, GL_DIFFUSE, material_Kd);
            glMaterialfv(GL_FRONT, GL_SPECULAR, material_Ks);
            glMaterialfv(GL_FRONT, GL_EMISSION, material_Ke);
            glMaterialf(GL_FRONT, GL_SHININESS, material_Se);
        }
        Vertex *v0, *v1, *v2;

        unsigned int verts = poly->numVertices( );
        if( verts < 3 ) continue;   // sanity check -> malformed poly & bad juju

        // the model is not tessellated, so have to tessellate for OpenGL
        // If poly is non-convex this won't work, but assume convex.

        // Select the first vertex as the root of all triangles in the poly
        v0 = mesh->vertex( poly->getVertex( 0 ) );

        glNormal3d( poly->normal.x(), poly->normal.y(), poly->normal.z() );
        glColor3d( poly->material->ambient.r(), poly->material->ambient.g(), poly->material->ambient.b() );

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
        //glNormal3d( poly->normal.x(), poly->normal.y(), poly->normal.z() );
    }
    glEnd();
}

//------------------------------------------------------------------------------
// Modified SimpleGLUT
//------------------------------------------------------------------------------
/// Initialization of OpenGL
void init(void)
{
    glClearColor( 0.25, 0.25, 0.25, 0.0 );
    glClearDepth( 1.0 );
    //glShadeModel( GL_SMOOTH );
    glShadeModel( GL_FLAT );
    glEnable( GL_LINE_SMOOTH );
    glEnable( GL_NORMALIZE );

    glEnable( GL_DEPTH_TEST );
    glDepthFunc( GL_LEQUAL );
    glHint( GL_PERSPECTIVE_CORRECTION_HINT, GL_NICEST );

    glEnable( GL_LIGHTING );

    glLightModeli( GL_LIGHT_MODEL_LOCAL_VIEWER, GL_TRUE );

    glEnable( GL_COLOR_MATERIAL );
    glColorMaterial( GL_FRONT, GL_AMBIENT_AND_DIFFUSE );

}

//------------------------------------------------------------------------------
/// Registered OpenGL Display Callback Function
void display(void)
{
    // clear the color and depth buffers
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

    // recompute the camera projection in case it was moved
    scene.camera.project();
    glMatrixMode( GL_PROJECTION );
    glLoadIdentity( );
    glLoadMatrixd( scene.camera.projection.arrayOpenGL() );

    // now process the scene
    glMatrixMode( GL_MODELVIEW );
    glLoadIdentity( );

    GLfloat LightAmbient[] =  { 0.4f, 0.4f, 0.4f, 1.0f};
    GLfloat LightDiffuse[] =  { 0.3f, 0.3f, 0.3f, 1.0f};
    GLfloat LightSpecular[] = { 0.0f, 0.0f, 0.0f, 1.0f};
    GLfloat LightPosition[] = { 0.0f, 0.0f, 0.0f, 1.0f};

    glLightfv(GL_LIGHT0, GL_AMBIENT, LightAmbient);
    glLightfv(GL_LIGHT0, GL_DIFFUSE, LightDiffuse);
    glLightfv(GL_LIGHT0, GL_SPECULAR, LightSpecular);
    glLightfv(GL_LIGHT0, GL_POSITION, LightPosition);
    glEnable( GL_LIGHT0 );

    for( unsigned int i = 0; i < scene.geometryCount(); i++ ) {
        Geometry* geometry = scene.geometry( i );
        if( geometry->geometry_type() == GEOMETRY_TYPE_BODY ) {
            Body* body = static_cast<Body*>( geometry );
            if( body->body_type() == BODY_TYPE_RIGID ) {
                RigidBody* rbody = static_cast<RigidBody*>( body );

                glPushMatrix();

                glLoadMatrixd( rbody->pose.transform.arrayOpenGL() );

                for( unsigned int i = 0; i < rbody->geometryCount(); i++ ) {
                    Geometry* geometry = rbody->geometry( i );
                    if( geometry->geometry_type() == GEOMETRY_TYPE_MESH ) {
                        Mesh* mesh = static_cast<Mesh*>( geometry );
                        draw( mesh );
                    }
                }

                glPopMatrix();
            }
        }
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

    glLoadMatrixd( scene.camera.projection.arrayOpenGL() );

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
        scene.camera.track_left( PI/64 );
        break;
    case 100:    // d
        scene.camera.track_right( PI/64 );
        break;
    case 115:    // s
        scene.camera.dolly_out( 5.0 );
        break;
    case 119:    // w
        scene.camera.dolly_in( 5.0 );
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

void get_collisions( Scene* s, std::vector<ContactEvent>& result ) {
    for( unsigned int i = 0; i < s->geometryCount(); i++ ) {
        Geometry* geometry = s->geometry( i );
        if( geometry->geometry_type() == GEOMETRY_TYPE_BODY ) {
            Body* body = static_cast<Body*>( geometry );
            if( body->body_type() == BODY_TYPE_RIGID ) {
                RigidBody* rbody = static_cast<RigidBody*>( body );

                for( unsigned int i = 0; i < s->geometryCount(); i++ ) {
                    Geometry* geometry = s->geometry( i );
                    if( geometry->geometry_type() == GEOMETRY_TYPE_BODY ) {
                        Body* body = static_cast<Body*>( geometry );
                        if( body->body_type() == BODY_TYPE_RIGID ) {
                            RigidBody* rbody2 = static_cast<RigidBody*>( body );
                            // eliminate any self test
                            if( rbody == rbody2 ) continue;

                            // search to see if this collision test has already been recorded
                            // Note, the search could be mitigated by using a map/key random access approach
                            bool found = false;
                            for( std::vector<ContactEvent>::iterator it = result.begin(); it != result.end(); it++ ) {
                                if( (it->body1 == rbody && it->body2 == rbody2) ||
                                    (it->body2 == rbody && it->body1 == rbody2) ) {
                                    found = true;
                                    break;
                                }
                            }
                            // if collision already recorded, eliminate it
                            if( found ) continue;

                            if( rbody->bv()->bounding_volume_type() == BV_TYPE_SPHERE && rbody2->bv()->bounding_volume_type() == BV_TYPE_SPHERE ) {
                                BoundingSphere* b1 = static_cast<BoundingSphere*>( rbody->bv() );
                                BoundingSphere* b2 = static_cast<BoundingSphere*>( rbody2->bv() );

                                b1->center = rbody->pose.position;
                                b2->center = rbody2->pose.position;

                                if( BoundingVolume::intersects( *b1, *b2 ) ) {
                                    Vector3 contact_pt;
                                    BoundingVolume::ClosestPtPointSphere( b2->center, *b1, contact_pt );
                                    ContactEvent event = ContactEvent( rbody, rbody2 );
                                    event.point = contact_pt;
                                    event.contact_event_type = CE_VERTEX_VERTEX;
                                    event.normal = b1->normal( b2->center );
                                    result.push_back( event );
                                }
                            } else if( rbody->bv()->bounding_volume_type() == BV_TYPE_AABB && rbody2->bv()->bounding_volume_type() == BV_TYPE_AABB ) {
                                if( rbody->rigidbody_type == RIGIDBODY_STATIC && rbody2->rigidbody_type == RIGIDBODY_STATIC ) continue;

                                AABB* b1 = static_cast<AABB*>( rbody->bv() );
                                AABB* b2 = static_cast<AABB*>( rbody2->bv() );

                                b1->center = rbody->pose.position;
                                b2->center = rbody2->pose.position;

                                if( BoundingVolume::intersects( *b1, *b2 ) ) {
                                    ContactEvent event = ContactEvent( rbody, rbody2 );
                                    result.push_back( event );
                                }
                            } else if( rbody->bv()->bounding_volume_type() == BV_TYPE_SPHERE && rbody2->bv()->bounding_volume_type() == BV_TYPE_AABB ) {
                                BoundingSphere* b1 = static_cast<BoundingSphere*>( rbody->bv() );
                                AABB* b2 = static_cast<AABB*>( rbody2->bv() );

                                b1->center = rbody->pose.position;
                                b2->center = rbody2->pose.position;

                                if( BoundingVolume::intersects( *b1, *b2 ) ) {
                                    Vector3 contact_pt;
                                    BoundingVolume::ClosestPtPointAABB( b1->center, *b2, contact_pt );
                                    ContactEvent event = ContactEvent( rbody, rbody2 );
                                    event.point = contact_pt;
                                    event.contact_event_type = CE_VERTEX_FACE;  // brute force assumption for now
                                    event.normal = b2->normal( contact_pt );
                                    result.push_back( event );
                                }
                            } else if( rbody->bv()->bounding_volume_type() == BV_TYPE_AABB && rbody2->bv()->bounding_volume_type() == BV_TYPE_SPHERE ) {
                                AABB* b1 = static_cast<AABB*>( rbody->bv() );
                                BoundingSphere* b2 = static_cast<BoundingSphere*>( rbody2->bv() );

                                b1->center = rbody->pose.position;
                                b2->center = rbody2->pose.position;

                                if( BoundingVolume::intersects( *b1, *b2 ) ) {
                                    Vector3 contact_pt;
                                    BoundingVolume::ClosestPtPointAABB( b2->center, *b1, contact_pt );
                                    ContactEvent event = ContactEvent( rbody2, rbody );
                                    event.point = contact_pt;
                                    event.contact_event_type = CE_VERTEX_FACE;  // brute force assumption for now
                                    event.normal = b1->normal( contact_pt );
                                    result.push_back( event );
                                }
                            }
                        }
                    }
                }
            }
        }
    }
}

//------------------------------------------------------------------------------
/// Registered OpenGL Idle Callback Function
/// Computes and Updates all component transformations
void idle( void )
{
    if( !sim_pause ) {
        frame_id++;

        Scene cache;

        for( unsigned int i = 0; i < scene.geometryCount(); i++ ) {
            Geometry* geometry = scene.geometry( i );
            if( geometry->geometry_type() == GEOMETRY_TYPE_BODY ) {
                Body* body = static_cast<Body*>( geometry );
                if( body->body_type() == BODY_TYPE_RIGID ) {
                    RigidBody* rbody = static_cast<RigidBody*>( body );
                    RigidBody* rbody_copy = new RigidBody();
                    rbody_copy->shallow_copy( rbody );
                    cache.insert( rbody_copy );
                }
            }
        }

        // update forces
        for( unsigned int i = 0; i < cache.geometryCount(); i++ ) {
            Geometry* geometry = cache.geometry( i );
            if( geometry->geometry_type() == GEOMETRY_TYPE_BODY ) {
                Body* body = static_cast<Body*>( geometry );
                if( body->body_type() == BODY_TYPE_RIGID ) {
                    RigidBody* rbody = static_cast<RigidBody*>( body );
                    if( rbody->rigidbody_type == RIGIDBODY_DYNAMIC )
                        Physics::compute_force_and_torque( sim_time, rbody );
                }
            }
        }

        // integrate & collision detection
        for( unsigned int i = 0; i < cache.geometryCount(); i++ ) {
            Geometry* geometry = cache.geometry( i );
            if( geometry->geometry_type() == GEOMETRY_TYPE_BODY ) {
                Body* body = static_cast<Body*>( geometry );
                if( body->body_type() == BODY_TYPE_RIGID ) {
                    RigidBody* rbody = static_cast<RigidBody*>( body );
                    if( rbody->rigidbody_type == RIGIDBODY_STATIC ) continue;

                    double t0 = sim_time;
                    double t1;
                    for( double step = 0.0; step < sim_time_step; step += sim_time_step/10.0 ) {
                        t1 = t0 + step;
                        Physics::ode( t0, t1, rbody );
                        std::vector<ContactEvent> collisions;
                        get_collisions( &cache, collisions );

                        if( collisions.size() > 0 ) {
                            // replay these bodies
                            for( std::vector<ContactEvent>::iterator it = collisions.begin(); it != collisions.end(); it++ ) {
                                RigidBody *rbody1 = it->body1;
                                RigidBody *rbody2 = it->body2;

                                // restore the state from before intersection
                                rbody1->copy_origin();
                                rbody2->copy_origin();

                                // collision response
                                ContactEvent ce = *it;
                                // Should be able to use, but not handling sphere/sphere contact as defined therefore remarked
                                //if( Physics::colliding( &ce ) ) {
                                    double coeff_num, coeff_denom, ratio_of_restitution;
                                    // a bit of a hack, but set up this way so energy is not added
                                    if( rbody1->coefficient_of_restitution > rbody2->coefficient_of_restitution ) {
                                        coeff_num = rbody2->coefficient_of_restitution;
                                        coeff_denom = rbody1->coefficient_of_restitution;
                                    } else {
                                        coeff_num = rbody1->coefficient_of_restitution;
                                        coeff_denom = rbody2->coefficient_of_restitution;
                                    }
                                    ratio_of_restitution = coeff_num / coeff_denom;
                                    Physics::collision( &ce, ratio_of_restitution );
                                    // replay
                                    Physics::ode( t0, t1, rbody1 );
                                    Physics::ode( t0, t1, rbody2 );
                                //}
                            }
                        }

                        t0 += step;
                    }
                    rbody->update_origin();
                }
            }
        }

        // update the scene state from the calculated cache
        for( unsigned int i = 0; i < cache.geometryCount(); i++ ) {
            Geometry* geometry = cache.geometry( i );
            if( geometry->geometry_type() == GEOMETRY_TYPE_BODY ) {
                Body* body = static_cast<Body*>( geometry );
                if( body->body_type() == BODY_TYPE_RIGID ) {
                    RigidBody* rbody = static_cast<RigidBody*>( body );
                    rbody->update_origin();
                }
            }
        }

        sim_time += sim_time_step;
    }
    glutPostRedisplay( );
}

//------------------------------------------------------------------------------
/// Program entry point
/// Initialize then start rendering
int main( int argc, char** argv )
{
    initScene( );

    // initialize OpenGL
    glutInit( &argc, argv );
    glutInitDisplayMode( GLUT_DOUBLE | GLUT_RGB |GLUT_DEPTH );
    glutInitWindowSize( Width, Height );
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
