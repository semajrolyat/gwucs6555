/*-----------------------THE GEORGE WASHINGTON UNIVERSITY-----------------------
James Taylor                                                         jrt@gwu.edu

Boids Unit Test

------------------------------------------------------------------------------*/

#include <iostream>

#include <CubicSpline.h>
#include <Constants.h>

#include <GL/glut.h>
#include <stdio.h>

#include <string>

#include <Vector3.h>
#include <Matrix4.h>

#include <Camera.h>
#include <Trajectory.h>

#include <EulerAngle.h>
#include <Keyframe.h>

#include <ArticulatedBody.h>
#include <stack>

#include <Mesh.h>
#include <MeshLoader.h>

#include <stdexcept>

#include <Color.h>
#include <Material.h>

#include <Utilities.h>
#include <GeometryMaker.h>

#include <Boid.h>
#include <Flock.h>
#include <Motivator.h>

//------------------------------------------------------------------------------
// Hardcoded Defines
//------------------------------------------------------------------------------

#define GENERATE_MOVIE  0               // No, do not generate a movie
//#define GENERATE_MOVIE  1               // Yes, do generate a movie

//------------------------------------------------------------------------------
// Macros
//------------------------------------------------------------------------------

#define DEG_PER_RAD  180.0/PI
#define RAD_PER_DEG  PI/180.0

//------------------------------------------------------------------------------
// Toggle Options
//------------------------------------------------------------------------------

bool opt_draw_actor = true;                 // toggle by pressing 1
//bool opt_draw_trajectories = true;
bool opt_draw_trajectories = false;         // toggle by pressing 2
bool sim_pause = false;                     // toggle by pressing p

Mesh* mesh_controlpt;

typedef std::vector<Vector3> KnotPointList;

KnotPointList knotpoints;

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

Geometry* arrow;

#define BOID_TYPE_BODY              0;
#define BOID_TYPE_ARTICULATED_BODY  1;

unsigned int boid_type;
Body* boid;

std::vector<Flock*> flocks;
std::vector<Boid*> boids;

Motivator motivator;
Mesh* motivator_mesh;

//------------------------------------------------------------------------------
// Camera Functions
//------------------------------------------------------------------------------
void initCamera( void ) {
    ///*
    camera.position = Vector3( 80.0, 5.0, 80.0 );
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
// Body Functions
//------------------------------------------------------------------------------

void initConicBoid( void ) {
    title = "Conic Boid";
    filetitle = "conic_boid";

    boid_type = BOID_TYPE_BODY;

    boid = new Body();

    boid->trajectory.construct_line_trajectory();
    boid->insert( MeshLoader::cone( 1.0, 3.0, 16 ) );

    //boid->trajectory.construct_random_trajectory( 5 );
}

//------------------------------------------------------------------------------

double angle_in_x_z( const Vector3& start_orientation, const Vector3& end_orientation ) {
    double start_theta = atan2( start_orientation.x(), start_orientation.z() );
    double end_theta = atan2( end_orientation.x(), end_orientation.z() );
    double theta = end_theta - start_theta;
    while (theta <= -PI)
        theta += 2.0 * PI;
    while (theta > PI)
        theta -= 2.0 * PI;
    return theta;
}

//------------------------------------------------------------------------------

void generate_flock( const Vector3& flock_center, const unsigned int& number_of_boids, const double& flock_extens ) {
    Boid* boid;
    Flock* flock = new Flock();
    flocks.push_back( flock );

    // assign to master list of boids
    double radius = 1.0;
    double height = 3.0;
    for( unsigned int i = 0; i < number_of_boids; i++ ) {
        boid = new Boid();
        boid->body.insert( MeshLoader::cone( radius, height, 16 ) );
        double x = flock_center.x() + ( (double) rand() / (double) RAND_MAX ) * flock_extens - flock_extens / 2.0;
        double y = 0.0;
        double z = flock_center.z() + ( (double) rand() / (double) RAND_MAX ) * flock_extens - flock_extens / 2.0;
        boid->body.pose.position = Vector3( x, y, z );

        boid->body.mass = 1.0;
        boid->goal = BOID_GOAL_LOCAL;
        boid->role = BOID_ROLE_FOLLOWER;
        boid->motivator_detector.position = Vector3( 0.0, 0.0, height );
        boid->base_orientation = Vector3( 0.0, PI/2.0, 0.0 );
        //boid->body.pose.orientation = Vector3( 0.0, PI/2.0, 0.0 );

        // back of the envelope reassessment if no collision anticipated
        boid->frames_till_reassess_trajectory = 5;
        // rate of yaw in terms of keyframes
        boid->yaw_rate_per_frame = RAD_PER_DEG;
        // forward velocity in terms of keyframes
        boid->max_forward_velocity_per_frame = 0.001;


        // add it to the master list of boids
        boids.push_back( boid );

        flock->add( boid );
        if( i == 0 ) {
            flock->leader = boid;
            boid->goal = BOID_GOAL_GLOBAL;
            boid->role = BOID_ROLE_LEADER;

            Vector3 base_orientation_vector = boid->motivator_detector.position;
            base_orientation_vector.normalize();
            Vector3 base_orientation_angle = boid->base_orientation;

            Vector3 orientation = boid->body.pose.orientation;

            Vector3 motivator_direction = motivator.position - boid->body.pose.position;
            motivator_direction.normalize();

            // calculate the angle between the direction to the motivator and the orientation of the boid
            double theta = angle_in_x_z( base_orientation_vector, motivator_direction );

            boid->body.pose.orientation = Vector3( 0.0, theta, 0.0 );

            //
            boid->body.trajectory.insert_keyframe( 1, boid->body.pose );
            Pose pose = Pose( motivator.position, boid->body.pose.orientation );
            boid->body.trajectory.insert_keyframe( 200, pose );

            boid->body.trajectory.construct_trajectory();

            boid->body.trajectory.prev_keyframe_id = 0;
            boid->body.trajectory.next_keyframe_id = 1;
            boid->body.trajectory.spline_basis = CubicSpline::basisUniformNonrationalBSpline();
            boid->body.trajectory.spline_arclength_map = CubicSpline::map_spline( CUBIC_SPLINE_B, boid->body.trajectory.controlpoints );
            boid->body.trajectory.cyclic = true;
        } else {
            Vector3 base_orientation_vector = boid->motivator_detector.position;
            base_orientation_vector.normalize();
            Vector3 boid_forward_movement_vector = base_orientation_vector * boid->max_forward_velocity_per_frame;
            // determine if left turn, right turn or straight.
            // draw a vector from the boid to the motivator to determine direction
            //Vector3 base_orientation_angle = boid->base_orientation;

            //Vector3 orientation = boid->body.pose.orientation;

            Vector3 motivator_direction = motivator.position - boid->body.pose.position;
            motivator_direction.normalize();

            // calculate the angle between the direction to the motivator and the orientation of the boid
            double theta = angle_in_x_z( base_orientation_vector, motivator_direction );
            double rads_per_frame = boid->yaw_rate_per_frame;
            assert( boid->frames_till_reassess_trajectory > 0 );
            if( theta < boid->yaw_rate_per_frame )
                rads_per_frame = theta / (double) boid->frames_till_reassess_trajectory;
            double rads_over_keyframe = rads_per_frame * boid->frames_till_reassess_trajectory;
            if( theta < 0 )
                rads_over_keyframe = -rads_over_keyframe;
            Quaternion q = Quaternion( sin(rads_over_keyframe/2.0), 0.0, cos(rads_over_keyframe/2.0), 0.0 );
            Matrix3 R = q.matrix3();
            Pose p1 = boid->body.pose;
            Vector3 position = boid->body.pose.position + R * boid_forward_movement_vector;
            double orientation_y = boid->body.pose.orientation.y() + rads_over_keyframe;
            Pose p2 = Pose( position, Vector3( 0.0, orientation_y, 0.0 ) );
            boid->body.trajectory.insert_keyframe( frame_id, p1 );
            boid->body.trajectory.insert_keyframe( frame_id + boid->frames_till_reassess_trajectory, p2 );
            boid->body.trajectory.construct_trajectory();

            boid->body.trajectory.prev_keyframe_id = 0;
            boid->body.trajectory.next_keyframe_id = 1;
            boid->body.trajectory.spline_basis = CubicSpline::basisUniformNonrationalBSpline();
            boid->body.trajectory.spline_arclength_map = CubicSpline::map_spline( CUBIC_SPLINE_B, boid->body.trajectory.controlpoints );
            boid->body.trajectory.cyclic = false;
        }
    }
}

//------------------------------------------------------------------------------

void initConicBoids( void ) {
    title = "Conic Flock";
    filetitle = "conic_flock";

    //motivator.position = Vector3( 100.0, 0.0, 0.0 );
    //motivator.position = Vector3( -100.0, 0.0, 0.0 );
    //motivator.position = Vector3( 0.0, 0.0, 100.0 );
    //motivator.position = Vector3( 0.0, 0.0, -100.0 );
    //motivator.position = Vector3( -100.0, 0.0, -100.0 );
    motivator.position = Vector3( 100.0, 0.0, 100.0 );
    motivator.type = MOTIVATOR_TYPE_ATTRACTOR;
    motivator_mesh = MeshLoader::box( 10.0, 10.0, 10.0 );

    //generate_flock( Vector3( 0.0, 0.0, 0.0 ), 50, 100.0 );

//    generate_flock( Vector3( 50.0, 0.0, 0.0 ), 10, 50.0 );

//    generate_flock( Vector3( -50.0, 0.0, 0.0 ), 10, 50.0 );

    generate_flock( Vector3( 50.0, 0.0, 0.0 ), 2, 50.0 );

    generate_flock( Vector3( -50.0, 0.0, 0.0 ), 2, 50.0 );

}


/// Initialize the actor for this program.  Builds the hierarchy of the articulated body
/// and adds any trajectories to the necessary joints
void initBody( void ) {
    title = "Articulated Boid";
    filetitle = "articulated_boid";

    boid_type = BOID_TYPE_ARTICULATED_BODY;

    unsigned int period_in_frames = 16;
    articulatedbody.init_boid( 5.0, period_in_frames );

    articulatedbody.trajectory.construct_random_trajectory( 5 );

    mesh_controlpt = MeshLoader::sphere( 1.0, 10, 5, &articulatedbody.trajectory.material, ML_MATERIAL_ASSIGN );
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

    //sim_pause = true;
}

//------------------------------------------------------------------------------

void draw_body( Body& body ) {

    // By requirements can only have one push and pop of OpenGL's matrix stack
    // Therefore do that here then descend the hierarchy
    glPushMatrix();

    glLoadMatrixd( body.pose.transform.arrayOpenGL() );

    for( unsigned int geometry_id = 0; geometry_id < body.geometryCount( ); geometry_id++ ) {
        Mesh* mesh = static_cast<Mesh*>( body.geometry( geometry_id ) );
        draw_mesh( mesh );
    }

    glPopMatrix();

    //sim_pause = true;
}

//------------------------------------------------------------------------------

void draw_motivator( ) {
    glPushMatrix();
    glLoadIdentity();

    Matrix4 transform = Matrix4::translationMatrix( motivator.position );
    glLoadMatrixd( transform.arrayOpenGL() );
    draw_mesh( motivator_mesh );

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

void arbitrate_boids( void ) {
    /*
    for( std::vector<Boid*>::iterator it = boids.begin(); it != boids.end(); it++ ) {
        Boid* boid = (*it);
        Vector3 base_orientation_vector = boid->motivator_detector.position;
        base_orientation_vector.normalize();
        Vector3 base_orientation_angle = boid->base_orientation;

        Vector3 orientation = boid->body.pose.orientation;

        Vector3 motivator_direction = motivator.position - boid->body.pose.position;
        motivator_direction.normalize();

        double theta = acos( Vector3::dot( motivator_direction, base_orientation_vector ) );

        boid->body.pose.orientation = Vector3( 0.0, theta, 0.0 );


    }
    */

    // move members between flocks
    //  if a flock is moving roughly in the same direction
    //    and the proximity between the two flocks is close


    // update the flock data
    for( std::vector<Flock*>::iterator it = flocks.begin(); it != flocks.end(); it++ ) {
        Flock* flock = (*it);
        unsigned int boids = flock->members.size();

        assert( boids > 0 );

        Vector3 center = Vector3( 0, 0, 0 ), velocity = Vector3( 0, 0, 0 );
        double mass = 0;
        for( std::vector<Boid*>::iterator bit = flock->members.begin(); bit != flock->members.end(); bit++ ) {
            Boid* boid = (*bit);
            center += boid->body.pose.position;
            velocity += boid->body.linear_velocity;
            mass += boid->body.mass;
        }

        flock->center = center / boids;
        flock->velocity = velocity / boids;
        flock->mass = mass / boids;
    }
    // update the leader

    // update the flock members
    // iterate over the flock
    //   update the list of neighbors for each boid
    //   go through each and apply 1/rr force for collision avoidance


    for( std::vector<Flock*>::iterator fit = flocks.begin(); fit != flocks.end(); fit++ ) {
        Flock* flock = (*fit);
        for( std::vector<Boid*>::iterator bit = flock->members.begin(); bit != flock->members.end(); bit++ ) {
            Boid* boid = (*bit);
            if( boid == flock->leader ) continue;
            //boid->get_neighbors( flock->members, 10.0 );

            // reassess trajectory
                // bias for moving forward i.e. a bird can't turn without moving forward
                // turn rate?  turn rate will have a bearing on calculating the trajectory
            // if at its end, build a new one; otherwise continue it
            if( boid->body.trajectory.frame_tic == boid->body.trajectory.keyframe( boid->body.trajectory.keyframes() - 1 ).frame ) {
                // at its end so build a new one
                boid->body.trajectory.clear();

                Vector3 base_orientation_vector = boid->motivator_detector.position;
                base_orientation_vector.normalize();
                Vector3 boid_forward_movement_vector = base_orientation_vector * boid->max_forward_velocity_per_frame;
                // determine if left turn, right turn or straight.
                // draw a vector from the boid to the motivator to determine direction
                //Vector3 base_orientation_angle = boid->base_orientation;

                //Vector3 orientation = boid->body.pose.orientation;

                Vector3 motivator_direction = motivator.position - boid->body.pose.position;
                motivator_direction.normalize();

                // calculate the angle between the direction to the motivator and the orientation of the boid
                double theta = angle_in_x_z( base_orientation_vector, motivator_direction );
                double rads_per_frame = boid->yaw_rate_per_frame;
                assert( boid->frames_till_reassess_trajectory > 0 );
                if( theta < boid->yaw_rate_per_frame )
                    rads_per_frame = theta / (double) boid->frames_till_reassess_trajectory;
                double rads_over_keyframe = rads_per_frame * boid->frames_till_reassess_trajectory;
                if( theta < 0 )
                    rads_over_keyframe = -rads_over_keyframe;
                Quaternion q = Quaternion( sin(rads_over_keyframe/2.0), 0.0, cos(rads_over_keyframe/2.0), 0.0 );
                Matrix3 R = q.matrix3();
                Pose p1 = boid->body.pose;
                Vector3 forward_step = R * boid_forward_movement_vector;
                Vector3 position = boid->body.pose.position + forward_step;
                std::cout << " pose: ";
                boid->body.pose.position.print();
                std::cout << " forward: ";
                forward_step.print();
                std::cout << " position: ";
                position.print();
                std::cout << std::endl;
                double orientation_y = boid->body.pose.orientation.y() + rads_over_keyframe;
                Pose p2 = Pose( position, Vector3( 0.0, orientation_y, 0.0 ) );
                boid->body.trajectory.insert_keyframe( frame_id, p1 );
                boid->body.trajectory.insert_keyframe( frame_id + boid->frames_till_reassess_trajectory, p2 );
                boid->body.trajectory.construct_trajectory();

                boid->body.trajectory.prev_keyframe_id = 0;
                boid->body.trajectory.next_keyframe_id = 1;
                boid->body.trajectory.spline_basis = CubicSpline::basisUniformNonrationalBSpline();
                boid->body.trajectory.spline_arclength_map = CubicSpline::map_spline( CUBIC_SPLINE_B, boid->body.trajectory.controlpoints );
                boid->body.trajectory.cyclic = false;
            }
        }
    }
/*
    // No spatial coherence -> O(nn)
    for( std::vector<Boid*>::iterator it = boids.begin(); it != boids.end(); it++ ) {
        Boid* boid = (*it);
        boid->get_neighbors( boids, 10.0 );
    }
*/
    /*
    // separation - keeping boids in relative separation
    // alignment - keeping boids in relative alignment
    // cohesion - keeping boids in relative proximity
    for( std::vector<Boid*>::iterator it = boids.begin(); it != boids.end(); it++ ) {
        Boid* boid = (*it);

        // query the size of the list of neighbors ONCE
        unsigned int neighbors = boid->neighbors.size();
        // if there are no neighbors, then there is no affect on this boid
        if( neighbors == 0 ) continue;

        Vector3 accum_position = boid->body.pose.position;
        Vector3 accum_orientation, avg_position, avg_orientation;
        Vector3 orientation;
        if( boid->body.pose.orientation.y() > 2 * PI )
            // !assumes the interval is no greater than 4PI!
            accum_orientation = Vector3( boid->body.pose.orientation.x(), boid->body.pose.orientation.y() - 2 * PI, boid->body.pose.orientation.z() );
        else if( boid->body.pose.orientation.y() < 2 * PI )
            // !assumes the interval is no less than -2PI!
            accum_orientation = Vector3( boid->body.pose.orientation.x(), boid->body.pose.orientation.y() + 2 * PI, boid->body.pose.orientation.z() );
        else
            accum_orientation = boid->body.pose.orientation;

        for( std::vector<Boid*>::iterator it2 = boid->neighbors.begin(); it2 != boid->neighbors.end(); it2++ ) {
            Boid* neighbor = (*it2);
            accum_position += neighbor->body.pose.position;  // cohesion
            orientation = neighbor->body.pose.orientation;
            if( orientation.y() > 2 * PI )
                // !assumes the interval is no greater than 4PI!
                accum_orientation += Vector3( orientation.x(), orientation.y() - 2 * PI, orientation.z() );
            else if( orientation.y() < 2 * PI )
                // !assumes the interval is no less than -2PI!
                accum_orientation += Vector3( orientation.x(), orientation.y() + 2 * PI, orientation.z() );
            else
                accum_orientation += orientation;

            Vector3 p = neighbor->body.pose.position - boid->body.pose.position;
            double dist = p.magnitude();

        }
        avg_position = accum_position / neighbors;
        avg_orientation = accum_orientation / neighbors;


    }
*/
}

//------------------------------------------------------------------------------
/// Transform the boids from the master list of boids
void transform_boids( void ) {
    for( std::vector<Boid*>::iterator it = boids.begin(); it != boids.end(); it++ ) {
        Boid* boid = (*it);
        boid->body.pose.transformByQuaternion();
    }
}

//------------------------------------------------------------------------------
/// Interpolate the articulated body trajectory.
void interpolate( Body* body ) {

    body->trajectory.frame_tic++;

    bool transition = false;

    // handle special transitions such as the cycle or intermediate transitions from keyframe to keyframe
    if( body->trajectory.frame_tic == body->trajectory.keyframe( body->trajectory.keyframes() - 1 ).frame ) {
        if( !body->trajectory.cyclic ) return;
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
/*
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
*/
//------------------------------------------------------------------------------
/// Interpolate the articulated body actor.
void interpolate_hierarchy( void ) {
    interpolate( boid );
    /*
    interpolate( &articulatedbody );

    for( unsigned int i = 0; i < articulatedbody.root_link->child_joints(); i++ ) {
        Joint* joint = articulatedbody.root_link->child_joint( i );
        interpolate( joint );
    }
    */
}

//------------------------------------------------------------------------------

void interpolate_boids( void ) {
    for( std::vector<Flock*>::iterator fit = flocks.begin(); fit != flocks.end(); fit++ ) {
        Flock *flock = (*fit);
        for( std::vector<Boid*>::iterator bit = flock->members.begin(); bit != flock->members.end(); bit++ ) {
            Boid* boid = (*bit);
            if( boid->body.trajectory.keyframes() != 0 ) {
                interpolate( &boid->body );
            }
        }
        //Boid* boid = flock->leader;
        //interpolate( &boid->body );
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

    draw_motivator();

    // Draw from the master boid list
    for( std::vector<Boid*>::iterator it = boids.begin(); it != boids.end(); it++ ) {
        Boid* boid = (*it);
        draw_body( boid->body );
    }

    /*
    // Draw Flocks
    Flock *flock;
    Boid *boid;

    for( std::vector<Flock*>::iterator it = flocks.begin(); it != flocks.end(); it++ ) {
        flock = (*it);
        for( std::vector<Boid*>::iterator bit = flock->members.begin(); bit != flock->members.end(); bit++ ) {
            boid = (*bit);
            draw_body( boid->body );
        }
    }
    */

    /*
    if( opt_draw_actor )
        //draw_articulatedbody( articulatedbody );
        draw_body( *boid );
*/
    /*
    if( opt_draw_trajectories ) {
        draw_trajectory( articulatedbody.trajectory, NULL );
        unsigned int num_ctlpts = articulatedbody.trajectory.controlpoints.size();
        for( unsigned int i = 0; i < num_ctlpts; i++ ) {
            ControlPoint cp = articulatedbody.trajectory.controlpoints.at( i );

            Matrix4 T = Matrix4::translationMatrix( cp.position );
            glLoadMatrixd( T.arrayOpenGL() );

            draw_mesh( mesh_controlpt );
        }
    }
    */
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
        camera.dolly_out( 5.0 );        // not functional at the moment
        break;
    case 119:    // w
        camera.dolly_in( 5.0 );         // not functional at the moment
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
        //interpolate_hierarchy( );
        //transform_hierarchy( );

        arbitrate_boids();
        interpolate_boids();
        // update flocks
        transform_boids();
    }
    glutPostRedisplay( );
}

//------------------------------------------------------------------------------
/// Program entry point
/// Initialize then start rendering
int main( int argc, char** argv )
{
    //initBody( );
    //initConicBoid( );
    initConicBoids( );
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

