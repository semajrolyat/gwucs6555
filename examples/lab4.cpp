
/*-----------------------THE GEORGE WASHINGTON UNIVERSITY-----------------------
James Taylor                                                         jrt@gwu.edu

Boids Unit Test

------------------------------------------------------------------------------*/

#include <iostream>

#include <cs6555/Constants.h>

#include <GL/glut.h>
#include <stdio.h>

#include <string>

#include <cs6555/Math/Vector3.h>
#include <cs6555/Math/Matrix4.h>

#include <cs6555/Camera.h>

#include <cs6555/Math/CubicSpline.h>
#include <cs6555/Trajectory.h>
#include <cs6555/Keyframe.h>

#include <cs6555/Math/EulerAngle.h>

#include <cs6555/Mesh.h>
#include <cs6555/MeshLoader.h>

#include <stdexcept>

#include <cs6555/Color.h>
#include <cs6555/Material.h>

#include <cs6555/Utilities.h>
#include <cs6555/GeometryMaker.h>

#include <cs6555/Boid.h>
#include <cs6555/Flock.h>
#include <cs6555/Motivator.h>

#include <cs6555/Renderer.h>
#include <cs6555/Movie.h>

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

Camera camera;                              // A camera actor object.  Generates projection matrix

#define BOID_TYPE_BODY              0;
#define BOID_TYPE_ARTICULATED_BODY  1;

unsigned int boid_type;
Body* boid;

std::vector<Flock*> flocks;
std::vector<Boid*> boids;

Material *boid_material;
Material *boid_material2;

Motivator motivator;
Mesh* motivator_mesh;

//------------------------------------------------------------------------------
// Transformation Operations
//------------------------------------------------------------------------------
/// normalize an angle in the interval [-PI,PI)
double normalize_angle( double theta ) {
    while (theta <= -PI)
        theta += 2.0 * PI;
    while (theta > PI)
        theta -= 2.0 * PI;
    return theta;
}

//------------------------------------------------------------------------------
/// returns the angle between two unit vectors located in the x,z plane
double angle_in_x_z( const Vector3& start_orientation, const Vector3& end_orientation ) {
    double start_theta = atan2( start_orientation.x(), start_orientation.z() );
    double end_theta = atan2( end_orientation.x(), end_orientation.z() );
    double theta = end_theta - start_theta;
    return normalize_angle( theta );
}

//------------------------------------------------------------------------------
/// down and dirty integration of orientation.
/// Note: passed by value
/// Artifact of earlier development but still necessary for initial state
Vector3 integrate_orientation( Vector3 local_orientation, Vector3 world_orientation, Vector3 desired_orientation, double max_dtheta ) {
    local_orientation.normalize();

    // transform the orientation toward the boid front in world space
    Matrix3 R_prev = EulerAngle::matrix3( EULER_ANGLE_XYZ, EULER_HANDED_LEFT, world_orientation.x(), world_orientation.y(), world_orientation.z() );
    Vector3 orientation = R_prev * local_orientation;
    orientation.normalize();

    desired_orientation.normalize();

    // calculate the angle between the direction to the motivator and the orientation of the boid
    double gross_theta = angle_in_x_z( orientation, desired_orientation );

    // determine the possible amount of yaw for this frame depending on the boid->yaw_rate_per_frame
    double dtheta = gross_theta;
    if( fabs(gross_theta) > max_dtheta ) {
        if( dtheta < 0 ) {
            dtheta = -max_dtheta;
        } else {
            dtheta = max_dtheta;
        }
    }
    // integrate the rotation
    double theta = world_orientation.y() + dtheta;
    return Vector3( 0.0, theta, 0.0 );
}

//------------------------------------------------------------------------------
/// down and dirty integration of velocity given a scalar acceleration and max velocity
/// Note: passed by value
/// Artifact of earlier development but still necessary for initial state
Vector3 integrate_velocity( Vector3 local_orientation, Vector3 world_orientation, Vector3 world_velocity, double max_ddx, double max_dx ) {
    local_orientation.normalize();

    // transform the orientation toward the boid front in world space
    Matrix3 R_now = EulerAngle::matrix3( EULER_ANGLE_XYZ, EULER_HANDED_LEFT, world_orientation.x(), world_orientation.y(), world_orientation.z() );
    Vector3 orientation_now = R_now * local_orientation;
    orientation_now.normalize();

    // integrate the velocity
    Vector3 v = world_velocity + orientation_now * max_ddx;
    double m = v.magnitude();

    if( fabs( m ) > max_dx ) {
        v.normalize();
        v *= max_dx;
    }

    return v;
}

//------------------------------------------------------------------------------
/// down and dirty integration of velocity given an acceleration vector.
/// Note: passed by value
/// Artifact of earlier development but still necessary for initial state
Vector3 integrate_velocity( Vector3 local_orientation, Vector3 world_orientation, Vector3 world_velocity, Vector3 max_ddx ) {
    local_orientation.normalize();

    // transform the orientation toward the boid front in world space
    Matrix3 R_now = EulerAngle::matrix3( EULER_ANGLE_XYZ, EULER_HANDED_LEFT, world_orientation.x(), world_orientation.y(), world_orientation.z() );
    Vector3 orientation_now = R_now * local_orientation;
    orientation_now.normalize();

    // integrate the velocity with impulse force max_ddx
    Vector3 v = world_velocity + max_ddx;

    return v;
}

//------------------------------------------------------------------------------
/// down and dirty integration of position
/// Note: passed by value
/// Artifact of earlier development but still necessary for initial state
Vector3 integrate_position( Vector3 local_orientation, Vector3 world_orientation, Vector3 world_position, double dx ) {
    local_orientation.normalize();

    // transform the orientation toward the boid front in world space
    Matrix3 R_now = EulerAngle::matrix3( EULER_ANGLE_XYZ, EULER_HANDED_LEFT, world_orientation.x(), world_orientation.y(), world_orientation.z() );
    Vector3 orientation_now = R_now * local_orientation;
    orientation_now.normalize();

    // integrate the position
    return world_position + orientation_now * dx;
}

//------------------------------------------------------------------------------
/// computation of an acceleration vector from a scalar acceleration and vector orientation
/// Note: passed by value
/// Artifact of earlier development but still necessary for initial state
Vector3 calculate_linear_acceleration( Vector3 local_orientation, Vector3 world_orientation, double ddx ) {
    local_orientation.normalize();

    // transform the orientation toward the boid front in world space
    Matrix3 R_now = EulerAngle::matrix3( EULER_ANGLE_XYZ, EULER_HANDED_LEFT, world_orientation.x(), world_orientation.y(), world_orientation.z() );
    Vector3 orientation_now = R_now * local_orientation;
    orientation_now.normalize();

    return orientation_now * ddx;
}

//------------------------------------------------------------------------------
// Camera Functions
//------------------------------------------------------------------------------
void initCamera( void ) {
    camera.position = Vector3( 0.0, 200.0, 0.0 );
    camera.viewpoint = Vector3( 0.0, 0.0, 0.0 );
    camera.up = Vector3( 0.0, 0.0, 1.0 );

    camera.project();
}

//------------------------------------------------------------------------------
// Boid Functions
//------------------------------------------------------------------------------
/// generates a randomized flock
void generate_flock( Material* material, const Color& color, const Vector3& flock_center, const unsigned int& number_of_boids, const double& flock_extens ) {
    Boid* boid;
    Flock* flock = new Flock();
    flocks.push_back( flock );

    // assign to master list of boids
    double radius = 1.0;
    double height = 3.0;
    for( unsigned int i = 0; i < number_of_boids; i++ ) {
        boid = new Boid();

        // create the mesh
        boid->body.insert( MeshLoader::cone( radius, height, 16, material, ML_MATERIAL_ASSIGN ) );

        // randomly generate the position, constrained to the flock extens
        double x = flock_center.x() + ( (double) rand() / (double) RAND_MAX ) * flock_extens - flock_extens / 2.0;
        double y = 0.0;
        double z = flock_center.z() + ( (double) rand() / (double) RAND_MAX ) * flock_extens - flock_extens / 2.0;
        boid->body.pose.position = Vector3( x, y, z );

        boid->member_of = flock;

        boid->body.mass = 1.0;
        boid->goal = BOID_GOAL_LOCAL;
        boid->role = BOID_ROLE_FOLLOWER;
        boid->motivator_detector.position = Vector3( 0.0, 0.0, height );

        // rate of yaw in terms of frames
        // Note: this is an artifact of previous generations, but still necessary due to the computation
        // of the initial state
        boid->max_yaw_rate_per_frame = RAD_PER_DEG / 1.0;

        // constraint on the velocity otherwise lim of velocity due to acceleration -> infinity
        //boid->max_forward_velocity_per_frame = 0.5;
        boid->max_forward_velocity_per_frame = 0.25;

        // constraint on the acceleration
        boid->max_acceleration_per_frame = boid->max_forward_velocity_per_frame / 10.0;

        boid->color = color;

        // bubbles
        boid->personal_space_radius = 3.0;
        boid->neighbor_perception_radius = 20.0;

        // suuggestion weights
        boid->motivator_force = 0.1;
        boid->avoidance_force = 0.5;
        boid->matching_force = 0.2;
        boid->centering_force = 0.2;

        // initial state
        boid->arbitrated_orientation = integrate_orientation( boid->motivator_detector.position, boid->body.pose.orientation, motivator.position - boid->body.pose.position, boid->max_yaw_rate_per_frame );
        boid->arbitrated_linear_acceleration = calculate_linear_acceleration( boid->motivator_detector.position, boid->arbitrated_orientation, boid->max_acceleration_per_frame );
        boid->arbitrated_linear_velocity = integrate_velocity( boid->motivator_detector.position, boid->arbitrated_orientation, Vector3(0,0,0), boid->max_acceleration_per_frame, boid->max_forward_velocity_per_frame );
        double velocity = boid->arbitrated_linear_velocity.magnitude();
        boid->arbitrated_position = integrate_position( boid->motivator_detector.position, boid->arbitrated_orientation, boid->body.pose.position, velocity );

        // add it to the master list of boids
        boids.push_back( boid );

        // add this boid to the flock
        flock->add( boid );
    }
}

//------------------------------------------------------------------------------

void initConicBoids( void ) {
    title = "Boid Movement";
    filetitle = "boid_movement";

    motivator.position = Vector3( 0.0, 0.0, 100.0 );
    motivator.type = MOTIVATOR_TYPE_ATTRACTOR;
    motivator_mesh = MeshLoader::box( 4.0, 4.0, 4.0 );

    boid_material = new Material( Color(0.0, 0.0, 1.0, 1.0), Color(0.0, 0.0, 1.0, 1.0), Color(0.0, 0.0, 0.0, 1.0), Color(0.0, 0.0, 0.0, 1.0), 10.0 );

    // generate 3 flocks
    generate_flock( boid_material, Color(1.0, 0.0, 0.0, 1.0), Vector3( 100.0, 0.0, -100.0 ), 20, 25.0 );

    generate_flock( boid_material, Color(1.0, 1.0, 0.0, 1.0), Vector3( -100.0, 0.0, -80.0 ), 20, 25.0 );

    generate_flock( boid_material, Color(1.0, 0.0, 1.0, 1.0), Vector3( -125.0, 0.0, 0.0 ), 20, 25.0 );

}

//------------------------------------------------------------------------------
/// iterates over the list of boids and computes all perception
/// O(n^2) time complexity
void compute_boid_perception( void ) {

    for( std::vector<Boid*>::iterator bit = boids.begin(); bit != boids.end(); bit++ ) {
        Boid* boid = (*bit);

        boid->get_neighbors( boid->member_of->members, boid->neighbor_perception_radius );
    }
}

//------------------------------------------------------------------------------
/// determines the strength of collision avoidance behavior for a given boid
/// determination is based upon the previously computed values
Vector3 suggest_goto_motivator( Boid* boid ) {

    Vector3 suggested_acceleration;

    // compute unit direction
    suggested_acceleration = motivator.position - boid->previous_position;
    suggested_acceleration.normalize();

    // scale by force
    suggested_acceleration *= boid->motivator_force;

    return suggested_acceleration;
}

//------------------------------------------------------------------------------
/// determines the strength of collision avoidance behavior for a given boid
/// determination is based upon the previously computed values
Vector3 suggest_collision_avoidance( Boid* boid ) {

    Vector3 suggested_acceleration = Vector3( 0.0, 0.0, 0.0 );

    // Rules:
    // 1. Compute the bubble.  If the bubble intersects with another bubble
    // 2. Only avoid hazards to the front
    // 2a. Compute the weight the bubble influences.
    // 2b. Scale the resultant acceleration based upon the weight of influence

    boid->clear_hazards();
    for( std::vector<Boid*>::iterator nit = boid->neighbors.begin(); nit != boid->neighbors.end(); nit++ ) {
        Boid* neighbor = (*nit);

        Vector3 v_distance_to = boid->previous_position - neighbor->previous_position;
        double distance_to = v_distance_to.magnitude();
        double buffer_distance = boid->personal_space_radius + neighbor->personal_space_radius;

        // bubbles intersect
        if( distance_to < buffer_distance ) {
            // if you are the boid in front, then don't consider the one behind for collision purposes

            // the heading of the boid
            Vector3 actual_heading = boid->previous_linear_velocity;
            actual_heading.normalize();
            // the heading to the intersection point between the boid and the neighbor
            Vector3 pseudo_heading = neighbor->previous_position - boid->previous_position;
            pseudo_heading.normalize();

            // if the inner product between the actual heading and the pseudo heading is
            // in the forward field then register a possible collision
            double dp = Vector3::dot( actual_heading, pseudo_heading );
            if( dp > 0.0 ) {
                boid->add_hazard( neighbor );

                // determine the direction
                double theta = -angle_in_x_z( actual_heading, pseudo_heading );
                Vector3 heading = Vector3( 0.0, boid->previous_orientation.y() + theta, 0.0 );

                Vector3 local_orientation = boid->motivator_detector.position;
                local_orientation.normalize();

                // transform the orientation toward the boid front in world space
                Matrix3 R = EulerAngle::matrix3( EULER_ANGLE_XYZ, EULER_HANDED_LEFT, heading.x(), heading.y(), heading.z() );
                Vector3 orientation = R * local_orientation;
                orientation.normalize();
                orientation.y( 0.0 );
                orientation.normalize();

                // determine the magnitude
                // basically R + R / r + r > 1.0 -> weight to be applied to acceleration
                double weight = buffer_distance / distance_to;

                suggested_acceleration += orientation * weight;
            }
        }
    }
    suggested_acceleration.normalize();
    suggested_acceleration *= boid->avoidance_force;
    return suggested_acceleration;
}

//------------------------------------------------------------------------------
/// determines the strength of velocity matching behavior for a given boid
/// determination is based upon the previously computed values
Vector3 suggest_velocity_matching( Boid* boid ) {

    Vector3 suggested_acceleration;

    Vector3 local_velocity = boid->previous_linear_velocity;

    for( std::vector<Boid*>::iterator nit = boid->neighbors.begin(); nit != boid->neighbors.end(); nit++ ) {
        Boid* neighbor = (*nit);
        // find the local heading with respect to this boid and its immediate neighbors
        local_velocity += neighbor->previous_linear_velocity;
    }
    local_velocity.normalize();                      // normalize to make unit length

    suggested_acceleration = local_velocity * boid->matching_force;

    return suggested_acceleration;
}

//------------------------------------------------------------------------------
/// determines the strength of flock centering behavior for a given boid
/// determination is based upon the previously computed values
Vector3 suggest_flock_centering( Boid* boid ) {

    Vector3 suggested_acceleration;

    Vector3 local_center = boid->previous_position;

    for( std::vector<Boid*>::iterator nit = boid->neighbors.begin(); nit != boid->neighbors.end(); nit++ ) {
        Boid* neighbor = (*nit);
        // find the local center with respect to this boid and its immediate neighbors
        local_center += (boid->previous_position - neighbor->previous_position) / 2.0;
    }
    Vector3 local_heading = local_center - boid->previous_position;
    local_heading.normalize();

    suggested_acceleration = local_heading * boid->centering_force;

    return suggested_acceleration;
}

//------------------------------------------------------------------------------
/// gather suggestions for each behavior for each boid
/// O(n) time complexity
void gather_suggestions() {
    // update previous for future error calculation
    for( std::vector<Boid*>::iterator bit = boids.begin(); bit != boids.end(); bit++ ) {
        Boid* boid = (*bit);

        boid->previous_linear_acceleration = boid->arbitrated_linear_acceleration;
        boid->previous_linear_velocity = boid->arbitrated_linear_velocity;
        boid->previous_position = boid->arbitrated_position;
        boid->previous_orientation = boid->arbitrated_orientation;
    }

    for( std::vector<Boid*>::iterator bit = boids.begin(); bit != boids.end(); bit++ ) {
        Boid* boid = (*bit);
        boid->goto_motivator_acceleration = suggest_goto_motivator( boid );
        boid->collision_avoidance_acceleration = suggest_collision_avoidance( boid );
        boid->velocity_matching_acceleration = suggest_velocity_matching( boid );
        boid->flock_centering_acceleration = suggest_flock_centering( boid );
    }
}

//------------------------------------------------------------------------------
/// arbitrate among all behavioral suggestions for each boid
void arbitrate_suggestions( void ) {

    // arbitrate between suggestions for each boid
    for( std::vector<Boid*>::iterator bit = boids.begin(); bit != boids.end(); bit++ ) {
        Boid* boid = (*bit);

        double mag_avoidance_suggestion = boid->collision_avoidance_acceleration.magnitude();
        double mag_matching_suggestion = boid->velocity_matching_acceleration.magnitude();
        double mag_centering_suggestion = boid->flock_centering_acceleration.magnitude();

        Vector3 arbitrated_acceleration_heading;
        double arbitrated_acceleration_magnitude;

        if( mag_avoidance_suggestion > mag_matching_suggestion &&
            mag_avoidance_suggestion > mag_centering_suggestion ) {
            // collision avoidance is priority
            arbitrated_acceleration_heading = boid->collision_avoidance_acceleration;

            // ?parcelling?
        } else if( mag_matching_suggestion > mag_avoidance_suggestion &&
                   mag_matching_suggestion > mag_centering_suggestion ) {
            // velocity matching is priority
            arbitrated_acceleration_heading = boid->velocity_matching_acceleration;

            // ?parcelling?
        } else if( mag_centering_suggestion > mag_avoidance_suggestion &&
                   mag_centering_suggestion > mag_matching_suggestion ) {
            // flock centering is priority
            arbitrated_acceleration_heading = boid->flock_centering_acceleration;

            // ?parcelling?
        } else {
            // otherwise goto motivator
            arbitrated_acceleration_heading = boid->goto_motivator_acceleration;

            // ?parcelling?
        }

        // now compute the action
        // first update acceleration
        arbitrated_acceleration_magnitude = arbitrated_acceleration_heading.magnitude();
        arbitrated_acceleration_heading.normalize();
        if( arbitrated_acceleration_magnitude > boid->max_acceleration_per_frame ) {
            arbitrated_acceleration_magnitude = boid->max_acceleration_per_frame;
        }
        boid->arbitrated_linear_acceleration = arbitrated_acceleration_heading * arbitrated_acceleration_magnitude;

        // second integrate velocity
        if( boid->previous_linear_velocity.magnitude() > boid->max_forward_velocity_per_frame ) {
            boid->arbitrated_linear_velocity = arbitrated_acceleration_heading * boid->max_forward_velocity_per_frame;
        } else {
            boid->arbitrated_linear_velocity = boid->previous_linear_velocity + boid->arbitrated_linear_acceleration;
        }

        // third integrate position
        boid->arbitrated_position = boid->previous_position + boid->arbitrated_linear_velocity;

        // fourth derive the orientation from the velocity vector
        Vector3 world_heading = boid->arbitrated_linear_velocity;
        world_heading.normalize();
        Vector3 local_heading = boid->motivator_detector.position;
        local_heading.normalize();
        double theta = -angle_in_x_z( world_heading, local_heading );
        theta = normalize_angle( theta );
        boid->arbitrated_orientation = Vector3( 0.0, theta, 0.0 );

    }

    // update the actual pose from the arbitrated values
    for( std::vector<Boid*>::iterator bit = boids.begin(); bit != boids.end(); bit++ ) {
        Boid* boid = (*bit);

        // update the pose
        boid->body.pose.orientation = boid->arbitrated_orientation;
        boid->body.pose.position = boid->arbitrated_position;
    }

    return;
}

//------------------------------------------------------------------------------
// Drawing Operations
//------------------------------------------------------------------------------

void draw_motivator( ) {
    glPushMatrix();
    glLoadIdentity();

    Matrix4 transform = Matrix4::translationMatrix( motivator.position );
    glLoadMatrixd( transform.arrayOpenGL() );
    Renderer::draw( motivator_mesh );

    glPopMatrix();
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
// Modified SimpleGLUT
//------------------------------------------------------------------------------
/// Initialization of OpenGL
void init(void)
{
   glClearColor( 0.0, 0.0, 0.0, 0.0 );
   glClearDepth( 1.0 );
   glShadeModel( GL_SMOOTH );
   glEnable( GL_LINE_SMOOTH );
   glEnable( GL_NORMALIZE );
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
    glDepthFunc( GL_LEQUAL );
    glHint( GL_PERSPECTIVE_CORRECTION_HINT, GL_NICEST );

    GLfloat LightAmbient[] =  { 0.4f, 0.4f, 0.4f, 1.0f};
    GLfloat LightDiffuse[] =  { 0.3f, 0.3f, 0.3f, 1.0f};
    GLfloat LightSpecular[] = { 0.4f, 0.4f, 0.4f, 1.0f};
    GLfloat LightPosition[] = { 0.0f, 5.0f, 0.0f, 1.0f};

    glClearColor( 0.25, 0.25, 0.25, 0.0 );
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

    glLightfv(GL_LIGHT0, GL_AMBIENT, LightAmbient);
    glLightfv(GL_LIGHT0, GL_DIFFUSE, LightDiffuse);
    glLightfv(GL_LIGHT0, GL_SPECULAR, LightSpecular);
    glLightfv(GL_LIGHT0, GL_POSITION, LightPosition);
    glEnable(GL_LIGHT0);
    glShadeModel(GL_SMOOTH);

    glLightModeli( GL_LIGHT_MODEL_LOCAL_VIEWER, GL_TRUE );

    glEnable( GL_COLOR_MATERIAL );

    glColor3d( 0.8, 0.8, 0.8 );

    draw_motivator();

    // Draw from the master boid list
    for( std::vector<Boid*>::iterator it = boids.begin(); it != boids.end(); it++ ) {
        Boid* boid = (*it);
        glColor3d( boid->color.r(), boid->color.g(), boid->color.b() );
        Renderer::draw( &boid->body );
    }

    glutSwapBuffers();

    if( GENERATE_MOVIE )
        Movie::write_frame( filename, frame_id, Width, Height );

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

        compute_boid_perception();
        gather_suggestions();
        arbitrate_suggestions();
        transform_boids();
    }
    glutPostRedisplay( );
}

//------------------------------------------------------------------------------
/// Program entry point
/// Initialize then start rendering
int main( int argc, char** argv )
{
    initConicBoids( );
    initCamera( );

    // initialize OpenGL
    glutInit( &argc, argv );
    glutInitDisplayMode( GLUT_DOUBLE | GLUT_RGBA | GLUT_DEPTH | GLUT_ALPHA );
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


