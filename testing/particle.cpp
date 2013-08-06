
/*-----------------------THE GEORGE WASHINGTON UNIVERSITY-----------------------
James Taylor                                                         jrt@gwu.edu

template program

------------------------------------------------------------------------------*/

#include <GL/glut.h>
#include <stdio.h>
#include <string>
#include <iostream>
#include <stdexcept>

#include <cs6555/Math/Vector3.h>
#include <cs6555/Math/Matrix4.h>
#include <cs6555/Math/CubicSpline.h>
#include <cs6555/Math/EulerAngle.h>

#include <cs6555/Constants.h>
#include <cs6555/Camera.h>
#include <cs6555/Trajectory.h>
#include <cs6555/Keyframe.h>
#include <cs6555/Mesh.h>
#include <cs6555/MeshLoader.h>
#include <cs6555/Color.h>
#include <cs6555/Material.h>
#include <cs6555/Utilities.h>
#include <cs6555/GeometryMaker.h>
#include <cs6555/Renderer.h>
#include <cs6555/Scene.h>

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

double sim_time = 0.0;
double sim_time_step = 1.0/30.0;
double sim_min_time_step = sim_time_step / 10.0;

unsigned int frames_drawn = 0;
//------------------------------------------------------------------------------

Scene scene;

//------------------------------------------------------------------------------
// Particles
//------------------------------------------------------------------------------

Particle particle;
std::vector<Particle> particles;

//------------------------------------------------------------------------------

void particle_ring( const double& max_radius,
                   const double& radius,
                   const unsigned int& arcsegments,
                   const double& z,
                   const double& max_x_jitter,
                   const double& max_y_jitter,
                   const double& max_z_jitter
                   ) {
    double dtheta = 2.0 * PI / (double) arcsegments;
    double theta = 0.0;
    double x, y;

    // seed offset
    theta = (double)rand() / (double)RAND_MAX * 2.0 * PI / (double) arcsegments;
    //double cvar = (double)rand() / (double)RAND_MAX * 0.05;

    for( unsigned int i = 0; i < arcsegments; i++ ) {
        x = cos(theta) * radius;
        y = sin(theta) * radius;
        theta += dtheta;

        double x_jitter = (double)rand() / (double)RAND_MAX * max_x_jitter * 2.0 - max_x_jitter;
        double y_jitter = (double)rand() / (double)RAND_MAX * max_y_jitter * 2.0 - max_y_jitter;
        double z_jitter = (double)rand() / (double)RAND_MAX * max_z_jitter * 2.0 - max_z_jitter;

        //particle.position = Vector3( x, y, z );
        particle.position = Vector3( x + x_jitter, y + y_jitter, z + z_jitter );

        /*
        // ion ring
        if( radius > 0.99 )
            particle.color = Color::black();
        else if( radius > 0.95 )
            particle.color = Color::red();
        else if( radius > 0.90 )
            particle.color = Color::yellow();
        else if( radius > 0.25 )
            particle.color = Color::cyan();
        else
            particle.color = Color::white();
        */
        // flame ring
        if( radius > 0.99 * max_radius )
            particle.color = Color::black();
        else if( radius > 0.70 * max_radius )
            particle.color = Color::red();
        else if( radius > 0.40 * max_radius )
            particle.color = Color::orange();
        else if( radius > 0.25 * max_radius )
            particle.color = Color::yellow();
        else
            particle.color = Color::white();

        particle.mass = 1e-18;
        //particle.mass = 0.00001;
        particle.frames_to_live = 500;
        particles.push_back( particle );
    }
}


//------------------------------------------------------------------------------
// Dynamics
//------------------------------------------------------------------------------
void compute_forces( const double& t, Particle* particle ) {
    // gravity
    const double g = -9.8;

    Vector3 acceleration = Vector3( 0.0, g, 0.0 );

    // F = ma
    particle->force = particle->mass * acceleration;

}

void ode( const double& t0, const double& t1, Particle* particle ) {
    double dt = t1 - t0;

    // --- Begin: Linear Integration ---
    // dp = F dt
    particle->linear_momentum += (particle->force * dt);

    // p = mv -> v = p/m
    particle->linear_velocity = particle->linear_momentum / particle->mass;

    // x1 = x0 + v*dt
    particle->position += (particle->linear_velocity * dt);
    // --- End: Linear Integration ---

}



//------------------------------------------------------------------------------
// Camera Functions
//------------------------------------------------------------------------------
void initScene( void ) {
    scene.camera.position = Vector3( 0.0, 0.0, 2.0 );
    scene.camera.viewpoint = Vector3( 0.0, 0.0, 0.0 );
    scene.camera.up = Vector3( 0.0, 1.0, 0.0 );

    scene.camera.project();

    double max_radius = 0.4;
    double max_height = 0.8;
    double z_translation = 0.0;

    for( int layer = 0; layer < 50; layer++ ) {
        double z = (double)rand() / (double)RAND_MAX * max_height + z_translation;
        for( int ring = 0; ring < 10; ring++ ) {
            double radius = (double)rand() / (double)RAND_MAX * max_radius;

            particle_ring( 0.4, radius, 32, z, 0.01, 0.01, 0.01 );
        }
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
    scene.camera.project();
    glMatrixMode( GL_PROJECTION );
    glLoadIdentity( );
    glLoadMatrixd( scene.camera.projection.arrayOpenGL() );

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

    glPushMatrix();
    Renderer::draw( &scene );
    glPopMatrix();
    //glPushMatrix();
    //Renderer::draw( &particle );
    //glPopMatrix();

    glPushMatrix();
    for( std::vector<Particle>::iterator it = particles.begin(); it != particles.end(); it++ ) {

        //if( it->frames_to_live ) it->frames_to_live--;
        //if( it->frames_to_live )
            Renderer::draw( &(*it) );
    }
    glPopMatrix();

    glutSwapBuffers();

    if( GENERATE_MOVIE ) {
        sprintf( filename, "%s_%.04d.tif",filetitle.c_str(), frame_id );
        printf( "%s\n", filename );
        Utilities::writetiff( filename, "movie", 0, 0, Width, Height, COMPRESSION_NONE );
    }

    frames_drawn++;

    //printf( "frame: %d\n", frames_drawn );
    if( frames_drawn % 30 == 0 )
        printf( "frame:%d, sim_time:%f\n", frames_drawn, sim_time );
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
        scene.camera.track_left( PI/16 );     // not functional at the moment
        break;
    case 100:    // d
        scene.camera.track_right( PI/16 );    // not functional at the moment
        break;
    case 115:    // s
        scene.camera.dolly_out( 5.0 );        // not functional at the moment
        break;
    case 119:    // w
        scene.camera.dolly_in( 5.0 );         // not functional at the moment
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
    //if( !sim_pause ) {
        frame_id++;

        // TODO : Transform here
        for( std::vector<Particle>::iterator it = particles.begin(); it != particles.end(); it++ ) {
            Particle* p = &(*it);

            compute_forces( sim_time, p );

            ode( sim_time, sim_time + sim_time_step, p );

            /*
            if( p->frames_to_live == 0 ) {
                particles.erase( it );
                //delete p;
            }
            */
        }

        sim_time += sim_time_step;

    //}
    glutPostRedisplay( );
}

//------------------------------------------------------------------------------
/// Program entry point
/// Initialize then start rendering
int main( int argc, char** argv )
{
    // TODO : Initialize here
    initScene( );

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




