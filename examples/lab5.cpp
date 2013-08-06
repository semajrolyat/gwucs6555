
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
#include <cs6555/Movie.h>

#include <cs6555/Motivator.h>
#include <cs6555/Particle.h>
#include <cs6555/ParticleCloud.h>
#include <cs6555/ParticleEmitter.h>

//------------------------------------------------------------------------------
// Hardcoded Defines
//------------------------------------------------------------------------------

#define GENERATE_MOVIE  0               // No, do not generate a movie
//#define GENERATE_MOVIE  1               // Yes, do generate a movie

#define DYNAMICS 1
//#define GRAVITY

//------------------------------------------------------------------------------
// Macros
//------------------------------------------------------------------------------

#define DEG_PER_RAD  180.0/PI
#define RAD_PER_DEG  PI/180.0

#define FRAMERATE    30

//------------------------------------------------------------------------------
// Toggle Options
//------------------------------------------------------------------------------

bool opt_draw_actor = true;                 // toggle by pressing 1
//bool opt_draw_trajectories = true;
bool opt_draw_trajectories = false;         // toggle by pressing 2
bool sim_pause = false;                     // toggle by pressing p

//------------------------------------------------------------------------------

static unsigned int frame = 0;              // current frame.  Counter
int Width;                                  // Width of OpenGL Viewport
int Height;                                 // Height of OpenGL Viewport
std::string title;                          // Window Title

//------------------------------------------------------------------------------

std::string moviename;

//------------------------------------------------------------------------------

double sim_time = 0.0;
double sim_time_step = 1.0/ ((double)(FRAMERATE * 100));
//double sim_min_time_step = sim_time_step / 10.0;

//------------------------------------------------------------------------------

Scene scene;

Motivator low_pressure, hi_pressure;

//------------------------------------------------------------------------------
// Particles
//------------------------------------------------------------------------------

//Particle particle;
//std::vector<Particle> particles;
ParticleEmitter emitter;
ParticleCloud cloud;

//------------------------------------------------------------------------------
// Dynamics
//------------------------------------------------------------------------------
void compute_forces( const double& t, Particle* particle ) {
    // gravity
#ifdef GRAVITY
    const double g = -9.8;
#else
    const double g =  0.0;
#endif

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

    title = "Particles";
    moviename = "particles";

    scene.camera.position = Vector3( 0.0, 0.0, 20.0 );
    scene.camera.viewpoint = Vector3( 0.0, 0.0, 0.0 );
    scene.camera.up = Vector3( 0.0, 1.0, 0.0 );

    scene.camera.project();

    low_pressure.position = Vector3( 10.0, 0.0, 0.0 );
    low_pressure.type = MOTIVATOR_TYPE_ATTRACTOR;
    low_pressure.insert( MeshLoader::sphere( 0.5, 16, 16 ) );

    hi_pressure.position = Vector3( -10.0, 0.0, 0.0 );
    hi_pressure.type = MOTIVATOR_TYPE_REPULSOR;
    hi_pressure.insert( MeshLoader::sphere( 0.5, 16, 16 ) );

    //double max_radius = 0.01;
    double max_radius = 1.0;
    double min_radius = 0.5;

    Vector3 center = Vector3( 0.0, 0.0, 0.0 );
    Particle prototype = Particle( );
    prototype.mass = 1e-32;
    //prototype.mass = 1e2;
    prototype.frames_to_live = 500;

    unsigned int particles = 5e4;

    CloudColorFun* colorfun = &ParticleCloud::blackhole;

    cloud.insert( ParticleCloud::disk( colorfun, prototype, Vector3( 0.0, 0.0, 0.0 ), 1.0, 0.2, particles ) );

    cloud.insert( ParticleCloud::sphere( &ParticleCloud::black, prototype, Vector3( 0.0, 0.0, 0.0 ), 0.2, particles ) );

    cloud.insert( ParticleCloud::ring( &ParticleCloud::flame, prototype, Vector3( 0.0, 0.0, 0.0 ), 0.92, 0.7, particles ) );

    /*
    cloud.insert( ParticleCloud::ring( colorfun, prototype, Vector3( 0.0, 0.0, 0.0 ), 0.5, 0.2, particles ) );

    cloud.insert( ParticleCloud::ring( colorfun, prototype, Vector3( 0.0, 0.0, 0.1 ), 0.55, 0.2, particles ) );

    cloud.insert( ParticleCloud::ring( colorfun, prototype, Vector3( 0.0, 0.0, 0.2 ), 0.6, 0.2, particles ) );

    cloud.insert( ParticleCloud::ring( colorfun, prototype, Vector3( 0.0, 0.0, 0.3 ), 0.65, 0.2, particles ) );

    cloud.insert( ParticleCloud::ring( colorfun, prototype, Vector3( 0.0, 0.0, 0.4 ), 0.7, 0.2, particles ) );

    cloud.insert( ParticleCloud::ring( colorfun, prototype, Vector3( 0.0, 0.0, 0.5 ), 0.75, 0.2, particles ) );

    cloud.insert( ParticleCloud::ring( colorfun, prototype, Vector3( 0.0, 0.0, 0.6 ), 0.8, 0.2, particles ) );

    cloud.insert( ParticleCloud::ring( colorfun, prototype, Vector3( 0.0, 0.0, 0.7 ), 0.85, 0.2, particles ) );

    cloud.insert( ParticleCloud::ring( colorfun, prototype, Vector3( 0.0, 0.0, 0.8 ), 0.9, 0.2, particles ) );

    cloud.insert( ParticleCloud::ring( colorfun, prototype, Vector3( 0.0, 0.0, 0.9 ), 0.95, 0.2, particles ) );
    */

    cloud.position = center;
    cloud.colorfun = colorfun;

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

    //Renderer::draw( &low_pressure );
    //Renderer::draw( &hi_pressure );

    //Renderer::draw( &scene );

    Renderer::draw( &cloud );

    glutSwapBuffers();

    if( GENERATE_MOVIE )
        Movie::write_frame( moviename, frame, Width, Height );

    if( frame % FRAMERATE == 0 )
        printf( "frame:%d, sim_time:%f\n", frame, sim_time );
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
    if( !sim_pause ) {
        frame++;

        if( DYNAMICS ) {
            for( unsigned int i = 0; i < cloud.geometryCount(); i++ ) {
                Particle* particle = (Particle*) cloud.geometry( i );

                ode( sim_time, sim_time + sim_time_step, particle );

                compute_forces( sim_time, particle );
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



