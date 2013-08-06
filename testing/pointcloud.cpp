
/*-----------------------THE GEORGE WASHINGTON UNIVERSITY-----------------------
James Taylor                                                         jrt@gwu.edu

PointCloud Unit Test

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

#include <cs6555/Renderer.h>
#include <cs6555/Math.h>

#include <cs6555/PointCloud.h>
#include <cs6555/ParticleSoup.h>
#include <cs6555/SpringMassSystem.h>
#include <cs6555/CloudMaker.h>

//------------------------------------------------------------------------------
// Hardcoded Defines
//------------------------------------------------------------------------------

#define GENERATE_MOVIE  0               // No, do not generate a movie
//#define GENERATE_MOVIE  1               // Yes, do generate a movie

//------------------------------------------------------------------------------
// Macros
//------------------------------------------------------------------------------


//------------------------------------------------------------------------------
// Toggle Options
//------------------------------------------------------------------------------

bool opt_draw_actor = true;                 // toggle by pressing 1
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

PointCloud* pc;
ParticleSoup* soup;
Material material;

//------------------------------------------------------------------------------
// Initialization
//------------------------------------------------------------------------------
void initTest( void ) {
    title = "PointCloud Test";
    filetitle = "pointcloud";

    camera.position = Vector3( 0.0, 2.0, 0.0 );
    camera.viewpoint = Vector3( 0.0, 0.0, 0.0 );
    camera.up = Vector3( 0.0, 0.0, 1.0 );
    camera.project();

    pc = CloudMaker::random_pointcloud( 10, Color( 1.0, 0.0, 0.0, 1.0 ), Vector3( 0.0, 0.0, 0.0 ), 5.0 );
    soup = CloudMaker::random_particle_soup( 50, Color( 1.0, 0.0, 0.0, 1.0 ), Vector3( 0.0, 0.0, 0.0 ), 5.0 );
    material.randomize();
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

    Renderer::activate( material );
    Renderer::draw( soup );

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

        // TODO :: state update
    }
    glutPostRedisplay( );
}

//------------------------------------------------------------------------------
/// Program entry point
/// Initialize then start rendering
int main( int argc, char** argv )
{
    initTest( );

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



