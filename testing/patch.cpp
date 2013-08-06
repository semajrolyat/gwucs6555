/*-----------------------THE GEORGE WASHINGTON UNIVERSITY-----------------------
James Taylor                                                         jrt@gwu.edu

Patch unit test

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
#include <cs6555/GeometryMaker.h>
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

#include <cs6555/Patch.h>

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
double sim_min_time_step = sim_time_step / 10.0
        ;
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

Patch* patch;

#include <cs6555/PerlinNoise.h>
#include <cs6555/Renderer.h>

void initPatch( void ) {
    patch = new Patch();

}

Mesh *start, *end;

Material *material1, *material2;

void initShapes( void ) {
    title = "LoadPly";
    filetitle = "loadply";

    scene.camera.position = Vector3( 0.0, 0.0, -0.0 );
    scene.camera.viewpoint = Vector3( 0.0, 0.0, -1.0 );
    scene.camera.up = Vector3( 0.0, 1.0, 0.0 );

    scene.camera.project();

    start = MeshLoader::loadPly( "mesh_start_ascii.ply" );
    end = MeshLoader::loadPly( "mesh_end_ascii.ply" );

    material1 = new Material(true);
    material2 = new Material(true);
}

//#define BACKFACE_CULL   0
#define BACKFACE_CULL   1

//------------------------------------------------------------------------------
// Drawing Operations
//------------------------------------------------------------------------------
/// Draws a mesh
void draw_pointcloud( Mesh* mesh, Material* material ) {

    glMaterialfv( GL_FRONT, GL_AMBIENT, (GLfloat*)material->ambient.arrayOpenGL() );
    glMaterialfv( GL_FRONT, GL_DIFFUSE, (GLfloat*)material->diffuse.arrayOpenGL() );
    glMaterialfv( GL_FRONT, GL_SPECULAR, (GLfloat*)material->specular.arrayOpenGL() );
    glMaterialfv( GL_FRONT, GL_EMISSION, (GLfloat*)material->emissive.arrayOpenGL() );
    glMaterialf( GL_FRONT, GL_SHININESS, (GLfloat)material->shininess );

    glPushMatrix();

    glBegin( GL_POINTS );

    for( unsigned int vertex_id = 0; vertex_id < mesh->vertexCount( ); vertex_id++ ) {
        Vertex* v = mesh->vertex( vertex_id );
        glVertex3d( v->position.x(), v->position.y(), v->position.z() );
    }

    glEnd();

    glPopMatrix();
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

   GLfloat LightAmbient[] =  { 0.4f, 0.4f, 0.4f, 1.0f};
   GLfloat LightDiffuse[] =  { 0.3f, 0.3f, 0.3f, 1.0f};
   GLfloat LightSpecular[] = { 0.0f, 0.0f, 0.0f, 1.0f};
   GLfloat LightPosition[] = { 0.0f, -100.0f, 0.0f, 1.0f};

   glLightfv(GL_LIGHT0, GL_AMBIENT, LightAmbient);
   glLightfv(GL_LIGHT0, GL_DIFFUSE, LightDiffuse);
   glLightfv(GL_LIGHT0, GL_SPECULAR, LightSpecular);
   glLightfv(GL_LIGHT0, GL_POSITION, LightPosition);
   glEnable( GL_LIGHT0 );

   glLightModeli(GL_LIGHT_MODEL_LOCAL_VIEWER,1);

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

    glClear( GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT );

    draw_pointcloud( start, material1 );
    draw_pointcloud( end, material2 );

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
    case 27:    // esc
        exit( 0 );
        break;
    case 49:
        opt_draw_actor = !opt_draw_actor;
        break;
    case 50:
        opt_draw_trajectories = !opt_draw_trajectories;
        break;
    case 97:    // a
        scene.camera.track_left( PI/16 );
        break;
    case 100:   // d
        scene.camera.track_right( PI/16 );
        break;
    case 115:   // s
        scene.camera.dolly_out( 5.0 );
        break;
    case 119:   // w
        scene.camera.dolly_in( 5.0 );
        break;
    case 112:   // p
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
        sim_time += sim_time_step;
    }
    glutPostRedisplay( );
}

//------------------------------------------------------------------------------
/// Program entry point
/// Initialize then start rendering
int main( int argc, char** argv )
{
    initShapes( );
    initPatch( );

    // initialize OpenGL
    glutInit( &argc, argv );
    glutInitDisplayMode( GLUT_DOUBLE | GLUT_RGBA | GLUT_DEPTH | GLUT_ALPHA );
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



