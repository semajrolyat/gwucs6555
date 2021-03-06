/*-----------------------THE GEORGE WASHINGTON UNIVERSITY-----------------------
James Taylor                                                         jrt@gwu.edu

Shpes unit test

Used to validate the primitive shapes generated by MeshLoader including both
vertex geometry and surface normals.
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

Mesh* sphere;
Mesh* box;
Mesh* cone;
Mesh* cylinder;
Mesh* disc;

Mesh* shape;

Material* material;

Geometry* arrow;

void initShapes( void ) {
    title = "Shapes";
    filetitle = "shapes";

//    scene.camera.position = Vector3( 0.0, 7.0, 7.0 );
//    scene.camera.viewpoint = Vector3( 0.0, 6.0, 6.0 );
//    scene.camera.up = Vector3( 0.0, 0.0, 1.0 );

    scene.camera.position = Vector3( 0.0, -7.0, -7.0 );
    scene.camera.viewpoint = Vector3( 0.0, 0.0, 0.0 );
    scene.camera.up = Vector3( 0.0, 1.0, 0.0 );

    scene.camera.project();

    const double radius = 1.0;

    material = new Material();

    material->randomize();
    /*
    material->ambient = Color( 0.0, 0.5, 0.0, 1.0 );
    material->diffuse = Color( material->ambient.red(), material->ambient.green(), material->ambient.blue(), 1.0 );
    material->specular = Color( 0.0, 0.0, 0.0, 1.0 );
    */

    //box = MeshLoader::box( radius, radius, radius, material, ML_MATERIAL_ASSIGN );
    box = MeshLoader::box( radius, radius, radius );

    //sphere = MeshLoader::sphere( radius, 32, 15, material, ML_MATERIAL_ASSIGN );
    sphere = MeshLoader::sphere( 1.0, 10, 5, material, ML_MATERIAL_ASSIGN );

    cylinder = MeshLoader::cylinder( 0.3, 3.5, 5, material, ML_MATERIAL_ASSIGN );

    cone = MeshLoader::cone( 0.5, 1.5, 8, material, ML_MATERIAL_ASSIGN );

    /*
    arrow = new Geometry();
    arrow->insert( sphere );
    cylinder->pose.position.z( 2.5 );
    arrow->insert( cylinder );
    cone->pose.position.z( 4.0 );
    arrow->insert( cone );

    arrow->scale = Vector3( 0.1, 0.1, 0.1 );
    */
    arrow = GeometryMaker::arrow();
}

//#define BACKFACE_CULL   0
#define BACKFACE_CULL   1

//------------------------------------------------------------------------------
// Drawing Operations
//------------------------------------------------------------------------------
/// Draws a mesh
void draw( Mesh* mesh, Matrix4& transform ) {
    Matrix3 R = transform.rotation();
    // backface cull
    Vector3 view = scene.camera.position -  scene.camera.viewpoint;

    for( unsigned int poly_id = 0; poly_id < mesh->polygonCount( ); poly_id++ ) {
        Polygon* poly = mesh->polygon( poly_id );
        Vector3 normal = R * poly->normal;

        if( Vector3::dot( view, normal ) >= 0.0 ) {
            Vertex* v = mesh->vertex( poly->getVertex( 0 ) );
            Vector3 pt = Vector3( v->position.x(), v->position.y(), v->position.z() );
            if( Vector3::dot( scene.camera.position - pt, normal ) >= 0.0 )
                poly->backface = true;
            else
                poly->backface = false;
        } else {
            poly->backface = false;
        }
    }

    glLoadMatrixd( transform.arrayOpenGL() );

    glBegin( GL_TRIANGLES );

    for( unsigned int poly_id = 0; poly_id < mesh->polygonCount( ); poly_id++ ) {

        // Select the current polygon
        Polygon* poly = mesh->polygon( poly_id );

        if(BACKFACE_CULL)
            if( poly->backface ) continue;

        /*
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
        */

        glColor3d( 0.6, 0.0, 0.0 );

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
            glNormal3d( v0->normal.x(), v0->normal.y(), v0->normal.z() );
            //glNormal3d( poly->normal.x(), poly->normal.y(), poly->normal.z() );

            glVertex3d( v1->position.x(), v1->position.y(), v1->position.z() );
            glNormal3d( v1->normal.x(), v1->normal.y(), v1->normal.z() );
            //glNormal3d( poly->normal.x(), poly->normal.y(), poly->normal.z() );

            glVertex3d( v2->position.x(), v2->position.y(), v2->position.z() );
            glNormal3d( v2->normal.x(), v2->normal.y(), v2->normal.z() );
            //glNormal3d( poly->normal.x(), poly->normal.y(), poly->normal.z() );
        }
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
   glShadeModel( GL_SMOOTH );
   //glShadeModel( GL_FLAT );
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

   glLightModeli( GL_LIGHT_MODEL_LOCAL_VIEWER, GL_TRUE );

   glEnable( GL_COLOR_MATERIAL );
   //glColorMaterial( GL_FRONT, GL_AMBIENT_AND_DIFFUSE );
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

    Matrix4 scaling_matrix, transform;
    if( arrow->scale != Vector3( 1.0, 1.0, 1.0 ) ) {
        scaling_matrix = Matrix4::scalingMatrix( arrow->scale.x(), arrow->scale.y(), arrow->scale.z() );
    }

    for( unsigned int i = 0; i < arrow->geometryCount(); i++ ) {
        Geometry* g = arrow->geometry( i );
        Mesh* m = static_cast<Mesh*>(g);
        glPushMatrix();

        m->pose.transformByQuaternion();
        transform = scaling_matrix * m->pose.transform;
        draw( m, transform );

        glPopMatrix();
    }

    // draw normals
    glLineWidth( 1 );

    Material material;
    glMaterialfv(GL_FRONT, GL_AMBIENT, (GLfloat*)material.ambient.arrayOpenGL());
    glMaterialfv(GL_FRONT, GL_DIFFUSE, (GLfloat*)material.diffuse.arrayOpenGL());
    glMaterialfv(GL_FRONT, GL_SPECULAR, (GLfloat*)material.specular.arrayOpenGL());
    glMaterialfv(GL_FRONT, GL_EMISSION, (GLfloat*)material.emissive.arrayOpenGL());
    glMaterialf(GL_FRONT, GL_SHININESS, (GLfloat)material.shininess);

    for( unsigned int j = 0; j < arrow->geometryCount(); j++ ) {
        Geometry* g = arrow->geometry( j );
        Mesh* m = static_cast<Mesh*>(g);
        m->pose.transformByQuaternion();
        transform = scaling_matrix * m->pose.transform;
        glLoadMatrixd( transform.arrayOpenGL() );

        glBegin(GL_LINES);

        for( unsigned int i = 0; i < m->vertexCount(); i++ ) {
            Vertex* v = m->vertex( i );
            Vector3 pt1 = Vector3( v->position.x(), v->position.y(), v->position.z() );
            Vector3 pt2 = pt1 + v->normal;

            glVertex3d( pt1.x(), pt1.y(), pt1.z() );
            glVertex3d( pt2.x(), pt2.y(), pt2.z() );
        }
        glEnd();
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

