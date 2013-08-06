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

#include <cs6555/Actor.h>
#include <cs6555/Mesh.h>
#include <cs6555/MeshLoader.h>

#include <stdexcept>

#include <cs6555/Color.h>
#include <cs6555/Material.h>

#include <cs6555/tiffio.h>

//------------------------------------------------------------------------------

#define CATMULL_ROM     0
#define B_SPLINE        1

//------------------------------------------------------------------------------

#define ROTFUN_EULERANGLES    0
#define ROTFUN_QUATERNION     1

//------------------------------------------------------------------------------

#define TRAJECTORY_LINE             0
#define TRAJECTORY_CIRCLE           1
#define TRAJECTORY_SINE             2
#define TRAJECTORY_DBLSINE          3
#define TRAJECTORY_IMMELMANN        4
#define TRAJECTORY_DBLIMMELMANN     5
#define TRAJECTORY_LOOP             6

//------------------------------------------------------------------------------

//#define DRAW_ACTOR      0
#define DRAW_ACTOR      1

#define ROTFUN    ROTFUN_EULERANGLES
//#define ROTFUN    ROTFUN_QUATERNION

#define SPLINE_BASIS    CATMULL_ROM
//#define SPLINE_BASIS    B_SPLINE

//#define TRAJECTORY      TRAJECTORY_LINE
//#define TRAJECTORY      TRAJECTORY_CIRCLE
//#define TRAJECTORY      TRAJECTORY_SINE
//#define TRAJECTORY      TRAJECTORY_DBLSINE
//#define TRAJECTORY      TRAJECTORY_IMMELMANN
#define TRAJECTORY      TRAJECTORY_DBLIMMELMANN
//#define TRAJECTORY      TRAJECTORY_LOOP

//------------------------------------------------------------------------------

#define GENERATE_MOVIE  0
//#define GENERATE_MOVIE  1

//------------------------------------------------------------------------------

static unsigned int frame_id = 0;
int Width;
int Height;
unsigned char *Pixels;
std::string title;
std::string filetitle;
char filename[128];
bool sim_pause = false;

//------------------------------------------------------------------------------

double catmullrom_s;
Eigen::MatrixXd catmullrom_M;
std::vector<SdS> catmullrom_arclength_map;

Material catmullrom_material = Material(
            Color( 1.0, 0.0, 0.0, 1.0 ),
            Color( 1.0, 0.0, 0.0, 1.0 ),
            Color( 1.0, 0.0, 0.0, 1.0 ),
            Color( 0.0, 0.0, 0.0, 1.0 ),
            10                           );

double bspline_s;
Eigen::MatrixXd bspline_M;
std::vector<SdS> bspline_arclength_map;

Material bspline_material = Material(
            Color( 1.0, 1.0, 0.0, 1.0 ),
            Color( 1.0, 1.0, 0.0, 1.0 ),
            Color( 1.0, 1.0, 0.0, 1.0 ),
            Color( 0.0, 0.0, 0.0, 1.0 ),
            10                           );

double line_width = 8.0;

Camera camera;
Actor actor;

Material actor_material = Material(
            Color( 1.0, 1.0, 1.0, 1.0 ),
            Color( 1.0, 1.0, 1.0, 1.0 ),
            Color( 1.0, 1.0, 1.0, 1.0 ),
            Color( 0.0, 0.0, 0.0, 1.0 ),
            10                           );

double actor_ds = 0.0;      // distance travelled by actor

Trajectory catmullrom_trajectory;
Trajectory bspline_trajectory;

//------------------------------------------------------------------------------
// Tests
//------------------------------------------------------------------------------

void lineTest( void ) {
    title = "Line Test";
    filetitle = "line";

    actor.trajectory.construct_line_trajectory();

    camera.position = Vector3( 0.0, 0.0, 4.0 );
    camera.viewpoint = Vector3( 0.0, 0.0, 3.0 );

}

//------------------------------------------------------------------------------

void circleTest( void ) {
    title = "Circle Test";
    filetitle = "circle";

    actor.trajectory.construct_circle_trajectory();

    camera.position = Vector3( 0.0, 0.0, 3.0 );
    camera.viewpoint = Vector3( 0.0, 0.0, 2.0 );

}

//------------------------------------------------------------------------------

void sineTest( void ) {
    title = "Sine Test";
    filetitle = "sine";

    actor.trajectory.construct_sine_trajectory();

    camera.position = Vector3( PI, 0.0, 6.0 );
    camera.viewpoint = Vector3( PI, 0.0, 5.0 );

}

//------------------------------------------------------------------------------

void compressedsineandsineTest( void ) {
    title = "Compressed Sine And Sine Test";
    filetitle = "dblsine";

    actor.trajectory.construct_multisine_trajectory();

    camera.position = Vector3( (PI+PI/2.0), 0.0, 8.0 );
    camera.viewpoint = Vector3( (PI+PI/2.0), 0.0, 7.0 );

}

//------------------------------------------------------------------------------

void immelmannTest( void ) {
    title = "Immelmann Test";
    filetitle = "immelmann";

    actor.trajectory.construct_immelmann_trajectory();

    camera.position = Vector3( -0.5, 0.0, 3.0 );
    camera.viewpoint = Vector3( -0.5, 0.0, 2.0 );

}

//------------------------------------------------------------------------------

void dblimmelmannTest( void ) {
    title = "Double Immelmann Test";
    filetitle = "dblimmelmann";

    actor.trajectory.construct_doubleimmelmann_trajectory();

    camera.position = Vector3( 0.0, 1.0, 4.0 );
    camera.viewpoint = Vector3( 0.0, 1.0, 3.0 );

}

//------------------------------------------------------------------------------

void loopTest( void ) {
    title = "Loop Test";
    filetitle = "loop";

    actor.trajectory.construct_loop_trajectory();

    camera.position = Vector3( 0.0, 0.0, 3.0 );
    camera.viewpoint = Vector3( 0.0, 0.0, 2.0 );

}

//------------------------------------------------------------------------------
// Spline Functions
//------------------------------------------------------------------------------
void draw_spline( const ECubicSplineBasis& basis ) {

    glPushMatrix();

    Eigen::MatrixXd M;
    GLfloat* material_Ka;
    GLfloat* material_Kd;
    GLfloat* material_Ks;
    GLfloat* material_Ke;
    GLfloat material_Se;

    switch( basis ) {
    case CUBIC_SPLINE_CATMULLROM:
    default:
        material_Ka = (GLfloat*)catmullrom_material.ambient.arrayOpenGL();
        material_Kd = (GLfloat*)catmullrom_material.diffuse.arrayOpenGL();
        material_Ks = (GLfloat*)catmullrom_material.specular.arrayOpenGL();
        material_Ke = (GLfloat*)catmullrom_material.emissive.arrayOpenGL();
        material_Se = (GLfloat)catmullrom_material.shininess;

        M = catmullrom_M;
        break;
    case CUBIC_SPLINE_B:
        material_Ka = (GLfloat*)bspline_material.ambient.arrayOpenGL();
        material_Kd = (GLfloat*)bspline_material.diffuse.arrayOpenGL();
        material_Ks = (GLfloat*)bspline_material.specular.arrayOpenGL();
        material_Ke = (GLfloat*)bspline_material.emissive.arrayOpenGL();
        material_Se = (GLfloat)bspline_material.shininess;

        M = bspline_M;
        break;
    }

    glMaterialfv(GL_FRONT, GL_AMBIENT, material_Ka);
    glMaterialfv(GL_FRONT, GL_DIFFUSE, material_Kd);
    glMaterialfv(GL_FRONT, GL_SPECULAR, material_Ks);
    glMaterialfv(GL_FRONT, GL_EMISSION, material_Ke);
    glMaterialf(GL_FRONT, GL_SHININESS, material_Se);

    Vector3 vi, vi1;
    unsigned int n = actor.trajectory.controlpoints.size();

    glLineWidth( line_width );

    glBegin(GL_LINES);

    for( unsigned int i = 2; i < n-1; i++ ) {
        Eigen::MatrixXd C = CubicSpline::blend( M, actor.trajectory.controlpoints, i );

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
// Camera Functions
//------------------------------------------------------------------------------
void initCamera( void ) {
    camera.up = Vector3( 0.0, 1.0, 0.0 );
    camera.project();
}

//------------------------------------------------------------------------------
// Actor Functions
//------------------------------------------------------------------------------

void initActor( void ) {
    actor_ds = 0.0;
    actor.trajectory.prev_keyframe_id = 0;
    actor.trajectory.next_keyframe_id = 1;

    Mesh* mesh;
    mesh = MeshLoader::load( "biplane.d", MeshLoader::FormatD );
    if( mesh == NULL )
        throw std::runtime_error( "Failed to load mesh" );
    actor.insert( mesh );
}

//------------------------------------------------------------------------------

void draw_actor( void ) {
    GLfloat* material_Ka = (GLfloat*)actor_material.ambient.arrayOpenGL();
    GLfloat* material_Kd = (GLfloat*)actor_material.diffuse.arrayOpenGL();
    GLfloat* material_Ks = (GLfloat*)actor_material.specular.arrayOpenGL();
    GLfloat* material_Ke = (GLfloat*)actor_material.emissive.arrayOpenGL();
    GLfloat material_Se = (GLfloat)actor_material.shininess;

    glPushMatrix();
    glMaterialfv(GL_FRONT, GL_AMBIENT, material_Ka);
    glMaterialfv(GL_FRONT, GL_DIFFUSE, material_Kd);
    glMaterialfv(GL_FRONT, GL_SPECULAR, material_Ks);
    glMaterialfv(GL_FRONT, GL_EMISSION, material_Ke);
    glMaterialf(GL_FRONT, GL_SHININESS, material_Se);

    glLoadMatrixd( actor.pose.transform.arrayOpenGL() );

    glBegin( GL_TRIANGLES );

    for( unsigned int geometry_id = 0; geometry_id < actor.geometryCount( ); geometry_id++ ) {
        // dangerous because of assumption that is a mesh.  Will revisit when more
        // drawable types are included
        Mesh* mesh = static_cast<Mesh*>( actor.geometry( geometry_id ) );
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
    }
    glEnd( );

    glPopMatrix();

}
//------------------------------------------------------------------------------

void interpolate_frame( void ) {

    // if the last keyframe is passed, then don't interpolate any more
    if( actor.trajectory.next_keyframe_id == actor.trajectory.keyframes() ) return;

    bool transition = false;
    // if the frame has reached a keyframe boundary, advance to next keyframe
    if( frame_id == actor.trajectory.keyframe( actor.trajectory.next_keyframe_id ).frame ) {
        actor.trajectory.prev_keyframe_id++;
        actor.trajectory.next_keyframe_id++;
        transition = true;
    }

    // special case when its the transition to the last keyframe
    if( actor.trajectory.next_keyframe_id == actor.trajectory.keyframes() ) return;

    // select the keyframe
    Keyframe keyframe0 = actor.trajectory.keyframe( actor.trajectory.prev_keyframe_id );
    Keyframe keyframe1 = actor.trajectory.keyframe( actor.trajectory.next_keyframe_id );

    // convert frame number to time
    double t0 = (double) keyframe0.frame;
    double t1 = (double) keyframe1.frame;

    // change in time between frames (for angular calculations)
    double dt = ((double)frame_id - t0) / ( t1 - t0 );

    // when at a transition simply use that pose
    // otherwise integrate
    if( transition ) {
        // reference the current pose defined by the keyframe
        actor.pose.orientation = keyframe0.pose.orientation;
        Eigen::MatrixXd C;
        switch( SPLINE_BASIS ) {
        case CATMULL_ROM:
        default:
            C = CubicSpline::blend( catmullrom_M, actor.trajectory.controlpoints, actor.trajectory.next_keyframe_id + 1 );
            actor_ds = catmullrom_arclength_map.at( actor.trajectory.prev_keyframe_id + 1 ).first;
            break;
        case B_SPLINE:
            C = CubicSpline::blend( bspline_M, actor.trajectory.controlpoints, actor.trajectory.next_keyframe_id + 1 );
            actor_ds = bspline_arclength_map.at( actor.trajectory.prev_keyframe_id + 1 ).first;
            break;
        }
        actor.pose.position = CubicSpline::position( C, 0.0 );
    } else {
        // integration

        // integrate rotation
        double drotxdt = keyframe1.pose.orientation.x() - keyframe0.pose.orientation.x();
        double drotydt = keyframe1.pose.orientation.y() - keyframe0.pose.orientation.y();
        double drotzdt = keyframe1.pose.orientation.z() - keyframe0.pose.orientation.z();

        actor.pose.orientation.x( keyframe0.pose.orientation.x() + drotxdt * dt );
        actor.pose.orientation.y( keyframe0.pose.orientation.y() + drotydt * dt );
        actor.pose.orientation.z( keyframe0.pose.orientation.z() + drotzdt * dt );

        // integrate position based upon interpolation along the arc
        SdS sds_frame0, sds_frame1;

        switch( SPLINE_BASIS ) {
        case CATMULL_ROM:
        default:
            sds_frame0 = catmullrom_arclength_map.at( actor.trajectory.prev_keyframe_id + 1 );
            sds_frame1 = catmullrom_arclength_map.at( actor.trajectory.next_keyframe_id + 1 );
            break;
        case B_SPLINE:
            sds_frame0 = bspline_arclength_map.at( actor.trajectory.prev_keyframe_id + 1 );
            sds_frame1 = bspline_arclength_map.at( actor.trajectory.next_keyframe_id + 1 );
            break;
        }

        double s_frame0 = sds_frame0.first;
        double dsdt = 1 / ( t1 - t0 );
        Eigen::MatrixXd C;

        switch( SPLINE_BASIS ) {
        case CATMULL_ROM:
        default:
            C = CubicSpline::blend( catmullrom_M, actor.trajectory.controlpoints, actor.trajectory.next_keyframe_id + 1 );
            break;
        case B_SPLINE:
            C = CubicSpline::blend( bspline_M, actor.trajectory.controlpoints, actor.trajectory.next_keyframe_id + 1 );
            break;
        }

        // calculate the new u
        double u = actor_ds - s_frame0 + dsdt;
        actor_ds += dsdt;
        actor.pose.position = CubicSpline::position( C, u );
    }

    switch( ROTFUN ) {
    case ROTFUN_EULERANGLES:
        actor.pose.transformByEulerAngle();
        break;
    case ROTFUN_QUATERNION:
    default:
        actor.pose.transformByQuaternion();
        break;
    }

    // used to shrink the oversized model to a compatible size
    Matrix4 actor_scale = Matrix4::scalingMatrix( 0.1, 0.1, 0.1 );
    actor.pose.transform = actor.pose.transform * actor_scale;
}

//------------------------------------------------------------------------------
// Modified SimpleGLUT
//------------------------------------------------------------------------------

int writetiff(const char *f_name, const char *description,
  int x, int y, int width, int height, int compression)
{
  TIFF *file;
  GLubyte *image, *p;
  int i;

  file = TIFFOpen(f_name, "w");
  if (file == NULL) {
    return 1;
  }
  //image = (GLubyte *) malloc(width * height * sizeof(GLubyte) * 3);
  image = new GLubyte[width * height * sizeof(GLubyte) * 3];
  glPixelStorei(GL_PACK_ALIGNMENT, 1);

  glReadPixels(x, y, width, height, GL_RGB, GL_UNSIGNED_BYTE, image);
  TIFFSetField(file, TIFFTAG_IMAGEWIDTH, (uint32) width);
  TIFFSetField(file, TIFFTAG_IMAGELENGTH, (uint32) height);
  TIFFSetField(file, TIFFTAG_BITSPERSAMPLE, 8);
  TIFFSetField(file, TIFFTAG_COMPRESSION, compression);
  TIFFSetField(file, TIFFTAG_PHOTOMETRIC, PHOTOMETRIC_RGB);
  TIFFSetField(file, TIFFTAG_SAMPLESPERPIXEL, 3);
  TIFFSetField(file, TIFFTAG_PLANARCONFIG, PLANARCONFIG_CONTIG);
  TIFFSetField(file, TIFFTAG_ROWSPERSTRIP, 1);
  TIFFSetField(file, TIFFTAG_IMAGEDESCRIPTION, description);
  p = image;
  for (i = height - 1; i >= 0; i--) {
    if (TIFFWriteScanline(file, p, i, 0) < 0) {
      //free(image);
        delete [] image;
      TIFFClose(file);
      return 1;
    }
    p += width * sizeof(GLubyte) * 3;
  }
  TIFFClose(file);
  return 0;
}

//------------------------------------------------------------------------------
void init(void)
{

   glClearColor (0.0, 0.0, 0.0, 0.0);
   glClearDepth (1.0);
   glShadeModel (GL_SMOOTH);
   glEnable( GL_LINE_SMOOTH );

   initCamera();
   initActor();
}

//------------------------------------------------------------------------------
void display(void)
{
    glClear (GL_COLOR_BUFFER_BIT);
    glEnable(GL_LIGHTING);
    glEnable(GL_DEPTH_TEST);

    GLfloat LightAmbient[] =  { 0.4f, 0.4f, 0.4f, 1.0f};
    GLfloat LightDiffuse[] =  { 0.3f, 0.3f, 0.3f, 1.0f};
    GLfloat LightSpecular[] = { 0.4f, 0.4f, 0.4f, 1.0f};
    GLfloat LightPosition[] = { 5.0f, 5.0f, 5.0f, 1.0f};

    //glClearColor(0.0,0.0,0.0,0.0);
    glClearColor( 0.25, 0.25, 0.25, 0.0 );
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

    glLightfv(GL_LIGHT0, GL_AMBIENT, LightAmbient);
    glLightfv(GL_LIGHT0, GL_DIFFUSE, LightDiffuse);
    glLightfv(GL_LIGHT0, GL_SPECULAR, LightSpecular);
    glLightfv(GL_LIGHT0, GL_POSITION, LightPosition);
    glEnable(GL_LIGHT0);
    glShadeModel(GL_SMOOTH);

    draw_spline( CUBIC_SPLINE_CATMULLROM );
    draw_spline( CUBIC_SPLINE_B );
    if( DRAW_ACTOR )
        draw_actor();

    glutSwapBuffers();

    if( GENERATE_MOVIE ) {
        std::string ssplinebasis;
        switch( SPLINE_BASIS ) {
        case CATMULL_ROM:
            ssplinebasis = "crom";
            break;
        case B_SPLINE:
        default:
            ssplinebasis = "bspl";
            break;
        }

        std::string srotfun;
        switch( ROTFUN ) {
        case ROTFUN_EULERANGLES:
            srotfun = "euler";
            break;
        case ROTFUN_QUATERNION:
        default:
            srotfun = "quat";
            break;
        }


        sprintf( filename, "%s_%s_%s_%.04d.tif",filetitle.c_str(), ssplinebasis.c_str(), srotfun.c_str(), frame_id );
        printf( "%s\n", filename );
        writetiff( filename, "movie", 0, 0, Width, Height, COMPRESSION_NONE );
    }

}
//------------------------------------------------------------------------------
void reshape (int w, int h)
{
    glViewport (0, 0, (GLsizei) w, (GLsizei) h);
    glMatrixMode (GL_PROJECTION);
    glLoadIdentity ();
    glLoadMatrixd( camera.projection.arrayOpenGL() );

    glMatrixMode(GL_MODELVIEW);
    glLoadIdentity();

    Width = w;
    Height = h;

}
//------------------------------------------------------------------------------
void keyboard (unsigned char key, int x, int y)
{
    switch (key) {
    case 27:
        exit(0);
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
void idle(void)
{
    if( !sim_pause ) {
        frame_id++;
        interpolate_frame();
    }
    glutPostRedisplay();
}

//------------------------------------------------------------------------------
int main(int argc, char** argv)
{
    switch( TRAJECTORY ) {
    case TRAJECTORY_LINE:
    default:
        lineTest();
        break;
    case TRAJECTORY_CIRCLE:
        circleTest();
        break;
    case TRAJECTORY_SINE:
        sineTest();
        break;
    case TRAJECTORY_DBLSINE:
        compressedsineandsineTest();
        break;
    case TRAJECTORY_IMMELMANN:
        immelmannTest();
        break;
    case TRAJECTORY_DBLIMMELMANN:
        dblimmelmannTest();
        break;
    case TRAJECTORY_LOOP:
        loopTest();
        break;
    }

    glutInit(&argc, argv);
    glutInitDisplayMode (GLUT_DOUBLE | GLUT_RGB |GLUT_DEPTH);
    glutInitWindowSize (600, 600);
    glutInitWindowPosition (100, 100);
    glutCreateWindow ( title.c_str() );

    catmullrom_s = CubicSpline::linearly_interpolate_arclength( CUBIC_SPLINE_CATMULLROM, actor.trajectory.controlpoints );
    catmullrom_M = CubicSpline::basisCatmullRom();
    catmullrom_arclength_map = CubicSpline::map_spline( CUBIC_SPLINE_CATMULLROM, actor.trajectory.controlpoints );
    std::cout << "catmullrom arclength s:" << catmullrom_s << std::endl;

    bspline_s = CubicSpline::linearly_interpolate_arclength( CUBIC_SPLINE_B, actor.trajectory.controlpoints );
    bspline_M = CubicSpline::basisUniformNonrationalBSpline();
    bspline_arclength_map = CubicSpline::map_spline( CUBIC_SPLINE_B, actor.trajectory.controlpoints );
    std::cout << "bspline arclength s:" << bspline_s << std::endl;

    camera.project();

    init ();
    glutDisplayFunc(display);
    glutReshapeFunc(reshape);
    glutKeyboardFunc(keyboard);
    glutIdleFunc(idle);
    glutMainLoop();
    return 0;
}

//------------------------------------------------------------------------------
