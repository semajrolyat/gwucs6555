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

#define GENERATE_MOVIE  0
//#define GENERATE_MOVIE  1

//------------------------------------------------------------------------------

static int frame_id = 0;
int Width;
int Height;
unsigned char *Pixels;
std::string title;
std::string filetitle;
char filename[128];
bool sim_pause = false;

//------------------------------------------------------------------------------

Eigen::MatrixXd catmullrom_M;
std::vector<SdS> catmullrom_arclength_map;

Material catmullrom_material = Material(
            Color( 1.0, 0.0, 0.0, 1.0 ),
            Color( 1.0, 0.0, 0.0, 1.0 ),
            Color( 1.0, 0.0, 0.0, 1.0 ),
            Color( 0.0, 0.0, 0.0, 1.0 ),
            10                           );

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

Trajectory catmullrom_trajectory;
Trajectory bspline_trajectory;

double actor_position_on_spline = 0.0;
double actor_distance_travelled = 0.0;

//------------------------------------------------------------------------------
// Tests
//------------------------------------------------------------------------------

void cycleTest( void ) {
    title = "Cycle Test";
    filetitle = "cycle";

    actor.trajectory.construct_cyclic_circle_trajectory( 200 );

    camera.position = Vector3( 0.0, 0.0, 2.0 );
    camera.viewpoint = Vector3( 0.0, 0.0, 1.0 );

}

//------------------------------------------------------------------------------
// Spline Functions
//------------------------------------------------------------------------------

void draw_spline( const ECubicSplineBasis& basis ) {

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

    glPushMatrix();
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
    cycleTest();

    catmullrom_M = CubicSpline::basisCatmullRom();
    catmullrom_arclength_map = CubicSpline::map_spline( CUBIC_SPLINE_CATMULLROM, actor.trajectory.controlpoints );

    bspline_M = CubicSpline::basisUniformNonrationalBSpline();
    bspline_arclength_map = CubicSpline::map_spline( CUBIC_SPLINE_B, actor.trajectory.controlpoints );

    actor_position_on_spline = 0.0;
    actor_distance_travelled = 0.0;

    actor.trajectory.prev_keyframe_id = 0;
    actor.trajectory.next_keyframe_id = 1;

    actor.trajectory.spline_basis = CubicSpline::basisUniformNonrationalBSpline();
    actor.trajectory.spline_arclength_map = CubicSpline::map_spline( CUBIC_SPLINE_B, actor.trajectory.controlpoints );
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

    glutSolidSphere( 0.2, 32, 16 );

    glEnd( );

    glPopMatrix();
}

//------------------------------------------------------------------------------

void forward_interpolation( const bool& transition ) {
    Keyframe keyframe0 = actor.trajectory.keyframe( actor.trajectory.prev_keyframe_id );
    Keyframe keyframe1 = actor.trajectory.keyframe( actor.trajectory.next_keyframe_id );

    double frame0 = (double) keyframe0.frame;
    double frame1 = (double) keyframe1.frame;
    double dframe = frame1 - frame0;
    double u = 0.0;
    Eigen::MatrixXd C;

    SdS sds_frame0 = bspline_arclength_map.at( actor.trajectory.prev_keyframe_id + 1 );
    SdS sds_frame1 = bspline_arclength_map.at( actor.trajectory.next_keyframe_id + 1 );

    double dsdframe = sds_frame1.second / dframe;

    C = CubicSpline::blend( bspline_M, actor.trajectory.controlpoints, actor.trajectory.next_keyframe_id + 1 );
    if( transition ) {
        actor_position_on_spline = bspline_arclength_map.at( actor.trajectory.prev_keyframe_id + 1 ).first;
    } else {

        double length_of_current_spline = sds_frame1.second;
        double position_on_current_spline = actor_position_on_spline - sds_frame0.first + dsdframe;

        u = position_on_current_spline / length_of_current_spline;
        actor_position_on_spline += dsdframe;
    }
    actor_distance_travelled += dsdframe;
    actor.pose.position = CubicSpline::position( C, u );
    actor.pose.transformByQuaternion();
}

//------------------------------------------------------------------------------

unsigned int frame_tic = 0;

void interpolate_frame( void ) {

    frame_tic++;

    if( frame_tic == actor.trajectory.keyframe( actor.trajectory.keyframes() - 1 ).frame ) {
        actor_position_on_spline = 0.0;
        actor.trajectory.prev_keyframe_id = 0;
        actor.trajectory.next_keyframe_id = 1;
        frame_tic = 0;
    }

    bool transition = false;

    if( frame_tic == actor.trajectory.keyframe( actor.trajectory.next_keyframe_id ).frame ) {
        actor.trajectory.prev_keyframe_id++;
        actor.trajectory.next_keyframe_id++;
        transition = true;
    }

    forward_interpolation( transition );
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

   frame_tic = 0;
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

    glClearColor( 0.25, 0.25, 0.25, 0.0 );
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

    glLightfv(GL_LIGHT0, GL_AMBIENT, LightAmbient);
    glLightfv(GL_LIGHT0, GL_DIFFUSE, LightDiffuse);
    glLightfv(GL_LIGHT0, GL_SPECULAR, LightSpecular);
    glLightfv(GL_LIGHT0, GL_POSITION, LightPosition);
    glEnable(GL_LIGHT0);
    glShadeModel(GL_SMOOTH);

    draw_spline( CUBIC_SPLINE_B );
    draw_actor();

    glutSwapBuffers();

    if( GENERATE_MOVIE ) {
        sprintf( filename, "%s_%.04d.tif",filetitle.c_str(), frame_id );
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
    initActor();
    initCamera();

    // initialize OpenGL
    glutInit(&argc, argv);
    glutInitDisplayMode (GLUT_DOUBLE | GLUT_RGB |GLUT_DEPTH);
    glutInitWindowSize (600, 600);
    glutInitWindowPosition (100, 100);
    glutCreateWindow ( title.c_str() );

    init( );

    glutDisplayFunc(display);
    glutReshapeFunc(reshape);
    glutKeyboardFunc(keyboard);
    glutIdleFunc(idle);
    glutMainLoop();
    return 0;
}
//------------------------------------------------------------------------------
