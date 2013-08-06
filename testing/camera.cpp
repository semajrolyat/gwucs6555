#include <iostream>

#include <cs6555/Math/CubicSpline.h>
#include <cs6555/Constants.h>

#include <GL/glut.h>
#include <stdio.h>

#include <string>

#include <cs6555/Math/Vector3.h>
#include <cs6555/Math/Matrix4.h>

#include <cs6555/Camera.h>

//------------------------------------------------------------------------------

double catmullrom_s;
Eigen::MatrixXd catmullrom_M;
std::vector<SdS> catmullrom_arclength_map;

Vector3 catmullrom_Ka = Vector3( 1.0, 0.0, 0.0 );
Vector3 catmullrom_Kd = Vector3( 1.0, 0.0, 0.0 );
Vector3 catmullrom_Ks = Vector3( 1.0, 0.0, 0.0 );
Vector3 catmullrom_Ke = Vector3( 0.0, 0.0, 0.0 );

double bspline_s;
Eigen::MatrixXd bspline_M;
std::vector<SdS> bspline_arclength_map;

Vector3 bspline_Ka = Vector3( 0.0, 1.0, 0.0 );
Vector3 bspline_Kd = Vector3( 0.0, 1.0, 0.0 );
Vector3 bspline_Ks = Vector3( 0.0, 1.0, 0.0 );
Vector3 bspline_Ke = Vector3( 0.0, 0.0, 0.0 );

ControlPointList cplist;
Vector3 camera_position;
double line_width = 8.0;
std::string title = "Camera Test";

Camera camera;


//------------------------------------------------------------------------------
// Camera Functions
//------------------------------------------------------------------------------

void initCamera( void ) {

    camera.position = Vector3( 0.0, 0.0, 10.0 );
    camera.viewpoint = Vector3( 0.0, 0.0, 9.0 );
    camera.up = Vector3( 0.0, 1.0, 0.0 );
    camera.project();
}

//------------------------------------------------------------------------------
// Actor Functions
//------------------------------------------------------------------------------

Vector3 actor_Ka = Vector3( 0.0, 0.0, 1.0 );
Vector3 actor_Kd = Vector3( 0.0, 0.0, 1.0 );
Vector3 actor_Ks = Vector3( 0.0, 0.0, 1.0 );
Vector3 actor_Ke = Vector3( 0.0, 0.0, 0.0 );

Matrix4 actor_transform;

//------------------------------------------------------------------------------

void initActor( void ) {
    actor_transform = Matrix4( 1.0,   0.0,   0.0,   5.0,
                               0.0,   1.0,   0.0,   0.0,
                               0.0,   0.0,   1.0,   0.0,
                               0.0,   0.0,   0.0,   1.0 );

}

//------------------------------------------------------------------------------

void draw_actor( void ) {
    GLfloat material_Ka[4] = { actor_Ka(0), actor_Ka(1), actor_Ka(2), 1.0 };
    GLfloat material_Kd[4] = { actor_Kd(0), actor_Kd(1), actor_Kd(2), 1.0 };
    GLfloat material_Ks[4] = { actor_Ks(0), actor_Ks(1), actor_Ks(2), 1.0 };
    GLfloat material_Ke[4] = { actor_Ke(0), actor_Ke(1), actor_Ke(2), 1.0 };
    GLfloat material_Se   = 10;

    glMaterialfv(GL_FRONT, GL_AMBIENT, material_Ka);
    glMaterialfv(GL_FRONT, GL_DIFFUSE, material_Kd);
    glMaterialfv(GL_FRONT, GL_SPECULAR, material_Ks);
    glMaterialfv(GL_FRONT, GL_EMISSION, material_Ke);
    glMaterialf(GL_FRONT, GL_SHININESS, material_Se);

    glPushMatrix();

    glLoadMatrixd( actor_transform.arrayOpenGL() );

    glutSolidTeapot(1.0);

    glPopMatrix();

}

//------------------------------------------------------------------------------
// Modified SimpleGLUT
//------------------------------------------------------------------------------
int Width;
int Height;
unsigned char *Pixels;

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

    glClearColor(0.0,0.0,0.0,0.0);
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

    glLightfv(GL_LIGHT0, GL_AMBIENT, LightAmbient);
    glLightfv(GL_LIGHT0, GL_DIFFUSE, LightDiffuse);
    glLightfv(GL_LIGHT0, GL_SPECULAR, LightSpecular);
    glLightfv(GL_LIGHT0, GL_POSITION, LightPosition);
    glEnable(GL_LIGHT0);
    glShadeModel(GL_SMOOTH);

    draw_actor();

    glutSwapBuffers();

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
      default:
         break;
   }
}
//------------------------------------------------------------------------------
void idle(void)
{
    glutPostRedisplay();
}

//------------------------------------------------------------------------------
int main(int argc, char** argv)
{
   glutInit(&argc, argv);
   glutInitDisplayMode (GLUT_DOUBLE | GLUT_RGB |GLUT_DEPTH);
   glutInitWindowSize (600, 600);
   glutInitWindowPosition (100, 100);
   glutCreateWindow ( title.c_str() );
   init ();
   glutDisplayFunc(display);
   glutReshapeFunc(reshape);
   glutKeyboardFunc(keyboard);
   glutIdleFunc(idle);
   glutMainLoop();
   return 0;
}

//------------------------------------------------------------------------------


