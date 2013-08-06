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
std::string title;

Camera camera;

//------------------------------------------------------------------------------
// Tests
//------------------------------------------------------------------------------

void lineTest( void ) {
    title = "Line Test";

    ControlPoint cp;

    cp = ControlPoint( Vector3(-5.0, 0.0, 0.0), CONTROL_POINT_END );
    cplist.push_back(cp);

    cp = ControlPoint( Vector3(-5.0, 0.0, 0.0), CONTROL_POINT_MIDDLE );
    cplist.push_back(cp);

    cp = ControlPoint( Vector3(0.0, 0.0, 0.0), CONTROL_POINT_MIDDLE );
    cplist.push_back(cp);

    cp = ControlPoint( Vector3(5.0, 0.0, 0.0), CONTROL_POINT_MIDDLE );
    cplist.push_back(cp);

    cp = ControlPoint( Vector3(5.0, 0.0, 0.0), CONTROL_POINT_END );
    cplist.push_back(cp);

    camera.position = Vector3( 0.0, 0.0, 10.0 );
    camera.viewpoint = Vector3( 0.0, 0.0, 9.0 );

}

//------------------------------------------------------------------------------

void circleTest( void ) {
    title = "Circle Test";

    ControlPoint cp;

    cp = ControlPoint( Vector3(0.0, -1.0, 0.0), CONTROL_POINT_END );
    cplist.push_back(cp);

    cp = ControlPoint( Vector3(0.0, -1.0, 0.0), CONTROL_POINT_MIDDLE );
    cplist.push_back(cp);

    cp = ControlPoint( Vector3(cos(PI/4), -sin(PI/4), 0.0), CONTROL_POINT_MIDDLE );
    cplist.push_back(cp);

    cp = ControlPoint( Vector3(1.0, 0.0, 0.0), CONTROL_POINT_MIDDLE );
    cplist.push_back(cp);

    cp = ControlPoint( Vector3(cos(PI/4), sin(PI/4), 0.0), CONTROL_POINT_MIDDLE );
    cplist.push_back(cp);

    cp = ControlPoint( Vector3(0.0, 1.0, 0.0), CONTROL_POINT_MIDDLE );
    cplist.push_back(cp);

    cp = ControlPoint( Vector3(-cos(PI/4), sin(PI/4), 0.0), CONTROL_POINT_MIDDLE );
    cplist.push_back(cp);

    cp = ControlPoint( Vector3(-1.0, 0.0, 0.0), CONTROL_POINT_MIDDLE );
    cplist.push_back(cp);

    cp = ControlPoint( Vector3(-cos(PI/4), -sin(PI/4), 0.0), CONTROL_POINT_MIDDLE );
    cplist.push_back(cp);

    cp = ControlPoint( Vector3(0.0,-1.0, 0.0), CONTROL_POINT_MIDDLE );
    cplist.push_back(cp);

    cp = ControlPoint( Vector3(0.0,-1.0, 0.0), CONTROL_POINT_END );
    cplist.push_back(cp);

    camera.position = Vector3( 0.0, 0.0, 3.0 );
    camera.viewpoint = Vector3( 0.0, 0.0, 2.0 );

}

//------------------------------------------------------------------------------

void sineTest( void ) {
    title = "Sine Test";

    ControlPoint cp;

    cp = ControlPoint( Vector3(0.0, 0.0, 0.0), CONTROL_POINT_END );
    cplist.push_back(cp);

    cp = ControlPoint( Vector3(0.0, 0.0, 0.0), CONTROL_POINT_MIDDLE );
    cplist.push_back(cp);

    cp = ControlPoint( Vector3(PI/2.0, 1.0, 0.0), CONTROL_POINT_MIDDLE );
    cplist.push_back(cp);

    cp = ControlPoint( Vector3(PI*3.0/2.0, -1.0, 0.0), CONTROL_POINT_MIDDLE );
    cplist.push_back(cp);

    cp = ControlPoint( Vector3(2*PI, 0.0, 0.0), CONTROL_POINT_MIDDLE );
    cplist.push_back(cp);

    cp = ControlPoint( Vector3(2*PI, 0.0, 0.0), CONTROL_POINT_END );
    cplist.push_back(cp);

    camera.position = Vector3( PI, 0.0, 6.0 );
    camera.viewpoint = Vector3( PI, 0.0, 5.0 );

}

//------------------------------------------------------------------------------

void compressedsineandsineTest( void ) {
    title = "Compressed Sine And Sine Test";

    ControlPoint cp;

    cp = ControlPoint( Vector3(0.0, 0.0, 0.0), CONTROL_POINT_END );
    cplist.push_back(cp);

    // compressed amplitude sine

    cp = ControlPoint( Vector3(0.0, 0.0, 0.0), CONTROL_POINT_MIDDLE );
    cplist.push_back(cp);

    cp = ControlPoint( Vector3(PI/4.0, 1.0, 0.0), CONTROL_POINT_MIDDLE );
    cplist.push_back(cp);

    cp = ControlPoint( Vector3(PI*3/4.0, -1.0, 0.0), CONTROL_POINT_MIDDLE );
    cplist.push_back(cp);

    // remarked for better continuity over both catmullrom and bspline
    //cp = ControlPoint( Vector3(PI, 0.0, 0.0), CONTROL_POINT_MIDDLE );
    //cplist.push_back(cp);

    // normal amplitude sine

    cp = ControlPoint( Vector3(PI+PI/2.0, 1.0, 0.0), CONTROL_POINT_MIDDLE );
    cplist.push_back(cp);

    cp = ControlPoint( Vector3(PI+PI*3.0/2.0, -1.0, 0.0), CONTROL_POINT_MIDDLE );
    cplist.push_back(cp);

    cp = ControlPoint( Vector3(PI+2*PI, 0.0, 0.0), CONTROL_POINT_MIDDLE );
    cplist.push_back(cp);

    cp = ControlPoint( Vector3(PI+2*PI, 0.0, 0.0), CONTROL_POINT_END );
    cplist.push_back(cp);

    camera.position = Vector3( (PI+PI/2.0), 0.0, 8.0 );
    camera.viewpoint = Vector3( (PI+PI/2.0), 0.0, 7.0 );

}

//------------------------------------------------------------------------------

void immelmannTest( void ) {
    title = "Immelmann Test";

    ControlPoint cp;

    cp = ControlPoint( Vector3(-1.0, -1.0, 0.0), CONTROL_POINT_END );
    cplist.push_back(cp);

    cp = ControlPoint( Vector3(-1.0, -1.0, 0.0), CONTROL_POINT_MIDDLE );
    cplist.push_back(cp);

    cp = ControlPoint( Vector3(0.0, -1.0, 0.0), CONTROL_POINT_MIDDLE );
    cplist.push_back(cp);

    cp = ControlPoint( Vector3(cos(PI/4), -sin(PI/4), 0.0), CONTROL_POINT_MIDDLE );
    cplist.push_back(cp);

    cp = ControlPoint( Vector3(1.0, 0.0, 0.0), CONTROL_POINT_MIDDLE );
    cplist.push_back(cp);

    cp = ControlPoint( Vector3(cos(PI/4), sin(PI/4), 0.0), CONTROL_POINT_MIDDLE );
    cplist.push_back(cp);

    cp = ControlPoint( Vector3(0.0, 1.0, 0.0), CONTROL_POINT_MIDDLE );
    cplist.push_back(cp);

    cp = ControlPoint( Vector3(-1.0, 1.0, 0.0), CONTROL_POINT_MIDDLE );
    cplist.push_back(cp);

    cp = ControlPoint( Vector3(-1.0, 1.0, 0.0), CONTROL_POINT_END );
    cplist.push_back(cp);

    camera.position = Vector3( 0.0, 0.0, 3.0 );
    camera.viewpoint = Vector3( 0.0, 0.0, 2.0 );

}

//------------------------------------------------------------------------------

void dblimmelmannTest( void ) {
    title = "Double Immelmann Test";

    ControlPoint cp;

    // first immelmann
    cp = ControlPoint( Vector3(-2.0, -1.0, 0.0), CONTROL_POINT_END );
    cplist.push_back(cp);

    cp = ControlPoint( Vector3(-2.0, -1.0, 0.0), CONTROL_POINT_MIDDLE );
    cplist.push_back(cp);

    cp = ControlPoint( Vector3(0.0, -1.0, 0.0), CONTROL_POINT_MIDDLE );
    cplist.push_back(cp);

    cp = ControlPoint( Vector3(cos(PI/4), -sin(PI/4), 0.0), CONTROL_POINT_MIDDLE );
    cplist.push_back(cp);

    cp = ControlPoint( Vector3(1.0, 0.0, 0.0), CONTROL_POINT_MIDDLE );
    cplist.push_back(cp);

    cp = ControlPoint( Vector3(cos(PI/4), sin(PI/4), 0.0), CONTROL_POINT_MIDDLE );
    cplist.push_back(cp);

    cp = ControlPoint( Vector3(0.0, 1.0, 0.0), CONTROL_POINT_MIDDLE );
    cplist.push_back(cp);

    // second immelmann

    cp = ControlPoint( Vector3(-cos(PI/4), 2.0-sin(PI/4), 0.0), CONTROL_POINT_MIDDLE );
    cplist.push_back(cp);

    cp = ControlPoint( Vector3(-1.0, 2.0, 0.0), CONTROL_POINT_MIDDLE );
    cplist.push_back(cp);

    cp = ControlPoint( Vector3(-cos(PI/4), 2.0+sin(PI/4), 0.0), CONTROL_POINT_MIDDLE );
    cplist.push_back(cp);

    cp = ControlPoint( Vector3(0.0, 3.0, 0.0), CONTROL_POINT_MIDDLE );
    cplist.push_back(cp);

    cp = ControlPoint( Vector3(2.0, 3.0, 0.0), CONTROL_POINT_MIDDLE );
    cplist.push_back(cp);

    cp = ControlPoint( Vector3(2.0, 3.0, 0.0), CONTROL_POINT_END );
    cplist.push_back(cp);

    camera.position = Vector3( 0.0, 1.0, 4.0 );
    camera.viewpoint = Vector3( 0.0, 1.0, 3.0 );

}

//------------------------------------------------------------------------------
// Spline Functions
//------------------------------------------------------------------------------
void draw_spline( const ECubicSplineBasis& basis ) {

    glPushMatrix();

    Eigen::MatrixXd M;

    GLfloat material_Ka[4] = { 0.0, 0.0, 0.0, 1.0 };
    GLfloat material_Kd[4] = { 0.0, 0.0, 0.0, 1.0 };
    GLfloat material_Ks[4] = { 0.0, 0.0, 0.0, 1.0 };
    GLfloat material_Ke[4] = { 0.0, 0.0, 0.0, 1.0 };
    GLfloat material_Se   = 10;

    switch( basis ) {
    case CUBIC_SPLINE_CATMULLROM:
    default:
        material_Ka[0] = catmullrom_Ka(0);
        material_Ka[1] = catmullrom_Ka(1);
        material_Ka[2] = catmullrom_Ka(2);

        material_Kd[0] = catmullrom_Kd(0);
        material_Kd[1] = catmullrom_Kd(1);
        material_Kd[2] = catmullrom_Kd(2);

        material_Ks[0] = catmullrom_Ks(0);
        material_Ks[1] = catmullrom_Ks(1);
        material_Ks[2] = catmullrom_Ks(2);

        material_Ke[0] = catmullrom_Ke(0);
        material_Ke[1] = catmullrom_Ke(1);
        material_Ke[2] = catmullrom_Ke(2);

        M = catmullrom_M;
        break;
    case CUBIC_SPLINE_B:
        material_Ka[0] = bspline_Ka(0);
        material_Ka[1] = bspline_Ka(1);
        material_Ka[2] = bspline_Ka(2);

        material_Kd[0] = bspline_Kd(0);
        material_Kd[1] = bspline_Kd(1);
        material_Kd[2] = bspline_Kd(2);

        material_Ks[0] = bspline_Ks(0);
        material_Ks[1] = bspline_Ks(1);
        material_Ks[2] = bspline_Ks(2);

        material_Ke[0] = bspline_Ke(0);
        material_Ke[1] = bspline_Ke(1);
        material_Ke[2] = bspline_Ke(2);

        M = bspline_M;
        break;
    }
    glMaterialfv(GL_FRONT, GL_AMBIENT, material_Ka);
    glMaterialfv(GL_FRONT, GL_DIFFUSE, material_Kd);
    glMaterialfv(GL_FRONT, GL_SPECULAR, material_Ks);
    glMaterialfv(GL_FRONT, GL_EMISSION, material_Ke);
    glMaterialf(GL_FRONT, GL_SHININESS, material_Se);


    Vector3 vi, vi1;
    unsigned int n = cplist.size();

    glLineWidth( line_width );

    glBegin(GL_LINES);

    for( unsigned int i = 2; i < n-1; i++ ) {
        Eigen::MatrixXd C = CubicSpline::blend( M, cplist, i );

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

Vector3 actor_Ka = Vector3( 0.0, 0.0, 1.0 );
Vector3 actor_Kd = Vector3( 0.0, 0.0, 1.0 );
Vector3 actor_Ks = Vector3( 0.0, 0.0, 1.0 );
Vector3 actor_Ke = Vector3( 0.0, 0.0, 0.0 );

double sim_t = 0.0;         // current time of simulation
double sim_dt = 0.1;        // time step of simulation
double actor_ds = 0.0;      // distance travelled by actor
double actor_dsdt = 0.1;    // rate of change along spline
Vector3 actor_position;     // current 3space position of actor

Vector3 actor_forward;
Vector3 actor_up;
Vector3 actor_left;

Matrix4 actor_transform;
Matrix4 camera_transform;

//------------------------------------------------------------------------------

void draw_actor( void ) {
    GLfloat material_Ka[4] = { actor_Ka(0), actor_Ka(1), actor_Ka(2), 1.0 };
    GLfloat material_Kd[4] = { actor_Kd(0), actor_Kd(1), actor_Kd(2), 1.0 };
    GLfloat material_Ks[4] = { actor_Ks(0), actor_Ks(1), actor_Ks(2), 1.0 };
    GLfloat material_Ke[4] = { actor_Ke(0), actor_Ke(1), actor_Ke(2), 1.0 };
    GLfloat material_Se   = 10;

    glPushMatrix();
    glMaterialfv(GL_FRONT, GL_AMBIENT, material_Ka);
    glMaterialfv(GL_FRONT, GL_DIFFUSE, material_Kd);
    glMaterialfv(GL_FRONT, GL_SPECULAR, material_Ks);
    glMaterialfv(GL_FRONT, GL_EMISSION, material_Ke);
    glMaterialf(GL_FRONT, GL_SHININESS, material_Se);

    glLoadMatrixd( actor_transform.arrayOpenGL() );

    glBegin(GL_TRIANGLES);

    glVertex3d( 1.0, 0.0, 0.0);
    glVertex3d(-1.0, 1.0, 0.0);
    glVertex3d(-1.0,-1.0, 0.0);

    glEnd();

    glPopMatrix();

}

//------------------------------------------------------------------------------
/// Simple Euler integration
void integrate_actor( void ) {
    int qi = 0;
    bool failed = true;
    SdS sds, sdsi;
    unsigned int n = cplist.size();
    for( unsigned int i = 2; i < n-1; i++ ) {
        sdsi = catmullrom_arclength_map.at( i );
        if( actor_ds < sdsi.first ) {
            sds = catmullrom_arclength_map.at( i-1 );
            qi = i;
            failed = false;
            break;
        }
    }

    if( failed ) {
        std::cout << "failed to find the next control point -> off the spline\n";
        return;   // failed to find the next control point -> off the spline
    }

    Eigen::MatrixXd C = CubicSpline::blend( catmullrom_M, cplist, qi );
    double qi_ds = sdsi.second;
    double qip_s = sds.first;

    // where the actor will be in terms of arclength after the step is taken
    double s1 = actor_dsdt * sim_dt + (actor_ds - qip_s);

    double u = (double)s1 / (double)qi_ds;

    Vector3 vi1 = CubicSpline::position( C, u );

    // update the actor position
    actor_position = vi1;
    // advance the actor for next tick
    actor_ds = s1 + qip_s;
    // advance the sim time
    sim_t += sim_dt;

    actor_forward = Vector3( 1.0, 0.0, 0.0 );
    actor_up =      Vector3( 0.0, 1.0, 0.0 );
    actor_left =    Vector3( 0.0, 0.0, 1.0 );

    actor_transform = Matrix4( 1.0,   0.0,   0.0,     actor_position.x(),
                               0.0,   1.0,   0.0,     actor_position.y(),
                               0.0,   0.0,   1.0,     actor_position.z(),
                               0.0,   0.0,   0.0,     1.0                 );

}

//------------------------------------------------------------------------------
// Modified SimpleGLUT
//------------------------------------------------------------------------------
static int framename = 0;
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

    draw_spline( CUBIC_SPLINE_CATMULLROM );
    draw_spline( CUBIC_SPLINE_B );
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
    framename++;

    integrate_actor();

    glutPostRedisplay();
}

//------------------------------------------------------------------------------
int main(int argc, char** argv)
{
    //lineTest();
    //circleTest();
    //sineTest();
    //immelmannTest();
    //dblimmelmannTest();
    compressedsineandsineTest();

   glutInit(&argc, argv);
   glutInitDisplayMode (GLUT_DOUBLE | GLUT_RGB |GLUT_DEPTH);
   glutInitWindowSize (600, 600);
   glutInitWindowPosition (100, 100);
   glutCreateWindow ( title.c_str() );

   catmullrom_s = CubicSpline::linearly_interpolate_arclength( CUBIC_SPLINE_CATMULLROM, cplist );
   catmullrom_M = CubicSpline::basisCatmullRom();
   catmullrom_arclength_map = CubicSpline::map_spline( CUBIC_SPLINE_CATMULLROM, cplist );
   std::cout << "catmullrom arclength s:" << catmullrom_s << std::endl;

   bspline_s = CubicSpline::linearly_interpolate_arclength( CUBIC_SPLINE_B, cplist );
   bspline_M = CubicSpline::basisUniformNonrationalBSpline();
   bspline_arclength_map = CubicSpline::map_spline( CUBIC_SPLINE_B, cplist );
   std::cout << "bspline arclength s:" << bspline_s << std::endl;

   ControlPointList::iterator it = cplist.begin()+1;
   actor_position = Vector3( it->position );
   actor_transform.identity();

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

