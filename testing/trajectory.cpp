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

//------------------------------------------------------------------------------

static unsigned int framename = 0;
int Width;
int Height;
unsigned char *Pixels;
std::string title;

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

bool sim_pause = false;



//------------------------------------------------------------------------------
// Tests
//------------------------------------------------------------------------------

void lineTest( void ) {
    title = "Line Test";

    /*
    Pose pose;

    pose = Pose( Vector3(-2.0, 0.0, 0.0), Vector3(0.0, PI, 0.0) );
    actor.insert_keyframe( 1, pose );

    pose = Pose( Vector3(0.0, 0.0, 0.0), Vector3(0.0, PI, 0.0) );
    actor.insert_keyframe( 100, pose );

    pose = Pose( Vector3(2.0, 0.0, 0.0), Vector3(0.0, PI, 0.0) );
    actor.insert_keyframe( 200, pose );

    actor.construct_trajectory();
    */

    actor.trajectory.construct_line_trajectory();

    camera.position = Vector3( 0.0, 0.0, 4.0 );
    camera.viewpoint = Vector3( 0.0, 0.0, 3.0 );

}

//------------------------------------------------------------------------------

void circleTest( void ) {
    title = "Circle Test";

    /*
    Pose pose;

    pose = Pose( Vector3(0.0, -1.0, 0.0), Vector3(0.0, PI, 0.0) );
    actor.insert_keyframe( 1, pose );

    pose = Pose( Vector3(cos(PI/4), -sin(PI/4), 0.0), Vector3(0.0, PI, -PI/4.0) );
    actor.insert_keyframe( 100, pose );

    pose = Pose( Vector3(1.0, 0.0, 0.0), Vector3(0.0, PI, -PI/2.0) );
    actor.insert_keyframe( 200, pose );

    pose = Pose( Vector3(cos(PI/4), sin(PI/4), 0.0), Vector3(0.0, PI, -PI*3.0/4.0) );
    actor.insert_keyframe( 300, pose );

    pose = Pose( Vector3(0.0, 1.0, 0.0), Vector3(0.0, PI, -PI) );
    actor.insert_keyframe( 400, pose );

    pose = Pose( Vector3(-cos(PI/4), sin(PI/4), 0.0), Vector3(0.0, PI, -PI-PI/4.0) );
    actor.insert_keyframe( 500, pose );

    pose = Pose( Vector3(-1.0, 0.0, 0.0), Vector3(0.0, PI, -PI-PI/2.0) );
    actor.insert_keyframe( 600, pose );

    pose = Pose( Vector3(-cos(PI/4), -sin(PI/4), 0.0), Vector3(0.0, PI, -PI-PI*3.0/4.0) );
    actor.insert_keyframe( 700, pose );

    pose = Pose( Vector3(0.0,-1.0, 0.0), Vector3(0.0, PI, -2.0*PI) );
    actor.insert_keyframe( 800, pose );

    actor.construct_trajectory();
    */

    actor.trajectory.construct_circle_trajectory();

    camera.position = Vector3( 0.0, 0.0, 3.0 );
    camera.viewpoint = Vector3( 0.0, 0.0, 2.0 );

}

//------------------------------------------------------------------------------

void sineTest( void ) {
    title = "Sine Test";

    /*
    Pose pose;

    pose = Pose( Vector3(0.0, 0.0, 0.0), Vector3(0.0, PI, 0.0) );
    actor.insert_keyframe( 1, pose );

    pose = Pose( Vector3(PI/2.0, 1.0, 0.0), Vector3(0.0, PI, 0.0) );
    actor.insert_keyframe( 300, pose );

    pose = Pose( Vector3(PI*3.0/2.0, -1.0, 0.0), Vector3(0.0, PI, 0.0) );
    actor.insert_keyframe( 600, pose );

    pose = Pose( Vector3(2*PI, 0.0, 0.0), Vector3(0.0, PI, 0.0) );
    actor.insert_keyframe( 900, pose );

    actor.construct_trajectory();
    */

    actor.trajectory.construct_sine_trajectory();

    camera.position = Vector3( PI, 0.0, 6.0 );
    camera.viewpoint = Vector3( PI, 0.0, 5.0 );

}

//------------------------------------------------------------------------------

void compressedsineandsineTest( void ) {
    title = "Compressed Sine And Sine Test";

    /*
    Pose pose;

    // compressed amplitude sine

    pose = Pose( Vector3(0.0, 0.0, 0.0), Vector3(0.0, PI, 0.0) );
    actor.insert_keyframe( 1, pose );

    pose = Pose( Vector3(PI/4.0, 1.0, 0.0), Vector3(0.0, PI, 0.0) );
    actor.insert_keyframe( 100, pose );

    pose = Pose( Vector3(PI*3/4.0, -1.0, 0.0), Vector3(0.0, PI, 0.0) );
    actor.insert_keyframe( 200, pose );

    //pose = Pose( Vector3(PI, 0.0, 0.0), Vector3(0.0, PI, 0.0) );
    //actor.insert_keyframe( 300, pose );

    // normal amplitude sine

    pose = Pose( Vector3(PI+PI/2.0, 1.0, 0.0), Vector3(0.0, PI, 0.0) );
    actor.insert_keyframe( 400, pose );

    pose = Pose( Vector3(PI+PI*3.0/2.0, -1.0, 0.0), Vector3(0.0, PI, 0.0) );
    actor.insert_keyframe( 600, pose );

    pose = Pose( Vector3(PI+2*PI, 0.0, 0.0), Vector3(0.0, PI, 0.0) );
    actor.insert_keyframe( 800, pose );

    actor.construct_trajectory();
    */

    actor.trajectory.construct_multisine_trajectory();

    camera.position = Vector3( (PI+PI/2.0), 0.0, 8.0 );
    camera.viewpoint = Vector3( (PI+PI/2.0), 0.0, 7.0 );

}

//------------------------------------------------------------------------------

void immelmannTest( void ) {
    title = "Immelmann Test";

    /*
    Pose pose;

    pose = Pose( Vector3(-2.0, -1.0, 0.0), Vector3(0.0, PI, 0.0) );
    actor.insert_keyframe( 1, pose );

    pose = Pose( Vector3(-1.0, -1.0, 0.0), Vector3(0.0, PI, 0.0) );
    actor.insert_keyframe( 100, pose );

    pose = Pose( Vector3(0.0, -1.0, 0.0), Vector3(0.0, PI, 0.0) );
    actor.insert_keyframe( 200, pose );

    pose = Pose( Vector3(cos(PI/4), -sin(PI/4), 0.0), Vector3(0.0, PI, -PI/4.0) );
    actor.insert_keyframe( 300, pose );

    pose = Pose( Vector3(1.0, 0.0, 0.0), Vector3(0.0, PI, -PI/2.0) );
    actor.insert_keyframe( 400, pose );

    pose = Pose( Vector3(cos(PI/4), sin(PI/4), 0.0), Vector3(0.0, PI, -PI*3.0/4.0) );
    actor.insert_keyframe( 500, pose );

    pose = Pose( Vector3(0.0, 1.0, 0.0), Vector3(0.0, PI, -PI) );
    actor.insert_keyframe( 600, pose );

    pose = Pose( Vector3(-1.0, 1.0, 0.0), Vector3(PI, PI, -PI) );
    actor.insert_keyframe( 700, pose );

    pose = Pose( Vector3(-2.0, 1.0, 0.0), Vector3(PI, PI, -PI) );
    actor.insert_keyframe( 800, pose );

    actor.construct_trajectory();
    */
    actor.trajectory.construct_immelmann_trajectory();

    camera.position = Vector3( -0.5, 0.0, 3.0 );
    camera.viewpoint = Vector3( -0.5, 0.0, 2.0 );

}

//------------------------------------------------------------------------------

void dblimmelmannTest( void ) {
    title = "Double Immelmann Test";

    /*
    Pose pose;

    // lower (first) immelmann
    pose = Pose( Vector3(-2.5, -1.0, 0.0), Vector3(0.0, PI, 0.0) );
    actor.insert_keyframe( 1, pose );

    pose = Pose( Vector3(0.5, -1.0, 0.0), Vector3(0.0, PI, 0.0) );
    actor.insert_keyframe( 100, pose );

    pose = Pose( Vector3(1.0+cos(PI/4), -sin(PI/4), 0.0), Vector3(0.0, PI, -PI/4.0) );
    actor.insert_keyframe( 150, pose );

    pose = Pose( Vector3(2.0, 0.0, 0.0), Vector3(0.0, PI, -PI/2.0) );
    actor.insert_keyframe( 200, pose );

    pose = Pose( Vector3(1.0+cos(PI/4), sin(PI/4), 0.0), Vector3(0.0, PI, -PI*3.0/4.0) );
    actor.insert_keyframe( 250, pose );

    pose = Pose( Vector3(1.0, 1.0, 0.0), Vector3(0.0, PI, -PI) );
    actor.insert_keyframe( 300, pose );

    // roll
    pose = Pose( Vector3(-1.0, 1.0, 0.0), Vector3(PI, PI, -PI) );
    actor.insert_keyframe( 400, pose );

    // upper (second) immelmann
    pose = Pose( Vector3(-1.0-cos(PI/4), 2.0-sin(PI/4), 0.0), Vector3(PI, PI, -PI*5.0/4.0) );
    actor.insert_keyframe( 450, pose );

    pose = Pose( Vector3(-2.0, 2.0, 0.0), Vector3(PI, PI, -PI*6.0/4.0) );
    actor.insert_keyframe( 500, pose );

    pose = Pose( Vector3(-1.0-cos(PI/4), 2.0+sin(PI/4), 0.0), Vector3(PI, PI, -PI*7.0/4.0) );
    actor.insert_keyframe( 550, pose );

    pose = Pose( Vector3(-0.5, 3.0, 0.0), Vector3(PI, PI, -2*PI) );
    actor.insert_keyframe( 600, pose );

    pose = Pose( Vector3(2.5, 3.0, 0.0), Vector3(0.0, PI, -2*PI) );
    actor.insert_keyframe( 700, pose );

    actor.construct_trajectory();
    */

    actor.trajectory.construct_doubleimmelmann_trajectory();

    camera.position = Vector3( 0.0, 1.0, 4.0 );
    camera.viewpoint = Vector3( 0.0, 1.0, 3.0 );

}

//------------------------------------------------------------------------------

void loopTest( void ) {
    title = "Loop Test";

    /*
    Pose pose;

    pose = Pose( Vector3(-2.0, -1.0, 0.0), Vector3(0.0, PI, 0.0) );
    actor.insert_keyframe( 1, pose );

    pose = Pose( Vector3(0.0, -1.0, 0.0), Vector3(0.0, PI, 0.0) );
    actor.insert_keyframe( 100, pose );

    pose = Pose( Vector3(cos(PI/4), -sin(PI/4), 0.0), Vector3(0.0, PI, -PI/4.0) );
    actor.insert_keyframe( 150, pose );

    pose = Pose( Vector3(1.0, 0.0, 0.0), Vector3(0.0, PI, -PI/2.0) );
    actor.insert_keyframe( 200, pose );

    pose = Pose( Vector3(cos(PI/4), sin(PI/4), 0.0), Vector3(0.0, PI, -PI*3.0/4.0) );
    actor.insert_keyframe( 250, pose );

    pose = Pose( Vector3(0.0, 1.0, 0.0), Vector3(0.0, PI, -PI) );
    actor.insert_keyframe( 300, pose );

    pose = Pose( Vector3(-cos(PI/4), sin(PI/4), 0.0), Vector3(0.0, PI, -PI-PI/4.0) );
    actor.insert_keyframe( 350, pose );

    pose = Pose( Vector3(-1.0, 0.0, 0.0), Vector3(0.0, PI, -PI-PI/2.0) );
    actor.insert_keyframe( 400, pose );

    pose = Pose( Vector3(-cos(PI/4), -sin(PI/4), 0.0), Vector3(0.0, PI, -PI-PI*3.0/4.0) );
    actor.insert_keyframe( 450, pose );

    pose = Pose( Vector3(0.0,-1.0, 0.0), Vector3(0.0, PI, -2*PI) );
    actor.insert_keyframe( 500, pose );

    pose = Pose( Vector3(2.0,-1.0, 0.0), Vector3(0.0, PI, -2*PI) );
    actor.insert_keyframe( 600, pose );

    actor.construct_trajectory();
    */
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

Material actor_material = Material(
            Color( 0.0, 0.0, 1.0, 1.0 ),
            Color( 0.0, 0.0, 1.0, 1.0 ),
            Color( 0.0, 0.0, 1.0, 1.0 ),
            Color( 0.0, 0.0, 0.0, 1.0 ),
            10                           );

double sim_t = 0.0;         // current time of simulation
double sim_dt = 0.1;        // time step of simulation
double actor_ds = 0.0;      // distance travelled by actor
double actor_dsdt = 0.1;    // rate of change along spline

void initActor( void ) {
    actor_ds = 0;
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

    //glLoadMatrixd( actor_transform.arrayOpenGL() );

    /*
    glBegin(GL_TRIANGLES);
    glVertex3d( 1.0, 0.0, 0.0);
    glVertex3d(-1.0, 1.0, 0.0);
    glVertex3d(-1.0,-1.0, 0.0);
    glEnd();
    */
    //glutSolidTeapot( 1.0 );
    glBegin( GL_TRIANGLES );

    for( unsigned int geometry_id = 0; geometry_id < actor.geometryCount( ); geometry_id++ ) {
        // again dangerous see above
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

#define CATMULL_ROM     0
#define B_SPLINE        1
//#define SPLINE_BASIS    CATMULL_ROM
#define SPLINE_BASIS    B_SPLINE

void interpolate_frame( void ) {

    // if the last keyframe is passed, then don't interpolate any more
    if( actor.trajectory.next_keyframe_id == actor.trajectory.keyframes() ) return;

    bool transition = false;
    // if the frame has reached a keyframe boundary, advance to next keyframe
    if( framename == actor.trajectory.keyframe( actor.trajectory.next_keyframe_id ).frame ) {
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
    double dt = ((double)framename - t0) / ( t1 - t0 );

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

    actor.pose.transformByQuaternion();
    //actor.pose.transformByEulerAngle();

    Matrix4 actor_scale = Matrix4::scalingMatrix( 0.1, 0.1, 0.1 );
    actor.pose.transform = actor.pose.transform * actor_scale;
}

//------------------------------------------------------------------------------

/// Simple Euler integration
void integrate_actor( void ) {
    if(sim_pause) return;

    int qi = 0;
    bool failed = true;
    SdS sds, sdsi;
    unsigned int n = actor.trajectory.controlpoints.size();
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

    Eigen::MatrixXd C = CubicSpline::blend( catmullrom_M, actor.trajectory.controlpoints, qi );
    double qi_ds = sdsi.second;
    double qip_s = sds.first;

    // where the actor will be in terms of arclength after the step is taken
    double s1 = actor_dsdt * sim_dt + (actor_ds - qip_s);

    double u = (double)s1 / (double)qi_ds;

    Vector3 vi1 = CubicSpline::position( C, u );

    // update the actor position
    actor.pose.position = vi1;
    // advance the actor for next tick
    actor_ds = s1 + qip_s;
    // advance the sim time
    sim_t += sim_dt;
    /*
    //-----------------
    Vector3 v_w = CubicSpline::tangent( C, u );
    v_w.normalize();

    double wdotup = Vector3::dot( v_w, actor_up );
    double wdotforward = Vector3::dot( v_w, actor_forward );
    double theta = acos(wdotforward);
    if( wdotup < 0.0 ) {
        theta = -theta;             // pitch down
    }

    Matrix3 R = Matrix3::rotZ( theta );
    actor_up = R * actor_up;
    actor_up.normalize();

    actor_forward = v_w;

    actor_left = Vector3::cross( actor_forward, actor_up );
    actor_left.normalize();

    //-----------------
    */
    ///*
    Matrix3 arot;
    arot  = Matrix3::rotX( actor.pose.orientation.x() );
    arot *= Matrix3::rotY( actor.pose.orientation.y() );
    arot *= Matrix3::rotZ( actor.pose.orientation.z() );

    Vector3 old_actor_forward = arot * Vector3( 1.0, 0.0, 0.0 );
    Vector3 old_actor_up = arot * Vector3( 0.0, 1.0, 0.0 );
    Vector3 old_actor_left = arot * Vector3( 0.0, 0.0, 1.0 );
    old_actor_forward.normalize();
    old_actor_up.normalize();
    old_actor_left.normalize();

    //if( sim_pause )
    //    bool this_is_a_breakpoint = true;

    Vector3 new_actor_forward = CubicSpline::tangent( C, u );
    new_actor_forward.normalize();

    Vector3 new_actor_up = old_actor_up - (new_actor_forward * Vector3::dot( old_actor_up, new_actor_forward ) );
    new_actor_up.normalize();

    Vector3 new_actor_left = Vector3::cross( new_actor_forward, new_actor_up );
    new_actor_left.normalize();

    double fwd_dot_fwd = Vector3::dot( new_actor_forward, old_actor_forward );
    double up_dot_up = Vector3::dot( new_actor_up, old_actor_up );
    double left_dot_left = Vector3::dot( new_actor_left, old_actor_left );

    double fwd_dot_left = Vector3::dot( new_actor_forward, old_actor_left );
    double up_dot_fwd = Vector3::dot( new_actor_up, old_actor_forward );
    double left_dot_up = Vector3::dot( new_actor_left, old_actor_up );

    double droll = acos( left_dot_left );
    double dpitch = acos( up_dot_up );
    double dyaw = acos( fwd_dot_fwd );

    if( left_dot_up < 0.0 )     droll = -droll;
    if( up_dot_fwd < 0.0 )      dpitch = -dpitch;
    if( fwd_dot_left < 0.0 )    dyaw = -dyaw;

    actor.pose.orientation(0) += droll;
    actor.pose.orientation(1) += dyaw;
    actor.pose.orientation(2) += dpitch;

    /*
    //double wdotup = Vector3::dot( new_actor_forward, old_actor_up );
    double wdotleft = Vector3::dot( new_actor_forward, old_actor_left );
    double wdotforward = Vector3::dot( new_actor_forward, old_actor_forward );
    double dtheta = acos(wdotforward);
    //if( wdotup < 0.0 ) {
    if( wdotleft < 0.0 ) {
        dtheta = -dtheta;
    }

    double vdotleft = Vector3::dot( new_actor_up, old_actor_left );
    double vdotup = Vector3::dot( new_actor_up, old_actor_up );
    double dphi = acos(vdotup);
    if( vdotleft < 0.0 ) {
        dphi = -dphi;
    }

    double udotforward = Vector3::dot( new_actor_left, old_actor_forward );
    double udotleft = Vector3::dot( new_actor_left, old_actor_left );
    double dpsi = acos(udotleft);
    if( udotforward < 0.0 ) {
        dpsi = -dpsi;
    }

    actor_orientation(0) += dtheta;
    actor_orientation(1) += dpsi;
    actor_orientation(2) += dphi;

//    actor_orientation(2) += dtheta;
//    actor_orientation(1) += dpsi;
//    actor_orientation(0) += dphi;

    */

    /*
    actor_rotation.identity();
    actor_rotation  = Matrix4::rotX( actor_orientation.x() );
    actor_rotation *= Matrix4::rotY( actor_orientation.y() );
    actor_rotation *= Matrix4::rotZ( actor_orientation.z() );

    //actor_rotation = EulerAngle::matrix4(EULER_ANGLE_XYZ,EULER_HANDED_LEFT,actor_orientation);
    //actor_rotation = EulerAngle::matrix4(EULER_ANGLE_ZXZ,EULER_HANDED_LEFT,actor_orientation);

//    actor_rotation = Matrix4( actor_forward(0),    actor_up(0),    actor_left(0),  0.0,
//                              actor_forward(1),    actor_up(1),    actor_left(1),  0.0,
//                              actor_forward(2),    actor_up(2),    actor_left(2),  0.0,
//                              0.0,                 0.0,            0.0,            1.0 );


    actor_translation.setTranslation( actor_position.x(), actor_position.y(), actor_position.z() );
    actor_transform = actor_translation * actor_rotation;
    */
}


//------------------------------------------------------------------------------
// Modified SimpleGLUT
//------------------------------------------------------------------------------


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
    framename++;

    interpolate_frame();
    //integrate_actor();

    glutPostRedisplay();
}

//------------------------------------------------------------------------------
int main(int argc, char** argv)
{
    //lineTest();
    //circleTest();
    //sineTest();
    //compressedsineandsineTest();
    //immelmannTest();
    dblimmelmannTest();
    //loopTest();

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


