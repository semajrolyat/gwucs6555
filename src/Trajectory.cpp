/*-----------------------THE GEORGE WASHINGTON UNIVERSITY-----------------------
James Taylor                                                         jrt@gwu.edu

Trajectory class implementation

------------------------------------------------------------------------------*/

#include <cs6555/Trajectory.h>

#include <assert.h>
#include <iostream>

#include <cs6555/Constants.h>
#include <cs6555/Math/EulerAngle.h>

//------------------------------------------------------------------------------
// Constructors
//------------------------------------------------------------------------------
/// Default Constructor
Trajectory::Trajectory( void ) {
    m_keyframe_count = 0;

    prev_keyframe_id = 0;
    next_keyframe_id = 0;
    cyclic = false;
    reversible = false;
    forward = true;
    frame_tic = 0;

    material = Material( Color( 1.0, 1.0, 0.0, 1.0 ),
                         Color( 1.0, 1.0, 0.0, 1.0 ),
                         Color( 1.0, 1.0, 0.0, 1.0 ),
                         Color( 0.0, 0.0, 0.0, 1.0 ),
                         10                           );
    line_width = 1.0;
}

//------------------------------------------------------------------------------
// Destructor
//------------------------------------------------------------------------------
/// Destructor
Trajectory::~Trajectory( void ) {

}

//------------------------------------------------------------------------------
// Keyframe Operations
//------------------------------------------------------------------------------
/// Creates and inserts a keyframe into the keyframe list
/// Given a pose and a frame number
void Trajectory::insert_keyframe( const int& frame, const Pose& pose ) {
    Keyframe kf = Keyframe( frame, pose );
    m_keyframes.push_back( kf );
    m_keyframe_count++;
}

//------------------------------------------------------------------------------
/// Returns a Keyframe from the keyframe list
/// Given an index out of that list
Keyframe& Trajectory::keyframe( const unsigned int& index ) {
    assert( index < m_keyframe_count );
    return m_keyframes.at( index );
}

//------------------------------------------------------------------------------
/// Returns the number of keyframes currently in the keyframe list
unsigned int Trajectory::keyframes( void ) {
    return m_keyframe_count;
}

void Trajectory::clear( void ) {
    m_keyframes.clear( );
    m_keyframe_count = 0;
    controlpoints.clear();
    poses.clear();
    spline_arclength_map.clear();
}

//------------------------------------------------------------------------------
// Trajectory Queries
//------------------------------------------------------------------------------
Vector3 Trajectory::position( const double& distance ) {
    assert( m_keyframe_count > 1 );
    int keyframe0_id = 0;
    int keyframe1_id = 0;

    SdS sds_at_keyframe0, sds_at_keyframe1;
    //if( !cyclic ) {
        for( unsigned int i = 1; i < spline_arclength_map.size(); i++ ) {
            std::vector<SdS>::iterator it = spline_arclength_map.begin() + i;
            if( it->first > distance ) {
                keyframe0_id = i-1;
                keyframe1_id = i;
                sds_at_keyframe1 = *it;
                sds_at_keyframe0 = *(it-1);
                break;
            }
        }

        Eigen::MatrixXd C = CubicSpline::blend( spline_basis, controlpoints, keyframe1_id );

        double u = (distance - sds_at_keyframe0.first); /// sds_at_keyframe1.second;

        //std::cout << " s@kf0:" << sds_at_keyframe0.first << " ds@kf1:" << sds_at_keyframe1.second << " Traj u: " << u << " Traj ds:" << distance;

        return CubicSpline::position( C, u );
        /*
    } else {
        std::vector<SdS>::iterator last = spline_arclength_map.end() - 1;
        double div = distance / last->second;
        unsigned int cycles = (unsigned int) div;
        double d = distance - ((double)cycles * last->second);

        int n = spline_arclength_map.size();
        for( unsigned int i = 1; i < n; i++ ) {
            std::vector<SdS>::iterator it = spline_arclength_map.begin() + i;
            if( it->first > d ) {
                keyframe0_id = i-1;
                keyframe1_id = i;
                sds_at_keyframe1 = *it;
                sds_at_keyframe0 = *(it-1);
                break;
            }
        }
        ControlPointPtrList ptrlist;
        ControlPoint *cp1, *cp2, *cp3, *cp4;
        if( keyframe1_id == 0 ) {
            cp1 = &controlpoints.at( controlpoints.size()-2 );
            cp2 = &controlpoints.at( controlpoints.size()-1 );
            cp3 = &controlpoints.at( 0 );
            cp4 = &controlpoints.at( 1 );
        } else if( keyframe1_id == 1 ) {
            cp1 = &controlpoints.at( controlpoints.size()-1 );
            cp2 = &controlpoints.at( 0 );
            cp3 = &controlpoints.at( 1 );
            cp4 = &controlpoints.at( 2 );
        } else if( keyframe1_id == keyframes() - 1 ) {
            cp1 = &controlpoints.at( controlpoints.size()-3 );
            cp2 = &controlpoints.at( controlpoints.size()-2 );
            cp3 = &controlpoints.at( controlpoints.size()-1 );
            cp4 = &controlpoints.at( 0 );
        } else {
            cp1 = &controlpoints.at( keyframe1_id-2 );
            cp2 = &controlpoints.at( keyframe1_id-1 );
            cp3 = &controlpoints.at( keyframe1_id );
            cp4 = &controlpoints.at( keyframe1_id+1 );
        }
        ptrlist.push_back( cp1 );
        ptrlist.push_back( cp2 );
        ptrlist.push_back( cp3 );
        ptrlist.push_back( cp4 );

        Eigen::MatrixXd C = CubicSpline::blend( spline_basis, ptrlist );

        double u = (d - sds_at_keyframe0.first); /// sds_at_keyframe1.second;

        //std::cout << " s@kf0:" << sds_at_keyframe0.first << " ds@kf1:" << sds_at_keyframe1.second << " Traj u: " << u << " Traj ds:" << distance;

        return CubicSpline::position( C, u );
    }
    */
}

//------------------------------------------------------------------------------

Vector3 Trajectory::tangent( const double& distance ) {

    return Vector3( 0.0, 0.0, 1.0 ); // bogus
}
//------------------------------------------------------------------------------

Vector3 Trajectory::normal( const double& distance ) {
    return Vector3( 0.0, 0.0, 1.0 ); // bogus
}

//------------------------------------------------------------------------------
// Trajectory Operations
//------------------------------------------------------------------------------
/// Constructs a trajectory from the populated keyframe list
bool Trajectory::construct_trajectory( void ) {
    assert( m_keyframe_count > 0 );

    KeyframeList::iterator it_kf1 = m_keyframes.begin();
    KeyframeList::iterator it_kfn = m_keyframes.end()-1;
    Pose pose;
    ControlPoint cp;

    //if( !cyclic ) {
        pose = it_kf1->pose;
        cp = ControlPoint( pose.position, CONTROL_POINT_END );
        controlpoints.push_back( cp );

        poses.push_back( pose );
        cp = ControlPoint( pose.position, CONTROL_POINT_MIDDLE );
        controlpoints.push_back( cp );

        for( KeyframeList::iterator it = it_kf1+1; it != it_kfn; it++ ) {
            pose = it->pose;
            poses.push_back( pose );
            cp = ControlPoint( pose.position, CONTROL_POINT_MIDDLE );
            controlpoints.push_back( cp );
        }

        pose = it_kfn->pose;
        poses.push_back( pose );
        cp = ControlPoint( pose.position, CONTROL_POINT_MIDDLE );
        controlpoints.push_back( cp );

        cp = ControlPoint( pose.position, CONTROL_POINT_END );
        controlpoints.push_back( cp );
        /*
    } else {
        pose = it_kf1->pose;
        poses.push_back( pose );
        cp = ControlPoint( pose.position, CONTROL_POINT_MIDDLE );
        controlpoints.push_back( cp );

        for( KeyframeList::iterator it = it_kf1+1; it != it_kfn; it++ ) {
            pose = it->pose;
            poses.push_back( pose );
            cp = ControlPoint( pose.position, CONTROL_POINT_MIDDLE );
            controlpoints.push_back( cp );
        }

        pose = it_kfn->pose;
        poses.push_back( pose );
        cp = ControlPoint( pose.position, CONTROL_POINT_MIDDLE );
        controlpoints.push_back( cp );
    }
    */
    return true;
}

//------------------------------------------------------------------------------
// Predefined Trajectories
//------------------------------------------------------------------------------
/// Populates the keyframe list with a line trajectory and constructs
/// the control points
void Trajectory::construct_line_trajectory( void ) {
    Pose pose;

    pose = Pose( Vector3(-200.0, 0.0, 0.0), Vector3(0.0, PI/2, 0.0) );
    insert_keyframe( 1, pose );

    pose = Pose( Vector3(0.0, 0.0, 0.0), Vector3(0.0, PI/2, 0.0) );
    insert_keyframe( 500, pose );

    pose = Pose( Vector3(200.0, 0.0, 0.0), Vector3(0.0, PI/2, 0.0) );
    insert_keyframe( 1000, pose );
/*
    pose = Pose( Vector3(-200.0, 0.0, 0.0), Vector3(0.0, PI, 0.0) );
    insert_keyframe( 1, pose );

    pose = Pose( Vector3(0.0, 0.0, 0.0), Vector3(0.0, PI, 0.0) );
    insert_keyframe( 500, pose );

    pose = Pose( Vector3(200.0, 0.0, 0.0), Vector3(0.0, PI, 0.0) );
    insert_keyframe( 1000, pose );
*/
    construct_trajectory();

    prev_keyframe_id = 0;
    next_keyframe_id = 1;
    spline_basis = CubicSpline::basisUniformNonrationalBSpline();
    spline_arclength_map = CubicSpline::map_spline( CUBIC_SPLINE_B, controlpoints );

}

//------------------------------------------------------------------------------
/// Populates the keyframe list with a circle trajectory and constructs
/// the control points
void Trajectory::construct_circle_trajectory( void ) {
    Pose pose;

    pose = Pose( Vector3(0.0, -1.0, 0.0), Vector3(0.0, PI, 0.0) );
    insert_keyframe( 1, pose );

    pose = Pose( Vector3(cos(PI/4), -sin(PI/4), 0.0), Vector3(0.0, PI, -PI/4.0) );
    insert_keyframe( 100, pose );

    pose = Pose( Vector3(1.0, 0.0, 0.0), Vector3(0.0, PI, -PI/2.0) );
    insert_keyframe( 200, pose );

    pose = Pose( Vector3(cos(PI/4), sin(PI/4), 0.0), Vector3(0.0, PI, -PI*3.0/4.0) );
    insert_keyframe( 300, pose );

    pose = Pose( Vector3(0.0, 1.0, 0.0), Vector3(0.0, PI, -PI) );
    insert_keyframe( 400, pose );

    pose = Pose( Vector3(-cos(PI/4), sin(PI/4), 0.0), Vector3(0.0, PI, -PI-PI/4.0) );
    insert_keyframe( 500, pose );

    pose = Pose( Vector3(-1.0, 0.0, 0.0), Vector3(0.0, PI, -PI-PI/2.0) );
    insert_keyframe( 600, pose );

    pose = Pose( Vector3(-cos(PI/4), -sin(PI/4), 0.0), Vector3(0.0, PI, -PI-PI*3.0/4.0) );
    insert_keyframe( 700, pose );

    pose = Pose( Vector3(0.0,-1.0, 0.0), Vector3(0.0, PI, -2.0*PI) );
    insert_keyframe( 800, pose );

    construct_trajectory();
}

//------------------------------------------------------------------------------
/// Populates the keyframe list with a sine trajectory and constructs
/// the control points
void Trajectory::construct_sine_trajectory( void ) {
    Pose pose;

    pose = Pose( Vector3(0.0, 0.0, 0.0), Vector3(0.0, PI, 0.0) );
    insert_keyframe( 1, pose );

    pose = Pose( Vector3(PI/2.0, 1.0, 0.0), Vector3(0.0, PI, 0.0) );
    insert_keyframe( 300, pose );

    pose = Pose( Vector3(PI*3.0/2.0, -1.0, 0.0), Vector3(0.0, PI, 0.0) );
    insert_keyframe( 600, pose );

    pose = Pose( Vector3(2*PI, 0.0, 0.0), Vector3(0.0, PI, 0.0) );
    insert_keyframe( 900, pose );

    construct_trajectory();
}

//------------------------------------------------------------------------------
void Trajectory::construct_cyclic_sine_trajectory( void ) {
    Pose pose;

    pose = Pose( Vector3(0.0, 0.0, 0.0), Vector3(0.0, PI, 0.0) );
    insert_keyframe( 1, pose );

    pose = Pose( Vector3(PI/2.0, 1.0, 0.0), Vector3(0.0, PI, 0.0) );
    insert_keyframe( 300, pose );

    pose = Pose( Vector3(PI*3.0/2.0, -1.0, 0.0), Vector3(0.0, PI, 0.0) );
    insert_keyframe( 600, pose );

    pose = Pose( Vector3(2*PI, 0.0, 0.0), Vector3(0.0, PI, 0.0) );
    insert_keyframe( 900, pose );

    //cyclic = true;
    construct_trajectory();

}

//------------------------------------------------------------------------------
/// Populates the keyframe list with a concatenated-sine trajectory
/// and constructs the control points
void Trajectory::construct_multisine_trajectory( void ) {
    Pose pose;

    // compressed amplitude sine

    pose = Pose( Vector3(0.0, 0.0, 0.0), Vector3(0.0, PI, 0.0) );
    insert_keyframe( 1, pose );

    pose = Pose( Vector3(PI/4.0, 1.0, 0.0), Vector3(0.0, PI, 0.0) );
    insert_keyframe( 100, pose );

    pose = Pose( Vector3(PI*3/4.0, -1.0, 0.0), Vector3(0.0, PI, 0.0) );
    insert_keyframe( 200, pose );

    //pose = Pose( Vector3(PI, 0.0, 0.0), Vector3(0.0, PI, 0.0) );
    //insert_keyframe( 300, pose );

    // normal amplitude sine

    pose = Pose( Vector3(PI+PI/2.0, 1.0, 0.0), Vector3(0.0, PI, 0.0) );
    insert_keyframe( 400, pose );

    pose = Pose( Vector3(PI+PI*3.0/2.0, -1.0, 0.0), Vector3(0.0, PI, 0.0) );
    insert_keyframe( 600, pose );

    pose = Pose( Vector3(PI+2*PI, 0.0, 0.0), Vector3(0.0, PI, 0.0) );
    insert_keyframe( 800, pose );

    construct_trajectory();
}

//------------------------------------------------------------------------------
/// Populates the keyframe list with an immelmann trajectory and constructs
/// the control points
void Trajectory::construct_immelmann_trajectory( void ) {
    Pose pose;

    pose = Pose( Vector3(-2.0, -1.0, 0.0), Vector3(0.0, PI, 0.0) );
    insert_keyframe( 1, pose );

    pose = Pose( Vector3(-1.0, -1.0, 0.0), Vector3(0.0, PI, 0.0) );
    insert_keyframe( 100, pose );

    pose = Pose( Vector3(0.0, -1.0, 0.0), Vector3(0.0, PI, 0.0) );
    insert_keyframe( 200, pose );

    pose = Pose( Vector3(cos(PI/4), -sin(PI/4), 0.0), Vector3(0.0, PI, -PI/4.0) );
    insert_keyframe( 300, pose );

    pose = Pose( Vector3(1.0, 0.0, 0.0), Vector3(0.0, PI, -PI/2.0) );
    insert_keyframe( 400, pose );

    pose = Pose( Vector3(cos(PI/4), sin(PI/4), 0.0), Vector3(0.0, PI, -PI*3.0/4.0) );
    insert_keyframe( 500, pose );

    pose = Pose( Vector3(0.0, 1.0, 0.0), Vector3(0.0, PI, -PI) );
    insert_keyframe( 600, pose );

    pose = Pose( Vector3(-1.0, 1.0, 0.0), Vector3(PI, PI, -PI) );
    insert_keyframe( 700, pose );

    pose = Pose( Vector3(-2.0, 1.0, 0.0), Vector3(PI, PI, -PI) );
    insert_keyframe( 800, pose );

    construct_trajectory();
}

//------------------------------------------------------------------------------
/// Populates the keyframe list with a double immelmann trajectory and
/// constructs the control points
void Trajectory::construct_doubleimmelmann_trajectory( void ) {
    Pose pose;

    // lower (first) immelmann
    pose = Pose( Vector3(-2.5, -1.0, 0.0), Vector3(0.0, PI, 0.0) );
    insert_keyframe( 1, pose );

    pose = Pose( Vector3(0.5, -1.0, 0.0), Vector3(0.0, PI, 0.0) );
    insert_keyframe( 100, pose );

    pose = Pose( Vector3(1.0+cos(PI/4), -sin(PI/4), 0.0), Vector3(0.0, PI, -PI/4.0) );
    insert_keyframe( 150, pose );

    pose = Pose( Vector3(2.0, 0.0, 0.0), Vector3(0.0, PI, -PI/2.0) );
    insert_keyframe( 200, pose );

    pose = Pose( Vector3(1.0+cos(PI/4), sin(PI/4), 0.0), Vector3(0.0, PI, -PI*3.0/4.0) );
    insert_keyframe( 250, pose );

    pose = Pose( Vector3(1.0, 1.0, 0.0), Vector3(0.0, PI, -PI) );
    insert_keyframe( 300, pose );

    // roll
    pose = Pose( Vector3(-1.0, 1.0, 0.0), Vector3(PI, PI, -PI) );
    insert_keyframe( 400, pose );

    // upper (second) immelmann
    pose = Pose( Vector3(-1.0-cos(PI/4), 2.0-sin(PI/4), 0.0), Vector3(PI, PI, -PI*5.0/4.0) );
    insert_keyframe( 450, pose );

    pose = Pose( Vector3(-2.0, 2.0, 0.0), Vector3(PI, PI, -PI*6.0/4.0) );
    insert_keyframe( 500, pose );

    pose = Pose( Vector3(-1.0-cos(PI/4), 2.0+sin(PI/4), 0.0), Vector3(PI, PI, -PI*7.0/4.0) );
    insert_keyframe( 550, pose );

    pose = Pose( Vector3(-0.5, 3.0, 0.0), Vector3(PI, PI, -2*PI) );
    insert_keyframe( 600, pose );

    pose = Pose( Vector3(2.5, 3.0, 0.0), Vector3(0.0, PI, -2*PI) );
    insert_keyframe( 700, pose );

    construct_trajectory();
}

//------------------------------------------------------------------------------
/// Populates the keyframe list with a loop trajectory and constructs
/// the control points
void Trajectory::construct_loop_trajectory( void ) {
    Pose pose;

    pose = Pose( Vector3(-2.0, -1.0, 0.0), Vector3(0.0, PI, 0.0) );
    insert_keyframe( 1, pose );

    pose = Pose( Vector3(0.0, -1.0, 0.0), Vector3(0.0, PI, 0.0) );
    insert_keyframe( 100, pose );

    pose = Pose( Vector3(cos(PI/4), -sin(PI/4), 0.0), Vector3(0.0, PI, -PI/4.0) );
    insert_keyframe( 150, pose );

    pose = Pose( Vector3(1.0, 0.0, 0.0), Vector3(0.0, PI, -PI/2.0) );
    insert_keyframe( 200, pose );

    pose = Pose( Vector3(cos(PI/4), sin(PI/4), 0.0), Vector3(0.0, PI, -PI*3.0/4.0) );
    insert_keyframe( 250, pose );

    pose = Pose( Vector3(0.0, 1.0, 0.0), Vector3(0.0, PI, -PI) );
    insert_keyframe( 300, pose );

    pose = Pose( Vector3(-cos(PI/4), sin(PI/4), 0.0), Vector3(0.0, PI, -PI-PI/4.0) );
    insert_keyframe( 350, pose );

    pose = Pose( Vector3(-1.0, 0.0, 0.0), Vector3(0.0, PI, -PI-PI/2.0) );
    insert_keyframe( 400, pose );

    pose = Pose( Vector3(-cos(PI/4), -sin(PI/4), 0.0), Vector3(0.0, PI, -PI-PI*3.0/4.0) );
    insert_keyframe( 450, pose );

    pose = Pose( Vector3(0.0,-1.0, 0.0), Vector3(0.0, PI, -2*PI) );
    insert_keyframe( 500, pose );

    pose = Pose( Vector3(2.0,-1.0, 0.0), Vector3(0.0, PI, -2*PI) );
    insert_keyframe( 600, pose );

    construct_trajectory();
}

//------------------------------------------------------------------------------

void Trajectory::construct_pendulum_trajectory( const unsigned int& period_in_frames, const bool& forward, const Vector3& orientation, const double& frequency, const double& amplitude ) {

    assert( period_in_frames % 4 == 0 );

    unsigned int frames_over_spline = period_in_frames / 4;

    this->forward = forward;
    if( !forward ) {
        frame_tic = frames_over_spline + 1;
        prev_keyframe_id = 2;
        next_keyframe_id = 1;
    } else {
        frame_tic = frames_over_spline + 1;
        prev_keyframe_id = 1;
        next_keyframe_id = 2;
    }

    Pose pose;
    ControlPoint cp;
    Vector3 pt, rot;
    Matrix3 A = EulerAngle::matrix3( EULER_ANGLE_XYZ, EULER_HANDED_RIGHT, orientation );

    // dummy end control point
    pt = A * Vector3(-frequency/2.0, amplitude, 0.0);
    rot = Vector3( PI/2, PI/2, 0.0 ) + orientation;
    pose = Pose( pt, rot );
    cp = ControlPoint( pose.position, CONTROL_POINT_END );
    controlpoints.push_back( cp );

    // begin visible spline control points
    pt = A * Vector3(-frequency/4.0, 0.0, 0.0);
    rot = Vector3( PI/2, PI/4, 0.0 ) + orientation;
    pose = Pose( pt, rot );
    poses.push_back( pose );
    insert_keyframe( 1, pose );
    cp = ControlPoint( pose.position, CONTROL_POINT_MIDDLE );
    controlpoints.push_back( cp );

    pt = A * Vector3(0.0, -amplitude, 0.0);
    rot = Vector3( PI/2, PI/2, 0.0 ) + orientation;
    pose = Pose( pt, rot );
    poses.push_back( pose );
    insert_keyframe( frames_over_spline + 1, pose );
    cp = ControlPoint( pose.position, CONTROL_POINT_MIDDLE );
    controlpoints.push_back( cp );

    pt = A * Vector3(frequency/4.0, 0.0, 0.0);
    rot = Vector3( PI/2, PI*3/4, 0.0 ) + orientation;
    pose = Pose( pt, rot );
    poses.push_back( pose );
    insert_keyframe( frames_over_spline * 2 + 1, pose );
    cp = ControlPoint( pose.position, CONTROL_POINT_MIDDLE );
    controlpoints.push_back( cp );
    // end visible spline control points

    // dummy end control point
    pt = A * Vector3(frequency/2.0, amplitude, 0.0);
    rot = Vector3( PI/2, PI/2, 0.0 ) + orientation;
    pose = Pose( pt, rot );
    cp = ControlPoint( pose.position, CONTROL_POINT_END );
    controlpoints.push_back( cp );

    cyclic = true;
    reversible = true;
}

//------------------------------------------------------------------------------

void Trajectory::construct_cyclic_circle_trajectory( const unsigned int& period_in_frames, const bool& forward, const Vector3& orientation, const double& radius, const Vector3& center ) {

    assert( period_in_frames % 8 == 0 );
    assert( radius > 0.0 );

    unsigned int frames_over_spline = period_in_frames / 8;

    frame_tic = 0;
    prev_keyframe_id = 0;
    next_keyframe_id = 1;

    Pose pose;
    ControlPoint cp;
    Vector3 pt, rot;
    Matrix3 A = EulerAngle::matrix3( EULER_ANGLE_XYZ, EULER_HANDED_RIGHT, orientation );

    // dummy end control point
    pt = A * Vector3( -cos(PI/4) * radius + center.x(), -sin(PI/4) * radius + center.y(), center.z() );
    rot = Vector3( PI/2, PI/4, 0.0 ) + orientation;
    pose = Pose( pt, rot );
    cp = ControlPoint( pose.position, CONTROL_POINT_END );
    controlpoints.push_back( cp );

    // begin visible spline control points
    pt = A * Vector3(center.x(), -1.0 * radius + center.y(), center.z());
    rot = Vector3( PI/2, PI/2, 0.0 ) + orientation;
    pose = Pose( pt, rot );
    poses.push_back( pose );
    insert_keyframe( 1, pose );
    cp = ControlPoint( pose.position, CONTROL_POINT_MIDDLE );
    controlpoints.push_back( cp );

    pt = A * Vector3(cos(PI/4) * radius + center.x(), -sin(PI/4) * radius + center.y(), center.z());
    rot = Vector3( PI/2, PI*3/4, 0.0 ) + orientation;
    pose = Pose( pt, rot );
    insert_keyframe( frames_over_spline + 1, pose );
    cp = ControlPoint( pose.position, CONTROL_POINT_MIDDLE );
    controlpoints.push_back( cp );

    pt = A * Vector3(1.0 * radius + center.x(), center.y(), center.z());
    rot = Vector3( PI/2, PI, 0.0 ) + orientation;
    pose = Pose( pt, rot );
    poses.push_back( pose );
    insert_keyframe( frames_over_spline * 2 + 1, pose );
    cp = ControlPoint( pose.position, CONTROL_POINT_MIDDLE );
    controlpoints.push_back( cp );

    pt = A * Vector3(cos(PI/4) * radius + center.x(), sin(PI/4) * radius + center.y(), center.z());
    rot = Vector3( PI/2, PI*5/4, 0.0 ) + orientation;
    pose = Pose( pt, rot );
    poses.push_back( pose );
    insert_keyframe( frames_over_spline * 3 + 1, pose );
    cp = ControlPoint( pose.position, CONTROL_POINT_MIDDLE );
    controlpoints.push_back( cp );

    pt = A * Vector3(center.x(), 1.0 * radius + center.y(), center.z());
    rot = Vector3( PI/2, PI*3/2, 0.0 ) + orientation;
    pose = Pose( pt, rot );
    poses.push_back( pose );
    insert_keyframe( frames_over_spline * 4 + 1, pose );
    cp = ControlPoint( pose.position, CONTROL_POINT_MIDDLE );
    controlpoints.push_back( cp );

    pt = A * Vector3(-cos(PI/4) * radius + center.x(), sin(PI/4) * radius + center.y(), center.z());
    rot = Vector3( PI/2, PI*7/4, 0.0 ) + orientation;
    pose = Pose( pt, rot );
    poses.push_back( pose );
    insert_keyframe( frames_over_spline * 5 + 1, pose );
    cp = ControlPoint( pose.position, CONTROL_POINT_MIDDLE );
    controlpoints.push_back( cp );

    pt = A * Vector3(-1.0 * radius + center.x(), center.y(), center.z());
    rot = Vector3( PI/2, PI*2, 0.0 ) + orientation;
    pose = Pose( pt, rot );
    poses.push_back( pose );
    insert_keyframe( frames_over_spline * 6 + 1, pose );
    cp = ControlPoint( pose.position, CONTROL_POINT_MIDDLE );
    controlpoints.push_back( cp );

    pt = A * Vector3(-cos(PI/4) * radius + center.x(), -sin(PI/4) * radius + center.y(), center.z());
    rot = Vector3( PI/2, PI*2+PI/4, 0.0 ) + orientation;
    pose = Pose( pt, rot );
    poses.push_back( pose );
    insert_keyframe( frames_over_spline * 7 + 1, pose );
    cp = ControlPoint( pose.position, CONTROL_POINT_MIDDLE );
    controlpoints.push_back( cp );

    pt = A * Vector3(center.x(),-1.0 * radius + center.y(), center.z());
    rot = Vector3( PI/2, PI*2+PI/2, 0.0 ) + orientation;
    pose = Pose( pt, rot );
    poses.push_back( pose );
    insert_keyframe( frames_over_spline * 8 + 1, pose );
    cp = ControlPoint( pose.position, CONTROL_POINT_MIDDLE );
    controlpoints.push_back( cp );
    // end visible spline control points

    // dummy end control point
    pt = A * Vector3(cos(PI/4) * radius + center.x(), -sin(PI/4) * radius + center.y(), center.z());
    rot = Vector3( PI/2, PI*2+PI*3/4, 0.0 ) + orientation;
    pose = Pose( pt, rot );
    cp = ControlPoint( pose.position, CONTROL_POINT_END );
    controlpoints.push_back( cp );

    cyclic = true;
}

//------------------------------------------------------------------------------
void Trajectory::construct_random_trajectory( const unsigned int& num_keyframes ) {
    assert( num_keyframes > 2 );

    for( unsigned int i = 0; i < num_keyframes; i++ ) {
        unsigned int frame = i * 1000;
        double x = (double) rand() / (double) RAND_MAX * 100.0;
        //double y = (double) rand() / (double) RAND_MAX * 100.0;
        double y = 0.0;
        double z = (double) rand() / (double) RAND_MAX * 100.0;
        //double theta = (double) rand() / (double) RAND_MAX * 2.0 * PI;
        //insert_keyframe( frame, Pose( Vector3( x, y, z ), Vector3( 0.0, theta, 0.0 ) ) );
        insert_keyframe( frame, Pose( Vector3( x, y, z ), Vector3( 0.0, 0.0, 0.0 ) ) );
    }
    construct_trajectory();
    prev_keyframe_id = 0;
    next_keyframe_id = 1;
    spline_basis = CubicSpline::basisUniformNonrationalBSpline();
    spline_arclength_map = CubicSpline::map_spline( CUBIC_SPLINE_B, controlpoints );

    // Reprocess the trajectory for orientations
/*
    for( unsigned int i = 0; i < keyframes(); i++ ) {
        Vector3 tangent, x_axis, y_axis;

        tangent = controlpoints.at( i + 2 ).position - controlpoints.at( i ).position;
        //tangent = controlpoints.at( i ).position - controlpoints.at( i + 2 ).position;
        tangent.normalize();
        //Vector3 orientation = Vector3::cross( tangent, Vector3( 0.0, 1.0, 0.0 ) );
        keyframe( i ).pose.orientation = tangent;
        //keyframe( i ).pose.orientation = orientation;
        std::cout << "Tangent: ";
        tangent.print();
        std::cout << "\n";
    }
*/
/*
    for( unsigned int i = 0; i < keyframes(); i++ ) {
        Vector3 tangent, x_axis, z_axis;
        double dp_x, dp_z, theta;

        tangent = controlpoints.at( i + 2 ).position - controlpoints.at( i ).position;
        tangent.normalize();
        x_axis = Vector3( 1.0, 0.0, 0.0 );
        z_axis = Vector3( 0.0, 0.0, 1.0 );

        dp_x = Vector3::dot( tangent, x_axis );
        dp_z = Vector3::dot( tangent, z_axis );
        theta = acos( dp_x );
        if( dp_z < 0.0 ) theta = -( 2 * PI - theta );
        //theta = 2 * PI - theta;
        std::cout << "Theta: " << theta << "\n";
        keyframe( i ).pose.orientation = Vector3( 0.0, theta, 0.0 );
    }
*/
///*

    Vector3 tangent, x_axis, z_axis;
    double dp_x, dp_z, theta;

    x_axis = Vector3( 1.0, 0.0, 0.0 );
    z_axis = Vector3( 0.0, 0.0, 1.0 );

    for( unsigned int i = 0; i < keyframes() - 1; i++ ) {
        Eigen::MatrixXd C = CubicSpline::blend( spline_basis, controlpoints, i + 2 );

        tangent = CubicSpline::tangent( C, 0.0 );
        tangent.normalize();

        dp_x = Vector3::dot( tangent, x_axis );
        dp_z = Vector3::dot( tangent, z_axis );
//        theta = acos( dp_x );
//        if( dp_z < 0.0 ) theta = -( 2 * PI - theta );
        theta = acos( dp_z );
        //if( dp_x < 0.0 ) theta = -( 2 * PI - theta );
        keyframe( i ).pose.orientation = Vector3( 0.0, theta, 0.0 );

        if( i == keyframes() - 2 ) {

            tangent = CubicSpline::tangent( C, 1.0 );
            tangent.normalize();

            dp_x = Vector3::dot( tangent, x_axis );
            dp_z = Vector3::dot( tangent, z_axis );
//            theta = acos( dp_x );
//            if( dp_z < 0.0 ) theta = -( 2 * PI - theta );
            theta = acos( dp_z );
            //if( dp_x < 0.0 ) theta = -( 2 * PI - theta );
            keyframe( i + 1 ).pose.orientation = Vector3( 0.0, theta, 0.0 );
        }
    }
//*/

}

//------------------------------------------------------------------------------
