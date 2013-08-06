#ifndef _MATH_H_
#define _MATH_H_

#include <cs6555/Math/EulerAngle.h>
#include <cs6555/Math/Quaternion.h>
#include <cs6555/Math/Vector3.h>
#include <cs6555/Math/Matrix3.h>
#include <cs6555/Math/Matrix4.h>

namespace Math {

static inline double bilinear_interpolate( const double& x, const double& y, const double& x0, const double& y0, const double& x1, const double& y1, const double& f00, const double& f10, const double& f01, const double& f11 ) {
    double p1 = f00/((x1 - x0)*(y1 - y0)) * (x1 - x) * (y1 - y);
    double p2 = f10/((x1 - x0)*(y1 - y0)) * (x - x0) * (y1 - y);
    double p3 = f01/((x1 - x0)*(y1 - y0)) * (x1 - x) * (y - y0);
    double p4 = f11/((x1 - x0)*(y1 - y0)) * (x - x0) * (y - y0);
    return p1 + p2 + p3 + p4;
}

static inline double trilinear_interpolate( const double& x, const double& y, const double& z, const double& x0, const double& y0, const double& z0, const double& x1, const double& y1, const double& z1, const double& f000, const double& f100, const double& f001, const double& f101, const double& f010, const double& f110, const double& f011, const double& f111 ) {
    double xd = (x - x0) / (x1 - x0);
    double yd = (y - y0) / (y1 - y0);
    double zd = (z - z0) / (z1 - z0);

    double c00 = f000 * (1 - xd) + f100 * xd;
    double c10 = f010 * (1 - xd) + f110 * xd;
    double c01 = f001 * (1 - xd) + f101 * xd;
    double c11 = f011 * (1 - xd) + f111 * xd;

    double c0 = c00 * (1 - yd) + c10 * yd;
    double c1 = c01 * (1 - yd) + c11 * yd;

    return c0 * (1 - zd) + c1 * zd;
}

#define DEG_PER_RAD  180.0/PI
#define RAD_PER_DEG  PI/180.0

/// normalize an angle in the interval [-PI,PI)
static inline double normalize_angle( double theta ) {
    while (theta <= -PI)
        theta += 2.0 * PI;
    while (theta > PI)
        theta -= 2.0 * PI;
    return theta;
}

//------------------------------------------------------------------------------
/// returns the angle between two unit vectors located in the x,z plane
static inline double angle_in_x_z( const Vector3& start_orientation, const Vector3& end_orientation ) {
    double start_theta = atan2( start_orientation.x(), start_orientation.z() );
    double end_theta = atan2( end_orientation.x(), end_orientation.z() );
    double theta = end_theta - start_theta;
    return normalize_angle( theta );
}

/*
//------------------------------------------------------------------------------
/// down and dirty integration of orientation.
/// Note: passed by value
/// Artifact of earlier development but still necessary for initial state
static Vector3 integrate_orientation( Vector3 local_orientation, Vector3 world_orientation, Vector3 desired_orientation, double max_dtheta ) {
    local_orientation.normalize();

    // transform the orientation toward the boid front in world space
    Matrix3 R_prev = EulerAngle::matrix3( EULER_ANGLE_XYZ, EULER_HANDED_LEFT, world_orientation.x(), world_orientation.y(), world_orientation.z() );
    Vector3 orientation = R_prev * local_orientation;
    orientation.normalize();

    desired_orientation.normalize();

    // calculate the angle between the direction to the motivator and the orientation of the boid
    double gross_theta = Math::angle_in_x_z( orientation, desired_orientation );

    // determine the possible amount of yaw for this frame depending on the boid->yaw_rate_per_frame
    double dtheta = gross_theta;
    if( fabs(gross_theta) > max_dtheta ) {
        if( dtheta < 0 ) {
            dtheta = -max_dtheta;
        } else {
            dtheta = max_dtheta;
        }
    }

    // integrate the rotation
    double theta = world_orientation.y() + dtheta;
    return Vector3( 0.0, theta, 0.0 );
}

//------------------------------------------------------------------------------
/// down and dirty integration of velocity given a scalar acceleration and max velocity
/// Note: passed by value
/// Artifact of earlier development but still necessary for initial state
static Vector3 integrate_velocity( Vector3 local_orientation, Vector3 world_orientation, Vector3 world_velocity, double max_ddx, double max_dx ) {
    local_orientation.normalize();

    // transform the orientation toward the boid front in world space
    Matrix3 R_now = EulerAngle::matrix3( EULER_ANGLE_XYZ, EULER_HANDED_LEFT, world_orientation.x(), world_orientation.y(), world_orientation.z() );
    Vector3 orientation_now = R_now * local_orientation;
    orientation_now.normalize();

    // integrate the velocity
    Vector3 v = world_velocity + orientation_now * max_ddx;
    double m = v.magnitude();

    if( fabs( m ) > max_dx ) {
        v.normalize();
        v *= max_dx;
    }

    return v;
}

//------------------------------------------------------------------------------
/// down and dirty integration of velocity given an acceleration vector.
/// Note: passed by value
/// Artifact of earlier development but still necessary for initial state
static Vector3 integrate_velocity( Vector3 local_orientation, Vector3 world_orientation, Vector3 world_velocity, Vector3 max_ddx ) {
    local_orientation.normalize();

    // transform the orientation toward the boid front in world space
    Matrix3 R_now = EulerAngle::matrix3( EULER_ANGLE_XYZ, EULER_HANDED_LEFT, world_orientation.x(), world_orientation.y(), world_orientation.z() );
    Vector3 orientation_now = R_now * local_orientation;
    orientation_now.normalize();

    // integrate the velocity with impulse force max_ddx
    Vector3 v = world_velocity + max_ddx;

    return v;
}
*/
//------------------------------------------------------------------------------
/// down and dirty integration of position
/// Note: passed by value
/// Artifact of earlier development but still necessary for initial state
static Vector3 integrate_position( Vector3 local_orientation, Vector3 world_orientation, Vector3 world_position, double dx ) {
    local_orientation.normalize();

    // transform the orientation toward the boid front in world space
    Matrix3 R_now = EulerAngle::matrix3( EULER_ANGLE_XYZ, EULER_HANDED_LEFT, world_orientation.x(), world_orientation.y(), world_orientation.z() );
    Vector3 orientation_now = R_now * local_orientation;
    orientation_now.normalize();

    // integrate the position
    return world_position + orientation_now * dx;
}
/*
//------------------------------------------------------------------------------
/// computation of an acceleration vector from a scalar acceleration and vector orientation
/// Note: passed by value
/// Artifact of earlier development but still necessary for initial state
static Vector3 calculate_linear_acceleration( Vector3 local_orientation, Vector3 world_orientation, double ddx ) {
    local_orientation.normalize();

    // transform the orientation toward the boid front in world space
    Matrix3 R_now = EulerAngle::matrix3( EULER_ANGLE_XYZ, EULER_HANDED_LEFT, world_orientation.x(), world_orientation.y(), world_orientation.z() );
    Vector3 orientation_now = R_now * local_orientation;
    orientation_now.normalize();

    return orientation_now * ddx;
}
*/

};  // namespace

#endif // _MATH_H_
