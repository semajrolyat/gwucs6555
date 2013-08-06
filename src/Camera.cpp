//------------------------------------------------------------------------------
#include <cs6555/Camera.h>

#include <cs6555/Math/Vector4.h>
#include <cs6555/Math/Quaternion.h>

//------------------------------------------------------------------------------
// Constructors
//------------------------------------------------------------------------------
/// Default Constructor
Camera::Camera( void ) {
    position = Vector3( 0.0, 0.0, 1.0 );
    viewpoint = Vector3( 0.0, 0.0, 0.0 );
    up = Vector3( 0.0, 1.0, 0.0 );

    view.identity();
    projection.identity();

    near = 1.0;
    far = 1000.0;

}

//------------------------------------------------------------------------------
// Destructor
//------------------------------------------------------------------------------
/// Destructor
Camera::~Camera( void ) {

}

//------------------------------------------------------------------------------
// Projection
//------------------------------------------------------------------------------
/// Determine the camera projection matrix
void Camera::project( void ) {
    Matrix4 T, R;
    Vector3 U, V, N;
    Vector3 Vprime;

    Vprime = up;

    N = viewpoint - position;
    N.normalize();

    V = Vprime - ( N * Vector3::dot( Vprime, N) );
    V.normalize();
    //up = V;

    U = Vector3::cross( N, V );
    U.normalize();

    Vector3 Vinv = Vector3::invert( position );

    T.identity();
    T.setTranslation( Vinv.x(), Vinv.y(), Vinv.z() );

    R.identity();
    R(0,0) = U.x();     R(0,1) = U.y();     R(0,2) = U.z();
    R(1,0) = V.x();     R(1,1) = V.y();     R(1,2) = V.z();
    R(2,0) = N.x();     R(2,1) = N.y();     R(2,2) = N.z();

    view = R * T;

    double h = 0.75;
    double d = near;
    double f = far;

    perspective.identity();
    perspective(0,0) = d / h;
    perspective(1,1) = d / h;
    perspective(2,2) = f / (f-d);
    perspective(2,3) = -d * f / (f-d);
    perspective(3,2) = 1.0;
    perspective(3,3) = 0.0;

    projection = perspective * view;
    projectionInv = Matrix4::inverse( projection );
}

//------------------------------------------------------------------------------
// Movement
//------------------------------------------------------------------------------
/// Tracking movement to the left
/// e.g. orbit the camera around the world y-axis while maintaining the viewpoint
void Camera::track_left( const double& step ) {
    if( up == Vector3( 1.0, 0.0, 0.0 ) ) {
        Matrix3 R = Matrix3::rotX( -step );
        Vector3 p = R * Vector3( 0.0, position.y(), position.z() );
        Vector3 v = R * Vector3( 0.0, viewpoint.y(), viewpoint.z() );
        position = Vector3( position.x(), p.y(), p.z() );
        viewpoint = Vector3( viewpoint.x(), v.y(), v.z() );
    } else if( up == Vector3( -1.0, 0.0, 0.0 )  ) {
        Matrix3 R = Matrix3::rotX( step );
        Vector3 p = R * Vector3( 0.0, position.y(), position.z() );
        Vector3 v = R * Vector3( 0.0, viewpoint.y(), viewpoint.z() );
        position = Vector3( position.x(), p.y(), p.z() );
        viewpoint = Vector3( viewpoint.x(), v.y(), v.z() );
    } else if( up == Vector3( 0.0, 1.0, 0.0 ) ) {
        Matrix3 R = Matrix3::rotY( -step );
        Vector3 p = R * Vector3( position.x(), 0.0, position.z() );
        Vector3 v = R * Vector3( viewpoint.x(), 0.0, viewpoint.z() );
        position = Vector3( p.x(), position.y(), p.z() );
        viewpoint = Vector3( v.x(), viewpoint.y(), v.z() );
    } else if( up == Vector3( 0.0, -1.0, 0.0 )  ) {
        Matrix3 R = Matrix3::rotY( step );
        Vector3 p = R * Vector3( position.x(), 0.0, position.z() );
        Vector3 v = R * Vector3( viewpoint.x(), 0.0, viewpoint.z() );
        position = Vector3( p.x(), position.y(), p.z() );
        viewpoint = Vector3( v.x(), viewpoint.y(), v.z() );
    } else if( up == Vector3( 0.0, 0.0, 1.0 ) ) {
        Matrix3 R = Matrix3::rotZ( -step );
        Vector3 p = R * Vector3( position.x(), position.y(), 0.0 );
        Vector3 v = R * Vector3( viewpoint.x(), viewpoint.y(), 0.0 );
        position = Vector3( p.x(), p.y(), position.z() );
        viewpoint = Vector3( v.x(), v.y(), viewpoint.z() );
    } else if( up == Vector3( 0.0, 0.0, -1.0 )  ) {
        Matrix3 R = Matrix3::rotZ( step );
        Vector3 p = R * Vector3( position.x(), position.y(), 0.0 );
        Vector3 v = R * Vector3( viewpoint.x(), viewpoint.y(), 0.0 );
        position = Vector3( p.x(), p.y(), position.z() );
        viewpoint = Vector3( v.x(), v.y(), viewpoint.z() );
    }
}

//------------------------------------------------------------------------------
/// Tracking movement to the right
/// e.g. orbit the camera around the world y-axis while maintaining the viewpoint
void Camera::track_right( const double& step ) {
    if( up == Vector3( 1.0, 0.0, 0.0 ) ) {
        Matrix3 R = Matrix3::rotX( step );
        Vector3 p = R * Vector3( 0.0, position.y(), position.z() );
        Vector3 v = R * Vector3( 0.0, viewpoint.y(), viewpoint.z() );
        position = Vector3( position.x(), p.y(), p.z() );
        viewpoint = Vector3( viewpoint.x(), v.y(), v.z() );
    } else if( up == Vector3( -1.0, 0.0, 0.0 )  ) {
        Matrix3 R = Matrix3::rotX( -step );
        Vector3 p = R * Vector3( 0.0, position.y(), position.z() );
        Vector3 v = R * Vector3( 0.0, viewpoint.y(), viewpoint.z() );
        position = Vector3( position.x(), p.y(), p.z() );
        viewpoint = Vector3( viewpoint.x(), v.y(), v.z() );
    } else if( up == Vector3( 0.0, 1.0, 0.0 ) ) {
        Matrix3 R = Matrix3::rotY( step );
        Vector3 p = R * Vector3( position.x(), 0.0, position.z() );
        Vector3 v = R * Vector3( viewpoint.x(), 0.0, viewpoint.z() );
        position = Vector3( p.x(), position.y(), p.z() );
        viewpoint = Vector3( v.x(), viewpoint.y(), v.z() );
    } else if( up == Vector3( 0.0, -1.0, 0.0 ) ) {
        Matrix3 R = Matrix3::rotY( -step );
        Vector3 p = R * Vector3( position.x(), 0.0, position.z() );
        Vector3 v = R * Vector3( viewpoint.x(), 0.0, viewpoint.z() );
        position = Vector3( p.x(), position.y(), p.z() );
        viewpoint = Vector3( v.x(), viewpoint.y(), v.z() );
    } else if( up == Vector3( 0.0, 0.0, 1.0 ) ) {
        Matrix3 R = Matrix3::rotZ( step );
        Vector3 p = R * Vector3( position.x(), position.y(), 0.0 );
        Vector3 v = R * Vector3( viewpoint.x(), viewpoint.y(), 0.0 );
        position = Vector3( p.x(), p.y(), position.z() );
        viewpoint = Vector3( v.x(), v.y(), viewpoint.z() );
    } else if( up == Vector3( 0.0, 0.0, -1.0 )  ) {
        Matrix3 R = Matrix3::rotZ( -step );
        Vector3 p = R * Vector3( position.x(), position.y(), 0.0 );
        Vector3 v = R * Vector3( viewpoint.x(), viewpoint.y(), 0.0 );
        position = Vector3( p.x(), p.y(), position.z() );
        viewpoint = Vector3( v.x(), v.y(), viewpoint.z() );
    }
}

//------------------------------------------------------------------------------
/// Dolly movement to toward the viewpoint
/// e.g. shorten the vector between the camera position and the viewpoint
/// Decrease the focal length
/// step is a percentage of focal length to step in
void Camera::dolly_in( const double& step ) {
    assert( step < 100.0 );
    Vector3 pv = position - viewpoint;
    double mag = pv.magnitude();
    pv.normalize();
    Vector3 p = pv * ( mag * (1.0 - step/100));
    position = viewpoint + p;
}

//------------------------------------------------------------------------------
/// Dolly movement to away from the viewpoint
/// e.g. lengthen the vector between the camera position and the viewpoint
/// Increase the focal length
/// step is a percentage of focal length to step out
void Camera::dolly_out( const double& step ) {
    assert( step < 100.0 );
    Vector3 pv = position - viewpoint;
    double mag = pv.magnitude();
    pv.normalize();
    Vector3 p = pv * ( mag * (1.0 + step/100));
    position = viewpoint + p;
}

//------------------------------------------------------------------------------
