/*-----------------------THE GEORGE WASHINGTON UNIVERSITY-----------------------
James Taylor                                                         jrt@gwu.edu

Pose class implementation

------------------------------------------------------------------------------*/

#include <cs6555/Pose.h>

#include <cs6555/Math/EulerAngle.h>
#include <cs6555/Math/Quaternion.h>

//------------------------------------------------------------------------------
// Constructors
//------------------------------------------------------------------------------
/// Default Constructor
Pose::Pose( void ) :
    position( 0.0, 0.0, 0.0 ),
    orientation( 0.0, 0.0, 0.0 )
{
    transform.identity( );
    orientation_type = POSE_BY_EULERANGLES;
}
//------------------------------------------------------------------------------
/// Copy Constructor
Pose::Pose( const Pose& pose ) {
    position = pose.position;
    orientation = pose.orientation;
    orientation_q = pose.orientation_q;
    transform = pose.transform;
    orientation_type = pose.orientation_type;
}
//------------------------------------------------------------------------------
/// Vector Constructor
Pose::Pose( const Vector3& position, const Vector3& orientation ) {
    this->position = position;
    this->orientation = orientation;
    transform.identity( );
    orientation_type = POSE_BY_EULERANGLES;
}

//------------------------------------------------------------------------------
/// Quaternion Constructor
Pose::Pose( const Vector3& position, const Quaternion& orientation ) {
    this->position = position;
    this->orientation_q = orientation;
    transform.identity( );
    orientation_type = POSE_BY_QUATERNION;
}
//------------------------------------------------------------------------------
/// Component Constructor
Pose::Pose( const double& x, const double& y, const double& z, const double& theta, const double& phi, const double& psi ) :
    position( x, y, z ),
    orientation( theta, phi, psi )
{
    transform.identity( );
    orientation_type = POSE_BY_EULERANGLES;
}
//------------------------------------------------------------------------------
Pose::Pose( const EPoseOrientationType& type ) {
    position = Vector3( 0.0, 0.0, 0.0 ),
    transform.identity( );
    orientation_type = type;
    switch( type ) {
    case POSE_BY_EULERANGLES:
    default:
        orientation = Vector3( 0.0, 0.0, 0.0 );
        break;
    case POSE_BY_QUATERNION:
        orientation_q = Quaternion( 0.0, 0.0, 0.0, 0.0 );
        break;
    }
}
//------------------------------------------------------------------------------
Pose::Pose( const EPoseOrientationType& type, const Vector3& position, const Vector3& orientation ) {
    this->position = position,
    transform.identity( );
    orientation_type = type;
    switch( type ) {
    case POSE_BY_EULERANGLES:
    default:
        this->orientation = Vector3( 0.0, 0.0, 0.0 );
        break;
    case POSE_BY_QUATERNION:
        Quaternion Qx = Quaternion( cos(orientation.x()/2), sin(orientation.x()/2), 0.0, 0.0 );
        Quaternion Qy = Quaternion( cos(orientation.y()/2), 0.0, sin(orientation.y()/2), 0.0 );
        Quaternion Qz = Quaternion( cos(orientation.z()/2), 0.0, 0.0, sin(orientation.z()/2) );
        Qx.normalize();
        Qy.normalize();
        Qz.normalize();
        orientation_q = Qx * Qy * Qz;
        break;
    }
}
//------------------------------------------------------------------------------
Pose::Pose( const EPoseOrientationType& type, const double& x, const double& y, const double& z, const double& theta, const double& phi, const double& psi ) {
    this->position = Vector3( x, y, z ),
    transform.identity( );
    orientation_type = type;
    switch( type ) {
    case POSE_BY_EULERANGLES:
    default:
        this->orientation = Vector3( theta, phi, psi );
        break;
    case POSE_BY_QUATERNION:
        Quaternion Qx = Quaternion( cos(theta/2), sin(theta/2), 0.0, 0.0 );
        Quaternion Qy = Quaternion( cos(phi/2), 0.0, sin(phi/2), 0.0 );
        Quaternion Qz = Quaternion( cos(psi/2), 0.0, 0.0, sin(psi/2) );
        Qx.normalize();
        Qy.normalize();
        Qz.normalize();
        orientation_q = Qx * Qy * Qz;
        break;
    }
}


//------------------------------------------------------------------------------
// Destructor
//------------------------------------------------------------------------------
/// Destructor
Pose::~Pose( void ) {

}

//------------------------------------------------------------------------------
// Transformations
//------------------------------------------------------------------------------
/// Returns a transformation matrix computed by Eular Angles
Matrix4 Pose::transformByEulerAngle( void ) {
    transform = EulerAngle::matrix4( EULER_ANGLE_XYZ, EULER_HANDED_RIGHT, orientation.x(), orientation.y(), orientation.z() );
    transform.setTranslation( position.x(), position.y(), position.z() );
    return transform;
}

//------------------------------------------------------------------------------
/// Returns a transformation matrix computed by Quaternion
Matrix4 Pose::transformByQuaternion( void ) {
    if( orientation_type == POSE_BY_EULERANGLES ) {
        Quaternion Qx = Quaternion( cos(orientation.x()/2), sin(orientation.x()/2), 0.0, 0.0 );
        Quaternion Qy = Quaternion( cos(orientation.y()/2), 0.0, sin(orientation.y()/2), 0.0 );
        Quaternion Qz = Quaternion( cos(orientation.z()/2), 0.0, 0.0, sin(orientation.z()/2) );
        Qx.normalize();
        Qy.normalize();
        Qz.normalize();

        Quaternion Q = Qx * Qy * Qz;
        transform = Q.matrix4();

        transform.setTranslation( position.x(), position.y(), position.z() );

        return transform;
    } else {        //POSE_BY_QUATERNION
        transform = orientation_q.matrix4();
        transform.setTranslation( position.x(), position.y(), position.z() );
        return transform;
    }
}

//------------------------------------------------------------------------------

Matrix3 Pose::matrix3( void ) {
    if( orientation_type == POSE_BY_EULERANGLES ) {
        Quaternion Qx = Quaternion( cos(orientation.x()/2), sin(orientation.x()/2), 0.0, 0.0 );
        Quaternion Qy = Quaternion( cos(orientation.y()/2), 0.0, sin(orientation.y()/2), 0.0 );
        Quaternion Qz = Quaternion( cos(orientation.z()/2), 0.0, 0.0, sin(orientation.z()/2) );
        Qx.normalize();
        Qy.normalize();
        Qz.normalize();

        Quaternion Q = Qx * Qy * Qz;
        Matrix3 rot = Q.matrix3();

        return rot;
    } else {    //POSE_BY_QUATERNION
        return orientation_q.matrix3();
    }

}
