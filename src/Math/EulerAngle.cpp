/*
Reference
    Eberly, D. - Euler Angle Formulas - www.geometrictools.com - 1999-2012 -
                 http://www.geometrictools.com/Documentation/EulerAngles.pdf

    Wikipedia - Euler Angles - en.wikipedia.org
                http://en.wikipedia.org/wiki/Euler_angles

Note: this implementation could have been simplified by concatenating rotation
helpers from matrix3 in the specified order; however, deriving from the factored
form will be faster as it will minimize the O(3n^2) matrix multiplication cost to
O(n).
*/

#include <cs6555/Math/EulerAngle.h>
#include <math.h>

/*
//------------------------------------------------------------------------------
// Constructors
//------------------------------------------------------------------------------
/// Default Constructor
EulerAngle::EulerAngle( void ) {

}
//------------------------------------------------------------------------------
// Destructor
//------------------------------------------------------------------------------
/// Destructor
EulerAngle::~EulerAngle( void ) {

}
*/
//------------------------------------------------------------------------------
// Generate Matrix3 Rotation Matrix
//------------------------------------------------------------------------------
/// Generate a Matrix3 Rotation Matrix by Components
Matrix3 EulerAngle::matrix3( const EEulerAngles angle, const EEulerHanded handed, const double& theta, const double& phi, const double& psi ) {
    Matrix4 result;

    switch( angle ) {
    case EULER_ANGLE_XZX:
        result = XZX( handed, theta, phi, psi );
        break;
    case EULER_ANGLE_XYX:
        result = XYX( handed, theta, phi, psi );
        break;
    case EULER_ANGLE_YXY:
        result = YXY( handed, theta, phi, psi );
        break;
    case EULER_ANGLE_YZY:
        result = YZY( handed, theta, phi, psi );
        break;
    case EULER_ANGLE_ZYZ:
        result = ZYZ( handed, theta, phi, psi );
        break;
    case EULER_ANGLE_ZXZ:
        result = ZXZ( handed, theta, phi, psi );
        break;
    case EULER_ANGLE_XZY:
        result = XZY( handed, theta, phi, psi );
        break;
    case EULER_ANGLE_XYZ:
    default:
        result = XYZ( handed, theta, phi, psi );
        break;
    case EULER_ANGLE_YXZ:
        result = YXZ( handed, theta, phi, psi );
        break;
    case EULER_ANGLE_YZX:
        result = YZX( handed, theta, phi, psi );
        break;
    case EULER_ANGLE_ZYX:
        result = ZYX( handed, theta, phi, psi );
        break;
    case EULER_ANGLE_ZXY:
        result = ZXY( handed, theta, phi, psi );
        break;
    }
    return result.rotation();
}
//------------------------------------------------------------------------------
/// Generate a Matrix3 Rotation Matrix by Vector | Vector3(x,y,z) -> EularAngle(theta,phi,psi)
Matrix3 EulerAngle::matrix3( const EEulerAngles angle, const EEulerHanded handed, const Vector3& u ) {
    return matrix3( angle, handed, u.x(), u.y(), u.z() );
}
//------------------------------------------------------------------------------
// Generate Matrix4 Rotation Matrix
//------------------------------------------------------------------------------
/// Generate a Matrix4 Rotation Matrix by Components
Matrix4 EulerAngle::matrix4( const EEulerAngles angle, const EEulerHanded handed, const double& theta, const double& phi, const double& psi ) {
    Matrix4 result;

    switch( angle ) {
    case EULER_ANGLE_XZX:
        result = XZX( handed, theta, phi, psi );
        break;
    case EULER_ANGLE_XYX:
        result = XYX( handed, theta, phi, psi );
        break;
    case EULER_ANGLE_YXY:
        result = YXY( handed, theta, phi, psi );
        break;
    case EULER_ANGLE_YZY:
        result = YZY( handed, theta, phi, psi );
        break;
    case EULER_ANGLE_ZYZ:
        result = ZYZ( handed, theta, phi, psi );
        break;
    case EULER_ANGLE_ZXZ:
        result = ZXZ( handed, theta, phi, psi );
        break;
    case EULER_ANGLE_XZY:
        result = XZY( handed, theta, phi, psi );
        break;
    case EULER_ANGLE_XYZ:
    default:
        result = XYZ( handed, theta, phi, psi );
        break;
    case EULER_ANGLE_YXZ:
        result = YXZ( handed, theta, phi, psi );
        break;
    case EULER_ANGLE_YZX:
        result = YZX( handed, theta, phi, psi );
        break;
    case EULER_ANGLE_ZYX:
        result = ZYX( handed, theta, phi, psi );
        break;
    case EULER_ANGLE_ZXY:
        result = ZXY( handed, theta, phi, psi );
        break;
    }
    return result;
}
//------------------------------------------------------------------------------
/// Generate a Matrix3 Rotation Matrix by Vector | Vector3(x,y,z) -> EularAngle(theta,phi,psi)
Matrix4 EulerAngle::matrix4( const EEulerAngles angle, const EEulerHanded handed, const Vector3& u ) {
    return matrix4( angle, handed, u.x(), u.y(), u.z() );
}
//------------------------------------------------------------------------------
// Class Utilities
//------------------------------------------------------------------------------
/// Get Sign Based on Handedness
double EulerAngle::get_sign( const EEulerHanded handed ) {
    if( handed == EULER_HANDED_RIGHT )
        return 1.0;         // RIGHT HANDED
    else
        return -1.0;        // LEFT HANDED
}
//------------------------------------------------------------------------------
// Eular Angle Formulations
//------------------------------------------------------------------------------
/// Form Rotation Matrix by XZX Convention
Matrix4 EulerAngle::XZX( const EEulerHanded handed, const double& theta, const double& phi, const double& psi ) {
    //double sign = get_sign( handed );
    get_sign( handed );
    return Matrix4( cos(phi),
                    -cos(psi)*sin(phi),
                    sin(phi)*sin(psi),
                    0,

                    cos(theta)*sin(phi),
                    cos(theta)*cos(phi)*cos(psi) - sin(theta)*sin(psi),
                    -cos(psi)*sin(theta) - cos(theta)*cos(phi)*sin(psi),
                    0,

                    sin(theta)*sin(phi),
                    cos(theta)*sin(psi) + cos(phi)*cos(psi)*sin(theta),
                    cos(theta)*cos(psi) - cos(phi)*sin(theta)*sin(psi),
                    0,

                    0,
                    0,
                    0,
                    1   );
}
//------------------------------------------------------------------------------
/// Form Rotation Matrix by XYX Convention
Matrix4 EulerAngle::XYX( const EEulerHanded handed, const double& theta, const double& phi, const double& psi ) {
    //double sign = get_sign( handed );
    get_sign( handed );
    return Matrix4( cos(phi),
                    sin(phi)*sin(psi),
                    cos(psi)*sin(phi),
                    0,

                    sin(theta)*sin(phi),
                    cos(theta)*cos(psi) - cos(phi)*sin(theta)*sin(psi),
                    -cos(theta)*sin(psi) - cos(phi)*cos(psi)*sin(theta),
                    0,

                    -cos(theta)*sin(phi),
                    cos(psi)*sin(theta) + cos(theta)*cos(phi)*sin(psi),
                    cos(theta)*cos(phi)*cos(psi) - sin(theta)*sin(psi),
                    0,

                    0,
                    0,
                    0,
                    1   );
}
//------------------------------------------------------------------------------
/// Form Rotation Matrix by YXY Convention
Matrix4 EulerAngle::YXY( const EEulerHanded handed, const double& theta, const double& phi, const double& psi ) {
    //double sign = get_sign( handed );
    get_sign( handed );
    return Matrix4( cos(theta)*cos(psi) - cos(phi)*sin(theta)*sin(psi),
                    sin(theta)*sin(phi),
                    cos(theta)*sin(psi) + cos(phi)*cos(psi)*sin(theta),
                    0,

                    sin(phi)*sin(psi),
                    cos(phi),
                    -cos(psi)*sin(phi),
                    0,

                    -cos(psi)*sin(theta) - cos(theta)*cos(phi)*sin(psi),
                    cos(theta)*sin(phi),
                    cos(theta)*cos(phi)*cos(psi) - sin(theta)*sin(psi),
                    0,

                    0,
                    0,
                    0,
                    1   );
}
//------------------------------------------------------------------------------
/// Form Rotation Matrix by YZY Convention
Matrix4 EulerAngle::YZY( const EEulerHanded handed, const double& theta, const double& phi, const double& psi ) {
    //double sign = get_sign( handed );
    get_sign( handed );
    return Matrix4( cos(theta)*cos(phi)*cos(psi) - sin(theta)*sin(psi),
                    -cos(theta)*sin(phi),
                    cos(psi)*sin(theta) + cos(theta)*cos(phi)*sin(psi),
                    0,

                    cos(psi)*sin(phi),
                    cos(phi),
                    sin(phi)*sin(psi),
                    0,

                    -cos(theta)*sin(psi) - cos(phi)*cos(psi)*sin(theta),
                    sin(theta)*sin(phi),
                    cos(theta)*cos(psi) - cos(phi)*sin(theta)*sin(psi),
                    0,

                    0,
                    0,
                    0,
                    1   );
}
//------------------------------------------------------------------------------
/// Form Rotation Matrix by ZYZ Convention
Matrix4 EulerAngle::ZYZ( const EEulerHanded handed, const double& theta, const double& phi, const double& psi ) {
    //double sign = get_sign( handed );
    get_sign( handed );
    return Matrix4( cos(theta)*cos(phi)*cos(psi) - sin(theta)*sin(psi),
                    -cos(psi)*sin(theta) - cos(theta)*cos(phi)*sin(psi),
                    cos(theta)*sin(phi),
                    0,

                    cos(theta)*sin(psi) + cos(phi)*cos(psi)*sin(theta),
                    cos(theta)*cos(psi) - cos(phi)*sin(theta)*sin(psi),
                    sin(theta)*sin(phi),
                    0,

                    -cos(psi)*sin(phi),
                    sin(phi)*sin(psi),
                    cos(phi),
                    0,

                    0,
                    0,
                    0,
                    1   );
}
//------------------------------------------------------------------------------
/// Form Rotation Matrix by ZXZ Convention
Matrix4 EulerAngle::ZXZ( const EEulerHanded handed, const double& theta, const double& phi, const double& psi ) {
    //double sign = get_sign( handed );
    get_sign( handed );
    return Matrix4( cos(theta)*cos(psi) - cos(phi)*sin(theta)*sin(psi),
                    -cos(theta)*sin(psi) - cos(phi)*cos(psi)*sin(theta),
                    sin(theta)*sin(phi),
                    0,

                    cos(psi)*sin(theta) + cos(theta)*cos(phi)*cos(psi),
                    cos(theta)*cos(phi)*cos(psi) - sin(theta)*sin(psi),
                    -cos(theta)*sin(phi),
                    0,

                    sin(phi)*sin(psi),
                    cos(psi)*sin(phi),
                    cos(phi),
                    0,

                    0,
                    0,
                    0,
                    1   );
}
//------------------------------------------------------------------------------
/// Form Rotation Matrix by XZY Convention
Matrix4 EulerAngle::XZY( const EEulerHanded handed, const double& theta, const double& phi, const double& psi ) {
    //double sign = get_sign( handed );
    get_sign( handed );
    return Matrix4( cos(phi)*cos(psi),
                    -sin(phi),
                    cos(phi)*sin(psi),
                    0,

                    sin(theta)*sin(psi) + cos(theta)*cos(psi)*sin(phi),
                    cos(theta)*cos(phi),
                    cos(theta)*sin(phi)*sin(psi) - cos(psi)*sin(theta),
                    0,

                    cos(psi)*sin(theta)*sin(phi) - cos(theta)*sin(psi),
                    cos(phi)*sin(theta),
                    cos(theta)*cos(psi) - sin(theta)*sin(phi)*sin(psi),
                    0,

                    0,
                    0,
                    0,
                    1   );
}
//------------------------------------------------------------------------------
/// Form Rotation Matrix by XYZ Convention
Matrix4 EulerAngle::XYZ( const EEulerHanded handed, const double& theta, const double& phi, const double& psi ) {
    //double sign = get_sign( handed );
    get_sign( handed );
    return Matrix4( cos(phi)*cos(psi),
                    -cos(phi)*sin(psi),
                    sin(phi),
                    0,

                    cos(theta)*sin(psi) + cos(psi)*sin(theta)*sin(phi),
                    cos(theta)*cos(psi) - sin(theta)*sin(phi)*sin(psi),
                    -cos(phi)*sin(theta),
                    0,

                    sin(theta)*sin(psi) - cos(theta)*cos(psi)*sin(phi),
                    cos(psi)*sin(theta) + cos(theta)*sin(phi)*sin(psi),
                    cos(theta)*cos(phi),
                    0,

                    0,
                    0,
                    0,
                    1   );
}
//------------------------------------------------------------------------------
/// Form Rotation Matrix by YXZ Convention
Matrix4 EulerAngle::YXZ( const EEulerHanded handed, const double& theta, const double& phi, const double& psi ) {
    //double sign = get_sign( handed );
    get_sign( handed );
    return Matrix4( cos(theta)*cos(psi) + sin(theta)*sin(phi)*sin(psi),
                    cos(psi)*sin(theta)*sin(phi) - cos(theta)*sin(psi),
                    cos(phi)*sin(theta),
                    0,

                    cos(phi)*sin(psi),
                    cos(phi)*cos(psi),
                    -sin(phi),
                    0,

                    cos(theta)*sin(phi)*sin(psi) - cos(psi)*sin(theta),
                    sin(theta)*sin(psi) + cos(theta)*cos(psi)*sin(phi),
                    cos(theta)*cos(phi),
                    0,

                    0,
                    0,
                    0,
                    1   );
}
//------------------------------------------------------------------------------
/// Form Rotation Matrix by YZX Convention
Matrix4 EulerAngle::YZX( const EEulerHanded handed, const double& theta, const double& phi, const double& psi ) {
    //double sign = get_sign( handed );
    get_sign( handed );
    return Matrix4( cos(theta)*cos(phi),
                    sin(theta)*sin(psi) - cos(theta)*cos(psi)*sin(phi),
                    cos(psi)*sin(theta) + cos(theta)*sin(phi)*sin(psi),
                    0,

                    sin(phi),
                    cos(phi)*cos(psi),
                    -cos(phi)*sin(psi),
                    0,

                    -cos(phi)*sin(theta),
                    cos(theta)*sin(psi) + cos(psi)*sin(theta)*sin(phi),
                    cos(theta)*cos(psi) - sin(theta)*sin(phi)*sin(psi),
                    0,

                    0,
                    0,
                    0,
                    1   );
}
//------------------------------------------------------------------------------
/// Form Rotation Matrix by ZYX Convention
Matrix4 EulerAngle::ZYX( const EEulerHanded handed, const double& theta, const double& phi, const double& psi ) {
    //double sign = get_sign( handed );
    get_sign( handed );
    return Matrix4( cos(theta)*cos(phi),
                    cos(theta)*sin(phi)*sin(psi) - cos(psi)*sin(theta),
                    sin(theta)*sin(psi) + cos(theta)*cos(psi)*sin(phi),
                    0,

                    cos(phi)*sin(theta),
                    cos(theta)*cos(psi) + sin(theta)*sin(phi)*sin(psi),
                    cos(psi)*sin(theta)*sin(phi) - cos(theta)*sin(psi),
                    0,

                    -sin(phi),
                    cos(phi)*sin(psi),
                    cos(phi)*cos(psi),
                    0,

                    0,
                    0,
                    0,
                    1   );
}
//------------------------------------------------------------------------------
/// Form Rotation Matrix by ZXY Convention
Matrix4 EulerAngle::ZXY( const EEulerHanded handed, const double& theta, const double& phi, const double& psi ) {
    //double sign = get_sign( handed );
    get_sign( handed );
    return Matrix4( cos(theta)*cos(psi) - sin(theta)*sin(phi)*sin(psi),
                    -cos(psi)*sin(theta),
                    cos(theta)*sin(psi) + cos(psi)*sin(theta)*sin(phi),
                    0,

                    cos(psi)*sin(theta) + cos(theta)*sin(phi)*sin(psi),
                    cos(theta)*cos(phi),
                    sin(theta)*sin(psi) - cos(theta)*cos(psi)*sin(phi),
                    0,

                    -cos(phi)*sin(psi),
                    sin(phi),
                    cos(phi)*cos(psi),
                    0,

                    0,
                    0,
                    0,
                    1   );
}
//------------------------------------------------------------------------------

