/*-----------------------THE GEORGE WASHINGTON UNIVERSITY-----------------------
James Taylor                                                         jrt@gwu.edu

EulerAngle class

------------------------------------------------------------------------------*/

#ifndef _EULERANGLE_H_
#define _EULERANGLE_H_

#include <cs6555/Math/Vector3.h>
#include <cs6555/Math/Matrix3.h>
#include <cs6555/Math/Matrix4.h>

//------------------------------------------------------------------------------

typedef enum {
    EULER_ANGLE_XZX,
    EULER_ANGLE_XYX,
    EULER_ANGLE_YXY,
    EULER_ANGLE_YZY,
    EULER_ANGLE_ZYZ,
    EULER_ANGLE_ZXZ,
    EULER_ANGLE_XZY,
    EULER_ANGLE_XYZ,
    EULER_ANGLE_YXZ,
    EULER_ANGLE_YZX,
    EULER_ANGLE_ZYX,
    EULER_ANGLE_ZXY
} EEulerAngles;

//------------------------------------------------------------------------------

typedef enum {
    EULER_HANDED_RIGHT,
    EULER_HANDED_LEFT
} EEulerHanded;

//------------------------------------------------------------------------------

class EulerAngle {
public:
    EulerAngle( void ) {}
    virtual ~EulerAngle( void ) {}

    // Generate Matrix3 Rotation Matrix
    static Matrix3 matrix3( const EEulerAngles angle, const EEulerHanded handed, const double& theta, const double& phi, const double& psi );
    static Matrix3 matrix3( const EEulerAngles angle, const EEulerHanded handed, const Vector3& u );

    // Generate Matrix4 Rotation Matrix
    static Matrix4 matrix4( const EEulerAngles angle, const EEulerHanded handed, const double& theta, const double& phi, const double& psi );
    static Matrix4 matrix4( const EEulerAngles angle, const EEulerHanded handed, const Vector3& u );

    // Eular Angle Formulations
    static Matrix4 XZX( const EEulerHanded handed, const double& theta, const double& phi, const double& psi );
    static Matrix4 XYX( const EEulerHanded handed, const double& theta, const double& phi, const double& psi );
    static Matrix4 YXY( const EEulerHanded handed, const double& theta, const double& phi, const double& psi );
    static Matrix4 YZY( const EEulerHanded handed, const double& theta, const double& phi, const double& psi );
    static Matrix4 ZYZ( const EEulerHanded handed, const double& theta, const double& phi, const double& psi );
    static Matrix4 ZXZ( const EEulerHanded handed, const double& theta, const double& phi, const double& psi );
    static Matrix4 XZY( const EEulerHanded handed, const double& theta, const double& phi, const double& psi );
    static Matrix4 XYZ( const EEulerHanded handed, const double& theta, const double& phi, const double& psi );
    static Matrix4 YXZ( const EEulerHanded handed, const double& theta, const double& phi, const double& psi );
    static Matrix4 YZX( const EEulerHanded handed, const double& theta, const double& phi, const double& psi );
    static Matrix4 ZYX( const EEulerHanded handed, const double& theta, const double& phi, const double& psi );
    static Matrix4 ZXY( const EEulerHanded handed, const double& theta, const double& phi, const double& psi );

private:
    // Class Utilities
    static double get_sign( const EEulerHanded handed );
};

//------------------------------------------------------------------------------

#endif // _EULERANGLE_H_
