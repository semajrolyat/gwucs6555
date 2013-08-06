/*-----------------------THE GEORGE WASHINGTON UNIVERSITY-----------------------
James Taylor                                                         jrt@gwu.edu

Pose class definition

------------------------------------------------------------------------------*/

#ifndef _POSE_H_
#define _POSE_H_

#include <cs6555/Math/Vector3.h>
#include <cs6555/Math/Matrix3.h>
#include <cs6555/Math/Matrix4.h>
#include <cs6555/Math/Quaternion.h>

//------------------------------------------------------------------------------

typedef enum {
    POSE_BY_EULERANGLES,
    POSE_BY_QUATERNION
} EPoseOrientationType;

//------------------------------------------------------------------------------

class Pose {
public:
    // Constructors
    Pose( void );
    Pose( const Pose& pose );
    Pose( const Vector3& position, const Vector3& orientation );
    Pose( const Vector3& position, const Quaternion& orientation );
    Pose( const double& x, const double& y, const double& z, const double& theta, const double& phi, const double& psi );

    Pose( const EPoseOrientationType& type );
    Pose( const EPoseOrientationType& type, const Vector3& position, const Vector3& orientation );
    Pose( const EPoseOrientationType& type, const double& x, const double& y, const double& z, const double& theta, const double& phi, const double& psi );

    // Destructor
    virtual ~Pose( void );

    // Transformation Functions
    Matrix4 transformByEulerAngle( void );
    Matrix4 transformByQuaternion( void );

    Matrix3 matrix3( void );

    // Member Data
    Vector3 position;           // 3-dimensional postion vector
    Vector3 orientation;        // 3-dimensional orientation vector
    Quaternion orientation_q;   // orientation quaternion

    Matrix4 transform;          // affine homogeneous transformation

    EPoseOrientationType orientation_type;
};

//------------------------------------------------------------------------------

#endif // _POSE_H_
