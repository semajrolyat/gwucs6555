/*-----------------------THE GEORGE WASHINGTON UNIVERSITY-----------------------
James Taylor                                                         jrt@gwu.edu

RigidBody class definition

------------------------------------------------------------------------------*/

#ifndef _RIGIDBODY_H_
#define _RIGIDBODY_H_

//------------------------------------------------------------------------------

#include <cs6555/Body.h>
#include <cs6555/Math/Vector3.h>
#include <cs6555/Math/Matrix3.h>
#include <cs6555/Math/Quaternion.h>

//------------------------------------------------------------------------------

typedef enum {
    RIGIDBODY_DYNAMIC,
    RIGIDBODY_STATIC
} ERigidBodyType;

//------------------------------------------------------------------------------

class RigidBody : public Body {
public:
    // Constructors
    RigidBody( void );
    RigidBody( const ERigidBodyType& type );

    // Destructor
    virtual ~RigidBody( void );

    // [Base Class] Body::Queries
    virtual EBodyType body_type( void );

    ERigidBodyType rigidbody_type;

    virtual void shallow_copy( RigidBody* rb );

    // Member Data
//    double      mass;

//    Matrix3     body_frame_inertial_tensor;
//    Matrix3     body_frame_inertial_tensor_inverse;

//    Vector3     position;
//    Matrix3     rotation;
//    Quaternion  q_rotation;
//    Vector3     linear_momentum;
//    Vector3     angular_momentum;

//    Matrix3     inertial_tensor_inverse;
//    Vector3     linear_velocity;
//    Vector3     angular_velocity;

//    Vector3     force;
//    Vector3     torque;

//    double      coefficient_of_restitution;

    void sphere( const double& mass, const double& radius );
    void sphere( const double& mass, const double& radius, const unsigned int& segments, const unsigned int& slices );
    void sphere( const double& mass, const double& radius, const unsigned int& segments, const unsigned int& slices, Material* material );

    void cube( const double& mass, const double& width, const double& height, const double& depth );
    void cube( const double& mass, const double& width, const double& height, const double& depth, Material* material );

    void update_origin( void );
    void copy_origin( void );

};

//------------------------------------------------------------------------------

#endif // _RIGIDBODY_H_
