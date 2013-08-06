/*-----------------------THE GEORGE WASHINGTON UNIVERSITY-----------------------
James Taylor                                                         jrt@gwu.edu

Physics class definition

------------------------------------------------------------------------------*/

#ifndef _PHYSICS_H_
#define _PHYSICS_H_

//------------------------------------------------------------------------------

#include <cs6555/Math/Vector3.h>
#include <cs6555/RigidBody.h>

#include <cs6555/BoundingVolume.h>
#include <cs6555/BoundingSphere.h>
#include <cs6555/AABB.h>
#include <cs6555/OBB.h>
#include <cs6555/ContactEvent.h>

//------------------------------------------------------------------------------

class Physics {
public:
    // Integrators
    static void ode( const double& t0, const double& t1, RigidBody* body );

    // Force and Torque Operations
    static void compute_force_and_torque( const double& t, RigidBody* body );

    // Collision Resolution
    static Vector3 pt_velocity( RigidBody* body, Vector3& point );
    static bool colliding( ContactEvent* contact );
    static void collision( ContactEvent* contact, const double& epsilon );

    // Obsolete Collision Resolution.  Bad Model.
    static void resolve_collision_event( RigidBody* body1, RigidBody* body2 );
private:
    static void resolve_collision_event( RigidBody* body1, const double& theta1, const Vector3& axis1, RigidBody* body2, const double& theta2, const Vector3& axis2 );
    static void resolve_Sphere_Sphere_collision_event( RigidBody* sphere1_body, RigidBody* sphere2_body );
    static void resolve_Sphere_AABB_collision_event( RigidBody* sphere_body, RigidBody* aabb_body );
    static void resolve_Sphere_OBB_collision_event( RigidBody* sphere_body, RigidBody* obb_body );
    static void resolve_AABB_AABB_collision_event( RigidBody* aabb1_body, RigidBody* aabb2_body );
    static void resolve_AABB_OBB_collision_event( RigidBody* aabb_body, RigidBody* obb_body );
    static void resolve_OBB_OBB_collision_event( RigidBody* obb1_body, RigidBody* obb2_body );


};

//------------------------------------------------------------------------------

#endif // _PHYSICS_H_
