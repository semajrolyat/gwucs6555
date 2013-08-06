/*-----------------------THE GEORGE WASHINGTON UNIVERSITY-----------------------
James Taylor                                                         jrt@gwu.edu

Physics class implementation

References:
#1 Baraff, D., "An Introduction to Physically Based Modeling: Rigid Body Simulation I - Unconstrained Rigid Body Dynamics"
------------------------------------------------------------------------------*/

#include <cs6555/Physics.h>
#include <cs6555/Constants.h>

//------------------------------------------------------------------------------
// Integrators
//------------------------------------------------------------------------------
// Ref #1
void Physics::ode( const double& t0, const double& t1, RigidBody* body ) {
    double dt = t1 - t0;

    // --- Begin: Linear Integration ---
    // dp = F dt
    body->linear_momentum += (body->force * dt);

    // p = mv -> v = p/m
    body->linear_velocity = body->linear_momentum / body->mass;

    // x1 = x0 + v*dt
    body->pose.position += (body->linear_velocity * dt);
    // --- End: Linear Integration ---

    // --- Begin: Angular Integration ---
    // L = torque * dt
    body->angular_momentum += (body->torque * dt);

    // derive rotation matrix R and Rt
    body->pose.orientation_q.normalize();
    Matrix3 rotation = body->pose.orientation_q.matrix3();
    Matrix3 rotation_transpose = Matrix3::transpose( rotation );

    // Iinv = R * Ibodyinv * Rt
    body->inertial_tensor_inverse = rotation * body->body_frame_inertial_tensor_inverse * rotation_transpose;

    // omega = Iinv * L
    body->angular_velocity = body->inertial_tensor_inverse * body->angular_momentum;

    // q/dt = 1/2 * ( omega * q )
    Quaternion qdot = 0.5 * ( body->angular_velocity * body->pose.orientation_q );

    // q1 = q0 + qdot * dt
    body->pose.orientation_q += (qdot * dt);
    // --- End: Angular Integration ---

    // --- Update the pose transformation ---
    body->pose.transformByQuaternion();
}

//------------------------------------------------------------------------------
// Force and Torque Operations
//------------------------------------------------------------------------------

void Physics::compute_force_and_torque( const double& t, RigidBody* body ) {
    // gravity
    const double g = -9.8;

    Vector3 acceleration = Vector3( 0.0, g, 0.0 );

    // F = ma
    body->force = body->mass * acceleration;

    body->torque = Vector3( 0.0, 0.0, 0.0 );
}

//------------------------------------------------------------------------------
// Collision Resolution
//------------------------------------------------------------------------------

void Physics::resolve_collision_event( RigidBody* body1, RigidBody* body2 ) {
    assert( body1 != NULL && body2 != NULL );
    assert( body1->bv() != NULL && body2->bv() != NULL );

    if( body1->bv()->bounding_volume_type() == BV_TYPE_SPHERE && body2->bv()->bounding_volume_type() == BV_TYPE_SPHERE ) {
        resolve_Sphere_Sphere_collision_event( body1, body2 );
    } else if( body1->bv()->bounding_volume_type() == BV_TYPE_SPHERE && body2->bv()->bounding_volume_type() == BV_TYPE_AABB ) {
        resolve_Sphere_AABB_collision_event( body1, body2 );
    } else if( body1->bv()->bounding_volume_type() == BV_TYPE_AABB && body2->bv()->bounding_volume_type() == BV_TYPE_SPHERE ) {
        resolve_Sphere_AABB_collision_event( body2, body1 );
    } else if( body1->bv()->bounding_volume_type() == BV_TYPE_SPHERE && body2->bv()->bounding_volume_type() == BV_TYPE_OBB ) {
        resolve_Sphere_OBB_collision_event( body1, body2 );
    } else if( body1->bv()->bounding_volume_type() == BV_TYPE_OBB && body2->bv()->bounding_volume_type() == BV_TYPE_SPHERE ) {
        resolve_Sphere_OBB_collision_event( body2, body1 );
    } else if( body1->bv()->bounding_volume_type() == BV_TYPE_AABB && body2->bv()->bounding_volume_type() == BV_TYPE_AABB ) {
        resolve_AABB_AABB_collision_event( body1, body2 );
    } else if( body1->bv()->bounding_volume_type() == BV_TYPE_AABB&& body2->bv()->bounding_volume_type() == BV_TYPE_OBB ) {
        resolve_AABB_OBB_collision_event( body1, body2 );
    } else if( body1->bv()->bounding_volume_type() == BV_TYPE_OBB && body2->bv()->bounding_volume_type() == BV_TYPE_AABB ) {
        resolve_AABB_OBB_collision_event( body2, body1 );
    } else if( body1->bv()->bounding_volume_type() == BV_TYPE_OBB && body2->bv()->bounding_volume_type() == BV_TYPE_OBB ) {
        resolve_OBB_OBB_collision_event( body1, body2 );
    }
}

//------------------------------------------------------------------------------
// Return the velocity of a point on a rigid body
Vector3 Physics::pt_velocity( RigidBody* body, Vector3& point ) {
    return body->linear_velocity + Vector3::cross( body->angular_velocity, point - body->pose.position );
}

//------------------------------------------------------------------------------
// Return true if bodies are in colliding contact
bool Physics::colliding( ContactEvent* contact ) {
    Vector3 padot = pt_velocity( contact->body1, contact->point );
    Vector3 pbdot = pt_velocity( contact->body2, contact->point );
    double vrel = Vector3::dot( contact->normal, padot - pbdot );

    //const double EPSILON = 1e-5;
    const double EPSILON = 1e-16;
    if( vrel > EPSILON )    // Moving away
        return false;
    if( vrel > -EPSILON )   // Resting contact
        return false;
    return true;            // Colliding contact
}

//------------------------------------------------------------------------------

void Physics::collision( ContactEvent* contact, const double& epsilon ) {
    Vector3 padot = pt_velocity( contact->body1, contact->point );
    Vector3 pbdot = pt_velocity( contact->body2, contact->point );
    Vector3 normal = contact->normal;
    Vector3 ra = contact->point - contact->body1->pose.position;
    Vector3 rb = contact->point - contact->body2->pose.position;
    double vrel = Vector3::dot( contact->normal, padot - pbdot );
    double numerator = -( 1 + epsilon ) * vrel;

    // calculate denominator in four parts
    double term1 = 1 / contact->body1->mass;
    double term2 = 1 / contact->body2->mass;
    double term3 = Vector3::dot( normal, Vector3::cross( contact->body1->inertial_tensor_inverse * Vector3::cross( ra, normal ), ra) );
    double term4 = Vector3::dot( normal, Vector3::cross( contact->body2->inertial_tensor_inverse * Vector3::cross( rb, normal ), rb) );

    // compute the impulse magnitude
    double j = numerator / ( term1 + term2 + term3 + term4);
    Vector3 force = j * normal;

    // apply the impulse to the bodies
    contact->body1->linear_momentum += force;
    contact->body2->linear_momentum -= force;
    contact->body1->angular_momentum += Vector3::cross( ra, force );
    contact->body2->angular_momentum -= Vector3::cross( rb, force );

    // recompute auxilliary variables
    contact->body1->linear_velocity = contact->body1->linear_momentum / contact->body1->mass;
    contact->body2->linear_velocity = contact->body2->linear_momentum / contact->body2->mass;

    contact->body1->angular_velocity = contact->body1->inertial_tensor_inverse * contact->body1->angular_momentum;
    contact->body2->angular_velocity = contact->body2->inertial_tensor_inverse * contact->body2->angular_momentum;
}




//------------------------------------------------------------------------------
// Note: The restitution below works well enough for instantaneous collisions,
// but for continuous contact, e.g. rolling contact, restitution will act as a
// form of friction and will cause the bodies to come to a stop so it is a rough
// application but it is physically unrealistic in the long run.
void Physics::resolve_collision_event( RigidBody* body1, const double& theta1, const Vector3& axis1, RigidBody* body2, const double& theta2, const Vector3& axis2 ) {
    const double EPSILON = 1e-5;
    // Note: theta != theta -> theta = NaN
    if( (theta1 != theta1 && theta2 != theta2) || (theta1 < EPSILON && theta2 < EPSILON) ) {
        // head on collision, exchange the momentums
        Vector3 temp = body1->linear_momentum;
        body1->linear_momentum = body2->linear_momentum;
        body2->linear_momentum = temp;

        body1->linear_momentum *= body1->coefficient_of_restitution;
        body2->angular_momentum *= body1->coefficient_of_restitution;

        body1->linear_momentum *= body2->coefficient_of_restitution;
        body2->angular_momentum *= body2->coefficient_of_restitution;

    } else if( (theta1 != theta1 || theta1 < EPSILON) && (fabs(theta2 - PI/2) < EPSILON) ) {
        // Note: with a huge differential in mass, this may not be physically representative
        // as the heavy body will likely still have some forward momentum after the collision
        // assuming it is the perpendicular collider

        // completely tangential, body 1 stops and body 2 gains all momentum
        body2->linear_momentum += body1->linear_momentum;
        body1->linear_momentum = Vector3( 0.0, 0.0, 0.0 );

        // technically p = 0 but may still have spin
        body1->linear_momentum *= body1->coefficient_of_restitution;
        body1->angular_momentum *= body1->coefficient_of_restitution;

        // p = mv1 + mv2
        body2->linear_momentum *= body2->coefficient_of_restitution;
        body2->angular_momentum *= body2->coefficient_of_restitution;

    } else if( (fabs(theta1 - PI/2) < EPSILON) && (theta2 != theta2 || theta2 < EPSILON) ) {
        // Note: with a huge differential in mass, this may not be physically representative
        // as the heavy body will likely still have some forward momentum after the collision
        // assuming it is the perpendicular collider

        // completely tangential, body 2 stops and body 1 gains all momentum
        body1->linear_momentum += body2->linear_momentum;
        body2->linear_momentum = Vector3( 0.0, 0.0, 0.0 );

        // p = mv1 + mv2
        body1->linear_momentum *= body1->coefficient_of_restitution;
        body1->angular_momentum *= body1->coefficient_of_restitution;

        // technically p = 0 but may still have spin
        body2->linear_momentum *= body2->coefficient_of_restitution;
        body2->angular_momentum *= body2->coefficient_of_restitution;

    } else {
        // Note: this is probably not the correct analytical solution but momentum
        // is conserved.  With two moving bodies, the linear system must be solved
        // to get the most accurate answer, but this is a reasonable approximation
        // for the time being

        // otherwise use axis angle to reflect by 2 * theta
        // Note: 2 * theta -> q terms are theta not theta/2
        Quaternion q1 = Quaternion( cos(theta1), axis1.x() * sin(theta1), axis1.y() * sin(theta1), axis1.z() * sin(theta1) );
        Matrix3 R1 = q1.matrix3();

        Quaternion q2 = Quaternion( cos(theta2), axis2.x() * sin(theta2), axis2.y() * sin(theta2), axis2.z() * sin(theta2) );
        Matrix3 R2 = q2.matrix3();

        Vector3 newdir_body1 = R1 * -body1->linear_momentum;
        newdir_body1.normalize();
        Vector3 newdir_body2 = R2 * -body2->linear_momentum;
        newdir_body2.normalize();

        body1->linear_momentum = R1 * -body1->linear_momentum;
        body1->linear_momentum *= body1->coefficient_of_restitution;
        body1->angular_momentum *= body1->coefficient_of_restitution;

        body2->linear_momentum = R2 * -body2->linear_momentum;
        body2->linear_momentum *= body2->coefficient_of_restitution;
        body2->angular_momentum *= body2->coefficient_of_restitution;
    }
}

//------------------------------------------------------------------------------
void Physics::resolve_Sphere_Sphere_collision_event( RigidBody* sphere1_body, RigidBody* sphere2_body ) {
    BoundingSphere* bsphere1 = static_cast<BoundingSphere*>( sphere1_body->bv() );
    BoundingSphere* bsphere2 = static_cast<BoundingSphere*>( sphere2_body->bv() );

    bsphere1->center = sphere1_body->pose.position;
    bsphere2->center = sphere2_body->pose.position;

    Vector3 sphere1_normal = bsphere1->normal( bsphere2->center );
    Vector3 sphere2_normal = bsphere2->normal( bsphere1->center );

    Vector3 dir_sphere1 = sphere1_body->linear_velocity;
    dir_sphere1.normalize();
    Vector3 dir_sphere2 = sphere2_body->linear_velocity;
    dir_sphere2.normalize();

    Vector3 axis_sphere1 = Vector3::cross( -dir_sphere1, sphere2_normal );
    axis_sphere1.normalize();
    double theta1 = acos( Vector3::dot( -dir_sphere1, sphere2_normal ) );

    Vector3 axis_sphere2 = Vector3::cross( -dir_sphere2, sphere1_normal );
    axis_sphere2.normalize();
    double theta2 = acos( Vector3::dot( -dir_sphere2, sphere1_normal ) );

    resolve_collision_event( sphere1_body, theta1, axis_sphere1, sphere2_body, theta2, axis_sphere2 );
}

//------------------------------------------------------------------------------

void Physics::resolve_Sphere_AABB_collision_event( RigidBody* sphere_body, RigidBody* aabb_body ) {
    BoundingSphere* bsphere = static_cast<BoundingSphere*>( sphere_body->bv() );
    AABB* aabb = static_cast<AABB*>( aabb_body->bv() );

    bsphere->center = sphere_body->pose.position;
    aabb->center = aabb_body->pose.position;

    if( aabb_body->rigidbody_type == RIGIDBODY_STATIC ) {

        Vector3 aabbpt;
        BoundingVolume::ClosestPtPointAABB( bsphere->center, *aabb, aabbpt );
        Vector3 sphere_normal, aabb_normal;
        sphere_normal = bsphere->normal( aabbpt );
        aabb_normal = aabb->normal( aabbpt );
        Vector3 dir_sphere = sphere_body->linear_velocity;
        dir_sphere.normalize();

        Vector3 axis = Vector3::cross( -dir_sphere, aabb_normal );
        axis.normalize();
        double theta = acos( Vector3::dot( -dir_sphere, aabb_normal ) );

        const double EPSILON = 1e-5;
        // Note: theta != theta -> theta = NaN
        if( theta != theta || theta < EPSILON ) {
            // reflection backward toward incoming direction so just flip incoming vector
            sphere_body->linear_momentum = -sphere_body->linear_momentum;
            // Note: in continuous contact with multiple contact points, this will ultimately be
            // unstable, but sphere has one point of contact with rigid bodies, so this is
            // basically an impulse away of the same momentum that was applied to incoming so
            // there is equilibrium in this particular configuration
            sphere_body->linear_momentum *= sphere_body->coefficient_of_restitution;
            sphere_body->angular_momentum *= sphere_body->coefficient_of_restitution;
        } else {
            // otherwise use axis angle to reflect by 2 * theta
            // Note: 2 * theta -> q terms are theta not theta/2
            Quaternion q = Quaternion( cos(theta), axis.x() * sin(theta), axis.y() * sin(theta), axis.z() * sin(theta) );
            Matrix3 R = q.matrix3();
            sphere_body->linear_momentum = R * -sphere_body->linear_momentum;
            sphere_body->linear_momentum *= sphere_body->coefficient_of_restitution;
            sphere_body->angular_momentum *= sphere_body->coefficient_of_restitution;
        }
    } else if( aabb_body->rigidbody_type == RIGIDBODY_DYNAMIC ) {
        // to be determined.  Not essential for lab3
    }
}

//------------------------------------------------------------------------------

void Physics::resolve_Sphere_OBB_collision_event( RigidBody* sphere_body, RigidBody* obb_body ) {

}

//------------------------------------------------------------------------------

void Physics::resolve_AABB_AABB_collision_event( RigidBody* aabb1_body, RigidBody* aabb2_body ) {
    AABB* aabb1 = static_cast<AABB*>( aabb1_body->bv() );
    AABB* aabb2 = static_cast<AABB*>( aabb2_body->bv() );

    aabb1->center = aabb1_body->pose.position;
    aabb2->center = aabb2_body->pose.position;

    Vector3 aabb1_point, aabb2_point;
    BoundingVolume::ClosestPtPointAABB( aabb2->center, *aabb1, aabb1_point );
    BoundingVolume::ClosestPtPointAABB( aabb1->center, *aabb2, aabb2_point );
    Vector3 aabb1_normal = aabb1->normal( aabb1_point );
    Vector3 aabb2_normal = aabb2->normal( aabb2_point );

    Vector3 dir_aabb1 = aabb1_body->linear_velocity;
    dir_aabb1.normalize();
    Vector3 dir_aabb2 = aabb2_body->linear_velocity;
    dir_aabb2.normalize();

    Vector3 axis_aabb1 = Vector3::cross( -dir_aabb1, aabb2_normal );
    axis_aabb1.normalize();
    double theta1 = acos( Vector3::dot( -dir_aabb1, aabb2_normal ) );

    Vector3 axis_aabb2 = Vector3::cross( -dir_aabb2, aabb1_normal );
    axis_aabb2.normalize();
    double theta2 = acos( Vector3::dot( -dir_aabb2, aabb1_normal ) );

    resolve_collision_event( aabb1_body, theta1, axis_aabb1, aabb2_body, theta2, axis_aabb2 );
}

//------------------------------------------------------------------------------

void Physics::resolve_AABB_OBB_collision_event( RigidBody* aabb_body, RigidBody* obb_body ) {

}

//------------------------------------------------------------------------------

void Physics::resolve_OBB_OBB_collision_event( RigidBody* obb1_body, RigidBody* obb2_body ) {

}

//------------------------------------------------------------------------------
