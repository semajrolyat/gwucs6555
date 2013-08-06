#include <cs6555/Boid.h>

#include <cs6555/Math.h>
#include <cs6555/Math/EulerAngle.h>
#include <cs6555/Motivator.h>
//------------------------------------------------------------------------------
// Constructors
//------------------------------------------------------------------------------
/// Default Constructor
Boid::Boid( void ) {
    role = BOID_ROLE_INDEPENDENT;
    goal = BOID_GOAL_GLOBAL;
    member_of = NULL;
}

//------------------------------------------------------------------------------
// Destructor
//------------------------------------------------------------------------------
/// Destructor
Boid::~Boid( void ) {

}

//------------------------------------------------------------------------------

void Boid::get_neighbors( std::vector<Boid*>& boids, const double& radius ) {
    // clear neighbors
    std::vector<Boid*> x;
    neighbors.swap( x );    // Note: swap won't call destructors on the pointers
                            // where clear would

    Vector3 my_heading = previous_linear_velocity;
    my_heading.normalize();

    for( std::vector<Boid*>::iterator it = boids.begin(); it != boids.end(); it++ ) {
        Boid* boid = (*it);
        if( boid == this ) continue;    // avoid adding a self-reference

        Vector3 p = boid->body.pose.position - body.pose.position;
        double dist = p.magnitude();
        if( dist <= radius ) {
            Vector3 boid_heading = boid->previous_linear_velocity;
            boid_heading.normalize();
            double dp = Vector3::dot( my_heading, boid_heading );
            // only consider a neighbor if the headings are not directly opposed
            if( dp > -0.5 )
                neighbors.push_back( boid );
        }
    }
}

//------------------------------------------------------------------------------

void Boid::clear_hazards( void ) {
    std::vector<Boid*> x;
    hazards.swap( x );    // Note: swap won't call destructors on the pointers
                            // where clear would
}

//------------------------------------------------------------------------------

void Boid::add_hazard( Boid* boid ) {
    hazards.push_back( boid );
}

//------------------------------------------------------------------------------
/// determines the strength of collision avoidance behavior for a given boid
/// determination is based upon the previously computed values
Vector3 Boid::suggest_goto_motivator( Boid* boid, Motivator* motivator ) {

    Vector3 suggested_acceleration;

    // compute unit direction
    suggested_acceleration = motivator->position - boid->previous_position;
    suggested_acceleration.normalize();

    // scale by force
    suggested_acceleration *= boid->motivator_force;

    return suggested_acceleration;
}

//------------------------------------------------------------------------------
/// determines the strength of collision avoidance behavior for a given boid
/// determination is based upon the previously computed values
Vector3 Boid::suggest_collision_avoidance( Boid* boid ) {

    Vector3 suggested_acceleration = Vector3( 0.0, 0.0, 0.0 );

    // Rules:
    // 1. Compute the bubble.  If the bubble intersects with another bubble
    // 2. Only avoid hazards to the front
    // 2a. Compute the weight the bubble influences.
    // 2b. Scale the resultant acceleration based upon the weight of influence

    boid->clear_hazards();
    for( std::vector<Boid*>::iterator nit = boid->neighbors.begin(); nit != boid->neighbors.end(); nit++ ) {
        Boid* neighbor = (*nit);

        Vector3 v_distance_to = boid->previous_position - neighbor->previous_position;
        double distance_to = v_distance_to.magnitude();
        double buffer_distance = boid->personal_space_radius + neighbor->personal_space_radius;

        // bubbles intersect
        if( distance_to < buffer_distance ) {
            // if you are the boid in front, then don't consider the one behind for collision purposes

            // the heading of the boid
            Vector3 actual_heading = boid->previous_linear_velocity;
            actual_heading.normalize();
            // the heading to the intersection point between the boid and the neighbor
            Vector3 pseudo_heading = neighbor->previous_position - boid->previous_position;
            pseudo_heading.normalize();

            // if the inner product between the actual heading and the pseudo heading is
            // in the forward field then register a possible collision
            double dp = Vector3::dot( actual_heading, pseudo_heading );
            if( dp > 0.0 ) {
                boid->add_hazard( neighbor );

                // determine the direction
                double theta = -Math::angle_in_x_z( actual_heading, pseudo_heading );
                Vector3 heading = Vector3( 0.0, boid->previous_orientation.y() + theta, 0.0 );

                Vector3 local_orientation = boid->motivator_detector.position;
                local_orientation.normalize();

                // transform the orientation toward the boid front in world space
                Matrix3 R = EulerAngle::matrix3( EULER_ANGLE_XYZ, EULER_HANDED_LEFT, heading.x(), heading.y(), heading.z() );
                Vector3 orientation = R * local_orientation;
                orientation.normalize();
                orientation.y( 0.0 );
                orientation.normalize();

                // determine the magnitude
                // basically R + R / r + r > 1.0 -> weight to be applied to acceleration
                double weight = buffer_distance / distance_to;

                suggested_acceleration += orientation * weight;
            }
        }
    }
    suggested_acceleration.normalize();
    suggested_acceleration *= boid->avoidance_force;
    return suggested_acceleration;
}

//------------------------------------------------------------------------------
/// determines the strength of velocity matching behavior for a given boid
/// determination is based upon the previously computed values
Vector3 Boid::suggest_velocity_matching( Boid* boid ) {

    Vector3 suggested_acceleration;

    Vector3 local_velocity = boid->previous_linear_velocity;

    for( std::vector<Boid*>::iterator nit = boid->neighbors.begin(); nit != boid->neighbors.end(); nit++ ) {
        Boid* neighbor = (*nit);
        // find the local heading with respect to this boid and its immediate neighbors
        local_velocity += neighbor->previous_linear_velocity;
    }
    local_velocity.normalize();                      // normalize to make unit length

    suggested_acceleration = local_velocity * boid->matching_force;

    return suggested_acceleration;
}

//------------------------------------------------------------------------------
/// determines the strength of flock centering behavior for a given boid
/// determination is based upon the previously computed values
Vector3 Boid::suggest_flock_centering( Boid* boid ) {

    Vector3 suggested_acceleration;

    Vector3 local_center = boid->previous_position;

    for( std::vector<Boid*>::iterator nit = boid->neighbors.begin(); nit != boid->neighbors.end(); nit++ ) {
        Boid* neighbor = (*nit);
        // find the local center with respect to this boid and its immediate neighbors
        local_center += (boid->previous_position - neighbor->previous_position) / 2.0;
    }
    Vector3 local_heading = local_center - boid->previous_position;
    local_heading.normalize();

    suggested_acceleration = local_heading * boid->centering_force;

    return suggested_acceleration;
}
