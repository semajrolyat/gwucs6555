#include <cs6555/Flock.h>

#include <cs6555/MeshLoader.h>
#include <cs6555/Math.h>
//------------------------------------------------------------------------------
// Constructors
//------------------------------------------------------------------------------
/// Default Constructor
Flock::Flock( void ) {
    leader = NULL;
}

//------------------------------------------------------------------------------
/// Copy Constructor
Flock::Flock( Flock& flock ) {
    for( std::vector<Boid*>::iterator it = flock.members.begin(); it != flock.members.end(); it++ ) {
        members.push_back( *it );
    }
}

//------------------------------------------------------------------------------
// Destructor
//------------------------------------------------------------------------------
/// Destructor
Flock::~Flock( void ) {
    leader = NULL;
}

//------------------------------------------------------------------------------
/// Add a boid to the flock
void Flock::add( Boid* boid ) {
    // check to see if already a member of this flock
    if( contains( boid ) ) return;

    // remove the boid from its current flock
    Flock* owner = boid->member_of;
    if( owner != NULL ) owner->remove( boid );

    // add it to this flock as a follower
    boid->role = BOID_ROLE_FOLLOWER;
    members.push_back( boid );

    // arbitrate

}

//------------------------------------------------------------------------------
/// Remove a given boid from the flock
/// Note: O(n) search inefficiency here.  Better approach would be to give every
/// boid a UID and handle through a map -> O(c), but this is down and dirty for now
/// Try to avoid using contains and remove as that will be O(cn).
void Flock::remove( Boid* boid ) {
    for( std::vector<Boid*>::iterator it = members.begin(); it != members.end(); it++ ) {
        if( (*it) == boid ) {
            // if the boid is the leader have to promote another
            // requires arbitration to determine who is new leader

            // remove the boid from the flock member list
            members.erase(it);
            break;
        }
    }
}

//------------------------------------------------------------------------------
/// Determine whether the flock contains a given boid
/// Note: O(n) search inefficiency here.  Better approach would be to give every
/// boid a UID and handle through a map -> O(c), but this is down and dirty for now
bool Flock::contains( Boid* boid ) {
    for( std::vector<Boid*>::iterator it = members.begin(); it != members.end(); it++ )
        if( (*it) == boid ) return true;
    return false;
}

//------------------------------------------------------------------------------
/// promote a given boid to be the leader
/// Note: may be better to return integer failure states rather than bool
bool Flock::promote( Boid* boid ) {
    // make sure boid is a member of this flock
    if( !contains( boid ) ) return false;

    // chance to validate member_of.  may be critical later for quick referencing
    if( boid->member_of != this ) {
        // Note: a little redundant to check then set, but more logic may be
        // needed if incongruity is detected
        boid->member_of = this;
    }

    // chance to validate whether search is necessary and get out early if not
    if( boid->role == BOID_ROLE_LEADER ) return true;

    for( std::vector<Boid*>::iterator it = members.begin(); it != members.end(); it++ ) {
        if( (*it) == boid ) {
            // promote the new leader
            boid->role = BOID_ROLE_LEADER;
            // demote the old leader
            if( leader != NULL )
                leader->role = BOID_ROLE_FOLLOWER;
            // swap the two
            leader = boid;
            return true;
        }
    }
    // should never get here.
    return false;
}

//------------------------------------------------------------------------------
/// Merges two seperate flocks into a new flock
/// Note, does not deallocate original flock pointers.  Deallocation is left up
/// to the provider of the flock pointers
Flock* Flock::merge( Flock* flock1, Flock* flock2 ) {
    Flock* result = new Flock( *flock1 );
    for( std::vector<Boid*>::iterator it = flock2->members.begin(); it != flock2->members.end(); it++ ) {
        result->add( *it );
    }
    // arbitrate between leaders and goals

    return result;
}

//------------------------------------------------------------------------------

/// generates a randomized flock
Flock* Flock::generate_flock( Material* material, const Color& color, const Vector3& flock_center, const unsigned int& number_of_boids, const double& flock_extens ) {
    Boid* boid;
    Flock* flock = new Flock();

    // assign to master list of boids
    double radius = 1.0;
    double height = 3.0;
    for( unsigned int i = 0; i < number_of_boids; i++ ) {
        boid = new Boid();

        // create the mesh
        boid->body.insert( MeshLoader::cone( radius, height, 16, material, ML_MATERIAL_ASSIGN ) );

        // randomly generate the position, constrained to the flock extens
        double x = flock_center.x() + ( (double) rand() / (double) RAND_MAX ) * flock_extens - flock_extens / 2.0;
        double y = 0.0;
        double z = flock_center.z() + ( (double) rand() / (double) RAND_MAX ) * flock_extens - flock_extens / 2.0;
        boid->body.pose.position = Vector3( x, y, z );

        boid->member_of = flock;

        boid->body.mass = 1.0;
        boid->goal = BOID_GOAL_LOCAL;
        boid->role = BOID_ROLE_FOLLOWER;
        boid->motivator_detector.position = Vector3( 0.0, 0.0, height );

        // rate of yaw in terms of frames
        // Note: this is an artifact of previous generations, but still necessary due to the computation
        // of the initial state
        boid->max_yaw_rate_per_frame = RAD_PER_DEG / 1.0;

        // constraint on the velocity otherwise lim of velocity due to acceleration -> infinity
        //boid->max_forward_velocity_per_frame = 0.5;
        boid->max_forward_velocity_per_frame = 0.25;

        // constraint on the acceleration
        boid->max_acceleration_per_frame = boid->max_forward_velocity_per_frame / 10.0;

        boid->color = color;

        // bubbles
        boid->personal_space_radius = 3.0;
        boid->neighbor_perception_radius = 20.0;

        // suuggestion weights
        boid->motivator_force = 0.1;
        boid->avoidance_force = 0.5;
        boid->matching_force = 0.2;
        boid->centering_force = 0.2;

        // initial state
        boid->arbitrated_orientation = Vector3( 0.0, 0.0, 1.0 );
        boid->arbitrated_linear_acceleration = Vector3( 0.0, 0.0, 0.0 );
        boid->arbitrated_linear_velocity = Vector3( 0.0, 0.0, 0.0 );
        double velocity = boid->arbitrated_linear_velocity.magnitude();
        boid->arbitrated_position = Math::integrate_position( boid->motivator_detector.position, boid->arbitrated_orientation, boid->body.pose.position, velocity );

        // add this boid to the flock
        flock->add( boid );
    }
    return flock;
}

//------------------------------------------------------------------------------
/// iterates over the list of boids and computes all perception
/// O(n^2) time complexity
void Flock::compute_boid_perception( void ) {
    for( std::vector<Boid*>::iterator bit = members.begin(); bit != members.end(); bit++ ) {
        Boid* boid = (*bit);
        boid->get_neighbors( boid->member_of->members, boid->neighbor_perception_radius );
    }
}

//------------------------------------------------------------------------------
/// gather suggestions for each behavior for each boid
/// O(n) time complexity
void Flock::gather_suggestions( Motivator* attractor ) {
    for( std::vector<Boid*>::iterator bit = members.begin(); bit != members.end(); bit++ ) {
        Boid* boid = (*bit);

        boid->previous_linear_acceleration = boid->arbitrated_linear_acceleration;
        boid->previous_linear_velocity = boid->arbitrated_linear_velocity;
        boid->previous_position = boid->arbitrated_position;
        boid->previous_orientation = boid->arbitrated_orientation;
    }

    for( std::vector<Boid*>::iterator bit = members.begin(); bit != members.end(); bit++ ) {
        Boid* boid = (*bit);
        boid->goto_motivator_acceleration = Boid::suggest_goto_motivator( boid, attractor );
        boid->collision_avoidance_acceleration = Boid::suggest_collision_avoidance( boid );
        boid->velocity_matching_acceleration = Boid::suggest_velocity_matching( boid );
        boid->flock_centering_acceleration = Boid::suggest_flock_centering( boid );
    }
}

//------------------------------------------------------------------------------
/// arbitrate among all behavioral suggestions for each boid
void Flock::arbitrate_suggestions( void ) {
    for( std::vector<Boid*>::iterator bit = members.begin(); bit != members.end(); bit++ ) {
        Boid* boid = (*bit);

        double mag_avoidance_suggestion = boid->collision_avoidance_acceleration.magnitude();
        double mag_matching_suggestion = boid->velocity_matching_acceleration.magnitude();
        double mag_centering_suggestion = boid->flock_centering_acceleration.magnitude();

        Vector3 arbitrated_acceleration_heading;
        double arbitrated_acceleration_magnitude;

        if( mag_avoidance_suggestion > mag_matching_suggestion &&
            mag_avoidance_suggestion > mag_centering_suggestion ) {
            // collision avoidance is priority
            arbitrated_acceleration_heading = boid->collision_avoidance_acceleration;

            // ?parcelling?
        } else if( mag_matching_suggestion > mag_avoidance_suggestion &&
                   mag_matching_suggestion > mag_centering_suggestion ) {
            // velocity matching is priority
            arbitrated_acceleration_heading = boid->velocity_matching_acceleration;

            // ?parcelling?
        } else if( mag_centering_suggestion > mag_avoidance_suggestion &&
                   mag_centering_suggestion > mag_matching_suggestion ) {
            // flock centering is priority
            arbitrated_acceleration_heading = boid->flock_centering_acceleration;

            // ?parcelling?
        } else {
            // otherwise goto motivator
            arbitrated_acceleration_heading = boid->goto_motivator_acceleration;

            // ?parcelling?
        }

        // now compute the action
        // first update acceleration
        arbitrated_acceleration_magnitude = arbitrated_acceleration_heading.magnitude();
        arbitrated_acceleration_heading.normalize();
        if( arbitrated_acceleration_magnitude > boid->max_acceleration_per_frame ) {
            arbitrated_acceleration_magnitude = boid->max_acceleration_per_frame;
        }
        boid->arbitrated_linear_acceleration = arbitrated_acceleration_heading * arbitrated_acceleration_magnitude;

        // second integrate velocity
        if( boid->previous_linear_velocity.magnitude() > boid->max_forward_velocity_per_frame ) {
            boid->arbitrated_linear_velocity = arbitrated_acceleration_heading * boid->max_forward_velocity_per_frame;
        } else {
            boid->arbitrated_linear_velocity = boid->previous_linear_velocity + boid->arbitrated_linear_acceleration;
        }

        // third integrate position
        boid->arbitrated_position = boid->previous_position + boid->arbitrated_linear_velocity;

        // fourth derive the orientation from the velocity vector
        Vector3 world_heading = boid->arbitrated_linear_velocity;
        world_heading.normalize();
        Vector3 local_heading = boid->motivator_detector.position;
        local_heading.normalize();
        double theta = -Math::angle_in_x_z( world_heading, local_heading );
        theta = Math::normalize_angle( theta );
        boid->arbitrated_orientation = Vector3( 0.0, theta, 0.0 );

    }

    // update the actual pose from the arbitrated values
    for( std::vector<Boid*>::iterator bit = members.begin(); bit != members.end(); bit++ ) {
        Boid* boid = (*bit);

        // update the pose
        boid->body.pose.orientation = boid->arbitrated_orientation;
        boid->body.pose.position = boid->arbitrated_position;
    }
    return;
}
