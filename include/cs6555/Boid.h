#ifndef _BOID_H_
#define _BOID_H_

//------------------------------------------------------------------------------

#include <cs6555/Body.h>
#include <cs6555/Sensor.h>
#include <cs6555/Motivator.h>

//------------------------------------------------------------------------------
typedef enum {
    BOID_GOAL_GLOBAL,
    BOID_GOAL_LOCAL
} EBoidGoalType;

typedef enum {
    BOID_ROLE_LEADER,
    BOID_ROLE_FOLLOWER,
    BOID_ROLE_INDEPENDENT
} EBoidRoleType;

//------------------------------------------------------------------------------
// Forward Declaration
//------------------------------------------------------------------------------
class Flock;

//------------------------------------------------------------------------------

class Navigation {
public:
    Navigation( void ) {}
    virtual ~Navigation( void ) {}

    Vector3 desired_acceleration;
};

//------------------------------------------------------------------------------

class Pilot {
    Pilot( void ) {}
    virtual ~Pilot( void ) {}

};

//------------------------------------------------------------------------------

class Flight {
    Flight( void ) {}
    virtual ~Flight( void ) {}

};

//------------------------------------------------------------------------------

class Boid {
public:
    // Constructors
    Boid( void );
    // Destructor
    virtual ~Boid( void );

    //Goal
    // if leader, it's the point
    // if follower, it's local to the leader
    // if independent then it'd looking for a flock

    // Behaviors
    void go_to( const Vector3& location );
    void avoid( std::vector<Boid*> boids );
    void follow( Boid* boid );
    void match_velocity( void );
    void center( Flock* flock );

    void get_neighbors( std::vector<Boid*>& boids, const double& radius );

    static const double max_velocity = 5.0;

    void clear_hazards( void );
    void add_hazard( Boid* boid );

    // Member Data
    Body            body;

    EBoidGoalType   goal;
    EBoidRoleType   role;

    Flock*          member_of;

    std::vector<Boid*> neighbors;
    std::vector<Boid*> hazards;

    Sensor motivator_detector;
    Vector3 base_orientation;

    double max_yaw_rate_per_frame;

    double max_forward_velocity_per_frame;
    double max_acceleration_per_frame;

    Color color;

    double personal_space_radius;
    double neighbor_perception_radius;

    Vector3 previous_linear_acceleration;
    Vector3 previous_linear_velocity;
    Vector3 previous_position;
    Vector3 previous_orientation;

    Vector3 arbitrated_linear_acceleration;
    Vector3 arbitrated_linear_velocity;
    Vector3 arbitrated_position;
    Vector3 arbitrated_orientation;

    Vector3 goto_motivator_acceleration;
    Vector3 collision_avoidance_acceleration;
    Vector3 velocity_matching_acceleration;
    Vector3 flock_centering_acceleration;

    double motivator_force;
    double avoidance_force;
    double matching_force;
    double centering_force;

    static Vector3 suggest_goto_motivator( Boid* boid, Motivator* motivator );
    static Vector3 suggest_collision_avoidance( Boid* boid );
    static Vector3 suggest_velocity_matching( Boid* boid );
    static Vector3 suggest_flock_centering( Boid* boid );
};

//------------------------------------------------------------------------------

#endif // _BOID_H_
