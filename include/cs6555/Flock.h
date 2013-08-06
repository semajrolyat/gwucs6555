#ifndef _FLOCK_H_
#define _FLOCK_H_

//------------------------------------------------------------------------------

#include <cs6555/Boid.h>

#include <vector>

#include <cs6555/Math/Vector3.h>
#include <cs6555/Color.h>
#include <cs6555/Material.h>

#include <cs6555/Motivator.h>

//------------------------------------------------------------------------------

class Flock {
public:
    // Constructors
    Flock( void );
    Flock( Flock& flock );
    // Destructor
    virtual ~Flock( void );

    //
    void add( Boid* boid );
    void remove( Boid* boid );
    bool contains( Boid* boid );
    bool promote( Boid* boid );
    static Flock* merge( Flock* flock1, Flock* flock2 );

    static Flock* generate_flock( Material* material, const Color& color, const Vector3& flock_center, const unsigned int& number_of_boids, const double& flock_extens );

    void compute_boid_perception( void );
    void gather_suggestions( Motivator* attractor );
    void arbitrate_suggestions( void );

    // Member Data
    Boid*               leader;
    std::vector<Boid*>  members;

    Vector3 center;
    Vector3 velocity;
    double mass;
};

//------------------------------------------------------------------------------

#endif // _FLOCK_H_
