
/*-----------------------THE GEORGE WASHINGTON UNIVERSITY-----------------------
James Taylor                                                         jrt@gwu.edu

Particle class implementation

------------------------------------------------------------------------------*/

#include <cs6555/Particle.h>

#include <assert.h>


//------------------------------------------------------------------------------
// Constructors
//------------------------------------------------------------------------------
/// Default Constructor
Particle::Particle( void ) {

}


//------------------------------------------------------------------------------
// Destructor
//------------------------------------------------------------------------------
/// Destructor
Particle::~Particle( void ) {

}

//------------------------------------------------------------------------------
// [Base Class] Geometry::Queries
//------------------------------------------------------------------------------
/// Geometry Type Query.  Implemented by inheritor.
EGeometryType Particle::geometry_type( void ) {
    return GEOMETRY_TYPE_PARTICLE;
}

//------------------------------------------------------------------------------
// [Base Class] Body::Queries
//------------------------------------------------------------------------------
/// Body Type Query.  Implemented by inheritor.
EBodyType Particle::body_type( void ) {
    return BODY_TYPE_POINTMASS;
}


//------------------------------------------------------------------------------
// [Base Class] Particle::Queries
//------------------------------------------------------------------------------
EParticleType Particle::particle_type( void ) {
    return PARTICLE_TYPE_QUANTUM;
}

