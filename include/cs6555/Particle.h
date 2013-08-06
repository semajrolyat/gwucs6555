#ifndef _PARTICLE_H_
#define _PARTICLE_H_

//------------------------------------------------------------------------------

#include <cs6555/Body.h>

//------------------------------------------------------------------------------

typedef enum {
    PARTICLE_TYPE_UNDEFINED,
    PARTICLE_TYPE_QUANTUM,
    PARTICLE_TYPE_CLOUD,
    PARTICLE_TYPE_EMITTER
} EParticleType;

//------------------------------------------------------------------------------

class Particle : public Body {
public:
    // Constructors
    Particle( void );
    // Destructor
    virtual ~Particle( void );

    // [Base Class] Geometry::Queries
    virtual EGeometryType geometry_type( void );

    // [Base Class] Body::Queries
    virtual EBodyType body_type( void );

    virtual EParticleType particle_type( void );

    unsigned int frames_to_live;

    double temperature;

    double radius;  // can be used for simple sphere collision without bringing in BoundingVolume and Physics
};

//------------------------------------------------------------------------------

#endif // _PARTICLE_H_
