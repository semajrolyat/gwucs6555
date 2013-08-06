#ifndef _PARTICLEEMITTER_H_
#define _PARTICLEEMITTER_H_

//------------------------------------------------------------------------------

#include <cs6555/Particle.h>

//------------------------------------------------------------------------------

class ParticleEmitter : public Particle {
public:
    ParticleEmitter( void ) { }
    virtual ~ParticleEmitter( void ) { }

    virtual EParticleType particle_type( void ) {
        return PARTICLE_TYPE_EMITTER;
    }

    Vector3 position;

    Particle prototype;

    unsigned int frames_until_emission;
    unsigned int period;
};

//------------------------------------------------------------------------------

#endif // _PARTICLEEMITTER_H_
