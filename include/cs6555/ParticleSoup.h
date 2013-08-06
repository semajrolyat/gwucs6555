#ifndef _PARTICLESOUP_H_
#define _PARTICLESOUP_H_

//------------------------------------------------------------------------------

#include <assert.h>
#include <vector>
#include <cs6555/Point3.h>
#include <cs6555/PointCloud.h>
#include <cs6555/Particle.h>

//------------------------------------------------------------------------------

class ParticleSoup : public PointCloud {
public:
    ParticleSoup( void ) {
        m_count = 0;
        prototype = new Particle();
        prototype->generate_mesh( 5.0 );
    }
    virtual ~ParticleSoup( void ) {
        delete prototype;
    }

    virtual Point3* point( const unsigned int& i ) {
        assert( i < m_count );
        return (Point3*) m_particles.at( i );
    }
    virtual unsigned int count( void ) {
        return m_count;
    }

    virtual void randomize( const unsigned int& n, const Color& color, const Vector3& center, const double& extens ) {
        for( unsigned int i = 0; i < n; i++ ) {
            m_particles.push_back( Particle::generate_particle( color, center, extens ) );
            m_count++;
        }
    }

    // really should be LOD
    virtual double size( void ) {
        return 5.0;
    }

    Particle* prototype;

protected:
    std::vector<Particle*> m_particles;
    unsigned int m_count;

};

//------------------------------------------------------------------------------

#endif // _PARTICLESOUP_H_
