#ifndef _PARTICLECLOUD_H_
#define _PARTICLECLOUD_H_

//------------------------------------------------------------------------------

#include <cs6555/Particle.h>

#include <vector>
#include <stdio.h>
#include <cs6555/Constants.h>

//------------------------------------------------------------------------------

typedef void CloudColorFun( Particle& particle, const double& radius, const double& max );

//------------------------------------------------------------------------------

class ParticleCloud : public Particle {
public:
    ParticleCloud( void ) {
        //colorfun = &ParticleCloud::flame;
        //colorfun = &ParticleCloud::ion;
    }

    ParticleCloud( CloudColorFun* colorfun ) {
        this->colorfun = colorfun;
    }

    virtual ~ParticleCloud( void ) { }

    virtual EParticleType particle_type( void ) {
        return PARTICLE_TYPE_CLOUD;
    }

    CloudColorFun* colorfun;

    static void water( Particle& particle, const double& radius, const double& max ) {
        if( radius > 0.96 * max )
            particle.color = Color::white();
        else if( radius > 0.85 * max )
            particle.color = Color::cyan();
        else if( radius > 0.1 * max )
            particle.color = Color::blue();
        else
            particle.color = Color::blue();
    }

    static void cloud( Particle& particle, const double& radius, const double& max ) {
        if( radius > 0.96 * max )
            particle.color = Color::white();
        else if( radius > 0.8 * max )
            particle.color = Color( 0.8, 0.8, 0.8, 1.0 );
        else if( radius > 0.5 * max )
            particle.color = Color( 0.6, 0.6, 0.6, 1.0 );
        else if( radius > 0.1 * max )
            particle.color = Color( 0.5, 0.5, 0.5, 1.0 );
        else
            particle.color = Color::black();
    }

    static void ion( Particle& particle, const double& radius, const double& max ) {
        if( radius > 0.98 * max )
            particle.color = Color::yellow();
        else if( radius > 0.5 * max )
            particle.color = Color::cyan();
        else
            particle.color = Color::white();
    }

    static void flame( Particle& particle, const double& radius, const double& max ) {
        // flame ring
        if( radius > 0.96 * max )
            particle.color = Color::black();
        else if( radius > 0.70 * max )
            particle.color = Color::red();
        else if( radius > 0.40 * max )
            particle.color = Color::orange();
        else if( radius > 0.25 * max )
            particle.color = Color::yellow();
        else if( radius > 0.24 * max )
            particle.color = Color::cyan();
        else
            particle.color = Color::white();
    }

    static void blackhole( Particle& particle, const double& radius, const double& max ) {
        if( radius > 0.96 * max )
            particle.color = Color( 0.2, 0.2, 0.2, 1.0 );
        else if( radius > 0.92 * max )
            particle.color = Color( 0.6, 0.6, 0.6, 1.0 );
        else if( radius > 0.70 * max )
            particle.color = Color::red();
        else if( radius > 0.40 * max )
            particle.color = Color::orange();
        else if( radius > 0.25 * max )
            particle.color = Color::yellow();
        else if( radius > 0.23 * max )
            particle.color = Color::green();
        else if( radius > 0.22 * max )
            particle.color = Color::blue();
        else if( radius > 0.20 * max )
            particle.color = Color::white();
        else
            particle.color = Color::black();
    }

    static void black( Particle& particle, const double& radius, const double& max ) {
            particle.color = Color::black();
    }
    static void red( Particle& particle, const double& radius, const double& max ) {
            particle.color = Color::red();
    }
    static void orange( Particle& particle, const double& radius, const double& max ) {
            particle.color = Color::orange();
    }
    static void yellow( Particle& particle, const double& radius, const double& max ) {
            particle.color = Color::yellow();
    }
    static void green( Particle& particle, const double& radius, const double& max ) {
            particle.color = Color::green();
    }
    static void blue( Particle& particle, const double& radius, const double& max ) {
            particle.color = Color::blue();
    }
    static void cyan( Particle& particle, const double& radius, const double& max ) {
            particle.color = Color::cyan();
    }
    static void white( Particle& particle, const double& radius, const double& max ) {
            particle.color = Color::white();
    }

    void jitter( double& x,
                 double& y,
                 double& z,
                 const double& max_x_jitter,
                 const double& max_y_jitter,
                 const double& max_z_jitter
    ) {
        x += (double)rand() / (double)RAND_MAX * max_x_jitter - max_x_jitter / 2.0;
        y += (double)rand() / (double)RAND_MAX * max_y_jitter - max_y_jitter / 2.0;
        z += (double)rand() / (double)RAND_MAX * max_z_jitter - max_z_jitter / 2.0;
    }

    void add( CloudColorFun* colorfun,
              const Particle& prototype,
              const Vector3& center,
              const double& max_radius,
              const double& radius,
              double& x,
              double& y,
              double& z )
    {
        Particle* particle = new Particle( );
        particle->mass = prototype.mass;
        particle->frames_to_live = prototype.frames_to_live;

        //jitter( x, y, z, 0.01, 0.01, 0.01 );

        particle->position = Vector3( x + center.x(), y + center.y(), z + center.z() );
        //----
        ///*
        particle->force = Vector3( x, y, z );
        particle->force.normalize();
        //particle->force *= 10.0 * radius / max_radius;
        particle->force *= 1e-28;
        //*/
        //----

        (*colorfun)( *particle, radius, max_radius );
        //particle->color = Color( 1.0, 1.0, 1.0, 1.0 );
        insert( particle );
    }

    static ParticleCloud* ring( CloudColorFun* colorfun,
                                const Particle& prototype,
                                const Vector3& center,
                                const double& max_radius,
                                const double& min_radius,
                                const unsigned int& particles )
    {
        assert( max_radius > 0.0 && min_radius >= 0 && max_radius > min_radius );

        ParticleCloud* pc = new ParticleCloud( colorfun );
        pc->position = center;

        double radius = (max_radius + min_radius) / 2.0;
        double ring_radius = (max_radius - min_radius) / 2.0;

        for( unsigned int i = 0; i < particles; i++ ) {
            double x1, x2, y1, y2, z1, z2, r1, r2;

            x1 = ( (double)rand() / (double)RAND_MAX * 2.0 - 1.0 );
            y1 = (double)rand() / (double)RAND_MAX * 2.0 - 1.0;
            z1 = (double)rand() / (double)RAND_MAX * 2.0 - 1.0;
            r1 = (double)rand() / (double)RAND_MAX * ring_radius;

            x2 = ( (double)rand() / (double)RAND_MAX * 2.0 - 1.0 );
            y2 = (double)rand() / (double)RAND_MAX * 2.0 - 1.0;
            z2 = 0.0;
            r2 = radius;

            Vector3 v1 = Vector3( x1, y1, z1 );
            v1.normalize();
            v1 *= r1;

            Vector3 v2 = Vector3( x2, y2, z2 );
            v2.normalize();
            v2 *= r2;

            Vector3 v3 = v2 + v1;

            double x, y, z, r;
            x = v3.x();
            y = v3.y();
            z = v3.z();
            r = r2 + r1;

            pc->add( colorfun, prototype, center, max_radius, r, x, y, z );
        }

        return pc;
    }

    static ParticleCloud* disk( CloudColorFun* colorfun,
                                const Particle& prototype,
                                const Vector3& center,
                                const double& max_radius,
                                const double& thickness,
                                const unsigned int& particles )
    {
        assert( max_radius > 0.0 );

        ParticleCloud* pc = new ParticleCloud( colorfun );
        pc->position = center;

        for( unsigned int i = 0; i < particles; i++ ) {
            double x, y, z, r;

            x = ( (double)rand() / (double)RAND_MAX * 2.0 - 1.0 );
            y = (double)rand() / (double)RAND_MAX * 2.0 - 1.0;
            z = (double)rand() / (double)RAND_MAX * thickness - thickness / 2.0;
            r = (double)rand() / (double)RAND_MAX * max_radius;

            Vector3 v = Vector3( x, y, z );
            v.normalize();
            v *= r;

            x = v.x();
            y = v.y();
            z = v.z();

            pc->add( colorfun, prototype, center, max_radius, r, x, y, z );
        }

        return pc;
    }

    static ParticleCloud* shell( CloudColorFun* colorfun,
                                  const Particle& prototype,
                                  const Vector3& center,
                                  const double& max_radius,
                                  const double& min_radius,
                                  const unsigned int& particles )
    {
        assert( max_radius > 0.0 && min_radius > 0 && max_radius > min_radius );

        ParticleCloud* pc = new ParticleCloud( colorfun );
        pc->position = center;

        double radius = (max_radius + min_radius) / 2.0;
        double ring_radius = (max_radius - min_radius) / 2.0;

        for( unsigned int i = 0; i < particles; i++ ) {

            double x1, x2, y1, y2, z1, z2, r1, r2;

            /*
            // Note: Cartesian aggregates around x=+-y=+-z
            x1 = ( (double)rand() / (double)RAND_MAX * 1.8 - 0.9 );
            y1 = (double)rand() / (double)RAND_MAX * 1.8 - 0.9;
            z1 = (double)rand() / (double)RAND_MAX * 1.8 - 0.9;
            r1 = (double)rand() / (double)RAND_MAX * ring_radius;

            x2 = ( (double)rand() / (double)RAND_MAX * 1.8 - 0.9 );
            y2 = (double)rand() / (double)RAND_MAX * 1.8 - 0.9;
            z2 = (double)rand() / (double)RAND_MAX * 1.8 - 0.9;
            r2 = radius;

            Vector3 v1 = Vector3( x1, y1, z1 );
            v1.normalize();
            v1 *= r1;

            Vector3 v2 = Vector3( x2, y2, z2 );
            v2.normalize();
            v2 *= r2;

            Vector3 v3 = v2 + v1;

            double x, y, z, r;
            x = v3.x();
            y = v3.y();
            z = v3.z();
            r = r2 + r1;

            pc->add( colorfun, prototype, center, max_radius, r, x, y, z );
            */
            ///*
            // Note: Polar aggregates around poles
            double theta1 = (double)rand() / (double)RAND_MAX * 2.0 * PI;
            //double phi1 = (double)rand() / (double)RAND_MAX * 98.0/100.0 * PI - 1.0/100.0 * PI;
            double phi1 = (double)rand() / (double)RAND_MAX * PI;
            double theta2 = (double)rand() / (double)RAND_MAX * 2.0 * PI;
            //double phi2 = (double)rand() / (double)RAND_MAX * 98.0/100.0 * PI + 1.0/100.0 * PI;
            double phi2 = (double)rand() / (double)RAND_MAX * PI;

            x1 = sin(phi1) * cos(theta1);
            y1 = cos(phi1);
            z1 = sin(phi1) * sin(theta1);
            r1 = (double)rand() / (double)RAND_MAX * ring_radius;

            x2 = sin(phi2) * cos(theta2);
            y2 = cos(phi2);
            z2 = sin(phi2) * sin(theta2);
            r2 = radius;

            Vector3 v1 = Vector3( x1, y1, z1 );
            v1.normalize();
            v1 *= r1;

            Vector3 v2 = Vector3( x2, y2, z2 );
            v2.normalize();
            v2 *= r2;

            Vector3 v3 = v2 + v1;

            double x, y, z, r;
            x = v3.x();
            y = v3.y();
            z = v3.z();
            r = r2 + r1;

            pc->add( colorfun, prototype, center, max_radius, r, x, y, z );
            //*/
        }

        return pc;
    }

    static ParticleCloud* sphere( CloudColorFun* colorfun,
                                  const Particle& prototype,
                                  const Vector3& center,
                                  const double& max_radius,
                                  const unsigned int& particles )
    {
        assert( particles > 0 );

        ParticleCloud* pc = new ParticleCloud( colorfun );
        pc->position = center;

        for( unsigned int i = 0; i < particles; i++ ) {
            double x, y, z, r;

            x = ( (double)rand() / (double)RAND_MAX * 2.0 - 1.0 );
            y = (double)rand() / (double)RAND_MAX * 2.0 - 1.0;
            z = (double)rand() / (double)RAND_MAX * 2.0 - 1.0;
            r = (double)rand() / (double)RAND_MAX * max_radius;

            Vector3 v = Vector3( x, y, z );
            v.normalize();
            v *= r;

            x = v.x();
            y = v.y();
            z = v.z();

            pc->add( colorfun, prototype, center, max_radius, r, x, y, z );
        }
        return pc;
    }

    // Note: Spherical use of colorfun
    static ParticleCloud* cube( CloudColorFun* colorfun,
                                const Particle& prototype,
                                const Vector3& center,
                                const double& max_radius,
                                const unsigned int& particles )
    {
        assert( particles > 0 );

        ParticleCloud* pc = new ParticleCloud( colorfun );
        pc->position = center;

        for( unsigned int i = 0; i < particles; i++ ) {
            double x, y, z, r;

            x = ( (double)rand() / (double)RAND_MAX * 2.0 - 1.0 );
            y = (double)rand() / (double)RAND_MAX * 2.0 - 1.0;
            z = (double)rand() / (double)RAND_MAX * 2.0 - 1.0;

            Vector3 v = Vector3( x, y, z );
            r = v.magnitude();

            pc->add( colorfun, prototype, center, max_radius, r, x, y, z );
        }
        return pc;
    }
};

//------------------------------------------------------------------------------


#endif // _PARTICLECLOUD_H_
