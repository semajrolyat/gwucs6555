/*-----------------------THE GEORGE WASHINGTON UNIVERSITY-----------------------
James Taylor                                                         jrt@gwu.edu

Renderer class definition

------------------------------------------------------------------------------*/

#ifndef _RENDERER_H_
#define _RENDERER_H_

#include <cs6555/Mesh.h>
#include <cs6555/Motivator.h>
#include <cs6555/Trajectory.h>
#include <cs6555/Body.h>
#include <cs6555/Link.h>
#include <cs6555/ArticulatedBody.h>
#include <cs6555/DeformableBody.h>
#include <cs6555/Scene.h>
#include <cs6555/Particle.h>
#include <cs6555/ParticleCloud.h>

//------------------------------------------------------------------------------

class Renderer {
public:
    static void activate( const Material& material );

    static void draw( Mesh* mesh );
    static void draw( Motivator* motivator );
    static void draw( Point3* point );
    static void draw( Trajectory* trajectory );
    static void draw( Body* body );
    static void draw( Link* link );
    static void draw( ArticulatedBody* body );
    static void draw( DeformableBody* body );
    static void draw( Scene* scene );
    static void draw( Particle* particle );
    static void draw( ParticleCloud* cloud );
};

//------------------------------------------------------------------------------

#endif // _RENDERER_H_
