/*-----------------------THE GEORGE WASHINGTON UNIVERSITY-----------------------
James Taylor                                                         jrt@gwu.edu

Actor class definition

The actor class unites geometry with trajectory allowing models to be given
movement definitions on top of geometrical definitions

------------------------------------------------------------------------------*/

#ifndef _ACTOR_H_
#define _ACTOR_H_

#include <cs6555/Geometry.h>
#include <cs6555/Trajectory.h>

//------------------------------------------------------------------------------

class Actor : public Geometry {
public:
    // Constructors
    Actor( void );

    // Destructor
    virtual ~Actor( void );

    // Member Data
    Trajectory trajectory;      // the trajectory for the actor to follow

};

//------------------------------------------------------------------------------

#endif // _ACTOR_H_
