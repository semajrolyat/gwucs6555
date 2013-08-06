/*-----------------------THE GEORGE WASHINGTON UNIVERSITY-----------------------
James Taylor                                                         jrt@gwu.edu

Keyframe class definition

Keyframes define the pose through which a body is predicted to pass at a given
point (frame) in time.
------------------------------------------------------------------------------*/

#ifndef _KEYFRAME_H_
#define _KEYFRAME_H_

//------------------------------------------------------------------------------

#include <cs6555/Pose.h>

//------------------------------------------------------------------------------

class Keyframe {
public:
    // Constructors
    Keyframe( void );
    Keyframe( const int& frame, Pose pose );

    // Destructor
    ~Keyframe( void );

    // Member Data
    Pose pose;          // The goal pose for the actor at this keyframe
    unsigned int frame;          // The frame identifier.  Equivalent to time
};

//------------------------------------------------------------------------------

#endif // _KEYFRAME_H_
