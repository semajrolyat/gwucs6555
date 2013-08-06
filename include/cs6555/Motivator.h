#ifndef _MOTIVATOR_H_
#define _MOTIVATOR_H_

//------------------------------------------------------------------------------

#include <cs6555/Math/Vector3.h>
#include <cs6555/Mesh.h>

//------------------------------------------------------------------------------

typedef enum {
    MOTIVATOR_TYPE_ATTRACTOR,
    MOTIVATOR_TYPE_REPULSOR
} EMotivatorType;

//------------------------------------------------------------------------------

class Motivator : public Mesh {
public:
    Motivator( void ) {
        //position = Vector3( 0.0, 0.0, 0.0 );
        type = MOTIVATOR_TYPE_ATTRACTOR;
    }
    ~Motivator( void ) { }

    //Vector3 position;
    EMotivatorType type;
};

//------------------------------------------------------------------------------

#endif // _MOTIVATOR_H_
