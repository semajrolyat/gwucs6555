#ifndef _SENSOR_H_
#define _SENSOR_H_

//------------------------------------------------------------------------------

#include <cs6555/Math/Vector3.h>

//------------------------------------------------------------------------------

class Sensor {
public:
    Sensor( void ) {
        position = Vector3( 0.0, 0.0, 0.0 );
    }
    virtual ~Sensor( void ) { }

    Vector3 position;
};

//------------------------------------------------------------------------------

#endif // _SENSOR_H_
