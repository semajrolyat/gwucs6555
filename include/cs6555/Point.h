/*-----------------------THE GEORGE WASHINGTON UNIVERSITY-----------------------
James Taylor                                                         jrt@gwu.edu

Point3 class definition and implementation

------------------------------------------------------------------------------*/

#ifndef _POINT3_H_
#define _POINT3_H_

//------------------------------------------------------------------------------

#include <cs6555/Math/Vector3.h>

//------------------------------------------------------------------------------

class Point3 {
public:
    // Constructors
    /// Default Constructor
    Point3( void ) {
        position = Vector3( 0.0, 0.0, 0.0 );
    }
    /// Component Constructor
    Point3( const Vector3& position ) {
        this->position = Vector3( position );
    }

    // Destructor
    /// Destructor
    virtual ~Point3( void ) { }

    // Member Data
    Vector3 position;               // contol point position in 3-space

};

//------------------------------------------------------------------------------

#endif // _POINT3_H_
