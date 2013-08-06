/*-----------------------THE GEORGE WASHINGTON UNIVERSITY-----------------------
James Taylor                                                         jrt@gwu.edu

Point3 class definition and implementation

------------------------------------------------------------------------------*/

#ifndef _POINT3_H_
#define _POINT3_H_

//------------------------------------------------------------------------------

#include <cs6555/Math/Vector3.h>
#include <cs6555/Color.h>

//------------------------------------------------------------------------------

class Point3 {
public:
    // Constructors
    /// Default Constructor
    Point3( void ) {
        position = Vector3( 0.0, 0.0, 0.0 );
        color = Color( 1.0, 1.0, 1.0, 1.0 );
    }
    /// Copy Constructor
    Point3( const Point3& point ) {
        this->position = point.position;
        this->color = point.color;
    }
    /// Component Constructor
    Point3( const Vector3& position ) {
        this->position = position;
        color = Color( 1.0, 1.0, 1.0, 1.0 );
    }
    /// Component Constructor
    Point3( const Color& color ) {
        position = Vector3( 0.0, 0.0, 0.0 );
        this->color = color;
    }
    /// Component Constructor
    Point3( const Vector3& position, const Color& color ) {
        this->position = position;
        this->color = color;
    }

    // Destructor
    /// Destructor
    virtual ~Point3( void ) { }

    static Point3* generate_point( const Color& color, const Vector3& center, const double& extens ) {
        Point3 *p = new Point3();

        double x = center.x() + ( (double) rand() / (double) RAND_MAX ) * extens - extens / 2.0;
        double y = 0.0;
        double z = center.z() + ( (double) rand() / (double) RAND_MAX ) * extens - extens / 2.0;

        p->position = Vector3( x, y, z );
        p->color = color;

        return p;
    }

    // Member Data
    Vector3 position;       // point position in 3-space
    Color color;

};

//------------------------------------------------------------------------------

#endif // _POINT3_H_
