/*-----------------------THE GEORGE WASHINGTON UNIVERSITY-----------------------
James Taylor                                                         jrt@gwu.edu

LineSegment class

------------------------------------------------------------------------------*/

#ifndef _LINESEGMENT_H_
#define _LINESEGMENT_H_

#include <cs6555/Math/Vector3.h>

class LineSegment {
public:
    static double length( const Vector3& p1, const Vector3& p2 ) {
        double dx = 0.0, dy = 0.0, dz = 0.0;

        dx = fabs( p2.x() - p1.x() );
        dy = fabs( p2.y() - p1.y() );
        dz = fabs( p2.z() - p1.z() );

        return sqrt( dx*dx + dy*dy + dz*dz );
    }

};

#endif // _LINESEGMENT_H_
