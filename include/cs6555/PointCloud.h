/*-----------------------THE GEORGE WASHINGTON UNIVERSITY-----------------------
James Taylor                                                         jrt@gwu.edu

PointCloud class

Octree
------------------------------------------------------------------------------*/

#ifndef _POINTCLOUD_H_
#define _POINTCLOUD_H_

//------------------------------------------------------------------------------

#include <cs6555/Point3.h>
#include <cs6555/Geometry.h>

#include <vector>
#include <assert.h>

//------------------------------------------------------------------------------

class PointCloud : public Geometry {
public:

    /// Default Constructor
    PointCloud( void ) {
        m_count = 0;
        m_level = 0;
        m_point = NULL;

        m_min = Vector3( std::numeric_limits<double>::min(), std::numeric_limits<double>::min(), std::numeric_limits<double>::min() );
        m_max = Vector3( std::numeric_limits<double>::max(), std::numeric_limits<double>::max(), std::numeric_limits<double>::max() );
        m_center = Vector3( 0.0, 0.0, 0.0 );
    }

    /// Top of recursion.  Subdivides and puts points in appropriate bin
    PointCloud( std::vector<Point3*> points ) {
        m_count = points.size();
        m_level = 0;
        m_point = NULL;

        m_min = Vector3( std::numeric_limits<double>::min(), std::numeric_limits<double>::min(), std::numeric_limits<double>::min() );
        m_max = Vector3( std::numeric_limits<double>::max(), std::numeric_limits<double>::max(), std::numeric_limits<double>::max() );
        m_center = Vector3( 0.0, 0.0, 0.0 );

        subdivide( m_level + 1, points, m_min, m_max, m_center );
    }

    /// Recursive step.  Subdivides and puts points in appropriate bin
    PointCloud( const unsigned int& level, std::vector<Point3*> points, const Vector3& min, const Vector3& max ) {
        m_count = points.size();
        m_level = level;
        m_point = NULL;

        m_min = min;
        m_max = max;
        m_center = ( m_max - m_min ) / 2.0;

        if( m_count == 0 )  {
            return;
        } else if( m_count == 1 ) {
            m_point = points.at( 0 );
        } else {
            subdivide( m_level + 1, points, m_min, m_max, m_center );
        }
    }

    virtual ~PointCloud( void ) {
        m_point = NULL;
    }

    void subdivide( const unsigned int& level, std::vector<Point3*> points, const Vector3& min, const Vector3& max, const Vector3& center ) {

        // 0:  x,  y,  z
        // 1:  x, -y,  z
        // 2:  x, -y, -z
        // 3:  x,  y, -z
        // 4: -x,  y,  z
        // 5: -x, -y,  z
        // 6: -x, -y, -z
        // 7: -x,  y, -z

        std::vector<Point3*> p0, p1, p2, p3, p4, p5, p6, p7;
        for( std::vector<Point3*>::iterator it = points.begin(); it != points.end(); it++ ) {
            Point3* point = (*it);

            if( point->position.x() >= center.x() && point->position.y() >= center.y() && point->position.z() >= center.z() ) {
                p0.push_back( point );
            } else if( point->position.x() >= center.x() && point->position.y() < center.y() && point->position.z() >= center.z() ) {
                p1.push_back( point );
            } else if( point->position.x() >= center.x() && point->position.y() < center.y() && point->position.z() < center.z() ) {
                p2.push_back( point );
            } else if( point->position.x() >= center.x() && point->position.y() >= center.y() && point->position.z() < center.z() ) {
                p3.push_back( point );
            } else if( point->position.x() < center.x() && point->position.y() >= center.y() && point->position.z() >= center.z() ) {
                p4.push_back( point );
            } else if( point->position.x() < center.x() && point->position.y() < center.y() && point->position.z() >= center.z() ) {
                p5.push_back( point );
            } else if( point->position.x() < center.x() && point->position.y() < center.y() && point->position.z() < center.z() ) {
                p6.push_back( point );
            } else if( point->position.x() < center.x() && point->position.y() >= center.y() && point->position.z() < center.z() ) {
                p7.push_back( point );
            }
        }
        Vector3 tmax0 = Vector3( max.x(), max.y(), max.z() );
        Vector3 tmin0 = Vector3( center.x(), center.y(), center.z() );

        Vector3 tmax1 = Vector3( max.x(), center.y(), max.z() );
        Vector3 tmin1 = Vector3( center.x(), min.y(), center.z() );

        Vector3 tmax2 = Vector3( max.x(), center.y(), center.z() );
        Vector3 tmin2 = Vector3( center.x(), min.y(), min.z() );

        Vector3 tmax3 = Vector3( max.x(), max.y(), center.z() );
        Vector3 tmin3 = Vector3( center.x(), center.y(), min.z() );

        Vector3 tmax4 = Vector3( center.x(), max.y(), max.z() );
        Vector3 tmin4 = Vector3( min.x(), center.y(), center.z() );

        Vector3 tmax5 = Vector3( center.x(), center.y(), max.z() );
        Vector3 tmin5 = Vector3( min.x(), min.y(), center.z() );

        Vector3 tmax6 = Vector3( center.x(), center.y(), center.z() );
        Vector3 tmin6 = Vector3( min.x(), min.y(), min.z() );

        Vector3 tmax7 = Vector3( center.x(), max.y(), center.z() );
        Vector3 tmin7 = Vector3( min.x(), center.y(), min.z() );

        m_pc[0] = new PointCloud( level + 1, p0, tmin0, tmax0 );
        m_pc[1] = new PointCloud( level + 1, p1, tmin1, tmax1 );
        m_pc[2] = new PointCloud( level + 1, p2, tmin2, tmax2 );
        m_pc[3] = new PointCloud( level + 1, p3, tmin3, tmax3 );
        m_pc[4] = new PointCloud( level + 1, p4, tmin4, tmax4 );
        m_pc[5] = new PointCloud( level + 1, p5, tmin5, tmax5 );
        m_pc[6] = new PointCloud( level + 1, p6, tmin6, tmax6 );
        m_pc[7] = new PointCloud( level + 1, p7, tmin7, tmax7 );

    }

    // [Base Class] Geometry::Type Queries
    virtual EGeometryType geometry_type( void ) { return GEOMETRY_TYPE_POINTCLOUD; }

    // return the set of points
    virtual std::vector<Point3*> points( void ) {
        std::vector<Point3*> pts;
        if( m_count == 0 ) {
            return pts;
        } else if( m_count == 1 ) {
            pts.push_back( m_point );
        } else {
            for( unsigned int i = 0; i < 8; i++ ) {
                std::vector<Point3*> p;
                p = m_pc[i]->points();
                // merge
                pts.insert( pts.end(), p.begin(), p.end() );
            }
        }
        return pts;
    }

    virtual unsigned int count( void ) {
        return m_count;
    }

    virtual void randomize( const unsigned int& n, const Color& color, const Vector3& center, const double& extens ) {
        std::vector<Point3*> points;
        Vector3 min = Vector3( std::numeric_limits<double>::max(), std::numeric_limits<double>::max(), std::numeric_limits<double>::max() );
        Vector3 max = Vector3( std::numeric_limits<double>::min(), std::numeric_limits<double>::min(), std::numeric_limits<double>::min() );

        for( unsigned int i = 0; i < n; i++ ) {
            Point3* p = Point3::generate_point( color, center, extens );
            points.push_back( p );

            min.x( std::min( p->position.x(), min.x() ) );
            min.y( std::min( p->position.y(), min.y() ) );
            min.z( std::min( p->position.z(), min.z() ) );
            max.x( std::max( p->position.x(), max.x() ) );
            max.y( std::max( p->position.y(), max.y() ) );
            max.z( std::max( p->position.z(), max.z() ) );

            m_count++;
        }

        m_min = min;
        m_max = max;
        subdivide( m_level + 1, points, m_min, m_max, center );
    }

    //const unsigned int MAX_DEPTH = 5;

protected:
    //std::vector<Points*> m_points[8];
    unsigned int m_count;
    Point3* m_point;
    PointCloud* m_pc[8];
    //PointCloud m_pc[8] = { {}, {}, {}, {}, {}, {}, {}, {} };
    unsigned int m_level;

    Vector3 m_min, m_max, m_center;        // extens

};

//------------------------------------------------------------------------------

#endif // _POINTCLOUD_H_
