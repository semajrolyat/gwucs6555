/*-----------------------THE GEORGE WASHINGTON UNIVERSITY-----------------------
James Taylor                                                         jrt@gwu.edu

PointCloud class implementation

------------------------------------------------------------------------------*/

#include <cs6555/PointCloud.h>

//------------------------------------------------------------------------------
// Constructors
//------------------------------------------------------------------------------
/// Default Constructor
/// Note: You should only use the default constructor if you really know what
/// you're doing.  Generally acts as a Point3.
PointCloud<class T>::PointCloud<class T>( void ) {
    m_bounding = POINTCLOUD_BOUNDING_UNBOUNDED;
    m_dimensionality = POINTCLOUD_DIMENSIONALITY_0D;
    m_setAllocated = false;

    m_width = 0;
    m_height = 0;
    m_depth = 0;
}

PointCloud<class T>::PointCloud<class T>( EPointCloudBounding bounding, EPointCloudDimensionality dimensionality ) {
    m_bounding = bounding;
    m_dimensionality = dimensionality;
    m_setAllocated = false;

    m_width = 0;
    m_height = 0;
    m_depth = 0;
}

//------------------------------------------------------------------------------
// Destructor
//------------------------------------------------------------------------------
/// Destructor
PointCloud<class T>::~PointCloud<class T>( void ) {

}

//------------------------------------------------------------------------------


