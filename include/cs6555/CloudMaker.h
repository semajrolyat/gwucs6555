#ifndef _CLOUDMAKER_H_
#define _CLOUDMAKER_H_

#include <cs6555/PointCloud.h>
#include <cs6555/ParticleSoup.h>

class CloudMaker {
public:
    static PointCloud* random_pointcloud( const unsigned int& n, const Color& color, const Vector3& center, const double& extens );
    static ParticleSoup* random_particle_soup( const unsigned int& n, const Color& color, const Vector3& center, const double& extens );
};

#endif // _CLOUDMAKER_H_
