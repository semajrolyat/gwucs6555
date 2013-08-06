#include <cs6555/CloudMaker.h>

PointCloud* CloudMaker::random_pointcloud( const unsigned int& n, const Color& color, const Vector3& center, const double& extens ) {
    PointCloud* pc = new PointCloud();
    pc->randomize( n, color, center, extens );
    return pc;
}

ParticleSoup* CloudMaker::random_particle_soup( const unsigned int& n, const Color& color, const Vector3& center, const double& extens ) {
    ParticleSoup* soup = new ParticleSoup();
    soup->randomize( n, color, center, extens );
    return soup;
}
