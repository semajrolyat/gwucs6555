#ifndef _MATH_H_
#define _MATH_H_

namespace Math {

static inline double bilinear_interpolate( const double& x, const double& y, const double& x0, const double& y0, const double& x1, const double& y1, const double& f00, const double& f10, const double& f01, const double& f11 ) {
    double p1 = f00/((x1 - x0)*(y1 - y0)) * (x1 - x) * (y1 - y);
    double p2 = f10/((x1 - x0)*(y1 - y0)) * (x - x0) * (y1 - y);
    double p3 = f01/((x1 - x0)*(y1 - y0)) * (x1 - x) * (y - y0);
    double p4 = f11/((x1 - x0)*(y1 - y0)) * (x - x0) * (y - y0);
    return p1 + p2 + p3 + p4;
}

static inline double trilinear_interpolate( const double& x, const double& y, const double& z, const double& x0, const double& y0, const double& z0, const double& x1, const double& y1, const double& z1, const double& f000, const double& f100, const double& f001, const double& f101, const double& f010, const double& f110, const double& f011, const double& f111 ) {
    double xd = (x - x0) / (x1 - x0);
    double yd = (y - y0) / (y1 - y0);
    double zd = (z - z0) / (z1 - z0);

    double c00 = f000 * (1 - xd) + f100 * xd;
    double c10 = f010 * (1 - xd) + f110 * xd;
    double c01 = f001 * (1 - xd) + f101 * xd;
    double c11 = f011 * (1 - xd) + f111 * xd;

    double c0 = c00 * (1 - yd) + c10 * yd;
    double c1 = c01 * (1 - yd) + c11 * yd;

    return c0 * (1 - zd) + c1 * zd;
}

};  // namespace

#endif // _MATH_H_
