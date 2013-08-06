#include <cs6555/Math/CubicSpline.h>

#include <math.h>
//#include <eigen3/Eigen/Dense>

#include <cs6555/Math/LineSegment.h>


// Note: Everything is static for this class so don't need ctor/dtor
//------------------------------------------------------------------------------
// Constructors
//------------------------------------------------------------------------------
/// Default Constructor
CubicSpline::CubicSpline( void ) {

}
//------------------------------------------------------------------------------
// Destructor
//------------------------------------------------------------------------------
/// Destructor
CubicSpline::~CubicSpline( void ) {

}
//------------------------------------------------------------------------------
//
//------------------------------------------------------------------------------
/// Returns the Catmull-Rom basis matrix M
Eigen::MatrixXd CubicSpline::basisCatmullRom( void ) {
    double a = 0.5;
    Eigen::MatrixXd M(4,4);

    M(0,0) = -1.0;      M(0,1) =  3.0;      M(0,2) = -3.0;      M(0,3) =  1.0;
    M(1,0) =  2.0;      M(1,1) = -5.0;      M(1,2) =  4.0;      M(1,3) = -1.0;
    M(2,0) = -1.0;      M(2,1) =  0.0;      M(2,2) =  1.0;      M(2,3) =  0.0;
    M(3,0) =  0.0;      M(3,1) =  2.0;      M(3,2) =  0.0;      M(3,3) =  0.0;

    return a * M;
}
//------------------------------------------------------------------------------
/// Returns the Uniform Non-Rational B-Spline basis matrix M
Eigen::MatrixXd CubicSpline::basisUniformNonrationalBSpline( void ) {
    double a = 1.0/6.0;
    Eigen::MatrixXd M(4,4);

    M(0,0) = -1.0;      M(0,1) =  3.0;      M(0,2) = -3.0;      M(0,3) =  1.0;
    M(1,0) =  3.0;      M(1,1) = -6.0;      M(1,2) =  3.0;      M(1,3) =  0.0;
    M(2,0) = -3.0;      M(2,1) =  0.0;      M(2,2) =  3.0;      M(2,3) =  0.0;
    M(3,0) =  1.0;      M(3,1) =  4.0;      M(3,2) =  1.0;      M(3,3) =  0.0;

    return a * M;
}
//------------------------------------------------------------------------------
/// Returns the Bezier basis matrix M
Eigen::MatrixXd CubicSpline::basisBezier( void ) {
    Eigen::MatrixXd M(4,4);

    M(0,0) = -1.0;      M(0,1) =  3.0;      M(0,2) = -3.0;      M(0,3) =  1.0;
    M(1,0) =  3.0;      M(1,1) = -6.0;      M(1,2) =  3.0;      M(1,3) =  0.0;
    M(2,0) = -3.0;      M(2,1) =  3.0;      M(2,2) =  0.0;      M(2,3) =  0.0;
    M(3,0) =  1.0;      M(3,1) =  0.0;      M(3,2) =  0.0;      M(3,3) =  0.0;

    return M;
}
//------------------------------------------------------------------------------
/// Returns the Hermite basis matrix M
Eigen::MatrixXd CubicSpline::basisHermite( void ) {
    Eigen::MatrixXd M(4,4);

    M(0,0) =  2.0;      M(0,1) = -2.0;      M(0,2) =  1.0;      M(0,3) =  1.0;
    M(1,0) = -3.0;      M(1,1) =  3.0;      M(1,2) = -2.0;      M(1,3) = -1.0;
    M(2,0) =  0.0;      M(2,1) =  0.0;      M(2,2) =  1.0;      M(2,3) =  0.0;
    M(3,0) =  1.0;      M(3,1) =  0.0;      M(3,2) =  0.0;      M(3,3) =  0.0;

    return M;
}

//------------------------------------------------------------------------------
/// Returns the 3-space position vector at point u along spline Q
/// given blended matrix C and point u
/// where u E [0,1]
Vector3 CubicSpline::position( const Eigen::MatrixXd& C, const double& u ) {
    double u2 = u * u;
    double u3 = u2 * u;

    Eigen::MatrixXd U(1,4);
    U(0,0) = u3;
    U(0,1) = u2;
    U(0,2) = u;
    U(0,3) = 1;

    Eigen::MatrixXd Q = U * C;

    return Vector3( Q(0,0), Q(0,1), Q(0,2) );
}

//------------------------------------------------------------------------------
/// Returns the 3-space directional tangent vector at point u along spline Q
/// given blended matrix C and point u
/// where u E [0,1]
Vector3 CubicSpline::tangent( const Eigen::MatrixXd& C, const double& u ) {
    double u2 = u * u;

    return Vector3( 3 * C(0,0) * u2 + 2 * C(1,0) * u + C(2,0),
                    3 * C(0,1) * u2 + 2 * C(1,1) * u + C(2,1),
                    3 * C(0,2) * u2 + 2 * C(1,2) * u + C(2,2) );
}
//------------------------------------------------------------------------------
/// Returns the 3-space directional normal vector at point u along spline Q
/// given blended matrix C and point u
/// where u E [0,1]
/// Note: this implementation is incorrect at the moment
/// TODO: derive correct 2nd derivative
Vector3 CubicSpline::normal( const Eigen::MatrixXd& C, const double& u ) {
    return Vector3( 6 * C(0,0) * u + 2 * C(1,0),
                    6 * C(0,1) * u + 2 * C(1,1),
                    6 * C(0,2) * u + 2 * C(1,2) );
}

//------------------------------------------------------------------------------
/// Returns the blended matrix C
/// given the basis matrix M, the ordered set of control points defining spline Q
/// and the index of the third control point of the desired component spline qid.
/// where C is the concatenation of basis matrix M and geometry matrix G, e.g. C = M * G,
/// and qid is an integer on the interval [2,n-1] | n = number of control points
Eigen::MatrixXd CubicSpline::blend( const Eigen::MatrixXd& M, ControlPointList ctlpts, const unsigned int& qid ) {
    assert( qid >= 2 && qid < ctlpts.size()-1 );

    Eigen::MatrixXd G = geometry_matrix( ctlpts, qid );

    return M * G;
}
//------------------------------------------------------------------------------
Eigen::MatrixXd CubicSpline::blend( const Eigen::MatrixXd& M, ControlPointPtrList ctlptptrs, const unsigned int& qid ) {
    assert( qid >= 2 && qid < ctlptptrs.size()-1 );

    Eigen::MatrixXd G = geometry_matrix( ctlptptrs, qid );

    return M * G;
}

//------------------------------------------------------------------------------
/// Returns the 4 x 3 geometry matrix (G)
/// given the ordered set of control points and the index of the third control
/// point of the desired component spline qid.
/// where qid is an integer on the interval [2,n-1] | n = number of control points
Eigen::MatrixXd CubicSpline::geometry_matrix( ControlPointList ctlpts, const unsigned int& qid ) {
    assert( qid >= 2 && qid < ctlpts.size() - 1 );

    ControlPointList::iterator it2 = ctlpts.begin() + qid;
    ControlPointList::iterator it3 = it2+1;
    ControlPointList::iterator it1 = it2-1;
    ControlPointList::iterator it0 = it2-2;

    Eigen::MatrixXd G(4,3);
    G(0,0) = it0->position.x(); G(0,1) = it0->position.y(); G(0,2) = it0->position.z();
    G(1,0) = it1->position.x(); G(1,1) = it1->position.y(); G(1,2) = it1->position.z();
    G(2,0) = it2->position.x(); G(2,1) = it2->position.y(); G(2,2) = it2->position.z();
    G(3,0) = it3->position.x(); G(3,1) = it3->position.y(); G(3,2) = it3->position.z();

    return G;
}

//------------------------------------------------------------------------------
Eigen::MatrixXd CubicSpline::geometry_matrix( ControlPointPtrList ctlptptrs, const unsigned int& qid ) {
    assert( qid >= 2 && qid < ctlptptrs.size() - 1 );

    ControlPointPtrList::iterator it2 = ctlptptrs.begin() + qid;
    ControlPointPtrList::iterator it3 = it2+1;
    ControlPointPtrList::iterator it1 = it2-1;
    ControlPointPtrList::iterator it0 = it2-2;

    Eigen::MatrixXd G(4,3);
    G(0,0) = (*it0)->position.x(); G(0,1) = (*it0)->position.y(); G(0,2) = (*it0)->position.z();
    G(1,0) = (*it1)->position.x(); G(1,1) = (*it1)->position.y(); G(1,2) = (*it1)->position.z();
    G(2,0) = (*it2)->position.x(); G(2,1) = (*it2)->position.y(); G(2,2) = (*it2)->position.z();
    G(3,0) = (*it3)->position.x(); G(3,1) = (*it3)->position.y(); G(3,2) = (*it3)->position.z();

    return G;
}

//------------------------------------------------------------------------------
/// Returns the linearly approximated arclength between two points on the spline
/// given the blended matrix (C), e.g. basis (M) * geometry (G), and parameter u
/// where u E [0,1]
double CubicSpline::linearly_approximate_arclength( const Eigen::MatrixXd& C, const double& u0, const double& u1 ) {
    assert( u0 >= 0.0 && u0 < u1 && u1 <= 1.0 );

    Vector3 v0 = position( C, u0 );
    Vector3 v1 = position( C, u1 );

    return LineSegment::length( v0, v1 );
}

//------------------------------------------------------------------------------
/// Returns the interpolated arclength along a spline
/// given the blended matrix (C), the start point (u0), the end point (u1)
/// and the step size (du)
/// where u0, u1 E [0,1] & u1 > u0 & du < u1 - u0
/// Note: du is tunable to minimize either accuracy or computation time
double CubicSpline::linearly_interpolate_arclength( const Eigen::MatrixXd& C, const double& u0, const double& u1, const double& du ) {
    assert( u0 >= 0.0 && u0 < u1 && u1 <= 1.0 );
    assert( du < u1 - u0 );

    std::vector<Vector3> points;
    double s = 0.0;
    Vector3 vi, vi1;

    // query the points and store for processing
    for( double u = u0; u < u1; u += du ) {
        points.push_back( position( C, u ) );
    }
    // step size may have not been a factor of u1-u0 so pick up odd case
    points.push_back( position( C, u1 ) );

    // process each pair of table entries and append length
    for( std::vector<Vector3>::iterator it = points.begin(); it != points.end(); it++ ) {
        if( it == points.begin() ) {
            vi = *it;
        } else {
            vi1 = *it;
            s += LineSegment::length( vi1, vi );
            vi = *it;
        }
    }

    // return the interpolated arclength s
    return s;
}

//------------------------------------------------------------------------------
/// Returns the interpolated arclength along a spline
/// given the basis and the ordered set of control points
double CubicSpline::linearly_interpolate_arclength( const ECubicSplineBasis& basis, ControlPointList ctlpts ) {
    double s = 0.0;

    Eigen::MatrixXd M;

    switch( basis ) {
    case CUBIC_SPLINE_B:
        M = basisUniformNonrationalBSpline();
        break;
    case CUBIC_SPLINE_CATMULLROM:
    default:
        M = basisCatmullRom();
        break;
    }

    unsigned int n = ctlpts.size();
    for( unsigned int i = 2; i < n - 1; i++ ) {
        Eigen::MatrixXd C = blend( M, ctlpts, i );
        s += linearly_interpolate_arclength( C, 0.0, 1.0, 0.05 );
    }
    return s;
}

//------------------------------------------------------------------------------
double CubicSpline::linearly_interpolate_arclength( const ECubicSplineBasis& basis, ControlPointPtrList ctlptptrs ) {
    double s = 0.0;

    Eigen::MatrixXd M;

    switch( basis ) {
    case CUBIC_SPLINE_B:
        M = basisUniformNonrationalBSpline();
        break;
    case CUBIC_SPLINE_CATMULLROM:
    default:
        M = basisCatmullRom();
        break;
    }

    unsigned int n = ctlptptrs.size();
    for( unsigned int i = 2; i < n - 1; i++ ) {
        Eigen::MatrixXd C = blend( M, ctlptptrs, i );
        s += linearly_interpolate_arclength( C, 0.0, 1.0, 0.05 );
    }
    return s;
}

//------------------------------------------------------------------------------
/// Returns a map of the control point identifiers qid, the arclength s up to
/// that point and the change in arclength ds over that component spline
/// given the basis and the ordered set of control points
std::vector<SdS> CubicSpline::map_spline( const ECubicSplineBasis& basis, ControlPointList ctlpts ) {
    double s = 0.0;

    std::vector<SdS> result;

    Eigen::MatrixXd M;

    switch( basis ) {
    case CUBIC_SPLINE_B:
        M = basisUniformNonrationalBSpline();
        break;
    case CUBIC_SPLINE_CATMULLROM:
    default:
        M = basisCatmullRom();
        break;
    }

    unsigned int n = ctlpts.size();
    SdS sds;

    // initial points
    sds.first = 0.0;
    sds.second = 0.0;
    result.push_back( sds );  // This is an end point
    result.push_back( sds );  // This is the first median point

    for( unsigned int i = 2; i < n-1; i++ ) {
        Eigen::MatrixXd C = blend( M, ctlpts, i );
        double ds = linearly_interpolate_arclength( C, 0.0, 1.0, 0.05 );
        s += ds;
        sds.first = s;
        sds.second = ds;
        result.push_back( sds );
    }

    // terminal point
    sds.first = s;
    sds.second = 0.0;
    result.push_back( sds );  // This is an end point

    return result;
}

//------------------------------------------------------------------------------
std::vector<SdS> CubicSpline::map_spline( const ECubicSplineBasis& basis, ControlPointPtrList ctlptptrs ) {
    double s = 0.0;

    std::vector<SdS> result;

    Eigen::MatrixXd M;

    switch( basis ) {
    case CUBIC_SPLINE_B:
        M = basisUniformNonrationalBSpline();
        break;
    case CUBIC_SPLINE_CATMULLROM:
    default:
        M = basisCatmullRom();
        break;
    }

    unsigned int n = ctlptptrs.size();
    SdS sds;

    // initial points
    sds.first = 0.0;
    sds.second = 0.0;
    result.push_back( sds );  // This is an end point
    result.push_back( sds );  // This is the first median point

    for( unsigned int i = 2; i < n-1; i++ ) {
        Eigen::MatrixXd C = blend( M, ctlptptrs, i );
        double ds = linearly_interpolate_arclength( C, 0.0, 1.0, 0.05 );
        s += ds;
        sds.first = s;
        sds.second = ds;
        result.push_back( sds );
    }

    // terminal point
    sds.first = s;
    sds.second = 0.0;
    result.push_back( sds );  // This is an end point

    return result;
}
/*
//------------------------------------------------------------------------------
/// Parametric Tangent 2nd Derivative Vector
/// s : arclength
Vector3 CubicSpline::d2tangent( const double& t,
                                   const double& ax, const double& bx,
                                   const double& ay, const double& by,
                                   const double& az, const double& bz ) {

    double x = 6 * ax * t + 2 * bx;
    double y = 6 * ay * t + 2 * by;
    double z = 6 * az * t + 2 * bz;

    return Vector3( x, y, z );
}
//------------------------------------------------------------------------------
/// Orientation Matrix
/// Where 1st column is x axis, 2nd column is y axis, 3rd column is z axis
/// Right Handed Coordinate System
/// s : arclength
Matrix3 CubicSpline::orientation( const double& s ) {
    // Vector3 w = tangent(t,...)
    // Vector3 u = Vector3::cross( w, d2tangent(t,...) )
    // Vector3 v = Vector3::cross( w, u )
}
//------------------------------------------------------------------------------
*/
