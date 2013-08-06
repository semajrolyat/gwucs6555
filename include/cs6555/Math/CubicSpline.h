/*-----------------------THE GEORGE WASHINGTON UNIVERSITY-----------------------
James Taylor                                                         jrt@gwu.edu

CubicSpline class
------------------------------------------------------------------------------*/

#ifndef _CUBICSPLINE_H_
#define _CUBICSPLINE_H_

#include <cs6555/Math/Matrix4.h>
#include <cs6555/Math/Matrix3.h>
#include <vector>
#include <cs6555/Math/Vector3.h>
#include <cs6555/Pose.h>

#include <cs6555/ControlPoint.h>
#include <eigen3/Eigen/Dense>

//#include <map>

//------------------------------------------------------------------------------

typedef enum {
    CUBIC_SPLINE_BEZIER,
    CUBIC_SPLINE_B,
    CUBIC_SPLINE_CATMULLROM,
    CUBIC_SPLINE_HERMITE
} ECubicSplineBasis;

//------------------------------------------------------------------------------

typedef std::pair<double,double> SdS;

typedef std::vector<ControlPoint> ControlPointList;
typedef std::vector<ControlPoint*> ControlPointPtrList;

//------------------------------------------------------------------------------

class CubicSpline {
public:
    CubicSpline( void );
    ~CubicSpline( void );

    static Eigen::MatrixXd basisCatmullRom( void );
    static Eigen::MatrixXd basisUniformNonrationalBSpline( void );
    static Eigen::MatrixXd basisBezier( void );
    static Eigen::MatrixXd basisHermite( void );

    static Vector3 position( const Eigen::MatrixXd& C, const double& u );
    static Vector3 tangent( const Eigen::MatrixXd& C, const double& u );
    static Vector3 normal( const Eigen::MatrixXd& C, const double& u );

    static Eigen::MatrixXd blend( const Eigen::MatrixXd& M, ControlPointList ctlpts, const unsigned int& qid );
    static Eigen::MatrixXd geometry_matrix( ControlPointList ctlpts, const unsigned int& qid );

    static Eigen::MatrixXd blend( const Eigen::MatrixXd& M, ControlPointPtrList ctlptptrs, const unsigned int& qid = 2 );
    static Eigen::MatrixXd geometry_matrix( ControlPointPtrList ctlptptrs, const unsigned int& qid = 2 );

    static double linearly_approximate_arclength( const Eigen::MatrixXd& C, const double& u0 = 0.0, const double& u1 = 1.0 );
    static double linearly_interpolate_arclength( const Eigen::MatrixXd& C, const double& u0 = 0.0, const double& u1 = 1.0, const double& du = 0.1 );
    static double linearly_interpolate_arclength( const ECubicSplineBasis& basis, ControlPointList ctlpts );

    static double linearly_interpolate_arclength( const ECubicSplineBasis& basis, ControlPointPtrList ctlptptrs );

    // map at each control point
    //  ds of the section preceding it
    //  s of the overall spline

    static std::vector<SdS> map_spline( const ECubicSplineBasis& basis, ControlPointList ctlpts );
    static std::vector<SdS> map_spline( const ECubicSplineBasis& basis, ControlPointPtrList ctlptptrs );

};

//------------------------------------------------------------------------------

#endif // _CUBICSPLINE_H_
