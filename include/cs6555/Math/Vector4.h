/*-----------------------THE GEORGE WASHINGTON UNIVERSITY-----------------------
James Taylor                                                         jrt@gwu.edu

Vector4 class

------------------------------------------------------------------------------*/

#ifndef _VECTOR4_H_
#define _VECTOR4_H_

#include <eigen3/Eigen/Dense>
#include <cs6555/Math/Vector3.h>

//------------------------------------------------------------------------------

class Vector4 {
public:
    // Constructors
    Vector4( void );
    Vector4( const Vector4& );
    Vector4( const double& x, const double& y, const double& z );
    Vector4( const double& x, const double& y, const double& z, const double& w );
    Vector4( Vector3 );
private:
    // Adapter based constructor
    Vector4( const Eigen::Vector4d& );
public:
    // Destructor
    virtual ~Vector4( void );

    // Debugging Functions
    void print( void );

    // Vector4 Comparison Functions
    static bool equal( const Vector4&, const Vector4& );

    // Vector4 Inversion Functions
    static Vector4 invert( const Vector4& );

    // Vector4 Addition Functions
    static Vector4 add( const Vector4&, const Vector4& );
    static Vector4 subtract( const Vector4&, const Vector4& );

    // Vector4 Multiplication Functions
    static Vector4 multiply( const Vector4&, const double& );
    static Vector4 multiply( const double&, const Vector4& );
    static double multiply( const Vector4&, const Vector4& );

    // Vector4 Inner Product Functions
    static double dot( const Vector4&, const Vector4& );

    // Vector4 Normalization Functions
    void normalize( void );

    // Component Get & Set
    double x( void ) const;
    double y( void ) const;
    double z( void ) const;
    double w( void ) const;
    void x( const double& value );
    void y( const double& value );
    void z( const double& value );
    void w( const double& value );

    // Get as Class
    Vector3 vector3( void );

    // Vector4 Operators
    double& operator() ( const unsigned int& index );

    bool operator==( const Vector4& );
    bool operator!=( const Vector4& );

    Vector4 operator+=( const Vector4& );
    Vector4 operator+( const Vector4& );

    double operator*( const Vector4& );

    Vector4 operator*=( const double& );
    Vector4 operator*( const double& );

private:
    // Member data
    Eigen::Vector4d data;
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};

//------------------------------------------------------------------------------

// Out of class Vector4 'friendly' operators
Vector4 operator*( const double&, const Vector4& );

//------------------------------------------------------------------------------

#endif // _VECTOR4_H_
