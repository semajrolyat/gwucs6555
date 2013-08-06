/*-----------------------THE GEORGE WASHINGTON UNIVERSITY-----------------------
James Taylor                                                         jrt@gwu.edu

Vector3 class

------------------------------------------------------------------------------*/

#ifndef _VECTOR3_H_
#define _VECTOR3_H_

#include <eigen3/Eigen/Dense>

//------------------------------------------------------------------------------

class Vector3 {
public:
    // Constructors
    Vector3( void );
    Vector3( const Vector3& );
    Vector3( const double& x, const double& y, const double& z );
private:
    // Adapter based constructor
    Vector3( const Eigen::Vector3d& );
public:
    // Destructor
    virtual ~Vector3( void );

    // Debugging Functions
    void print( void );

    // Vector3 Comparison Functions
    static bool equal( const Vector3&, const Vector3& );

    // Vector3 Inversion Functions
    static Vector3 invert( const Vector3& );

    // Vector3 Addition Functions
    static Vector3 add( const Vector3&, const Vector3& );
    static Vector3 subtract( const Vector3&, const Vector3& );

    // Vector3 Multiplication Functions
    static Vector3 multiply( const Vector3&, const double& );
    static Vector3 multiply( const double&, const Vector3& );
    static double multiply( const Vector3&, const Vector3& );

    // Vector3 Inner Product Functions
    static double dot( const Vector3&, const Vector3& );

    // Vector3 Cross Product Functions
    static Vector3 cross( const Vector3&, const Vector3& );

    // Vector3 Magnitude
    double magnitude( void );

    // Vector3 Normalization Functions
    void normalize( void );

    // Component Get & Set
    double x( void ) const;
    double y( void ) const;
    double z( void ) const;
    void x( const double& value );
    void y( const double& value );
    void z( const double& value );

    // Vector3 Operators
    double& operator() ( const unsigned int& index );

    bool operator==( const Vector3& ) const;
    bool operator!=( const Vector3& ) const;

    Vector3 operator-( );

    Vector3 operator+=( const Vector3& );
    Vector3 operator+( const Vector3& ) const;

    Vector3 operator-=( const Vector3& );
    Vector3 operator-( const Vector3& ) const;

    double operator*( const Vector3& ) const;

    Vector3 operator*=( const double& );
    Vector3 operator*( const double& ) const;

    Vector3 operator/=( const double& );
    Vector3 operator/( const double& ) const;

private:
    // Member data
    Eigen::Vector3d data;
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};

//------------------------------------------------------------------------------

// Out of class Vector3 'friendly' operators
Vector3 operator*( const double&, const Vector3& );

//------------------------------------------------------------------------------

#endif // _VECTOR3_H_
