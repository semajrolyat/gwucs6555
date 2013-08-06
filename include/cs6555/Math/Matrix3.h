/*-----------------------THE GEORGE WASHINGTON UNIVERSITY-----------------------
James Taylor                                                         jrt@gwu.edu

Matrix3 class

------------------------------------------------------------------------------*/

#ifndef _MATRIX3_H_
#define _MATRIX3_H_

#include <eigen3/Eigen/Dense>
#include <cs6555/Math/Vector3.h>

//------------------------------------------------------------------------------

class Matrix3 {
public:
    // Constructors
    Matrix3( void );
    Matrix3( const Matrix3& );
    Matrix3( const Vector3& i, const Vector3& j, const Vector3& k );
    Matrix3( const double&, const double&, const double&,
             const double&, const double&, const double&,
             const double&, const double&, const double& );
private:
    // Adapter Copy Constructor (private)
    Matrix3( const Eigen::Matrix3d& );
public:
    // Destructor
    virtual ~Matrix3( void );

    // Debugging Functions
    void print( void );

    // Inplace Initialization Functions
    void zero( void );
    void identity( void );

    // Matrix Comparison Functions
    static bool equal( const Matrix3&, const Matrix3& );

    // Matrix Addition Functions
    static Matrix3 add( const Matrix3& A, const Matrix3& B );

    // Matrix Multiplication Functions
    static Matrix3 multiply( const Matrix3& A, const Matrix3& B );
    static Matrix3 multiply( const Matrix3& A, const double& c );
    static Vector3 multiply( const Matrix3& A, const Vector3& x );
    static Vector3 multiply( const Vector3& x, const Matrix3& A );

    // Matrix Transposition Functions
    static Matrix3 transpose( Matrix3& A );
    void transpose( void );

    // Matrix Inversion Functions
    static Matrix3 inverse( Matrix3& A );

    // Row & Column, Get & Set
    void row( const unsigned int& i, Vector3& v );
    Vector3 row( const unsigned int& i );
    void column( const unsigned int& i, Vector3& v );
    Vector3 column( const unsigned int& i );

    // Rotation Matrices
    static Matrix3 rotX( const double& );
    static Matrix3 rotY( const double& );
    static Matrix3 rotZ( const double& );

    // Utilities
    static Matrix3 skew( const Vector3& v);

    // Operators
    double& operator() (const unsigned int& row, const unsigned int& col );

    bool operator==( const Matrix3& );
    bool operator!=( const Matrix3& );

    Matrix3 operator+=( const Matrix3& );
    Matrix3 operator+( const Matrix3& ) const;

    Matrix3 operator*=( const Matrix3& );
    Matrix3 operator*( const Matrix3& ) const;

    Matrix3 operator*=( const double& c );
    Matrix3 operator*( const double& c ) const;

    Vector3 operator*( const Vector3& x ) const;

private:
    // Member data
    Eigen::Matrix3d A;
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};

//------------------------------------------------------------------------------

// Out of class 'friendly' operators
Matrix3 operator*( const double& c, const Matrix3& A );
Vector3 operator*( const Vector3& x, const Matrix3& A );

//------------------------------------------------------------------------------

#endif // _MATRIX3_H_
