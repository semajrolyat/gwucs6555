/*-----------------------THE GEORGE WASHINGTON UNIVERSITY-----------------------
James Taylor                                                         jrt@gwu.edu

Matrix4 class

------------------------------------------------------------------------------*/

#ifndef _MATRIX4_H_
#define _MATRIX4_H_

#include <eigen3/Eigen/Dense>
#include <cs6555/Math/Vector3.h>
#include <cs6555/Math/Vector4.h>
#include <cs6555/Math/Matrix3.h>

//------------------------------------------------------------------------------

class Matrix4 {
public:
    // Constructors
    Matrix4( void );
    Matrix4( const Matrix4& );
    Matrix4( const double&, const double&, const double&, const double&,
             const double&, const double&, const double&, const double&,
             const double&, const double&, const double&, const double&,
             const double&, const double&, const double&, const double& );

private:
    // Adapter Copy Constructor (private)
    Matrix4( const Eigen::Matrix4d& );
public:
    // Destructor
    virtual ~Matrix4( void );

    // Debugging Functions
    void print( void );

    // Inplace Initialization Functions
    void zero( void );
    void identity( void );

    // Matrix Comparison Functions
    static bool equal( const Matrix4&, const Matrix4& );

    // Matrix Addition Functions
    static Matrix4 add( const Matrix4& A, const Matrix4& B );

    // Matrix Multiplication Functions
    static Matrix4 multiply( const Matrix4& A, const Matrix4& B );
    static Matrix4 multiply( const Matrix4& A, const double& c );
    static Vector4 multiply( const Matrix4& A, const Vector4& x );
    static Vector4 multiply( const Vector4& x, const Matrix4& A );

    // Matrix Transposition Functions
    static Matrix4 transpose( Matrix4& A );
    void transpose( void );

    // Matrix Inversion Functions
    static Matrix4 inverse( Matrix4& A );

    // Row & Column, Get & Set
    void row( const unsigned int& i, Vector4& v );
    Vector4 row( const unsigned int& i );
    void column( const unsigned int& i, Vector4& v );
    Vector4 column( const unsigned int& i );

    // Rotation Matrices
    static Matrix4 rotX( const double& );
    static Matrix4 rotY( const double& );
    static Matrix4 rotZ( const double& );
    Matrix3 rotation( void );

    // Translation Matrices
    static Matrix4 translationMatrix( const double& x, const double& y, const double& z );
    static Matrix4 translationMatrix( const Vector3& v );

    // Translation Get & Set
    void setTranslation( const double& x, const double& y, const double& z );
    void setTranslation( const Vector3& v );
    Vector3 translation( void );

    // Translation Modification
    void addTranslation( const double& x, const double& y, const double& z );
    void addTranslation( const Vector3& v );

    // Scaling Matrices
    static Matrix4 scalingMatrix( const double& x, const double& y, const double& z );

    // Interface Utilities
    double* arrayOpenGL( void );

    // Operators
    double& operator() (const unsigned int& row, const unsigned int& col );

    bool operator==( const Matrix4& );
    bool operator!=( const Matrix4& );

    Matrix4 operator+=( const Matrix4& );
    Matrix4 operator+( const Matrix4& );

    Matrix4 operator*=( const Matrix4& );
    Matrix4 operator*( const Matrix4& );

    Matrix4 operator*=( const double& c );
    Matrix4 operator*( const double& c );

    Vector4 operator*( const Vector4& x );

private:
    // Member data
    Eigen::Matrix4d A;
    double m_linearized[16];
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};

//------------------------------------------------------------------------------

// Out of class 'friendly' operators
Matrix4 operator*( const double& c, const Matrix4& A );
Vector4 operator*( Vector4& x, const Matrix4& A );

//------------------------------------------------------------------------------

#endif // _MATRIX4_H_
