#include <cs6555/Math/Matrix3.h>
#include <assert.h>
#include <iostream>
#include <math.h>

//------------------------------------------------------------------------------
// Member Functions
//------------------------------------------------------------------------------
// Constructors
//------------------------------------------------------------------------------
/// Default Constructor
Matrix3::Matrix3( void ) {
    A = Eigen::Matrix3d::Identity();
}
//------------------------------------------------------------------------------
/// Copy Constructor
Matrix3::Matrix3( const Matrix3& A ) {
    this->A = Eigen::Matrix3d( A.A );
}
//------------------------------------------------------------------------------
/// Basis Vector Constructor
Matrix3::Matrix3( const Vector3& i, const Vector3& j, const Vector3& k ) {
    A(0,0) = i.x();   A(0,1) = j.x();   A(0,2) = k.x();
    A(1,0) = i.y();   A(1,1) = j.y();   A(1,2) = k.y();
    A(2,0) = i.z();   A(2,1) = j.z();   A(2,2) = k.z();
}
//------------------------------------------------------------------------------
/// Component Constructor
Matrix3::Matrix3( const double& a00, const double& a01, const double& a02,
         const double& a10, const double& a11, const double& a12,
         const double& a20, const double& a21, const double& a22 ) {
    A(0,0) = a00;   A(0,1) = a01;   A(0,2) = a02;
    A(1,0) = a10;   A(1,1) = a11;   A(1,2) = a12;
    A(2,0) = a20;   A(2,1) = a21;   A(2,2) = a22;
}
//------------------------------------------------------------------------------
/// Adapter Copy Constructor (private)
Matrix3::Matrix3( const Eigen::Matrix3d& A ) {
    this->A = Eigen::Matrix3d( A );
}
//------------------------------------------------------------------------------
// Destructor
//------------------------------------------------------------------------------
/// Destructor
Matrix3::~Matrix3( void ) {

}
//------------------------------------------------------------------------------
// Debugging Functions
//------------------------------------------------------------------------------
/// Print to the console for debugging
void Matrix3::print( void ) {
    std::cout << A;
}
//------------------------------------------------------------------------------
// Inplace Initialization
//------------------------------------------------------------------------------
/// Inplace Initialize As Zero Matrix
void Matrix3::zero( void ) {
    A = Eigen::Matrix3d::Zero();
}
//------------------------------------------------------------------------------
/// Inplace Initialize As Identity Matrix
void Matrix3::identity( void ) {
    A = Eigen::Matrix3d::Identity();
}
//------------------------------------------------------------------------------
// Matrix Comparison
//------------------------------------------------------------------------------
/// Matrix-Matrix Equality Comparison Function
bool Matrix3::equal( const Matrix3& A, const Matrix3& B ) {
    // some error in fp math so use epsilon to ensure reasonable comparison
    float EPSILON = 1e-5;
    for( int i = 0; i < 3; i++ )
        for( int j = 0; j < 3; j++ )
            if( A.A(i,j) + EPSILON < B.A(i,j) || A.A(i,j) - EPSILON > B.A(i,j) )
                return false;
    return true;
}
//------------------------------------------------------------------------------
// Matrix Addition
//------------------------------------------------------------------------------
/// Matrix-Matrix Addition Function
Matrix3 Matrix3::add( const Matrix3& A, const Matrix3& B ) {
    return Matrix3( A.A + B.A );
}
//------------------------------------------------------------------------------
// Matrix Multiplication
//------------------------------------------------------------------------------
/// Matrix-Matrix Multiplication Function
/// Note: Not Commutative -> Matrix order critical
Matrix3 Matrix3::multiply( const Matrix3& A, const Matrix3& B ) {
    return Matrix3( A.A * B.A );
}
//------------------------------------------------------------------------------
/// Matrix-Scalar Multiplication Function
Matrix3 Matrix3::multiply( const Matrix3& A, const double& c ) {
    return Matrix3( A.A * c );
}
//------------------------------------------------------------------------------
/// Matrix-Vector Multiplication Function
Vector3 Matrix3::multiply( const Matrix3& A, const Vector3& x ) {
    Eigen::Vector3d v = Eigen::Vector3d( x.x(), x.y(), x.z() );
    Eigen::Vector3d b = A.A * v;
    return Vector3( b(0), b(1), b(2) );
}
//------------------------------------------------------------------------------
/// Vector-Matrix Multiplication Function
Vector3 Matrix3::multiply( const Vector3& x, const Matrix3& A ) {
    Eigen::Vector3d v = Eigen::Vector3d( x.x(), x.y(), x.z() );
    Eigen::Vector3d b = v.transpose() * A.A;
    return Vector3( b(0), b(1), b(2) );
}
//------------------------------------------------------------------------------
// Matrix Transposition
//------------------------------------------------------------------------------
/// Matrix Transpose Function
Matrix3 Matrix3::transpose( Matrix3& A ) {
    return Matrix3( A.A.transpose() );
}
//------------------------------------------------------------------------------
/// Inplace Matrix Transpose Function
void Matrix3::transpose( void ) {
    this->A = this->A.transpose();
}
//------------------------------------------------------------------------------
// Matrix Inversion
//------------------------------------------------------------------------------
/// Matrix Inverse Function
Matrix3 Matrix3::inverse( Matrix3& A ) {
    return Matrix3( A.A.inverse() );
}
//------------------------------------------------------------------------------
// Row & Column, Get & Set
//------------------------------------------------------------------------------
/// Set a Row
void Matrix3::row( const unsigned int& i, Vector3& v ) {
    assert( i < 3 );
    A( i, v(0) );
    A( i, v(1) );
    A( i, v(2) );
}
//------------------------------------------------------------------------------
/// Get a Row
Vector3 Matrix3::row( const unsigned int& i ) {
    assert( i < 3 );
    return Vector3( A( i, 0 ), A( i, 1 ), A( i, 2 ) );
}
//------------------------------------------------------------------------------
/// Set a Column
void Matrix3::column( const unsigned int& i, Vector3& v ) {
    assert( i < 3 );
    A( v(0), i );
    A( v(1), i );
    A( v(2), i );
}
//------------------------------------------------------------------------------
/// Get a Column
Vector3 Matrix3::column( const unsigned int& i ) {
    assert( i < 3 );
    return Vector3( A( 0, i ), A( 1, i ), A( 2, i ) );
}
//------------------------------------------------------------------------------
// Rotation Matrices
//------------------------------------------------------------------------------
/// Rotation Matrix Around X
Matrix3 Matrix3::rotX( const double& theta ) {
    Matrix3 A;
    A.identity();
    A(1,1) = cos(theta);
    A(1,2) = -sin(theta);
    A(2,1) = sin(theta);
    A(2,2) = cos(theta);
    return A;
}
//------------------------------------------------------------------------------
/// Rotation Matrix Around Y
Matrix3 Matrix3::rotY( const double& theta ) {
    Matrix3 A;
    A.identity();
    A(0,0) = cos(theta);
    A(0,2) = sin(theta);
    A(2,0) = -sin(theta);
    A(2,2) = cos(theta);
    return A;
}
//------------------------------------------------------------------------------
/// Rotation Matrix Around Z
Matrix3 Matrix3::rotZ( const double& theta ) {
    Matrix3 A;
    A.identity();
    A(0,0) = cos(theta);
    A(0,1) = -sin(theta);
    A(1,0) = sin(theta);
    A(1,1) = cos(theta);
    return A;
}

//------------------------------------------------------------------------------
// Utilities
//------------------------------------------------------------------------------
/// Skew Symmetric Matrix
Matrix3 Matrix3::skew( const Vector3& v) {
    Matrix3 A;
    A(0,0) =  0.0;      A(0,1) = -v.z();    A(0,2) =  v.y();
    A(0,0) =  v.z();    A(0,1) =  0.0;      A(0,2) = -v.x();
    A(0,0) = -v.y();    A(0,1) =  v.x();    A(0,2) =  0.0;
    return A;
}

//------------------------------------------------------------------------------
// Matrix3 Operators
//------------------------------------------------------------------------------
/// Parenthetical Access to Member Data
double& Matrix3::operator() ( const unsigned int& row, const unsigned int& col ) {
    assert( row < 3 && col < 3 );
    return A( row, col );
}
//------------------------------------------------------------------------------
/// Matrix-Matrix Equality Operator
bool Matrix3::operator==( const Matrix3& B ) {
    return equal( *this, B );
}
//------------------------------------------------------------------------------
/// Matrix-Matrix Inequality Operator
bool Matrix3::operator!=( const Matrix3& B ) {
    return !equal( *this, B );
}
//------------------------------------------------------------------------------
/// Inplace Matrix-Matrix Addition Operator
Matrix3 Matrix3::operator+=( const Matrix3& B ) {
    Matrix3 C = add( *this, B );
    A = C.A;
    return *this;
}
//------------------------------------------------------------------------------
/// Matrix-Matrix Addition Operator
Matrix3 Matrix3::operator+( const Matrix3& B ) const {
    Matrix3 C = *this;
    C += B;
    return C;
}
//------------------------------------------------------------------------------
/// Inplace Matrix-Matrix Multiplication Operator
Matrix3 Matrix3::operator*=( const Matrix3& B ) {
    Matrix3 C = multiply( *this, B );
    A = C.A;
    return *this;
}
//------------------------------------------------------------------------------
/// Matrix-Matrix Multiplication Operator
Matrix3 Matrix3::operator*( const Matrix3& B ) const {
    Matrix3 C = *this;
    C *= B;
    return C;
}
//------------------------------------------------------------------------------
/// Inplace Matrix-Scalar Multiplication Operator
Matrix3 Matrix3::operator*=( const double& c ) {
    Matrix3 C = multiply( *this, c );
    A = C.A;
    return *this;
}
//------------------------------------------------------------------------------
/// Matrix-Scalar Multiplication Operator
Matrix3 Matrix3::operator*( const double& c ) const {
    Matrix3 C = *this;
    C *= c;
    return C;
}
//------------------------------------------------------------------------------
/// Matrix-Vector Multiplication Operator
Vector3 Matrix3::operator*( const Vector3& x ) const {
    return multiply( *this, x );
}
//------------------------------------------------------------------------------
// Out of class Matrix3 'friendly' operators
//------------------------------------------------------------------------------
/// Scalar-Matrix Multiplication Operator
Matrix3 operator*( const double& c, const Matrix3& A ) {
    return Matrix3::multiply( A, c );
}
//------------------------------------------------------------------------------
/// Vector-Matrix Multiplication Operator
Vector3 operator*( const Vector3& x, const Matrix3& A ) {
    return Matrix3::multiply( x, A );
}
//------------------------------------------------------------------------------
