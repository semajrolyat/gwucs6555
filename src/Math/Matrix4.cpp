#include <cs6555/Math/Matrix4.h>
#include <assert.h>
#include <iostream>

//------------------------------------------------------------------------------
// Member Functions
//------------------------------------------------------------------------------
// Constructors
//------------------------------------------------------------------------------
/// Default Constructor
Matrix4::Matrix4( void ) {
    A = Eigen::Matrix4d::Identity();
}
//------------------------------------------------------------------------------
/// Copy Constructor
Matrix4::Matrix4( const Matrix4& A ) {
    this->A = Eigen::Matrix4d( A.A );
}
//------------------------------------------------------------------------------
/// Component Constructor
Matrix4::Matrix4( const double& a00, const double& a01, const double& a02, const double& a03,
                  const double& a10, const double& a11, const double& a12, const double& a13,
                  const double& a20, const double& a21, const double& a22, const double& a23,
                  const double& a30, const double& a31, const double& a32, const double& a33 ) {
    A(0,0) = a00;   A(0,1) = a01;   A(0,2) = a02;   A(0,3) = a03;
    A(1,0) = a10;   A(1,1) = a11;   A(1,2) = a12;   A(1,3) = a13;
    A(2,0) = a20;   A(2,1) = a21;   A(2,2) = a22;   A(2,3) = a23;
    A(3,0) = a30;   A(3,1) = a31;   A(3,2) = a32;   A(3,3) = a33;
}
//------------------------------------------------------------------------------
/// Adapter Copy Constructor (private)
Matrix4::Matrix4( const Eigen::Matrix4d& A ) {
    this->A = Eigen::Matrix4d( A );
}
//------------------------------------------------------------------------------
// Destructor
//------------------------------------------------------------------------------
/// Destructor
Matrix4::~Matrix4( void ) {

}
//------------------------------------------------------------------------------
// Debugging Functions
//------------------------------------------------------------------------------
/// Print to the console for debugging
void Matrix4::print( void ) {
    std::cout << A;
}
//------------------------------------------------------------------------------
// Inplace Initialization
//------------------------------------------------------------------------------
/// Inplace Initialize As Zero Matrix
void Matrix4::zero( void ) {
    A = Eigen::Matrix4d::Zero();
}
//------------------------------------------------------------------------------
/// Inplace Initialize As Identity Matrix
void Matrix4::identity( void ) {
    A = Eigen::Matrix4d::Identity();
}
//------------------------------------------------------------------------------
// Matrix Comparison
//------------------------------------------------------------------------------
/// Matrix-Matrix Equality Comparison Function
bool Matrix4::equal( const Matrix4& A, const Matrix4& B ) {
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
Matrix4 Matrix4::add( const Matrix4& A, const Matrix4& B ) {
    return Matrix4( A.A + B.A );
}
//------------------------------------------------------------------------------
// Matrix Multiplication
//------------------------------------------------------------------------------
/// Matrix-Matrix Multiplication Function
/// Note: Not Commutative -> Matrix order critical
Matrix4 Matrix4::multiply( const Matrix4& A, const Matrix4& B ) {
    return Matrix4( A.A * B.A );
}
//------------------------------------------------------------------------------
/// Matrix-Scalar Multiplication Function
Matrix4 Matrix4::multiply( const Matrix4& A, const double& c ) {
    return Matrix4( A.A * c );
}
//------------------------------------------------------------------------------
/// Matrix-Vector Multiplication Function
Vector4 Matrix4::multiply( const Matrix4& A, const Vector4& x ) {
    Eigen::Vector4d v = Eigen::Vector4d( x.x(), x.y(), x.z(), x.w() );
    Eigen::Vector4d b = A.A * v;
    return Vector4( b(0), b(1), b(2), b(3) );
}
//------------------------------------------------------------------------------
/// Vector-Matrix Multiplication Function
Vector4 Matrix4::multiply( const Vector4& x, const Matrix4& A ) {
    Eigen::Vector4d v = Eigen::Vector4d( x.x(), x.y(), x.z(), x.w() );
    Eigen::Vector4d b = v.transpose() * A.A;
    return Vector4( b(0), b(1), b(2), b(3) );
}
//------------------------------------------------------------------------------
// Matrix Transposition
//------------------------------------------------------------------------------
/// Matrix Transpose Function
Matrix4 Matrix4::transpose( Matrix4& A ) {
    return Matrix4( A.A.transpose() );
}
//------------------------------------------------------------------------------
/// Inplace Matrix Transpose Function
void Matrix4::transpose( void ) {
    Eigen::Matrix4d m = A.transpose();
    A = m;
    //this->A = this->A.transpose();
}
//------------------------------------------------------------------------------
// Matrix Inversion
//------------------------------------------------------------------------------
/// Matrix Inverse Function
Matrix4 Matrix4::inverse( Matrix4& A ) {
    return Matrix4( A.A.inverse() );
}
//------------------------------------------------------------------------------
// Row & Column, Get & Set
//------------------------------------------------------------------------------
/// Set a Row
void Matrix4::row( const unsigned int& i, Vector4& v ) {
    assert( i < 4 );
    A( i, v(0) );
    A( i, v(1) );
    A( i, v(2) );
    A( i, v(3) );
}
//------------------------------------------------------------------------------
/// Get a Row
Vector4 Matrix4::row( const unsigned int& i ) {
    assert( i < 4 );
    return Vector4( A( i, 0 ), A( i, 1 ), A( i, 2 ), A( i, 3 ) );
}
//------------------------------------------------------------------------------
/// Set a Column
void Matrix4::column( const unsigned int& i, Vector4& v ) {
    assert( i < 4 );
    A( v(0), i );
    A( v(1), i );
    A( v(2), i );
    A( v(3), i );
}
//------------------------------------------------------------------------------
/// Get a Column
Vector4 Matrix4::column( const unsigned int& i ) {
    assert( i < 4 );
    return Vector4( A( 0, i ), A( 1, i ), A( 2, i ), A( 3, i ) );
}
//------------------------------------------------------------------------------
// Rotation Matrices
//------------------------------------------------------------------------------
/// Rotation Matrix Around X
/// Right-Handed
Matrix4 Matrix4::rotX( const double& theta ) {
    Matrix4 A;
    A.identity();
    A(1,1) = cos(theta);
    A(1,2) = -sin(theta);
    A(2,1) = sin(theta);
    A(2,2) = cos(theta);
    return A;
}
//------------------------------------------------------------------------------
/// Rotation Matrix Around Y
/// Right-Handed
Matrix4 Matrix4::rotY( const double& theta ) {
    Matrix4 A;
    A.identity();
    A(0,0) = cos(theta);
    A(0,2) = sin(theta);
    A(2,0) = -sin(theta);
    A(2,2) = cos(theta);
    return A;
}
//------------------------------------------------------------------------------
/// Rotation Matrix Around Z
/// Right-Handed
Matrix4 Matrix4::rotZ( const double& theta ) {
    Matrix4 A;
    A.identity();
    A(0,0) = cos(theta);
    A(0,1) = -sin(theta);
    A(1,0) = sin(theta);
    A(1,1) = cos(theta);
    return A;
}
//------------------------------------------------------------------------------
/// Get Rotation Component of This Matrix
Matrix3 Matrix4::rotation( void ) {
    return Matrix3( A(0,0), A(0,1), A(0,2),
                    A(1,0), A(1,1), A(1,2),
                    A(2,0), A(2,1), A(2,2) );
}
//------------------------------------------------------------------------------
// Translation Matrices
//------------------------------------------------------------------------------
/// Generate a Transformation Matrix Only
Matrix4 Matrix4::translationMatrix( const double& x, const double& y, const double& z ) {
    Matrix4 A;
    A.identity();
    A(0,3) = x;
    A(1,3) = y;
    A(2,3) = z;
    return A;
}

Matrix4 Matrix4::translationMatrix( const Vector3& u ) {
    Matrix4 A;
    A.identity();
    A(0,3) = u.x();
    A(1,3) = u.y();
    A(2,3) = u.z();
    return A;
}
//------------------------------------------------------------------------------
// Translation Get & Set
//------------------------------------------------------------------------------
/// Set Translation Component (Only) of This Matrix by x, y, z displacements
void Matrix4::setTranslation( const double& x, const double& y, const double& z ) {
    A(0,3) = x;
    A(1,3) = y;
    A(2,3) = z;
}
//------------------------------------------------------------------------------
/// Set Translation Component (Only) of This Matrix by displacement vector
void Matrix4::setTranslation( const Vector3& v ) {
    A(0,3) = v.x();
    A(1,3) = v.y();
    A(2,3) = v.z();
}
//------------------------------------------------------------------------------
/// Get Translation Component of This Matrix
Vector3 Matrix4::translation( void ) {
    return Vector3( A(0,3), A(1,3), A(2,3) );
}

//------------------------------------------------------------------------------
// Translation Modification
//------------------------------------------------------------------------------
/// Add to existing translation component of this matrix by x, y, z displacements
void Matrix4::addTranslation( const double& x, const double& y, const double& z ) {
    A(0,3) += x;
    A(1,3) += y;
    A(2,3) += z;
}
//------------------------------------------------------------------------------
/// Add to existing translation component of this matrix by displacement vector
void Matrix4::addTranslation( const Vector3& v ) {
    A(0,3) += v.x();
    A(1,3) += v.y();
    A(2,3) += v.z();
}

//------------------------------------------------------------------------------
// Scaling Matrices
//------------------------------------------------------------------------------
/// Generate a Scaling Matrix Only
Matrix4 Matrix4::scalingMatrix( const double& x, const double& y, const double& z ) {
    Matrix4 A;
    A.identity();
    A(0,0) = x;
    A(1,1) = y;
    A(2,2) = z;
    return A;
}
//------------------------------------------------------------------------------
// Interface Utilities
//------------------------------------------------------------------------------
/// Linearize Matrix to Array in OpenGL Compatible Format
double* Matrix4::arrayOpenGL( void ) {
    // OpenGL is column major
    // Therefore transpose A to get compatible matrix.
    //Eigen::Matrix4d At = A.transpose();
    //Eigen::Matrix4d At = A;
    for( unsigned int i = 0; i < 16; i++ ) {
        /*
        unsigned int row = i/4;
        unsigned int col = i%4;
        m_linearized[i] = At( row, col );
        */
        unsigned int row = i % 4;
        unsigned int col = i / 4;
        //unsigned int row = i / 4;
        //unsigned int col = i % 4;
        m_linearized[i] = A( row, col );
    }
    return m_linearized;
}
//------------------------------------------------------------------------------
// Matrix3 Operators
//------------------------------------------------------------------------------
/// Parenthetical Access to Member Data
double& Matrix4::operator() ( const unsigned int& row, const unsigned int& col ) {
    assert( row < 4 && col < 4 );
    return A( row, col );
}
//------------------------------------------------------------------------------
/// Matrix-Matrix Equality Operator
bool Matrix4::operator==( const Matrix4& B ) {
    return equal( *this, B );
}
//------------------------------------------------------------------------------
/// Matrix-Matrix Inequality Operator
bool Matrix4::operator!=( const Matrix4& B ) {
    return !equal( *this, B );
}
//------------------------------------------------------------------------------
/// Inplace Matrix-Matrix Addition Operator
Matrix4 Matrix4::operator+=( const Matrix4& B ) {
    Matrix4 C = add( *this, B );
    A = C.A;
    return *this;
}
//------------------------------------------------------------------------------
/// Matrix-Matrix Addition Operator
Matrix4 Matrix4::operator+( const Matrix4& B ) {
    Matrix4 C = *this;
    C += B;
    return C;
}
//------------------------------------------------------------------------------
/// Inplace Matrix-Matrix Multiplication Operator
Matrix4 Matrix4::operator*=( const Matrix4& B ) {
    Matrix4 C = multiply( *this, B );
    A = C.A;
    return *this;
}
//------------------------------------------------------------------------------
/// Matrix-Matrix Multiplication Operator
Matrix4 Matrix4::operator*( const Matrix4& B ) {
    Matrix4 C = *this;
    C *= B;
    return C;
}
//------------------------------------------------------------------------------
/// Inplace Matrix-Scalar Multiplication Operator
Matrix4 Matrix4::operator*=( const double& c ) {
    Matrix4 C = multiply( *this, c );
    A = C.A;
    return *this;
}
//------------------------------------------------------------------------------
/// Matrix-Scalar Multiplication Operator
Matrix4 Matrix4::operator*( const double& c ) {
    Matrix4 C = *this;
    C *= c;
    return C;
}
//------------------------------------------------------------------------------
/// Matrix-Vector Multiplication Operator
Vector4 Matrix4::operator*( const Vector4& x ) {
    return multiply( *this, x );
}
//------------------------------------------------------------------------------
// Out of class Matrix4 'friendly' operators
//------------------------------------------------------------------------------
/// Scalar-Matrix Multiplication Operator
Matrix4 operator*( const double& c, const Matrix4& A ) {
    return Matrix4::multiply( A, c );
}
//------------------------------------------------------------------------------
/// Vector-Matrix Multiplication Operator
Vector4 operator*( Vector4& x, const Matrix4& A ) {
    return Matrix4::multiply( x, A );
}
//------------------------------------------------------------------------------
