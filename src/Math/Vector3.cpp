#include <cs6555/Math/Vector3.h>

#include <math.h>
#include <assert.h>
#include <stdio.h>
#include <iostream>

//------------------------------------------------------------------------------
// Constructors
//------------------------------------------------------------------------------
/// Default Constructor
Vector3::Vector3( void ) {
    data = Eigen::Vector3d( 0, 0, 0 );
}
//------------------------------------------------------------------------------
/// Copy Constructor
Vector3::Vector3( const Vector3& u ) {
    data = Eigen::Vector3d( u.data );
}
//------------------------------------------------------------------------------
/// Component Constructor
Vector3::Vector3( const double& x, const double& y, const double& z ) {
    data = Eigen::Vector3d( x, y, z );
}
//------------------------------------------------------------------------------
/// Adapter Copy Constructor (private)
Vector3::Vector3( const Eigen::Vector3d& v ) {
    data = Eigen::Vector3d( v );
}
//------------------------------------------------------------------------------
// Destructor
//------------------------------------------------------------------------------
/// Destructor
Vector3::~Vector3( void ) {

}
//------------------------------------------------------------------------------
// Debugging Functions
//------------------------------------------------------------------------------
/// Print to the console for debugging
void Vector3::print( void ) {
    printf( "%f, %f, %f", data(0), data(1), data(2) );
}
//------------------------------------------------------------------------------
// Vector3 Comparison Functions
//------------------------------------------------------------------------------
/// Equality comparison
bool Vector3::equal( const Vector3& u, const Vector3& v ) {
    // error intrinsic to IEEE 754 double precision fp math
    // so use epsilon to ensure reasonable comparison
    double EPSILON = 1.0e-5;  // epsilon any smaller fails unit testing
    if( u.data(0) + EPSILON >= v.data(0) && u.data(0) - EPSILON <= v.data(0) &&
            u.data(1) + EPSILON >= v.data(1) && u.data(1) - EPSILON <= v.data(1) &&
            u.data(2) + EPSILON >= v.data(2) && u.data(2) - EPSILON <= v.data(2) )
        	return true;
    return false;
}
//------------------------------------------------------------------------------
// Vector3 Inversion Functions
//------------------------------------------------------------------------------
/// Inversion/"Flip"
Vector3 Vector3::invert( const Vector3& u ) {
    return Vector3( -u.data );
}
//------------------------------------------------------------------------------
// Vector3 Addition Functions
//------------------------------------------------------------------------------
/// Vector-Vector Addition Function
Vector3 Vector3::add( const Vector3& u, const Vector3& v ) {
    return Vector3( u.data + v.data );
}
//------------------------------------------------------------------------------
/// Vector-Vector Subtraction Function
Vector3 Vector3::subtract( const Vector3& u, const Vector3& v ) {
    return Vector3( u.data - v.data );
}
//------------------------------------------------------------------------------
// Vector3 Multiplication Functions
//------------------------------------------------------------------------------
/// Vector-Scalar Multiplication (Scaling) Function
Vector3 Vector3::multiply( const Vector3& u, const double& c ) {
    return Vector3( u.data * c );
}
//------------------------------------------------------------------------------
/// Scalar-Vector Multiplication (Scaling) Function
Vector3 Vector3::multiply( const double& c, const Vector3& u ) {
    return Vector3( c * u.data );
}
//------------------------------------------------------------------------------
/// Vector-Vector Multiplication Function
double Vector3::multiply( const Vector3& u, const Vector3& v ) {
    Eigen::MatrixXd w = u.data.transpose()*v.data;
    return w(0,0);
}
//------------------------------------------------------------------------------
// Vector3 Inner Product Functions
//------------------------------------------------------------------------------
/// Vector Dot-Product Function
double Vector3::dot( const Vector3& u, const Vector3& v ) {
    return u.data.dot( v.data );
}
//------------------------------------------------------------------------------
// Vector3 Cross Product Functions
//------------------------------------------------------------------------------
/// Vector Cross-Product Function
Vector3 Vector3::cross( const Vector3& u, const Vector3& v ) {
    return Vector3( u.data.cross( v.data ) );
}

//------------------------------------------------------------------------------
// Vector3 Magnitude
//------------------------------------------------------------------------------
double Vector3::magnitude( void ) {
    return data.norm();
}

//------------------------------------------------------------------------------
// Vector3 Normalization Functions
//------------------------------------------------------------------------------
/// Normalize This Vector
void Vector3::normalize( void ) {
    data.normalize();
}
//------------------------------------------------------------------------------
// Component Get & Set
//------------------------------------------------------------------------------
/// Get x Component
double Vector3::x( void ) const { return data(0); }
//------------------------------------------------------------------------------
/// Get y Component
double Vector3::y( void ) const { return data(1); }
//------------------------------------------------------------------------------
/// Get z Component
double Vector3::z( void ) const { return data(2); }
//------------------------------------------------------------------------------
/// Set x Component
void Vector3::x( const double& value ) { data(0) = value; }
//------------------------------------------------------------------------------
/// Set y Component
void Vector3::y( const double& value ) { data(1) = value; }
//------------------------------------------------------------------------------
/// Set z Component
void Vector3::z( const double& value ) { data(2) = value; }
//------------------------------------------------------------------------------
// Vector3 Operators
//------------------------------------------------------------------------------
/// Parenthetical access to member data
double& Vector3::operator() ( const unsigned int& index ) {
    assert( index < 3 );
    return data(index);
}
//------------------------------------------------------------------------------
/// Vector-Vector Equality Operator
bool Vector3::operator==( const Vector3& u ) const {
    return equal( *this, u );
}
//------------------------------------------------------------------------------
/// Vector-Vector Inequality Operator
bool Vector3::operator!=( const Vector3& u ) const {
    return !equal( *this, u );
}
//------------------------------------------------------------------------------
/// Unary Inversion Operator
Vector3 Vector3::operator-( ) {
    return Vector3::invert( *this );
}

//------------------------------------------------------------------------------
/// Inplace Vector-Vector Addition Operator
Vector3 Vector3::operator+=( const Vector3& u ) {
    Vector3 v = add( *this, u );
    data = v.data;
    return *this;
}
//------------------------------------------------------------------------------
/// Vector-Vector Addition Operator
Vector3 Vector3::operator+( const Vector3& u ) const {
    Vector3 v = *this;
    v += u;
    return v;
}
//------------------------------------------------------------------------------
/// Inplace Vector-Vector Subtraction Operator
Vector3 Vector3::operator-=( const Vector3& u ) {
    Vector3 v = subtract( *this, u );
    data = v.data;
    return *this;
}
//------------------------------------------------------------------------------
/// Vector-Vector Subtraction Operator
Vector3 Vector3::operator-( const Vector3& u ) const {
    Vector3 v = *this;
    v -= u;
    return v;
}
//------------------------------------------------------------------------------
/// Vector-Vector Multiplication Operator
double Vector3::operator*( const Vector3& u ) const {
    return multiply( *this, u );
}
//------------------------------------------------------------------------------
/// Inplace Vector-Scalar Multiplication Operator
Vector3 Vector3::operator*=( const double& c ) {
    Vector3 v = multiply( *this, c );
    data = v.data;
    return *this;
}
//------------------------------------------------------------------------------
/// Vector-Scalar Multiplication Operator
Vector3 Vector3::operator*( const double& c ) const {
    Vector3 v = *this;
    v.data *= c;
    return v;
}
//------------------------------------------------------------------------------
/// Inplace Vector-Scalar Division Operator
Vector3 Vector3::operator/=( const double& c ) {
    assert( c != 0 );
    Vector3 v = multiply( *this, 1/c );
    data = v.data;
    return *this;
}
//------------------------------------------------------------------------------
/// Vector-Scalar Division Operator
Vector3 Vector3::operator/( const double& c ) const {
    Vector3 v = *this;
    v.data /= c;
    return v;
}

//------------------------------------------------------------------------------
// Out of class Vector3 'friendly' operators
//------------------------------------------------------------------------------
/// Inplace Scalar-Vector Multiplication Operator
Vector3 operator*( const double& c, const Vector3& u ) {
    return Vector3::multiply( u, c );
}
//------------------------------------------------------------------------------

