#include <cs6555/Math/Vector4.h>

#include <math.h>
#include <assert.h>
#include <stdio.h>

//------------------------------------------------------------------------------
// Constructors
//------------------------------------------------------------------------------
/// Default Constructor
Vector4::Vector4( void ) {
    data = Eigen::Vector4d( 0, 0, 0, 0 );
}
//------------------------------------------------------------------------------
/// Copy Constructor
Vector4::Vector4( const Vector4& u ) {
    data = Eigen::Vector4d( u.data );
}
//------------------------------------------------------------------------------
/// Component Constructor
Vector4::Vector4( const double& x, const double& y, const double& z ) {
    data = Eigen::Vector4d( x, y, z, 1 );
}
//------------------------------------------------------------------------------
/// Component Constructor
Vector4::Vector4( const double& x, const double& y, const double& z, const double& w ) {
    data = Eigen::Vector4d( x, y, z, w );
}
//------------------------------------------------------------------------------
/// Vector3 Constructor
Vector4::Vector4( Vector3 v3 ) {
    data = Eigen::Vector4d( v3(0), v3(1), v3(2), 1 );
}
//------------------------------------------------------------------------------
/// Adapter copy Constructor (private)
Vector4::Vector4( const Eigen::Vector4d& u ) {
    data = Eigen::Vector4d( u );
}
//------------------------------------------------------------------------------
// Destructor
//------------------------------------------------------------------------------
/// Destructor
Vector4::~Vector4( void ) {

}
//------------------------------------------------------------------------------
// Debugging Functions
//------------------------------------------------------------------------------
/// Print to the console for debugging
void Vector4::print( void ) {
    printf( "%f, %f, %f, %f", data(0), data(1), data(2), data(3) );
}
//------------------------------------------------------------------------------
// Vector4 Comparison Functions
//------------------------------------------------------------------------------
/// Equality comparison
bool Vector4::equal( const Vector4& u, const Vector4& v ) {
    // error intrinsic to IEEE 754 double precision fp math
    // so use epsilon to ensure reasonable comparison
    double EPSILON = 1.0e-5;  // epsilon any smaller fails unit testing
    if( u.data(0) + EPSILON >= v.data(0) && u.data(0) - EPSILON <= v.data(0) &&
            u.data(1) + EPSILON >= v.data(1) && u.data(1) - EPSILON <= v.data(1) &&
            u.data(2) + EPSILON >= v.data(2) && u.data(2) - EPSILON <= v.data(2) &&
            u.data(3) + EPSILON >= v.data(3) && u.data(3) - EPSILON <= v.data(3) )
            return true;
    return false;
}
//------------------------------------------------------------------------------
// Vector4 Inversion Functions
//------------------------------------------------------------------------------
/// Inversion/"Flip"
Vector4 Vector4::invert( const Vector4& u ) {
    return Vector4( -u.data );
}
//------------------------------------------------------------------------------
// Vector4 Addition Functions
//------------------------------------------------------------------------------
/// Vector-Vector Addition Function
Vector4 Vector4::add( const Vector4& u, const Vector4& v ) {
    return Vector4( u.data + v.data );
}
//------------------------------------------------------------------------------
/// Vector-Vector Subtraction Function
Vector4 Vector4::subtract( const Vector4& u, const Vector4& v ) {
    return Vector4( u.data - v.data );
}
//------------------------------------------------------------------------------
// Vector4 Multiplication Functions
//------------------------------------------------------------------------------
/// Vector-Scalar Multiplication (Scaling) Function
Vector4 Vector4::multiply( const Vector4& u, const double& c ) {
    return Vector4( u.data * c );
}
//------------------------------------------------------------------------------
/// Scalar-Vector Multiplication (Scaling) Function
Vector4 Vector4::multiply( const double& c, const Vector4& u ) {
    return Vector4( c * u.data );
}
//------------------------------------------------------------------------------
/// Vector-Vector Multiplication Function
double Vector4::multiply( const Vector4& u, const Vector4& v ) {
    Eigen::MatrixXd w = u.data.transpose()*v.data;
    return w(0,0);
}
//------------------------------------------------------------------------------
// Vector4 Inner Product Functions
//------------------------------------------------------------------------------
/// Vector Dot-Product
double Vector4::dot( const Vector4& u, const Vector4& v ) {
    return u.data.dot( v.data );
}
//------------------------------------------------------------------------------
// Vector4 Normalization Functions
//------------------------------------------------------------------------------
/// Normalize This Vector
void Vector4::normalize( void ) {
    data.normalize();
}
//------------------------------------------------------------------------------
// Component Get & Set
//------------------------------------------------------------------------------
/// Get x Component
double Vector4::x( void ) const { return data(0); }
//------------------------------------------------------------------------------
/// Get y Component
double Vector4::y( void ) const { return data(1); }
//------------------------------------------------------------------------------
/// Get z Component
double Vector4::z( void ) const { return data(2); }
//------------------------------------------------------------------------------
/// Get w Component
double Vector4::w( void ) const { return data(3); }
//------------------------------------------------------------------------------
/// Set x Component
void Vector4::x( const double& value ) { data(0) = value; }
//------------------------------------------------------------------------------
/// Set y Component
void Vector4::y( const double& value ) { data(1) = value; }
//------------------------------------------------------------------------------
/// Set z Component
void Vector4::z( const double& value ) { data(2) = value; }
//------------------------------------------------------------------------------
/// Set w Component
void Vector4::w( const double& value ) { data(3) = value; }
//------------------------------------------------------------------------------
// Get as Class
//------------------------------------------------------------------------------
/// Get a Vector3
Vector3 Vector4::vector3( void ) {
    return Vector3( x(), y(), z() );
}
//------------------------------------------------------------------------------
// Vector4 Operators
//------------------------------------------------------------------------------
/// Parenthetical access to member data
double& Vector4::operator() ( const unsigned int& index ) {
    assert( index < 4 );
    return data(index);
}
//------------------------------------------------------------------------------
/// Vector-Vector Equality Operator
bool Vector4::operator==( const Vector4& u ) {
    return equal( *this, u );
}
//------------------------------------------------------------------------------
/// Vector-Vector Inequality Operator
bool Vector4::operator!=( const Vector4& u ) {
    return !equal( *this, u );
}
//------------------------------------------------------------------------------
/// Inplace Vector-Vector Addition Operator
Vector4 Vector4::operator+=( const Vector4& u ) {
    Vector4 v = add( *this, u );
    data = v.data;
    return *this;
}
//------------------------------------------------------------------------------
/// Vector-Vector Addition Operator
Vector4 Vector4::operator+( const Vector4& u ) {
    Vector4 v = *this;
    v += u;
    return v;
}
//------------------------------------------------------------------------------
/// Vector-Vector Multiplication Operator
double Vector4::operator*( const Vector4& u ) {
    return multiply( *this, u );
}
//------------------------------------------------------------------------------
/// Inplace Vector-Scalar Multiplication Operator
Vector4 Vector4::operator*=( const double& c ) {
    Vector4 v = multiply( *this, c );
    data = v.data;
    return *this;
}
//------------------------------------------------------------------------------
/// Vector-Scalar Multiplication Operator
Vector4 Vector4::operator*( const double& c ) {
    Vector4 v = *this;
    v.data *= c;
    return v;
}
//------------------------------------------------------------------------------
// Out of class Vector4 'friendly' operators
//------------------------------------------------------------------------------
/// Inplace Scalar-Vector Multiplication Operator
Vector4 operator*( const double& c, const Vector4& u ) {
    return Vector4::multiply( c, u );
}
//------------------------------------------------------------------------------
