/*
Reference
    Eberly, D. - 3D Game Engine Design - Morgan Kaufman - 2000
*/

#include <cs6555/Math/Quaternion.h>
#include <assert.h>
#include <stdio.h>

//------------------------------------------------------------------------------
// Constructors
//------------------------------------------------------------------------------
/// Default Constructor
Quaternion::Quaternion( void ) {
    identity();
}
//------------------------------------------------------------------------------
/// Copy Constructor
Quaternion::Quaternion( const Quaternion& q ) {
    data = Vector4( q.x(), q.y(), q.z(), q.w() );
}
//------------------------------------------------------------------------------
/// Component Constructor
Quaternion::Quaternion( const double& w, const double& x, const double& y, const double& z ) {
    data = Vector4( x, y, z, w );
}
//------------------------------------------------------------------------------
/// Selection/Vector Constructor
Quaternion::Quaternion( const double& w, const Vector3& v ) {
    data = Vector4( v.x(), v.y(), v.z(), w );
}
//------------------------------------------------------------------------------
/// Vector Constructor
Quaternion::Quaternion( const Vector4& v ) {
    data = Vector4( v );
}
//------------------------------------------------------------------------------
/// Matrix3 Constructor
Quaternion::Quaternion( Matrix3& m ) {
    set( m );
}
//------------------------------------------------------------------------------
// Destructor
//------------------------------------------------------------------------------
/// Destructor
Quaternion::~Quaternion( void ) {

}
//------------------------------------------------------------------------------

void Quaternion::set( Matrix3& m ) {
    double tr, s;

    tr = m(0,0) + m(1,1) + m(2,2);

    if( tr >= 0.0 ) {
        s = sqrt( tr + 1 );
        w( 0.5 * s );
        s = 0.5 / s;
        x( ( m(2,1) - m(1,2) ) * s );
        y( ( m(0,2) - m(2,0) ) * s );
        z( ( m(1,0) - m(0,1) ) * s );
    } else {
        int i = 0;
        if( m(1,1) > m(0,0) )
            i = 1;
        if( m(2,2) > m(i,i) )
            i = 2;

        switch( i ) {
        case 0:
            s = sqrt( (m(0,0) - (m(1,1) + m(2,2)) ) + 1 );
            x( 0.5 * s );
            s = 0.5 / s;
            y( (m(0,1) + m(1,0)) * s );
            z( (m(2,0) + m(0,2)) * s );
            w( (m(2,1) - m(1,2)) * s );
            break;
        case 1:
            s = sqrt( (m(1,1) - (m(2,2) + m(0,0)) ) + 1 );
            y( 0.5 * s );
            s = 0.5 / s;
            z( (m(1,2) + m(2,1)) * s );
            x( (m(0,1) + m(1,0)) * s );
            w( (m(0,2) - m(2,0)) * s );
            break;
        case 2:
            s = sqrt( (m(2,2) - (m(0,0) + m(1,1)) ) + 1 );
            z( 0.5 * s );
            s = 0.5 / s;
            x( (m(2,0) + m(0,2)) * s );
            y( (m(1,2) + m(2,1)) * s );
            w( (m(1,0) - m(0,1)) * s );
            break;
        }
    }
}

//------------------------------------------------------------------------------
// Debugging Functions
//------------------------------------------------------------------------------
/// Print to the console for debugging
void Quaternion::print( void ) {
    printf( "w:%f x:%f y:%f z:%f", w(), x(), y(), z() );
}
//------------------------------------------------------------------------------
// Inplace Initialization Functions
//------------------------------------------------------------------------------
/// Identity Initializer
void Quaternion::identity( void ) {
    data = Vector4( 0, 0, 0, 1 );
}
//------------------------------------------------------------------------------
// Quaternion Comparison Functions
//------------------------------------------------------------------------------
/// Equality comparison
bool Quaternion::equal( const Quaternion& q1, const Quaternion& q2 ) {
    return( q1.x() == q2.x() && q1.y() == q2.y() && q1.z() == q2.z() && q1.w() == q2.w() );
}
//------------------------------------------------------------------------------
// Quaternion Conjugation Functions
//------------------------------------------------------------------------------
/// Conjugate/"Flip"
Quaternion Quaternion::conjugate( void ) {
    return Quaternion( w(), -x(), -y(), -z() );
}
//------------------------------------------------------------------------------
// Quaternion Inversion Functions
//------------------------------------------------------------------------------
/// Inversion
Quaternion Quaternion::inverse( void ) {
    double n = norm();
    Quaternion q = conjugate();
    return Quaternion( q.w()/n, q.x()/n, q.y()/n, q.z()/n );
}
//------------------------------------------------------------------------------
// Quaternion Addition Functions
//------------------------------------------------------------------------------
/// Quaternion-Quaternion Addition Operation
Quaternion Quaternion::add( const Quaternion& q1, const Quaternion& q2 ) {
    return Quaternion( q1.w() + q2.w(), q1.x() + q2.x(), q1.y() + q2.y(), q1.z() + q2.z() );
}
//------------------------------------------------------------------------------
/// Quaternion-Quaternion Subtraction Operation
Quaternion Quaternion::subtract( const Quaternion& q1, const Quaternion& q2 ) {
    return Quaternion( q1.w() - q2.w(), q1.x() - q2.x(), q1.y() - q2.y(), q1.z() - q2.z() );
}
//------------------------------------------------------------------------------
// Quaternion Multiplication Functions
//------------------------------------------------------------------------------
/// Quaternion-Quaternion Multiplication (Concatenation) Function
Quaternion Quaternion::multiply( const Quaternion& q1, const Quaternion& q2 ) {
    // Eberly (2000) - p.11
    return Quaternion( q1.w() * q2.w() - q1.x() * q2.x() - q1.y() * q2.y() - q1.z() * q2.z(),
                       q1.w() * q2.x() + q1.x() * q2.w() + q1.y() * q2.z() - q1.z() * q2.y(),
                       q1.w() * q2.y() - q1.x() * q2.z() + q1.y() * q2.w() + q1.z() * q2.x(),
                       q1.w() * q2.z() + q1.x() * q2.y() - q1.y() * q2.x() + q1.z() * q2.w() );
    /*
    // Alternate from Lecture Notes and Eberly (2000) p.12
    Vector3 v1 = q1.v();
    Vector3 v2 = q2.v();
    return Quaternion( q1.w()*q2.w() - Vector3::dot(v1,v2), q1.w()*v2 + q2.w()*v1 + Vector3::cross(v1, v2) );
    */

}
/*
  /// Quaternion-Vector Multiplication Operation
Vector4 Quaternion::multiply( const Quaternion& q, const Vector4& v ) {
    Matrix4 A = Matrix4( v );
}
*/
//------------------------------------------------------------------------------
/// Quaternion-Scalar Multiplication (Scaling) Function
Quaternion Quaternion::multiply( const Quaternion& q, const double& c ) {
    return Quaternion( q.w() * c, q.x() * c, q.y() * c, q.z() * c );
}
//------------------------------------------------------------------------------

Quaternion Quaternion::multiply( const Quaternion& q, const Vector3& v ) {
    return multiply( q, Quaternion( 0.0, v.x(), v.y(), v.z() ) );
}
//------------------------------------------------------------------------------
// Quaternion Norm Functions
//------------------------------------------------------------------------------
/// Compute This Quaternion's Norm
double Quaternion::norm( void ) {
    return w() * w() + x() * x() + y() * y() + z() * z();
}
//------------------------------------------------------------------------------
// Quaternion Inner Product Functions
//------------------------------------------------------------------------------
/// Quaternion Dot-Product (Inner-Product)
double Quaternion::dot( const Quaternion& q1, const Quaternion& q2 ) {
    return q1.w() * q2.w() + q1.x() * q2.x() + q1.y() * q2.y() + q1.z() * q2.z();
}
//------------------------------------------------------------------------------
// Quaternion Normalization Functions
//------------------------------------------------------------------------------
/// Normalize This Quaternion
bool Quaternion::normalize( void ) {
    double nn = norm();
    // may be some numerical problems if don't use epsilon, but for now just check if zero
    if( nn == 0.0 ) return false;

    double invn = 1.0 / sqrt( nn );
    for( unsigned int i = 0; i < 4; i++ )
        data(i) *= invn;
    return true;
}
//------------------------------------------------------------------------------
// Component Get & Set
//------------------------------------------------------------------------------
/// Get Axis x Component
double Quaternion::x( void ) const {
    return data.x();
}
//------------------------------------------------------------------------------
/// Get Axis y Component
double Quaternion::y( void ) const {
    return data.y();
}
//------------------------------------------------------------------------------
/// Get Axis z Component
double Quaternion::z( void ) const {
    return data.z();
}
//------------------------------------------------------------------------------
/// Get Angular w Component (Selection Function)
double Quaternion::w( void ) const {
    return data.w();
}
//------------------------------------------------------------------------------
/// Set Axis x Component
void Quaternion::x( const double& value ) { data.x( value ); }
//------------------------------------------------------------------------------
/// Set Axis y Component
void Quaternion::y( const double& value ) { data.y( value ); }
//------------------------------------------------------------------------------
/// Set Axis z Component
void Quaternion::z( const double& value ) { data.z( value ); }
//------------------------------------------------------------------------------
/// Set Axis w Component
void Quaternion::w( const double& value ) { data.w( value ); }
//------------------------------------------------------------------------------
// Get as Class
//------------------------------------------------------------------------------
/// Get Axis Vector
Vector3 Quaternion::vector3( void ) const {
    return Vector3( x(), y(), z() );
}
//------------------------------------------------------------------------------
/// Get the Comparable Rotation Matrix
Matrix3 Quaternion::matrix3( void ) {
    // Lecture Notes and Eberly (2000) p.17
    return Matrix3( 1 - 2*y()*y() - 2*z()*z(),  2*x()*y() - 2*w()*z(),      2*x()*z() + 2*w()*y(),
                    2*x()*y() + 2*w()*z(),      1 - 2*x()*x() - 2*z()*z(),  2*y()*z() - 2*w()*x(),
                    2*x()*z() - 2*w()*y(),      2*y()*z() + 2*w()*x(),      1 - 2*x()*x() - 2*y()*y() );
}
//------------------------------------------------------------------------------
/// Get the Homogeneous Affine Transformation (Rotation) Matrix
Matrix4 Quaternion::matrix4( void ) {
    // Lecture Notes and Eberly (2000) p.17
    ///*
    return Matrix4( 1 - 2*y()*y() - 2*z()*z(),  2*x()*y() - 2*w()*z(),      2*x()*z() + 2*w()*y(),      0.0,
                    2*x()*y() + 2*w()*z(),      1 - 2*x()*x() - 2*z()*z(),  2*y()*z() - 2*w()*x(),      0.0,
                    2*x()*z() - 2*w()*y(),      2*y()*z() + 2*w()*x(),      1 - 2*x()*x() - 2*y()*y(),  0.0,
                    0.0,                        0.0,                        0.0,                        1.0 );
    //*/
    /*
    return Matrix4( 1 - 2*eq.y()*eq.y() - 2*eq.z()*eq.z(),  2*eq.x()*eq.y() - 2*eq.w()*eq.z(),      2*eq.x()*eq.z() + 2*eq.w()*eq.y(),      0.0,
                    2*eq.x()*eq.y() + 2*eq.w()*eq.z(),      1 - 2*eq.x()*eq.x() - 2*eq.z()*eq.z(),  2*eq.y()*eq.z() - 2*eq.w()*eq.x(),      0.0,
                    2*eq.x()*eq.z() - 2*eq.w()*eq.y(),      2*eq.y()*eq.z() + 2*eq.w()*eq.x(),      1 - 2*eq.x()*eq.x() - 2*eq.y()*eq.y(),  0.0,
                    0.0,                        0.0,                        0.0,                        1.0 );
    */
}
//------------------------------------------------------------------------------
// Interpolation
//------------------------------------------------------------------------------
/// Spherical Linear Interpolation
/// u E [0,1]
/// p (returned) is the linear combination where p = a * q1 + b * q2
Quaternion Quaternion::slerp( const double& u, const Quaternion& q1, const Quaternion& q2 ) {
    assert( u >= 0.0 && u <= 1.0 );

    bool flip = false;

    double comega = dot( q1, q2 );
    // cos(omega) < 0 -> p = a * q1 - b * q2
    // Note: may be a numerical problem simply comparing to zero.
    //       machine epsilon may be best
    if( comega < 0.0 ) {
        comega = -comega;
        flip = true;
    }
    double omega = acos( comega );  // acos -> omega E [0,1]
    // q1 dot p = cos(theta), theta = omega * u | u E [0,1]
    double somega = sin( omega );
    double theta = omega * u;
    double a = sin( (1.0 - u) * omega ) / somega;
    double b = sin( theta ) / somega;
    if( flip ) b = -b;
    Quaternion p = a * q1 + b * q2;
    // Note: length of p should be 1
    return p;
}
//------------------------------------------------------------------------------
// Quaternion Operators
//------------------------------------------------------------------------------
/// Parenthetical Access to Member Data
double& Quaternion::operator() (const unsigned int& index ) {
    assert( index < 4 );
    return data(index);
}
//------------------------------------------------------------------------------
/// Inline Quaternion-Quaternion Addition Operator
Quaternion Quaternion::operator+=( const Quaternion& q ) {
    Quaternion q0 = add( *this, q );
    data = q0.data;
    return *this;
}
//------------------------------------------------------------------------------
/// Quaternion-Quaternion Addition Operator
Quaternion Quaternion::operator+( const Quaternion& q ) const {
    Quaternion q0 = *this;
    q0 += q;
    return q0;
}
//------------------------------------------------------------------------------
/// Inline Quaternion-Quaternion Subtraction Operator
Quaternion Quaternion::operator-=( const Quaternion& q ) {
    Quaternion q0 = subtract( *this, q );
    data = q0.data;
    return *this;
}
//------------------------------------------------------------------------------
/// Quaternion-Quaternion Subtraction Operator
Quaternion Quaternion::operator-( const Quaternion& q ) const {
    Quaternion q0 = *this;
    q0 -= q;
    return q0;
}
//------------------------------------------------------------------------------
/// Inline Quaternion-Quaternion Multiplication (Concatenation) Operator
Quaternion Quaternion::operator*=( const Quaternion& q ) {
    Quaternion q0 = multiply( *this, q );
    data = q0.data;
    return *this;
}
//------------------------------------------------------------------------------
/// Quaternion-Quaternion Multiplication (Concatenation) Operator
Quaternion Quaternion::operator*( const Quaternion& q ) const {
    Quaternion q0 = *this;
    q0 *= q;
    return q0;
}
//------------------------------------------------------------------------------
/// Inline Quaternion-Scalar Multiplication (Scaling) Operator
Quaternion Quaternion::operator*=( const double& c ) {
    Quaternion q0 = multiply( *this, c );
    data = q0.data;
    return *this;
}
//------------------------------------------------------------------------------
/// Quaternion-Scalar Multiplication (Scaling) Operator
Quaternion Quaternion::operator*( const double& c ) const {
    Quaternion q0 = *this;
    q0 *= c;
    return q0;
}
//------------------------------------------------------------------------------
// Out of class Quaternion 'friendly' operators
//------------------------------------------------------------------------------
/// Scalar-Quaternion Multiplication (Scaling) Operator
Quaternion operator*( const double& c, const Quaternion& q ) {
    Quaternion q0 = Quaternion( q );
    q0 *= c;
    return q0;
}
//------------------------------------------------------------------------------

Quaternion operator*( const Vector3& v, const Quaternion& q ) {
    return Quaternion::multiply( Quaternion( 0.0, v.x(), v.y(), v.z() ), q );
}

//------------------------------------------------------------------------------
