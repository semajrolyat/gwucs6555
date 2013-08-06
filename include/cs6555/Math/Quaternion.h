/*-----------------------THE GEORGE WASHINGTON UNIVERSITY-----------------------
James Taylor                                                         jrt@gwu.edu

Quaternion class

------------------------------------------------------------------------------*/

#ifndef _QUATERNION_H_
#define _QUATERNION_H_

#include <cs6555/Math/Vector3.h>
#include <cs6555/Math/Vector4.h>
#include <cs6555/Math/Matrix3.h>
#include <cs6555/Math/Matrix4.h>

//------------------------------------------------------------------------------

class Quaternion {
public:
    // Constructors
    Quaternion( void );
    Quaternion( const Quaternion& );
    Quaternion( const double& w, const double& x, const double& y, const double& z );
    Quaternion( const double& w, const Vector3& v );
    Quaternion( const Vector4& v );
    Quaternion( Matrix3& m );

    // Destructor
    virtual ~Quaternion( void );

    // Debugging Functions
    void print( void );

    // Inplace Initialization Functions
    void identity( void );

    // Quaternion Comparison Functions
    static bool equal( const Quaternion&, const Quaternion& );

    // Quaternion Conjugation Functions
    Quaternion conjugate( void );

    // Quaternion Inversion Functions
    Quaternion inverse( void );

    // Quaternion Addition Functions
    static Quaternion add( const Quaternion&, const Quaternion& );
    static Quaternion subtract( const Quaternion&, const Quaternion& );

    // Quaternion Multiplication Functions
    static Quaternion multiply( const Quaternion&, const Quaternion& );    
    static Quaternion multiply( const Quaternion&, const double& );
    //static Vector4 multiply( const Quaternion&, const Vector4& );

    static Quaternion multiply( const Quaternion&, const Vector3& );

    // Quaternion Norm Functions
    double norm( void );

    // Quaternion Inner Product Functions
    static double dot( const Quaternion&, const Quaternion& );

    // Quaternion Normalization Functions
    bool normalize( void );

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
    Vector3 vector3( void ) const;
    Matrix3 matrix3( void );
    Matrix4 matrix4( void );

    void set( Matrix3& m );

    // Interpolation
    static Quaternion slerp( const double& t, const Quaternion& q0, const Quaternion& q1 );

    // Quaternion Operators
    double& operator() (const unsigned int& index );

    Quaternion operator+=( const Quaternion& );
    Quaternion operator+( const Quaternion& ) const;

    Quaternion operator-=( const Quaternion& );
    Quaternion operator-( const Quaternion& ) const;

    Quaternion operator*=( const Quaternion& );
    Quaternion operator*( const Quaternion& ) const;

    Quaternion operator*=( const double& );
    Quaternion operator*( const double& ) const;

private:
    // Member data
    Vector4 data;
};

//------------------------------------------------------------------------------

// Out of class Quaternion 'friendly' operators
Quaternion operator*( const double& c, const Quaternion& q );
Quaternion operator*( const Vector3& v, const Quaternion& q );

//------------------------------------------------------------------------------

#endif // _QUATERNION_H_
