#include <stdio.h>
#include <string>
#include <cs6555/Math/Quaternion.h>
#include <eigen3/Eigen/Geometry>
#include <eigen3/Eigen/Dense>
#include <iostream>
/*
bool report( std::string test_name, Vector3 u, Vector3 v, Vector3 experimental, Vector3 expected ) {
    if( !Vector3::equal( expected, experimental ) ) {
        printf( "  failed %s testing\n", test_name.c_str() );
        printf( "u: " );
        u.print( );
        printf( "v: " );
        v.print( );
        printf( "Expected: " );
        expected.print();
        printf( "Experimental: " );
        experimental.print();
        printf( "\n" );
	return false;
    } 
    printf( "  passed %s testing\n", test_name.c_str() );
    return true;
}

bool report( std::string test_name, Vector3 u, Vector3 v, double experimental, double expected ) {
    double EPSILON = 1e-5;
    if( expected + EPSILON < experimental || expected - EPSILON > experimental ) {
        printf( "  failed %s testing\n", test_name.c_str() );
        printf( "u: " );
        u.print( );
        printf( "v: " );
        v.print( );
        printf( "Expected: %f\n", expected );
        printf( "Experimental: %f\n", experimental );
        printf( "\n" );
	return false;
    }
    printf( "  passed %s testing\n", test_name.c_str() );
    return true;
}

bool report( std::string test_name, Vector3 u, double c, Vector3 experimental, Vector3 expected ) {
    if( !Vector3::equal( expected, experimental ) ) {
        printf( "  failed %s testing\n", test_name.c_str() );
        printf( "u: " );
        u.print( );
        printf( "c: %f\n", c );
        printf( "Expected: " );
        expected.print();
        printf( "Experimental: " );
        experimental.print();
        printf( "\n" );
	return false;
    }
    printf( "  passed %s testing\n", test_name.c_str() );
    return true;
}

int main( void ) {

    printf( "Unit testing class Vector3\n" );

    Vector3 i = Vector3( 1, 0, 0);
    Vector3 j = Vector3( 0, 1, 0);
    Vector3 k = Vector3( 0, 0, 1);

    Vector3 experimental, expected;
    double fexperimental, fexpected, c;

    Vector3 u = Vector3( 0.066336, 0.623016, 0.769146 );
    Vector3 v = Vector3( 0.221833, 0.092967, 0.929076 );
    c = 2;

    int fails = 0;

    experimental = Vector3::invert( u );
    expected = Vector3( -u.x, -u.y, -u.z );
    if( !report( std::string("inversion"), u, v, experimental, expected ) ) fails++;

    experimental = Vector3::add( u, v );
    expected = Vector3( 0.28817, 0.71598, 1.69822 );
    if( !report( std::string("addition"), u, v, experimental, expected ) ) fails++;

    experimental = Vector3::subtract( u, v );
    expected = Vector3( -0.15550, 0.53005, -0.15993 );
    if( !report( std::string("subtraction"), u, v, experimental, expected ) ) fails++;

    experimental = Vector3::multiply( u, c );
    expected = Vector3( 0.13267, 1.24603, 1.53829 );
    if( !report( std::string("scalar multiplication"), u, c, experimental, expected ) ) fails++;

    experimental = Vector3::cross( u, v );
    expected = Vector3( 0.50732, 0.10899, -0.13204 );
    if( !report( std::string("cross-product"), u, v, experimental, expected ) ) fails++;

    fexperimental = Vector3::dot( u, v );
    fexpected = 0.78723;
    if( !report( std::string("dot-product"), u, v, fexperimental, fexpected ) ) fails++;

    //Outstanding Tests
    //static Vector3 multiply( const Vector3&, const Vector3& );
    //static Vector3 normalize( const Vector3& );

    if( fails ) 
    	printf( "Vector3 failed %d unit tests\n", fails );
    else 
    	printf( "Vector3 passed unit testing\n" );


    return 0;
}
*/

int main( void ) {
    printf( "Unit testing class Quaternion\n" );

    Eigen::Quaterniond eqx, eqy, eqz;
    Quaternion qx, qy, qz;

    eqy = Eigen::Quaterniond( 0.0, 0.0, 1.0, 0.0 );
    eqz = Eigen::Quaterniond( 0.0, 0.0, 0.0, 1.0 );
    eqx = eqy * eqz;
    printf( "w:%f x:%f y:%f z:%f\n", eqx.w(), eqx.x(), eqx.y(), eqx.z() );

    qy = Quaternion( 0.0, 0.0, 1.0, 0.0 );
    qz = Quaternion( 0.0, 0.0, 0.0, 1.0 );

    qx = Quaternion::multiply( qy, qz );
    printf( "w:%f x:%f y:%f z:%f\n", qx.w(), qx.x(), qx.y(), qx.z() );

    qx = qy * qz;
    printf( "w:%f x:%f y:%f z:%f\n", qx.w(), qx.x(), qx.y(), qx.z() );

    Eigen::Matrix3d emat = eqx.toRotationMatrix();
    Matrix3 mat = qx.matrix3();



    return 0;
}
