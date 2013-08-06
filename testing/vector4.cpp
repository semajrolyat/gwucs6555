#include <stdio.h>
#include <string>
#include <cs6555/Math/Vector4.h>

bool report( std::string test_name, Vector4 u, Vector4 v, Vector4 experimental, Vector4 expected ) {
    if( !Vector4::equal( expected, experimental ) ) {
        printf( "  failed %s testing\n", test_name.c_str() );
        printf( "  u: " );
        u.print( );
        printf( "\n" );
        printf( "  v: " );
        v.print( );
        printf( "\n" );
        printf( "  Expected: " );
        expected.print();
        printf( "\n" );
        printf( "  Experimental: " );
        experimental.print();
        printf( "\n\n" );
    return false;
    }
    printf( "  passed %s testing\n", test_name.c_str() );
    return true;
}

bool report( std::string test_name, Vector4 u, Vector4 v, double experimental, double expected ) {
    double EPSILON = 1e-4;
    if( expected + EPSILON < experimental || expected - EPSILON > experimental ) {
        printf( "  failed %s testing\n", test_name.c_str() );
        printf( "  u: " );
        u.print( );
        printf( "\n" );
        printf( "  v: " );
        v.print( );
        printf( "\n" );
        printf( "  Expected: %f\n", expected );
        printf( "  Experimental: %f\n", experimental );
        printf( "\n\n" );
    return false;
    }
    printf( "  passed %s testing\n", test_name.c_str() );
    return true;
}

bool report( std::string test_name, Vector4 u, double c, Vector4 experimental, Vector4 expected ) {
    if( !Vector4::equal( expected, experimental ) ) {
        printf( "  failed %s testing\n", test_name.c_str() );
        printf( "  u: " );
        u.print( );
        printf( "\n" );
        printf( "  c: %f\n", c );
        printf( "  Expected: " );
        expected.print();
        printf( "\n" );
        printf( "  Experimental: " );
        experimental.print();
        printf( "\n\n" );
    return false;
    }
    printf( "  passed %s testing\n", test_name.c_str() );
    return true;
}

int main( void ) {

    printf( "Unit testing class Vector4\n" );

    Vector4 experimental, expected;
    double dexperimental, dexpected, c;

    Vector4 u = Vector4( 0.95471, 0.80330, 0.63371, 0.36202 );
    Vector4 v = Vector4( 0.7483311, 0.2116794, 0.8661326, 0.0021241 );
    c = 2;

    int fails = 0;

    experimental = Vector4::invert( u );
    expected = Vector4( -u(0), -u(1), -u(2), -u(3) );
    if( !report( std::string("inversion"), u, v, experimental, expected ) ) fails++;

    experimental = Vector4::add( u, v );
    expected = Vector4( 1.70304, 1.01498, 1.49984, 0.36414 );
    if( !report( std::string("addition"), u, v, experimental, expected ) ) fails++;

    experimental = Vector4::subtract( u, v );
    expected = Vector4( 0.20638, 0.59162, -0.23243, 0.35989 );
    if( !report( std::string("subtraction"), u, v, experimental, expected ) ) fails++;

    experimental = Vector4::multiply( u, c );
    expected = Vector4( 1.90942, 1.60660, 1.26741, 0.72404 );
    if( !report( std::string("scalar multiplication"), u, c, experimental, expected ) ) fails++;

    dexperimental = Vector4::dot( u, v );
    dexpected = 1.4341;
    if( !report( std::string("dot-product"), u, v, dexperimental, dexpected ) ) fails++;

    //Outstanding Tests
    //static Vector4 normalize( const Vector4& );

    if( fails )
        printf( "Vector4 failed %d unit tests\n", fails );
    else
        printf( "Vector4 passed unit testing\n" );

    return 0;
}
