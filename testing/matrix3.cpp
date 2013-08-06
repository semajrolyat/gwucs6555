/********************** THE GEORGE WASHINGTON UNIVERSITY ***********************
* James Taylor                                                     jrt@gwu.edu *
*                                                                              *
* A Test Program to validate the Matrix3 class member data and functions       *
*******************************************************************************/

#include <cs6555/Math/Vector3.h>
#include <cs6555/Math/Matrix3.h>

#include <stdio.h>
#include <string>

void report( std::string test_name, Matrix3 experimental, Matrix3 expected ) {
    if( !Matrix3::equal( expected, experimental ) ) {
        printf( "Matrix3 failed %s test\n", test_name.c_str() );
        printf( "Expected:\n" );
        expected.print();
        printf( "\n" );
        printf( "Experimental:\n" );
        experimental.print();
        printf( "\n" );
    } else {
      printf( "Matrix3 passed %s test\n", test_name.c_str() );
    }
}

void report( std::string test_name, Matrix3 A, Matrix3 experimental, Matrix3 expected ) {
    if( !Matrix3::equal( expected, experimental ) ) {
        printf( "Matrix3 failed %s test\n", test_name.c_str() );
        printf( "A:\n" );
        A.print( );
        printf( "\n" );
        printf( "Expected:\n" );
        expected.print();
        printf( "\n" );
        printf( "Experimental:\n" );
        experimental.print();
        printf( "\n" );
    } else {
      printf( "Matrix3 passed %s test\n", test_name.c_str() );
    }
}

void report( std::string test_name, Matrix3 A, Matrix3 B, Matrix3 experimental, Matrix3 expected ) {
    if( !Matrix3::equal( expected, experimental ) ) {
        printf( "Matrix3 failed %s test\n", test_name.c_str() );
        printf( "A:\n" );
        A.print( );
        printf( "\n" );
        printf( "B:\n" );
        B.print( );
        printf( "\n" );
        printf( "Expected:\n" );
        expected.print();
        printf( "\n" );
        printf( "Experimental:\n" );
        experimental.print();
        printf( "\n" );
    } else {
      printf( "Matrix3 passed %s test\n", test_name.c_str() );
    }
}

void report( std::string test_name, Matrix3 A, float c, Matrix3 experimental, Matrix3 expected ) {
    if( !Matrix3::equal( expected, experimental ) ) {
        printf( "Matrix3 failed %s test\n", test_name.c_str() );
        printf( "A:\n" );
        A.print( );
        printf( "\n" );
        printf( "c: %f\n", c );
        printf( "Expected:\n" );
        expected.print();
        printf( "\n" );
        printf( "Experimental:\n" );
        experimental.print();
        printf( "\n" );
    } else {
      printf( "Matrix3 passed %s test\n", test_name.c_str() );
    }
}

void report( std::string test_name, Matrix3 A, Vector3 x, Vector3 experimental, Vector3 expected ) {
    if( !Vector3::equal( expected, experimental ) ) {
        printf( "Matrix3 failed %s test\n", test_name.c_str() );
        printf( "A:\n" );
        A.print( );
        printf( "\n" );
        printf( "x: " );
        x.print( );
        printf( "\n" );
        printf( "Expected:\n" );
        expected.print();
        printf( "\n" );
        printf( "Experimental:\n" );
        experimental.print();
        printf( "\n" );
    } else {
      printf( "Matrix3 passed %s test\n", test_name.c_str() );
    }
}

int main( void ) {

    Matrix3 A;
    Matrix3 B;
    Matrix3 experimental;
    Matrix3 C;
    float c;

    // zero
    A.zero();

    C(0,0) = 0;   C(0,1) = 0;   C(0,2) = 0;
    C(1,0) = 0;   C(1,1) = 0;   C(1,2) = 0;
    C(2,0) = 0;   C(2,1) = 0;   C(2,2) = 0;

    report( std::string("zero matrix"), A, C );

    // identity
    A.identity();

    C(0,0) = 1;   C(0,1) = 0;   C(0,2) = 0;
    C(1,0) = 0;   C(1,1) = 1;   C(1,2) = 0;
    C(2,0) = 0;   C(2,1) = 0;   C(2,2) = 1;

    report( std::string("identity matrix"), A, C );

    // A and B matrices generated by random number generation in Octave
    // expected result matrix (e.g. C) then calculated by running corresponding
    // Octave function on A and B

    // matrix x matrix
    A(0,0) = 0.414941;  A(0,1) = 0.118973;  A(0,2) = 0.569040;
    A(1,0) = 0.076377;  A(1,1) = 0.303944;  A(1,2) = 0.759433;
    A(2,0) = 0.621115;  A(2,1) = 0.808128;  A(2,2) = 0.018372;

    B(0,0) = 0.098720;  B(0,1) = 0.972411;  B(0,2) = 0.846574;
    B(1,0) = 0.738911;  B(1,1) = 0.380881;  B(1,2) = 0.397504;
    B(2,0) = 0.593094;  B(2,1) = 0.288630;  B(2,2) = 0.353810;

    experimental = Matrix3::multiply( A, B );

    C(0,0) = 0.46637;   C(0,1) = 0.61305;   C(0,2) = 0.59990;
    C(1,0) = 0.68254;   C(1,1) = 0.40923;   C(1,2) = 0.45417;
    C(2,0) = 0.66935;   C(2,1) = 0.91708;   C(2,2) = 0.85355;

    report( std::string("matrix-matrix multiplication"), A, B, experimental, C );

    // matrix x matrix by operator

    experimental = A * B;

    report( std::string("matrix-matrix operator multiplication"), A, B, experimental, C );

    // matrix x constant
    c = 2;
    experimental = Matrix3::multiply( A, c );

    C(0,0) = 0.829881;   C(0,1) = 0.237946;   C(0,2) = 1.138081;
    C(1,0) = 0.152754;   C(1,1) = 0.607888;   C(1,2) = 1.518867;
    C(2,0) = 1.242229;   C(2,1) = 1.616255;   C(2,2) = 0.036743;

    report( std::string("matrix-constant multiplication"), A, c, experimental, C );

    experimental = A * c;

    report( std::string("matrix-constant operator multiplication"), A, c, experimental, C );

    experimental = c * A;

    report( std::string("constant-matrix operator multiplication"), A, c, experimental, C );

    // b = Ax
    Vector3 x;  // therefore input
    Vector3 b;  // and experimental result
    Vector3 bexpected;

    x = Vector3( 0.697905, 0.292496, 0.045169 );
    bexpected = Vector3( 0.35009, 0.17651, 0.67068 );

    b = Matrix3::multiply( A, x );

    report( std::string("matrix-vector multiplication"), A, x, b, bexpected );

    // matrix-vector operator multiplication

    b = A * x;

    report( std::string("matrix-vector operator multiplication"), A, x, b, bexpected );

    // b = xA
    bexpected = Vector3( 0.33998, 0.20844, 0.62010 );

    b = Matrix3::multiply( x, A );

    report( std::string("vector-matrix multiplication"), A, x, b, bexpected );

    // vector-matrix operator multiplication
    b = x * A;

    report( std::string("vector-matrix operator multiplication"), A, x, b, bexpected );

    // transpose
    experimental = Matrix3::transpose( A );

    C(0,0) = 0.414941;  C(1,0) = 0.118973;  C(2,0) = 0.569040;
    C(0,1) = 0.076377;  C(1,1) = 0.303944;  C(2,1) = 0.759433;
    C(0,2) = 0.621115;  C(1,2) = 0.808128;  C(2,2) = 0.018372;

    report( std::string("transpose"), A, experimental, C );

    // inversion
/*
    // Not really a valid test mostly because of inaccuracies in fp math
    // and variability in implementations of matrix inversion algorithms
    // Can't verify the test case inversion verses the result inversion
    experimental = Matrix3::inverse( A );

    C(0,0) =  2.26332;  C(1,0) = -1.70334;  C(2,0) =  0.30743;
    C(0,1) = -1.75031;  C(1,1) =  1.28704;  C(2,1) =  1.01104;
    C(0,2) =  0.47289;  C(1,2) =  0.97297;  C(2,2) = -0.43556;

    report( std::string("inverse"), A, experimental, C );
*/
    // Test based on I = inverse(A) * A
    Matrix3 Ainv = Matrix3::inverse( A );
    experimental = Matrix3::multiply( Ainv, A );

    Matrix3 I;
    I.identity();

    report( std::string("inverse"), A, experimental, I );

    return 0;
}

