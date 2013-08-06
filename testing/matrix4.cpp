#include <stdio.h>
#include <string>
#include <cs6555/Math/Matrix4.h>
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


int main( void ) {

    printf( "Unit testing class Matrix4\n" );
    return 0;
}
*/
#include <stdio.h>
#include <string>

void report( std::string test_name, Matrix4 experimental, Matrix4 expected ) {
    if( !Matrix4::equal( expected, experimental ) ) {
        printf( "Matrix4 failed %s test\n", test_name.c_str() );
        printf( "Expected:\n" );
        expected.print();
        printf( "Experimental:\n" );
        experimental.print();
        printf( "\n" );
    } else {
      printf( "Matrix4 passed %s test\n", test_name.c_str() );
    }
}

void report( std::string test_name, Matrix4 A, Matrix4 experimental, Matrix4 expected ) {
    if( !Matrix4::equal( expected, experimental ) ) {
        printf( "Matrix4 failed %s test\n", test_name.c_str() );
        printf( "A:\n" );
        A.print( );
        printf( "Expected:\n" );
        expected.print();
        printf( "Experimental:\n" );
        experimental.print();
        printf( "\n" );
    } else {
      printf( "Matrix4 passed %s test\n", test_name.c_str() );
    }
}

void report( std::string test_name, Matrix4 A, Matrix4 B, Matrix4 experimental, Matrix4 expected ) {
    if( !Matrix4::equal( expected, experimental ) ) {
        printf( "Matrix4 failed %s test\n", test_name.c_str() );
        printf( "A:\n" );
        A.print( );
        printf( "B:\n" );
        B.print( );
        printf( "Expected:\n" );
        expected.print();
        printf( "Experimental:\n" );
        experimental.print();
        printf( "\n" );
    } else {
      printf( "Matrix4 passed %s test\n", test_name.c_str() );
    }
}

void report( std::string test_name, Matrix4 A, float c, Matrix4 experimental, Matrix4 expected ) {
    if( !Matrix4::equal( expected, experimental ) ) {
        printf( "Matrix4 failed %s test\n", test_name.c_str() );
        printf( "A:\n" );
        A.print( );
        printf( "c: %f\n", c );
        printf( "Expected:\n" );
        expected.print();
        printf( "Experimental:\n" );
        experimental.print();
        printf( "\n" );
    } else {
      printf( "Matrix4 passed %s test\n", test_name.c_str() );
    }
}

void report( std::string test_name, Matrix4 A, Vector4 x, Vector4 experimental, Vector4 expected ) {
    if( !Vector4::equal( expected, experimental ) ) {
        printf( "Matrix4 failed %s test\n", test_name.c_str() );
        printf( "A:\n" );
        A.print( );
        printf( "x: " );
        x.print( );
        printf( "Expected:\n" );
        expected.print();
        printf( "Experimental:\n" );
        experimental.print();
        printf( "\n" );
    } else {
      printf( "Matrix4 passed %s test\n", test_name.c_str() );
    }
}

int main( void ) {

    printf( "Unit testing class Matrix4\n" );

    Matrix4 A;
    Matrix4 B;
    Matrix4 experimental;
    Matrix4 C;
    float c;

    A.zero();

    C(0,0) = 0;   C(0,1) = 0;   C(0,2) = 0;   C(0,3) = 0;
    C(1,0) = 0;   C(1,1) = 0;   C(1,2) = 0;   C(1,3) = 0;
    C(2,0) = 0;   C(2,1) = 0;   C(2,2) = 0;   C(2,3) = 0;
    C(3,0) = 0;   C(3,1) = 0;   C(3,2) = 0;   C(3,3) = 0;

    report( std::string("zero matrix"), A, C );

    A.identity();

    C(0,0) = 1;   C(0,1) = 0;   C(0,2) = 0;   C(0,3) = 0;
    C(1,0) = 0;   C(1,1) = 1;   C(1,2) = 0;   C(1,3) = 0;
    C(2,0) = 0;   C(2,1) = 0;   C(2,2) = 1;   C(2,3) = 0;
    C(3,0) = 0;   C(3,1) = 0;   C(3,2) = 0;   C(3,3) = 1;

    report( std::string("identity matrix"), A, C );

    // A and B matrices generated by random number generation in Octave
    // expected result matrix (e.g. C) then calculated by running corresponding
    // Octave function on A and B

    A(0,0) = 0.713641;  A(0,1) = 0.923981;  A(0,2) = 0.199143;  A(0,3) = 0.762288;
    A(1,0) = 0.744559;  A(1,1) = 0.405468;  A(1,2) = 0.552963;  A(1,3) = 0.497023;
    A(2,0) = 0.794030;  A(2,1) = 0.787646;  A(2,2) = 0.144648;  A(2,3) = 0.255559;
    A(3,0) = 0.046210;  A(3,1) = 0.418579;  A(3,2) = 0.950220;  A(3,3) = 0.704161;

    B(0,0) = 0.403320;  B(0,1) = 0.338039;  B(0,2) = 0.351747;  B(0,3) = 0.170443;
    B(1,0) = 0.437334;  B(1,1) = 0.880311;  B(1,2) = 0.972666;  B(1,3) = 0.940108;
    B(2,0) = 0.497617;  B(2,1) = 0.585278;  B(2,2) = 0.434827;  B(2,3) = 0.933336;
    B(3,0) = 0.612455;  B(3,1) = 0.023317;  B(3,2) = 0.884795;  B(3,3) = 0.596342;

    experimental = Matrix4::multiply( A, B );

    C(0,0) = 1.25788;   C(0,1) = 1.18896;   C(0,2) = 1.91081;   C(0,3) = 1.63073;
    C(1,0) = 1.05719;   C(1,1) = 0.94385;   C(1,2) = 1.33649;   C(1,3) = 1.32058;
    C(2,0) = 0.89321;   C(2,1) = 1.05240;   C(2,2) = 1.33443;   C(2,3) = 1.16321;
    C(3,0) = 1.10581;   C(3,1) = 0.95666;   C(3,2) = 1.45961;   C(3,3) = 1.70818;

    report( std::string("matrix-matrix multiplication"), A, B, experimental, C );

    // matrix x matrix by operator

    experimental = A * B;

    report( std::string("matrix-matrix operator multiplication"), A, B, experimental, C );

    c = 2;
    experimental = Matrix4::multiply( A, c );

    C(0,0) = 1.427282;   C(0,1) = 1.847961;   C(0,2) = 0.398287;   C(0,3) = 1.524576;
    C(1,0) = 1.489118;   C(1,1) = 0.810937;   C(1,2) = 1.105925;   C(1,3) = 0.994046;
    C(2,0) = 1.588059;   C(2,1) = 1.575291;   C(2,2) = 0.289296;   C(2,3) = 0.511119;
    C(3,0) = 0.092420;   C(3,1) = 0.837159;   C(3,2) = 1.900439;   C(3,3) = 1.408323;

    report( std::string("matrix-constant multiplication"), A, c, experimental, C );

    experimental = A * c;

    report( std::string("matrix-constant operator multiplication"), A, c, experimental, C );

    experimental = c * A;

    report( std::string("constant-matrix operator multiplication"), A, c, experimental, C );

    // b = Ax
    Vector4 x;  // therefore input
    Vector4 b;  // and experimental result
    Vector4 bexpected;

    x = Vector4( 0.37712, 0.25053, 0.53328, 0.71829 );
    bexpected = Vector4( 1.15435, 1.03426, 0.75747, 1.13482 );

    b = Matrix4::multiply( A, x );

    report( std::string("matrix-vector multiplication"), A, x, b, bexpected );

    // matrix-vector operator multiplication

    b = A * x;

    report( std::string("matrix-vector operator multiplication"), A, x, b, bexpected );

    // b = xA
    bexpected = Vector4( 0.91230, 1.17073, 0.97331, 1.05407 );

    b = Matrix4::multiply( x, A );

    report( std::string("vector-matrix multiplication"), A, x, b, bexpected );

    // vector-matrix operator multiplication
    b = x * A;

    report( std::string("vector-matrix operator multiplication"), A, x, b, bexpected );

    experimental = Matrix4::transpose( A );

    C(0,0) = 0.713641;  C(1,0) = 0.923981;  C(2,0) = 0.199143;  C(3,0) = 0.762288;
    C(0,1) = 0.744559;  C(1,1) = 0.405468;  C(2,1) = 0.552963;  C(3,1) = 0.497023;
    C(0,2) = 0.794030;  C(1,2) = 0.787646;  C(2,2) = 0.144648;  C(3,2) = 0.255559;
    C(0,3) = 0.046210;  C(1,3) = 0.418579;  C(2,3) = 0.950220;  C(3,3) = 0.704161;

    report( std::string("transpose"), A, experimental, C );
/*
    // test pseudoinverse
    Vector3 U = Vector3( -5.7447e-01, 2.7756e-17, 8.1853e-01 );
    Vector3 V = Vector3( -0.49865, 0.79301, -0.34997 );
    Vector3 N = Vector3( 0.64910, 0.60920, 0.45556 );

    Matrix3 Mrot;
    Mrot.row( 1, U );
    Mrot.row( 2, V );
    Mrot.row( 3, N );

    Vector3 Vtrans = Vector3( 0.12286, 0.56437, 0.57331 );
    Matrix3 MrotT;
    Matrix3 negMrotT;
    Vector3 Vtransprime = Vector3( -0.020136, -0.796810, -0.164228 );

    MrotT(1,1) = -5.7447e-01;   MrotT(1,2) = 2.7756e-17;    MrotT(1,3) = 8.1853e-01;
    MrotT(2,1) = -0.49865;      MrotT(2,2) = 0.79301;       MrotT(2,3) = -0.34997;
    MrotT(3,1) = 0.64910;       MrotT(3,2) = 0.60920;       MrotT(3,3) = 0.45556;

    negMrotT(1,1) = 5.7447e-01;   negMrotT(1,2) = -2.7756e-17;    negMrotT(1,3) = -8.1853e-01;
    negMrotT(2,1) = 0.49865;      negMrotT(2,2) = -0.79301;       negMrotT(2,3) = 0.34997;
    negMrotT(3,1) = -0.64910;     negMrotT(3,2) = -0.60920;       negMrotT(3,3) = -0.45556;

    Matrix4 P;
    P(1,1) = -0.57447; P(1,2) = 0.0f;     P(1,3) = 0.81853;  P(1,4) = 0.12286;
    P(2,1) = -0.49865; P(2,2) = 0.79301;  P(2,3) = -0.34997; P(2,4) = 0.56437;
    P(3,1) = 0.64910;  P(3,2) = 0.60920;  P(3,3) = 0.45556;  P(3,4) = 0.57331;
    P(4,1) = 0.0f;     P(4,2) = 0.0f;     P(4,3) = 0.0f;     P(4,4) = 1.0f;

    Matrix4 PI;

    PI(1,1) = -0.57447; PI(1,2) = -0.49865; PI(1,3) = 0.64910;  PI(1,4) = -0.02013;
    PI(2,1) = 0.0f;     PI(2,2) = 0.79301;  PI(2,3) = 0.60920;  PI(2,4) = -0.79681;
    PI(3,1) = 0.81853;  PI(3,2) = -0.34997; PI(3,3) = 0.45556;  PI(3,4) = -0.16423;
    PI(4,1) = 0.0f;     PI(4,2) = 0.0f;     PI(4,3) = 0.0f;     PI(4,4) = 1.0f;

    experimental = Matrix4::pseudoInverse( P );
    report( std::string("pesudoinverse"), P, experimental, PI );
*/
    // test determinant
    // test inverse
    A(0,0) = 0.447129;  A(0,1) = 0.200975;  A(0,2) = 0.589304;  A(0,3) = 0.150047;
    A(1,0) = 0.430449;  A(1,1) = 0.411979;  A(1,2) = 0.620010;  A(1,3) = 0.775876;
    A(2,0) = 0.090493;  A(2,1) = 0.818122;  A(2,2) = 0.818219;  A(2,3) = 0.113565;
    A(3,0) = 0.801656;  A(3,1) = 0.206533;  A(3,2) = 0.825334;  A(3,3) = 0.843162;

    /*
    // Octave:: A=rand(4,4)
    0.447129   0.200975   0.589304   0.150047
    0.430449   0.411979   0.620010   0.775876
    0.090493   0.818122   0.818219   0.113565
    0.801656   0.206533   0.825334   0.843162
    */
    // det = 0.0098637
    /*
    // Octave:: C=inverse(A)
    22.8348   19.9634  -10.3509  -21.0397
    18.7776   19.0633   -7.9898  -19.8075
   -20.4239  -20.8142   10.0889   21.4289
    -6.3182   -3.2761    1.9229    5.0661
    */

    //float det = Matrix4::determinant(A);

    C(0,0) = 22.8348;  C(0,1) = 19.9634;  C(0,2) = -10.3509;  C(0,3) = -21.0397;
    C(1,0) = 18.7776;  C(1,1) = 19.0633;  C(1,2) = -7.9898;   C(1,3) = -19.8075;
    C(2,0) = -20.4239; C(2,1) = -20.8142; C(2,2) = 10.0889;   C(2,3) = 21.4289;
    C(3,0) = -6.3182;  C(3,1) = -3.2761;  C(3,2) = 1.9229;    C(3,3) = 5.0661;

    //experimental = Matrix4::inverse( A );
    //report( std::string("inverse"), A, experimental, C );

    //Matrix4 CA = Matrix4::multiply( C, A );
    //Matrix4 Ica;
    //Ica.identity( );

    //report( std::string("inverse-expected-identity"), CA, Ica );

    Matrix4 Ainv = Matrix4::inverse( A );
    experimental = Matrix4::multiply( Ainv, A );

    Matrix4 I;
    I.identity();

    report( std::string("inverse"), A, experimental, I );



    return 0;
}
