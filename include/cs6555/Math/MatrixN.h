#ifndef _MATRIXN_H_
#define _MATRIXN_H_

#include <eigen3/Eigen/Dense>

//------------------------------------------------------------------------------

class MatrixN {
public:
    // Constructors
    MatrixN( const unsigned int& ROWS, const unsigned int& COLS );
private:
    // Adapter Copy Constructor (private)
    MatrixN( const Eigen::MatrixXd& );
public:
    // Desctructor
    virtual ~MatrixN( void );

    // Dimensional Queries
    unsigned int rows( void ) const;
    unsigned int columns( void ) const;

    // Inplace Initialization Functions
    void zero( void );
    //void identity( void );

    // Matrix Comparison Functions
    static bool equal( const MatrixN&, const MatrixN& );

    // Matrix Addition Functions
    static MatrixN add( const MatrixN& A, const MatrixN& B );

    // Matrix Multiplication Functions
    static MatrixN multiply( const MatrixN& A, const MatrixN& B );
    static MatrixN multiply( const MatrixN& A, const double& c );
    static Vector4 multiply( const MatrixN& A, const Vector4& x );
    static Vector4 multiply( const Vector4& x, const MatrixN& A );

    // Matrix Transposition Functions
    static MatrixN transpose( MatrixN& A );
    void transpose( void );

    // Matrix Inversion Functions
    static MatrixN inverse( MatrixN& A );

    // Operators
    double& operator() (const unsigned int& ROW, const unsigned int& COL );

    bool operator==( const MatrixN& );
    bool operator!=( const MatrixN& );

    MatrixN operator+=( const MatrixN& );
    MatrixN operator+( const MatrixN& );

    MatrixN operator*=( const MatrixN& );
    MatrixN operator*( const MatrixN& );

    MatrixN operator*=( const double& c );
    MatrixN operator*( const double& c );

    MatrixN operator*( const Vector4& x );


private:
    unsigned int m_rows;
    unsigned int m_columns;
    Eigen::MatrixXd A;
};

// Out of class 'friendly' operators
MatrixN operator*( const double& c, const MatrixN& A );
//Vector4 operator*( Vector4& x, const MatrixN& A );

//------------------------------------------------------------------------------

#endif // _MATRIXN_H_
