#include <cs6555/Math/MatrixN.h>

#include <assert.h>

//------------------------------------------------------------------------------

MatrixN( const unsigned int& ROWS, const unsigned int& COLS ) {
    assert( ROWS > 0 && COLS > 0 );

    m_rows = ROWS;
    m_columns = COLS;

    A = Eigen::MatrixX(m_rows, m_cols);
}

//------------------------------------------------------------------------------

MatrixN::~MatrixN( void ) {

}

//------------------------------------------------------------------------------
