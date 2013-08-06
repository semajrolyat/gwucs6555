#include <cs6555/Viewport.h>

Viewport::Viewport( const int& width, const int& height ) {
    m_iwidth = width;
    m_iheight = height;
}

Viewport::~Viewport( void ) {

}

int Viewport::width( void ) {
    return m_iwidth;
}

int Viewport::height( void ) {
    return m_iheight;
}
