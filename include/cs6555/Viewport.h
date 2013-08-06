#ifndef _VIEWPORT_H_
#define _VIEWPORT_H_

class Viewport {
public:
    Viewport( const int& width, const int& height );
    virtual ~Viewport( void );

    int width( void );
    int height( void );
private:
    int m_iwidth;
    int m_iheight;
}

#endif // _VIEWPORT_H_
