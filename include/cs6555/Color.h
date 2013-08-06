/*-----------------------THE GEORGE WASHINGTON UNIVERSITY-----------------------
James Taylor                                                         jrt@gwu.edu

Color class definition & implementation

A color contains the four channels red, green, blue and alpha as single
precision floats.
------------------------------------------------------------------------------*/

#ifndef _COLOR_H_
#define _COLOR_H_

//------------------------------------------------------------------------------

class Color {
public:
    // Constructors
    /// Default Constructor
    Color( void ){
        r( 0.0 );
        g( 0.0 );
        b( 0.0 );
        a( 1.0 );
    }

    /// Copy Constructor
    Color( const Color& color ) {
        r( color.r() );
        g( color.g() );
        b( color.b() );
        a( color.a() );
    }

    /// Component Constructor
    Color( const double& red, const double& green, const double& blue, const double& alpha ) {
        r( red );
        g( green );
        b( blue );
        a( alpha );
    }

    // Destructor
    /// Destructor
    virtual ~Color( void ) { }

    // Interface Utilities
    /// Exposes an OpenGL compatible array
    const float* arrayOpenGL( void ) const {
        return m_data;
    }

    // Component Get/Set
    /// Get Red Channel
    float r( void ) const { return m_data[0]; }

    /// Get Green Channel
    float g( void ) const { return m_data[1]; }

    /// Get Blue Channel
    float b( void ) const { return m_data[2]; }

    /// Get Alpha Channel
    float a( void ) const { return m_data[3]; }

    /// Set Red Channel
    void r( const float& value ) { m_data[0] = value; }

    /// Set Green Channel
    void g( const float& value ) { m_data[1] = value; }

    /// Set Blue Channel
    void b( const float& value ) { m_data[2] = value; }

    /// Set Alpha Channel
    void a( const float& value ) { m_data[3] = value; }

    static Color white( void ) { return Color( 1.0, 1.0, 1.0, 1.0 ); }
    static Color black( void ) { return Color( 0.0, 0.0, 0.0, 1.0 ); }
    //static Color magenta( void ) { return Color( 1.0, 1.0, 1.0, 1.0 ); }
    static Color red( void ) { return Color( 1.0, 0.0, 0.0, 1.0 ); }
    static Color orange( void ) { return Color( 1.0, 0.6, 0.0, 1.0 ); }
    static Color yellow( void ) { return Color( 1.0, 1.0, 0.0, 1.0 ); }
    static Color green( void ) { return Color( 0.0, 1.0, 0.0, 1.0 ); }
    static Color blue( void ) { return Color( 0.0, 0.0, 1.0, 1.0 ); }
    static Color cyan( void ) { return Color( 0.0, 1.0, 1.0, 1.0 ); }
    //static Color indigo( void ) { return Color( 1.0, 1.0, 1.0, 1.0 ); }
    //static Color violet( void ) { return Color( 1.0, 1.0, 1.0, 1.0 ); }

private:
    // Member Data
    float m_data[4];        // the array of color channels
};

//------------------------------------------------------------------------------

#endif // _COLOR_H_
