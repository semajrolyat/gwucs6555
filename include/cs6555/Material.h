/*-----------------------THE GEORGE WASHINGTON UNIVERSITY-----------------------
James Taylor                                                         jrt@gwu.edu

Material class definition and implementation

Datastructure for rendering materials on geometry
------------------------------------------------------------------------------*/

#ifndef _MATERIAL_H_
#define _MATERIAL_H_

//------------------------------------------------------------------------------

#include <cs6555/Color.h>

//------------------------------------------------------------------------------

class Material {
public:

    // Constructors
    /// Default Constructor
    Material( const bool& random = false ) {
        ambient = Color( 0.0, 0.0, 0.0, 1.0 );
        diffuse = Color( 0.0, 0.0, 0.0, 1.0 );
        specular = Color( 0.0, 0.0, 0.0, 1.0 );
        emissive = Color( 0.0, 0.0, 0.0, 1.0 );
        shininess = 10.0;

        if( random ) randomize();
    }
    /// Copy Constructor
    Material( const Material& material ) {
        ambient = Color( material.ambient );
        diffuse = Color( material.diffuse );
        specular = Color( material.specular );
        emissive = Color( material.emissive );
        shininess = material.shininess;
    }
    /// Component Constructor
    Material( const Color& amb, const Color& dif, const Color& spec, const Color& emis, const double& shine ) {
        ambient = Color( amb );
        diffuse = Color( dif );
        specular = Color( spec );
        emissive = Color( emis );
        shininess = shine;
    }

    // Destructor
    /// Destructor
    ~Material( void ) { }

    void randomize( void ) {
        ambient = Color( (double)rand() / (double)RAND_MAX, (double)rand() / (double)RAND_MAX, (double)rand() / (double)RAND_MAX, 1.0 );
        diffuse = Color( ambient.r(), ambient.g(), ambient.b(), 1.0 );
        specular = Color( 0.0, 0.0, 0.0, 1.0 );
    }

    // Member Data
    Color ambient;          // Ambient color coefficients
    Color diffuse;          // Diffuse color coefficients
    Color specular;         // Specular color coefficients
    Color emissive;         // Emissive color coefficients
    double shininess;       // Shininess coefficient
};

//------------------------------------------------------------------------------

#endif // _MATERIAL_H_
