/*-----------------------THE GEORGE WASHINGTON UNIVERSITY-----------------------
James Taylor                                                         jrt@gwu.edu

Scene class definition

------------------------------------------------------------------------------*/

#ifndef _SCENE_H
#define _SCENE_H

//------------------------------------------------------------------------------

#include <cs6555/Geometry.h>
#include <cs6555/Camera.h>

//------------------------------------------------------------------------------

class Scene : public Geometry {
public:
    // Constructor
    Scene( void );

    // Destructor
    virtual ~Scene( void );

    // Member Data
    Camera camera;

    // Material Insertion and Queries
    int appendMaterial( Material* m );
    unsigned int materialCount( void );
    Material* material( const unsigned int& i );

    // Subdivision

protected:

    unsigned int m_materialCount;

    std::vector<Material*> m_materials;

};

//------------------------------------------------------------------------------

#endif // _SCENE_H
