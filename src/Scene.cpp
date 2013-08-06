/*-----------------------THE GEORGE WASHINGTON UNIVERSITY-----------------------
James Taylor                                                         jrt@gwu.edu

Scene class implementation

------------------------------------------------------------------------------*/

#include <cs6555/Scene.h>


//------------------------------------------------------------------------------
// Constructor
//------------------------------------------------------------------------------
/// Default Constructor
Scene::Scene( void ) {
    m_materialCount = 0;
}

//------------------------------------------------------------------------------
// Destructor
//------------------------------------------------------------------------------
/// Destructor
Scene::~Scene( void ) {
    Material* m;
    while( !m_materials.empty( ) ) {
        m = *m_materials.end( );
        m_materials.pop_back( );
        //if( m != NULL ) delete m;
        delete m;
    }
}

//------------------------------------------------------------------------------
// Material Insertion and Queries
//------------------------------------------------------------------------------
/// Query for number of materials
unsigned int Scene::materialCount( void ) {
    return m_materialCount;
}

//------------------------------------------------------------------------------
/// Insert a pointer to a material into the linked list of polygons
int Scene::appendMaterial( Material* m ) {
    m_materials.push_back( m );
    m_materialCount++;
    return 0;
}

//------------------------------------------------------------------------------
/// Query for a material by id where id the position in the linked list
Material* Scene::material( const unsigned int& i ) {
    assert( i < m_materialCount );
    return m_materials.at( i );
}

//------------------------------------------------------------------------------
