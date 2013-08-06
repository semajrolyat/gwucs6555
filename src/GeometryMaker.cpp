#include <cs6555/GeometryMaker.h>

#include <cs6555/MeshLoader.h>
#include <cs6555/Constants.h>

Geometry* GeometryMaker::arrow( void ) {
    Geometry* geometry = new Geometry();
    Material* material = new Material();
    Mesh* mesh;

    material->randomize();

    /*
    mesh = MeshLoader::sphere( 1.0, 10, 5, material, ML_MATERIAL_ASSIGN );
    geometry->insert( mesh );

    mesh = MeshLoader::cylinder( 0.3, 3.5, 5, material, ML_MATERIAL_ASSIGN );
    mesh->pose.position.z( 2.5 );
    geometry->insert( mesh );

    mesh = MeshLoader::cone( 0.5, 1.5, 8, material, ML_MATERIAL_ASSIGN );
    mesh->pose.position.z( 4.0 );
    geometry->insert( mesh );
    */

    //mesh = MeshLoader::sphere( 0.75, 10, 5, material, ML_MATERIAL_ASSIGN );
    mesh = MeshLoader::sphere( 0.75, 6, 3, material, ML_MATERIAL_ASSIGN );
    mesh->pose.orientation = Vector3( PI/2.0, 0.0, 0.0 );
    geometry->insert( mesh );
///*
    //mesh = MeshLoader::cylinder( 0.3, 3.5, 10, material, ML_MATERIAL_ASSIGN );
    mesh = MeshLoader::cylinder( 0.3, 3.5, 4, material, ML_MATERIAL_ASSIGN );
    mesh->pose.position.z( 2.25 );
    geometry->insert( mesh );

    //mesh = MeshLoader::cone( 0.5, 1.5, 10, material, ML_MATERIAL_ASSIGN );
    mesh = MeshLoader::cone( 0.5, 1.5, 4, material, ML_MATERIAL_ASSIGN );
    mesh->pose.position.z( 4.0 );
    geometry->insert( mesh );
//*/
    //geometry->scale = Vector3( 0.1, 0.1, 0.1 );

    return geometry;
}
