/*-----------------------THE GEORGE WASHINGTON UNIVERSITY-----------------------
James Taylor                                                         jrt@gwu.edu

MeshLoader class definition

Meshloader is a factory for constructing instances of the mesh class
Supports loading and parsing some types from file or constructing standard
shapes parametrically
------------------------------------------------------------------------------*/

#ifndef MESHLOADER_H
#define MESHLOADER_H

//------------------------------------------------------------------------------

#include <string>

#include <cs6555/Mesh.h>
#include <cs6555/Material.h>

#include <cs6555/DeformableBody.h>

//------------------------------------------------------------------------------

typedef enum {
    ML_MATERIAL_RANDOM,
    ML_MATERIAL_ASSIGN
} EMeshLoaderMaterialOp;

//------------------------------------------------------------------------------

class MeshLoader
{
public:
    typedef enum {
        FormatD,
        FormatPLY
    } MeshType;

    // Primitives
    static Mesh* quad( const double& width, const double& height, Material* material = NULL, const EMeshLoaderMaterialOp& material_op = ML_MATERIAL_RANDOM );
    static Mesh* circle( const double& radius, const unsigned int& arcsegments, Material* material = NULL, const EMeshLoaderMaterialOp& material_op = ML_MATERIAL_RANDOM );
    static Mesh* cylinder( const double& radius, const double& height, const unsigned int& arcsegments, Material* material = NULL, const EMeshLoaderMaterialOp& material_op = ML_MATERIAL_RANDOM );
    static Mesh* cone( const double& radius, const double& height, const unsigned int& arcsegments, Material* material = NULL, const EMeshLoaderMaterialOp& material_op = ML_MATERIAL_RANDOM );
    static Mesh* box( const double& width, const double& height, const double& depth, Material* material = NULL, const EMeshLoaderMaterialOp& material_op = ML_MATERIAL_RANDOM );
    static Mesh* sphere( const double& radius, const unsigned int& arcsegments, const unsigned int& slices, Material* material = NULL, const EMeshLoaderMaterialOp& material_op = ML_MATERIAL_RANDOM );

    // Files
    static Mesh* load( const std::string& path, const MeshType& type );
    static Mesh* loadD( const std::string& path );
    static Mesh* loadPly( const std::string& path );

    static DeformableBody* tetrahedron( void );

private:
    static void assign_material( Mesh* mesh, Polygon* poly, Material* material, const EMeshLoaderMaterialOp& material_op );
};

//------------------------------------------------------------------------------

#endif // MESHLOADER_H
