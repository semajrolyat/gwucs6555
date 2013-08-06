/*-----------------------THE GEORGE WASHINGTON UNIVERSITY-----------------------
James Taylor                                                         jrt@gwu.edu

MeshLoader class implementation

------------------------------------------------------------------------------*/

#include <cs6555/MeshLoader.h>

#include <cs6555/Constants.h>

#include <stdlib.h>
#include <stdio.h>
#include <math.h>
#include <string.h>

#include <stdexcept>

#include <cs6555/Trigon.h>
#include <cs6555/Node.h>
#include <cs6555/Spring.h>
#include <cs6555/Tetrahedron.h>

//------------------------------------------------------------------------------
void MeshLoader::assign_material( Mesh* mesh, Polygon* poly, Material* material, const EMeshLoaderMaterialOp& material_op ) {
    if( material_op == ML_MATERIAL_ASSIGN )
        assert( material != NULL );

    if( material_op == ML_MATERIAL_ASSIGN ) {
        poly->material = material;
    } else if( material_op == ML_MATERIAL_RANDOM ) {
        poly->material = new Material();
        mesh->appendMaterial( poly->material );
        poly->material->randomize();
    }
}

//------------------------------------------------------------------------------
/// Parametrically load a Quad mesh, e.g. a flat square
Mesh* MeshLoader::quad( const double& width, const double& height, Material* material, const EMeshLoaderMaterialOp& material_op ) {
    Mesh* mesh = new Mesh();

    // ----- Generate Vertices ------
    double halfx = width / 2.0;
    double halfy = height / 2.0;

    mesh->appendVertex( new Vertex( -halfx, -halfy, 0.0 ) );
    mesh->appendVertex( new Vertex(  halfx, -halfy, 0.0 ) );
    mesh->appendVertex( new Vertex(  halfx,  halfy, 0.0 ) );
    mesh->appendVertex( new Vertex( -halfx,  halfy, 0.0 ) );

    mesh->appendTextureVertex( new Vector3( 1.0, 0.0, 0.0 ) ); // vert 0
    mesh->appendTextureVertex( new Vector3( 0.0, 0.0, 0.0 ) ); // vert 1
    mesh->appendTextureVertex( new Vector3( 0.0, 1.0, 0.0 ) ); // vert 2
    mesh->appendTextureVertex( new Vector3( 1.0, 1.0, 0.0 ) ); // vert 3
    // ------------------------------

    // ----- Generate Polygons ------
    Polygon* poly = new Polygon();
    assign_material( mesh, poly, material, material_op );

    poly->addVertex( 0 );
    poly->addVertex( 1 );
    poly->addVertex( 2 );
    poly->addVertex( 3 );

    poly->addTextureVertex( 0 );
    poly->addTextureVertex( 1 );
    poly->addTextureVertex( 2 );
    poly->addTextureVertex( 3 );

    mesh->appendPolygon( poly );
    // ------------------------------

    mesh->calculate_polygon_normals();
    mesh->calculate_vertex_normals();

    return mesh;
}

//------------------------------------------------------------------------------
/// Parametrically load a circle mesh, e.g. a flat circle
Mesh* MeshLoader::circle( const double& radius, const unsigned int& arcsegments, Material* material, const EMeshLoaderMaterialOp& material_op ) {

    Mesh* mesh = new Mesh();

    // ----- Generate Vertices ------
    double dtheta = 2.0 * PI / (double) arcsegments;
    double theta = 0.0;
    double x, y;
    for( unsigned int i = 0; i < arcsegments; i++ ) {
        x = cos(theta) * radius;
        y = sin(theta) * radius;
        theta += dtheta;
        mesh->appendVertex( new Vertex( x, y, 0.0 ) );
    }
    // ------------------------------

    // ----- Generate Polygons ------
    Polygon* poly = new Polygon();
    assign_material( mesh, poly, material, material_op );
    for( unsigned int i = 0; i < arcsegments; i++ ) {
        poly->addVertex( i );
    }
    mesh->appendPolygon( poly );
    // ------------------------------

    mesh->calculate_polygon_normals();
    mesh->calculate_vertex_normals();

    return mesh;
}

//------------------------------------------------------------------------------
/// Parametrically load a cylinder mesh
Mesh* MeshLoader::cylinder( const double& radius, const double& height, const unsigned int& arcsegments, Material* material, const EMeshLoaderMaterialOp& material_op ) {

    Mesh* mesh = new Mesh();

    // ----- Generate Vertices ------
    double halfz = height / 2.0;
    double dtheta = 2.0 * PI / (double) arcsegments;
    double theta = 0.0;
    double x, y;

    for( unsigned int i = 0; i < arcsegments; i++ ) {
        x = cos(theta) * radius;
        y = sin(theta) * radius;
        theta += dtheta;
        mesh->appendVertex( new Vertex( x, y,  halfz ) );
        mesh->appendVertex( new Vertex( x, y, -halfz ) );
    }
    // ------------------------------

    // ----- Generate Polygons ------
    Polygon* poly;
    // cap
    poly = new Polygon();
    assign_material( mesh, poly, material, material_op );
    for( unsigned int i = 0; i < arcsegments; i++ ) {
        poly->addVertex( i * 2 );
    }
    poly->flip();
    mesh->appendPolygon( poly );

    for( unsigned int i = 0; i < arcsegments; i++ ) {
        poly = new Polygon();
        assign_material( mesh, poly, material, material_op );
        if(i == 0) {
            poly->addVertex( 2 );
            poly->addVertex( 3 );
            poly->addVertex( 1 );
            poly->addVertex( 0 );
        } else if( i == arcsegments - 1) {
            poly->addVertex( i*2 );
            poly->addVertex( 0 );
            poly->addVertex( 1 );
            poly->addVertex( i*2+1 );
        } else {
            poly->addVertex( (i+1)*2 );
            poly->addVertex( (i+1)*2+1 );
            poly->addVertex( i*2+1 );
            poly->addVertex( i*2 );
        }
        mesh->appendPolygon( poly );
    }

    // cap
    poly = new Polygon();
    assign_material( mesh, poly, material, material_op );
    for( unsigned int i = 0; i < arcsegments; i++ ) {
        poly->addVertex( i * 2 + 1 );
    }
    //poly->flip();
    mesh->appendPolygon( poly );
    // ------------------------------

    mesh->calculate_polygon_normals();
    mesh->calculate_vertex_normals();

    return mesh;
}

//------------------------------------------------------------------------------
/// Parametrically load a conical mesh
Mesh* MeshLoader::cone( const double& radius, const double& height, const unsigned int& arcsegments, Material* material, const EMeshLoaderMaterialOp& material_op ) {

    Mesh* mesh = new Mesh();

    // ----- Generate Vertices ------
    // peak is 0 vertex
    mesh->appendVertex( new Vertex( 0.0, 0.0, height ) );

    // bottom disk
    double dtheta = 2.0 * PI / (double) arcsegments;
    double theta = 0.0;
    for( unsigned int i = 0; i < arcsegments; i++ ) {
        double x = cos(theta) * radius;
        double y = sin(theta) * radius;
        theta += dtheta;
        mesh->appendVertex( new Vertex( x, y, 0.0 ) );
    }
    // ------------------------------

    // ----- Generate Polygons ------
    Polygon* poly;

    // cap
    poly = new Polygon();
    assign_material( mesh, poly, material, material_op );

    for( unsigned int i = 1; i <= arcsegments; i++ ) {
        poly->addVertex( i );
    }
    mesh->appendPolygon( poly );

    // sides
    for( unsigned int i = 1; i <= arcsegments; i++ ) {
        poly = new Polygon();
        assign_material( mesh, poly, material, material_op );

        if( i < arcsegments ) {
            poly->addVertex( 0 );  // peak
            poly->addVertex( i+1 );
            poly->addVertex( i );
        } else {
            poly->addVertex( 0 );  // peak
            poly->addVertex( 1 );
            poly->addVertex( i );
        }
        mesh->appendPolygon( poly );
    }
    // ------------------------------

    mesh->calculate_polygon_normals();
    mesh->calculate_vertex_normals();

    return mesh;
}

//------------------------------------------------------------------------------
/// Parametrically load a box mesh
Mesh* MeshLoader::box( const double& width, const double& height, const double& depth, Material* material, const EMeshLoaderMaterialOp& material_op ) {

    Mesh* mesh = new Mesh();

    // ----- Generate Vertices ------
    double halfx, halfy, halfz;

    halfx = width / 2.0;
    halfy = height / 2.0;
    halfz = depth / 2.0;

    mesh->appendVertex( new Vertex( -halfx, -halfy, -halfz ) );
    mesh->appendVertex( new Vertex(  halfx, -halfy, -halfz ) );
    mesh->appendVertex( new Vertex(  halfx,  halfy, -halfz ) );
    mesh->appendVertex( new Vertex( -halfx,  halfy, -halfz ) );

    mesh->appendVertex( new Vertex( -halfx, -halfy,  halfz ) );
    mesh->appendVertex( new Vertex(  halfx, -halfy,  halfz ) );
    mesh->appendVertex( new Vertex(  halfx,  halfy,  halfz ) );
    mesh->appendVertex( new Vertex( -halfx,  halfy,  halfz ) );
    // ------------------------------

    // ----- Generate Polygons ------
    for ( unsigned int i = 0; i < 6; i++ ) {
        Polygon* poly = new Polygon();
        assign_material( mesh, poly, material, material_op );

        if( i == 0 ) {
            poly->addVertex( 0 );
            poly->addVertex( 1 );
            poly->addVertex( 2 );
            poly->addVertex( 3 );
        } else if( i == 1 ) {
            poly->addVertex( 0 );
            poly->addVertex( 4 );
            poly->addVertex( 5 );
            poly->addVertex( 1 );
        } else if( i == 2 ) {
            poly->addVertex( 0 );
            poly->addVertex( 3 );
            poly->addVertex( 7 );
            poly->addVertex( 4 );
        } else if( i == 3 ) {
            poly->addVertex( 6 );
            poly->addVertex( 5 );
            poly->addVertex( 4 );
            poly->addVertex( 7 );
        } else if( i == 4 ) {
            poly->addVertex( 6 );
            poly->addVertex( 7 );
            poly->addVertex( 3 );
            poly->addVertex( 2 );
        } else if( i == 5 ) {
            poly->addVertex( 6 );
            poly->addVertex( 2 );
            poly->addVertex( 1 );
            poly->addVertex( 5 );
        }
        mesh->appendPolygon( poly );
    }
    // ------------------------------

    //calculate the surface normals
    mesh->calculate_polygon_normals();
    mesh->calculate_vertex_normals();

    return mesh;
}

//------------------------------------------------------------------------------
/// Parametrically load a sphere mesh
/// where arcsegments are the number of arclengths around the sphere, e.g. longitude
/// and slices are the number of cross sections from pole to pole, e.g. latitude
Mesh* MeshLoader::sphere( const double& radius, const unsigned int& arcsegments, const unsigned int& slices, Material* material, const EMeshLoaderMaterialOp& material_op ) {
    assert( radius > 0.0 && arcsegments > 1 && slices > 0 );

    Mesh* mesh = new Mesh();

    // ----- Generate Vertices ------
    double dtheta = 2.0 * PI / (double) arcsegments;
    double theta = 0.0;
    double dphi = (double) PI / (double) (slices + 1);
    double phi = 0.0;
    double x, y, z;

    // Top pole vertex
    mesh->appendVertex( new Vertex( 0.0, radius, 0.0 ) );

    // Arc segment and slice vertices
    for( unsigned int i = 0; i < slices; i++ ) {
        theta = 0.0;
        phi += dphi;
        for( unsigned int i = 0; i < arcsegments; i++ ) {
            x = sin(phi) * cos(theta);
            y = cos(phi);
            z = sin(phi) * sin(theta);
            theta += dtheta;
            mesh->appendVertex( new Vertex( x * radius, y * radius, z * radius ) );
        }
    }
    // Bottom pole vertex
    mesh->appendVertex( new Vertex( 0.0, -radius, 0.0 ) );
    // ------------------------------

    // ----- Generate Polygons ------
    for( unsigned int slice = 0; slice < slices + 1; slice++ ) {
        for( unsigned int i = 0; i < arcsegments; i++ ) {
            Polygon* p = new Polygon();
            assign_material( mesh, p, material, material_op );

            if( slice == 0 ) {
                // Top Cap - Triads
                p->addVertex( 0 );
                if( i < arcsegments - 1 )
                    p->addVertex( i + 2 );
                else
                    p->addVertex( 1 );
                p->addVertex( i + 1 );
            } else if( slice > 0 && slice < slices ) {
                // Quads
                p->addVertex( arcsegments * (slice - 1) + i + 1 );
                if( i < arcsegments - 1 ) {
                    p->addVertex( arcsegments * (slice - 1) + i + 2 );
                    p->addVertex( arcsegments * (slice) + i + 2 );

                } else {
                    p->addVertex( arcsegments * (slice - 1) + 1 );
                    p->addVertex( arcsegments * (slice) + 1 );
                }
                p->addVertex( arcsegments * (slice) + i + 1 );
            } else if( slice == slices ) {
                // Bottom Cap - Triads
                p->addVertex( arcsegments * (slice - 1) + i + 1 );
                if( i < arcsegments - 1 ) {
                    p->addVertex( arcsegments * (slice - 1) + i + 2 );
                } else {
                    p->addVertex( arcsegments * (slice - 1) + 1 );
                }
                p->addVertex( mesh->vertexCount() - 1 );
            }

            p->flip();      // the sphere is wrong handed for OpenGL. Flip here solves for all polys
            mesh->appendPolygon( p );
        }
    }
    // ------------------------------

    mesh->calculate_polygon_normals();
    mesh->calculate_vertex_normals();

    return mesh;
}

//------------------------------------------------------------------------------
/// Load a mesh from a file
Mesh* MeshLoader::load( const std::string& path, const MeshType& type ) {
    switch( type ) {
    case FormatD:
        return loadD( path );
    case FormatPLY:
        break;
    }
    return NULL;
}

//------------------------------------------------------------------------------
/// Load a mesh from a file in D format
Mesh* MeshLoader::loadD( const std::string& path ) {
    Mesh* mesh = new Mesh();

    FILE* file;
    file = fopen( path.c_str(),"r" );
    if ( file == NULL ) return NULL;

    char buffer[1024];
    std::string token, delim = "\t ";

    int line = 0;
    int verts = 0;
    int polys = 0;

    while( fgets( buffer, 1024, file ) ) {
        if( line == 0 ) {
            token = strtok( buffer, delim.c_str() );
            token = strtok( NULL, delim.c_str() );
            verts = atoi( token.c_str() );
            token = strtok( NULL, delim.c_str() );
            polys = atoi( token.c_str() );
        } else {
            if( line < verts + 1 ) {
                double x, y, z;
                token = strtok( buffer, delim.c_str() );
                x = atof( token.c_str() );
                token = strtok( NULL, delim.c_str() );
                y = atof( token.c_str() );
                token = strtok( NULL, delim.c_str() );
                z = atof( token.c_str() );
                mesh->appendVertex( new Vertex( x, y, z ) );
            } else {
                token = strtok( buffer, delim.c_str() );
                int vertsInPoly = atoi( token.c_str() );
                int vert;
                //Polygon* p;
                //if( vertsInPoly ) p = new Polygon( );
                if( !vertsInPoly ) continue;

                Polygon* p = new Polygon( );
                for( int i = 0; i < vertsInPoly; i++ ) {
                    token = strtok( NULL, delim.c_str() );
                    vert = atoi( token.c_str() );
                    // vertex ids are all zero based due to vector container
                    // so adjust as they are read in by subtracting 1
                    p->addVertex( vert - 1 );
                }
                mesh->appendPolygon( p );
            }
        }
        line++;
    }

    fclose( file );

    return mesh;
}

//------------------------------------------------------------------------------
// Ref http://paulbourke.net/dataformats/ply/
Mesh* MeshLoader::loadPly( const std::string& path ) {
/*
ply
format ascii 1.0
comment VCGLIB generated
element vertex 1343
property float x
property float y
property float z
property uchar red
property uchar green
property uchar blue
property uchar alpha
element face 0
property list uchar int vertex_indices
end_header
-0.214557 0.0542856 -0.689 148 23 48 255
-0.212646 0.054128 -0.687 169 69 76 255
-0.211357 0.054128 -0.687 180 60 85 255
-0.214557 0.0529931 -0.689 169 59 83 255
-0.212646 0.0528393 -0.687 186 72 81 255
...
*/

    Mesh* mesh = new Mesh();

    FILE* file;
    file = fopen( path.c_str(),"r" );
    if ( file == NULL ) return NULL;

    char buffer[1024];
    std::string token, delim = "\n\t ";

    int line = 1;
    int verts = 0;
    int vert = 0;
    int polys = 0;
    bool header = true;
    unsigned int properties = 0;
    int prop_map[7];
    for( unsigned int i = 0; i < 7; i++ )
        prop_map[i] = 0;

    const int X = 1;
    const int Y = 2;
    const int Z = 3;
    const int RED = 4;
    const int GREEN = 5;
    const int BLUE = 6;
    const int ALPHA = 7;

    while( fgets( buffer, 1024, file ) ) {
        if( line == 1 ) {
            // ply
            token = strtok( buffer, delim.c_str() );
            if( token != "ply" )
                throw std::runtime_error( "Expected ply format.  File not a ply file." );
        }
        if( line == 2 ) {
            // one of [ "format ascii 1.0", "format binary_little_endian 1.0", "format binary_big_endian 1.0" ]

            token = strtok( buffer, delim.c_str() );
            if( token != "format" )
                throw std::runtime_error( "Expected ply format.  File malformed" );
            token = strtok( NULL, delim.c_str() );
            if( token != "ascii" && token != "binary_little_endian" && token != "binary_big_endian" )
                throw std::runtime_error( "Expected ply format.  File malformed" );
            if( token != "ascii" )
                throw std::runtime_error( "Expected ply in ascii format.  Ascii parsing only supported at this time" );
        }

        if( line > 2 && header ) {
            token = strtok( buffer, delim.c_str() );
            if( token == "end_header" ) {
                header = false;
            } else if( token == "element" ) {
                token = strtok( NULL, delim.c_str() );
                if( token == "vertex" ) {
                    token = strtok( NULL, delim.c_str() );
                    verts = atoi( token.c_str() );
                } else if( token == "face" ) {
                    polys = atoi( token.c_str() );
                }
            } else if( token == "property" ) {
                std::string property_type = strtok( NULL, delim.c_str() );
                /*
                name        type        number of bytes
                ---------------------------------------
                char       character                 1
                uchar      unsigned character        1
                short      short integer             2
                ushort     unsigned short integer    2
                int        integer                   4
                uint       unsigned integer          4
                float      single-precision float    4
                double     double-precision float    8

                -or-

                list
                */

                if( property_type != "list" ) {
                    std::string property_name = strtok( NULL, delim.c_str() );

                    if( property_name == "x" ) {
                        prop_map[properties++]  = X;
                    } else if( property_name == "y" ) {
                        prop_map[properties++]  = Y;
                    } else if( property_name == "z" ) {
                        prop_map[properties++]  = Z;
                    } else if( property_name == "red" ) {
                        prop_map[properties++]  = RED;
                    } else if( property_name == "green" ) {
                        prop_map[properties++]  = GREEN;
                    } else if( property_name == "blue" ) {
                        prop_map[properties++]  = BLUE;
                    } else if( property_name == "alpha" ) {
                        prop_map[properties++]  = ALPHA;
                    }
                } else {

                }
            }
        } else if( !header ) {
            double x = 0.0, y = 0.0, z = 0.0, red = 1.0, green = 1.0, blue = 1.0, alpha = 1.0;
            for( unsigned int i = 0; i < properties; i++ ) {
                if( i == 0 )
                    token = strtok( buffer, delim.c_str() );
                else
                    token = strtok( NULL, delim.c_str() );

                switch( prop_map[i] ) {
                case X:
                default:
                    x = atof( token.c_str() );
                case Y:
                    y = atof( token.c_str() );
                case Z:
                    z = atof( token.c_str() );
                case RED:
                    red = (double) atof( token.c_str() ) / (double) 255.0;
                case GREEN:
                    green = (double) atof( token.c_str() ) / (double) 255.0;
                case BLUE:
                    blue = (double) atof( token.c_str() ) / (double) 255.0;
                case ALPHA:
                    alpha = (double) atof( token.c_str() ) / (double) 255.0;
                    break;
                }
            }

            Vertex *v = new Vertex( x, y, z);
            v->color = Color( red, green, blue, alpha );

            mesh->appendVertex( v );

            vert++;
        }
        line++;
    }

    assert( vert == verts );
    return mesh;
}

//------------------------------------------------------------------------------

#define TETRA_SPRING_COEFFICIENT    100
#define TETRA_DAMPING_COEFFICIENT   10

DeformableBody* MeshLoader::tetrahedron( void ) {
    DeformableBody* body = new DeformableBody( );
    Mesh* mesh = new Mesh( );
    body->mesh = mesh;

    // -- build the mesh  --
    /*
    // x unit vector.  Will rotate it around the unit sphere in x and z
    Vector3 i_hat = Vector3( 1.0, 0.0, 0.0 );
    // y unit vector.  Will be the top of the tetrahedron
    Vector3 j_hat = Vector3( 0.0, 1.0, 0.0 );

    Vector3 u;

    // rotate x_hat around z axis by 30 degrees for first vertex
    u = Matrix3::rotZ( -1.0 * 1.0/12.0 * 2.0 * PI ) * i_hat;
    Vertex* v1 = new Vertex( u, u );

    // now rotate u around the y axis 120 degrees for second vertex
    u = Matrix3::rotY( 1.0/3.0 * 2.0 * PI ) * u;
    Vertex* v2 = new Vertex( u, u );

    // now rotate u around the y axis another 120 degrees for third vertex
    u = Matrix3::rotY( 1.0/3.0 * 2.0 * PI ) * u;
    Vertex* v3 = new Vertex( u, u );

    Vertex* v4 = new Vertex( j_hat, j_hat );
    */

    // Note the following is defined by Schneider & Eberly : Geometric Tools for Computer Graphics p. 347
    Vector3 u;

    u = Vector3( 2.0*sqrt(2.0)/3.0, -1.0/3.0, 0.0 );
    Vertex* v1 = new Vertex( u, u );

    u = Vector3( -sqrt(2.0)/3.0, -1.0/3.0, sqrt(6.0)/3.0 );
    Vertex* v2 = new Vertex( u, u );

    u = Vector3( -sqrt(2.0)/3.0, -1.0/3.0, -sqrt(6.0)/3.0 );
    Vertex* v3 = new Vertex( u, u );

    u = Vector3( 0.0, 1.0, 0.0 );
    Vertex* v4 = new Vertex( u, u );

    ///*
    v1->position.print();
    printf("\n");
    v2->position.print();
    printf("\n");
    v3->position.print();
    printf("\n");
    v4->position.print();
    printf("\n");
    //*/

    Trigon* t;
    t = new Trigon( );
    t->addVertex( 0 );
    t->addVertex( 1 );
    t->addVertex( 3 );
    mesh->appendPolygon( t );

    t = new Trigon( );
    t->addVertex( 1 );
    t->addVertex( 2 );
    t->addVertex( 3 );
    mesh->appendPolygon( t );

    t = new Trigon( );
    t->addVertex( 2 );
    t->addVertex( 0 );
    t->addVertex( 3 );
    mesh->appendPolygon( t );

    t = new Trigon( );
    t->addVertex( 0 );
    t->addVertex( 1 );
    t->addVertex( 2 );
    mesh->appendPolygon( t );

    // -- build the body --
    Node* node1, *node2, *node3, *node4;
    Spring* edge1, *edge2, *edge3, *edge4, *edge5, *edge6;
    Tetrahedron* tetra;

    mesh->appendVertex( v1 );
    node1 = new Node( );
    node1->vertex = v1;
    node1->mass = 0.25;
    body->appendNode( node1 );

    mesh->appendVertex( v2 );
    node2 = new Node( );
    node2->vertex = v2;
    node2->mass = 0.25;
    body->appendNode( node2 );

    mesh->appendVertex( v3 );
    node3 = new Node( );
    node3->vertex = v3;
    node3->mass = 0.25;
    body->appendNode( node3 );

    mesh->appendVertex( v4 );
    node4 = new Node( );
    node4->vertex = v4;
    node4->mass = 0.25;
    body->appendNode( node4 );

    edge1 = new Spring( node1, node2, TETRA_SPRING_COEFFICIENT, TETRA_DAMPING_COEFFICIENT );
    edge2 = new Spring( node2, node3, TETRA_SPRING_COEFFICIENT, TETRA_DAMPING_COEFFICIENT );
    edge3 = new Spring( node3, node1, TETRA_SPRING_COEFFICIENT, TETRA_DAMPING_COEFFICIENT );
    edge4 = new Spring( node1, node4, TETRA_SPRING_COEFFICIENT, TETRA_DAMPING_COEFFICIENT );
    edge5 = new Spring( node2, node4, TETRA_SPRING_COEFFICIENT, TETRA_DAMPING_COEFFICIENT );
    edge6 = new Spring( node3, node4, TETRA_SPRING_COEFFICIENT, TETRA_DAMPING_COEFFICIENT );

    body->appendEdge( edge1 );
    body->appendEdge( edge2 );
    body->appendEdge( edge3 );
    body->appendEdge( edge4 );
    body->appendEdge( edge5 );
    body->appendEdge( edge6 );

    tetra = new Tetrahedron( );
    tetra->insert( node1 );
    tetra->insert( node2 );
    tetra->insert( node3 );
    tetra->insert( node4 );
    tetra->insert( edge1 );
    tetra->insert( edge2 );
    tetra->insert( edge3 );
    tetra->insert( edge4 );
    tetra->insert( edge5 );
    tetra->insert( edge6 );
    body->appendTetrahedron( tetra );

    return body;
}
