/********************** THE GEORGE WASHINGTON UNIVERSITY ***********************
* James Taylor                                                     jrt@gwu.edu *
*                                                                              *
*******************************************************************************/

#ifndef PERLINNOISE_H
#define PERLINNOISE_H

#include <stdlib.h>
#include <math.h>

#include <cs6555/Math.h>

class PerlinNoise
{
public:
    static double** lattice( const unsigned int& width, const unsigned int& height, const unsigned int& seed, const unsigned int& lattice_step ) {
        srand( seed );

        unsigned int lattice_width = width / lattice_step;
        unsigned int lattice_height = height / lattice_step;

        double** plattice = new double*[lattice_height];
        for( unsigned int y = 0; y < lattice_height; y++ ) {
            plattice[y] = new double[lattice_width];
        }
        for( unsigned int y = 0; y < lattice_height; y++ ) {
            for( unsigned int x = 0; x < lattice_width; x++ ) {
                plattice[y][x] = ( (double)rand() / (double)RAND_MAX );
            }
        }

        return plattice;
    }

    static double*** lattice( const unsigned int& width, const unsigned int& height, const unsigned int& depth, const unsigned int& seed, const unsigned int& lattice_step ) {
        srand( seed );

        unsigned int lattice_width = width / lattice_step;
        unsigned int lattice_height = height / lattice_step;
        unsigned int lattice_depth = depth / lattice_step;

        double*** plattice = new double**[lattice_depth];
        for( unsigned int z = 0; z < lattice_depth; z++ ) {
            plattice[z] = new double*[lattice_height];
        }
        for( unsigned int z = 0; z < lattice_depth; z++ ) {
            for( unsigned int y = 0; y < lattice_height; y++ ) {
                plattice[z][y] = new double[lattice_width];
            }
        }
        for( unsigned int z = 0; z < lattice_depth; z++ ) {
            for( unsigned int y = 0; y < lattice_height; y++ ) {
                for( unsigned int x = 0; x < lattice_width; x++ ) {
                    plattice[z][y][x] = ( (double)rand() / (double)RAND_MAX );
                }
            }
        }
        return plattice;
    }
/*
    static Image3D* image( const unsigned int& width, const unsigned int& height, const unsigned int& depth, const unsigned int& seed, const unsigned int& lattice_step ) {

        Image3D* img = new Image3D( width, height, depth );
        if( !img->Allocated() ) {
            delete img;
            return NULL;
        }

        double*** plattice = lattice( width, height, depth, seed, lattice_step );

        for( unsigned int z = 0; z < depth; z++ ) {
            for( unsigned int y = 0; y < height; y++ ) {
                for( unsigned int x = 0; x < width; x++ ) {
                    double value = std::min( turbulence( x, y, z, width, height, depth, plattice, lattice_step ), 1.0f );

                    Pixel* pixel = img->pixel( x, y, z );
                    pixel->red = value;
                    pixel->green = value;
                    pixel->blue = value;
                }
            }
        }
        delete plattice;

        return img;
    }

    static Image3D* image( Image3D* img, const unsigned int& seed, const unsigned int& lattice_step ) {

        unsigned int width = img->Width();
        unsigned int height = img->Height();
        unsigned int depth = img->Depth();

        double*** plattice = lattice( width, height, depth, seed, lattice_step );

        for( unsigned int z = 0; z < depth; z++ ) {
            for( unsigned int y = 0; y < height; y++ ) {
                for( unsigned int x = 0; x < width; x++ ) {
                    double value = turbulence( x, y, z, width, height, depth, plattice, lattice_step );

                    Pixel* pixel = img->pixel( x, y, z );
                    pixel->red = std::max( std::min( pixel->red + value, 1.0f), 0.0f );
                    pixel->green = std::max( std::min( pixel->green + value, 1.0f), 0.0f );
                    pixel->blue = std::max( std::min( pixel->blue + value, 1.0f), 0.0f );
                }
            }
        }
        delete plattice;

        return img;
    }

    static Image* image( const unsigned int& width, const unsigned int& height, const unsigned int& seed, const unsigned int& lattice_step ) {

        Image* img = new Image( width, height );
        if( !img->Allocated() ) {
            delete img;
            return NULL;
        }

        double** plattice = lattice( width, height, seed, lattice_step );

        for( unsigned int y = 0; y < height; y++ ) {
            for( unsigned int x = 0; x < width; x++ ) {
                double value = std::min( turbulence( x, y, width, height, plattice, lattice_step ), 1.0f );

                Pixel* pixel = img->pixel( x, y );
                pixel->red = value;
                pixel->green = value;
                pixel->blue = value;
            }
        }

        delete plattice;

        return img;
    }

    static Image* image( Image* img, const unsigned int& seed, const unsigned int& lattice_step ) {

        unsigned int width = img->Width();
        unsigned int height = img->Height();

        double** plattice = lattice( width, height, seed, lattice_step );

        for( unsigned int y = 0; y < height; y++ ) {
            for( unsigned int x = 0; x < width; x++ ) {
                double value = std::min( turbulence( x, y, width, height, plattice, lattice_step ), 1.0f );

                Pixel* pixel = img->pixel( x, y );
                pixel->red = std::min( pixel->red + value, 1.0f);
                pixel->green = std::min( pixel->green + value, 1.0f);
                pixel->blue = std::min( pixel->blue + value, 1.0f);
            }
        }

        delete plattice;

        return img;
    }
*/
    static double turbulence( const unsigned int& x, const unsigned int& y, const unsigned int& width, const unsigned int& height, double** lattice, const unsigned int& lattice_step ) {
        double noise1x = noise( x, y, width, height, lattice, lattice_step * 16 );
        double noise2x = noise( x, y, width, height, lattice, lattice_step * 8 );
        double noise4x = noise( x, y, width, height, lattice, lattice_step * 4 );
        double noise8x = noise( x, y, width, height, lattice, lattice_step * 2 );
        double noise16x = noise( x, y, width, height, lattice, lattice_step * 1 );

        double turbulence1x  = ( ( noise1x - 0.5f ) / 2.0f );
        double turbulence2x  = ( ( noise2x - 0.5f ) / 4.0f );
        double turbulence4x  = ( ( noise4x - 0.5f ) / 8.0f );
        double turbulence8x  = ( ( noise8x - 0.5f ) / 16.0f );
        double turbulence16x = ( ( noise16x - 0.5f ) / 32.0f );

        return turbulence1x + turbulence2x + turbulence4x + turbulence8x + turbulence16x + 0.5f;
    }

    static double turbulence( const unsigned int& x, const unsigned int& y, const unsigned int& z, const unsigned int& width, const unsigned int& height, const unsigned int& depth, double*** lattice, const unsigned int& lattice_step ) {
        //more complex than this
        double noise1x = noise( x, y, z, width, height, depth, lattice, lattice_step * 16 );
        double noise2x = noise( x, y, z, width, height, depth, lattice, lattice_step * 8 );
        double noise4x = noise( x, y, z, width, height, depth, lattice, lattice_step * 4 );
        double noise8x = noise( x, y, z, width, height, depth, lattice, lattice_step * 2 );
        double noise16x = noise( x, y, z, width, height, depth, lattice, lattice_step * 1 );

        double turbulence1x  = ( ( noise1x - 0.5f ) / 2.0f );
        double turbulence2x  = ( ( noise2x - 0.5f ) / 4.0f );
        double turbulence4x  = ( ( noise4x - 0.5f ) / 8.0f );
        double turbulence8x  = ( ( noise8x - 0.5f ) / 16.0f );
        double turbulence16x = ( ( noise16x - 0.5f ) / 32.0f );

        return turbulence1x + turbulence2x + turbulence4x + turbulence8x + turbulence16x + 0.5f;
    }

    static double noise( const unsigned int& x, const unsigned int& y, const unsigned int& width, const unsigned int& height, double** lattice, const unsigned int& lattice_step ) {
        unsigned int this_lattice_x = x / lattice_step;
        unsigned int this_lattice_y = y / lattice_step;
        unsigned int interp_lattice_x = this_lattice_x + 1;
        unsigned int next_lattice_x = interp_lattice_x % ( width / lattice_step );
        unsigned int interp_lattice_y = this_lattice_y + 1;
        unsigned int next_lattice_y = interp_lattice_y % ( height / lattice_step );

        unsigned int x0 = this_lattice_x * lattice_step;
        unsigned int y0 = this_lattice_y * lattice_step;
        unsigned int x1 = interp_lattice_x * lattice_step;
        unsigned int y1 = interp_lattice_y * lattice_step;
        double f00 = lattice[this_lattice_y][this_lattice_x];
        double f10 = lattice[this_lattice_y][next_lattice_x];
        double f01 = lattice[next_lattice_y][this_lattice_x];
        double f11 = lattice[next_lattice_y][next_lattice_x];

        using namespace Math;
        return bilinear_interpolate( x, y, x0, y0, x1, y1, f00, f10, f01,f11 );
    }

    static double noise( const unsigned int& x, const unsigned int& y, const unsigned int& z, const unsigned int& width, const unsigned int& height, const unsigned int& depth, double*** lattice, const unsigned int& lattice_step ) {
        unsigned int this_lattice_x = x / lattice_step;
        unsigned int this_lattice_y = y / lattice_step;
        unsigned int this_lattice_z = z / lattice_step;
        unsigned int interp_lattice_x = this_lattice_x + 1;
        unsigned int next_lattice_x = interp_lattice_x % ( width / lattice_step );
        unsigned int interp_lattice_y = this_lattice_y + 1;
        unsigned int next_lattice_y = interp_lattice_y % ( height / lattice_step );
        unsigned int interp_lattice_z = this_lattice_z + 1;
        unsigned int next_lattice_z = interp_lattice_z % ( depth / lattice_step );

        unsigned int x0 = this_lattice_x * lattice_step;
        unsigned int y0 = this_lattice_y * lattice_step;
        unsigned int z0 = this_lattice_z * lattice_step;
        unsigned int x1 = interp_lattice_x * lattice_step;
        unsigned int y1 = interp_lattice_y * lattice_step;
        unsigned int z1 = interp_lattice_z * lattice_step;
        double f000 = lattice[this_lattice_z][this_lattice_y][this_lattice_x];
        double f100 = lattice[this_lattice_z][this_lattice_y][next_lattice_x];
        double f010 = lattice[this_lattice_z][next_lattice_y][this_lattice_x];
        double f110 = lattice[this_lattice_z][next_lattice_y][next_lattice_x];
        double f001 = lattice[next_lattice_z][this_lattice_y][this_lattice_x];
        double f101 = lattice[next_lattice_z][this_lattice_y][next_lattice_x];
        double f011 = lattice[next_lattice_z][next_lattice_y][this_lattice_x];
        double f111 = lattice[next_lattice_z][next_lattice_y][next_lattice_x];

        using namespace Math;
        return trilinear_interpolate( x, y, z, x0, y0, z0, x1, y1, z1, f000, f100, f001, f101, f010, f110, f011, f111 );
    }
};

#endif // PERLINNOISE_H
