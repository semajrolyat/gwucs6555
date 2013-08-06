#ifndef _MOVIE_H_
#define _MOVIE_H_

#include <stdio.h>
#include <string>
#include <cs6555/Utilities.h>

class Movie {
public:

    static void write_frame( const std::string& name,
                      const unsigned int& frame,
                      const unsigned int& width,
                      const unsigned int& height )
    {
        char file[128];

        sprintf( file, "%s_%.04d.tif",name.c_str(), frame );
        //printf( "%s\n", file );
        Utilities::writetiff( file, "movie", 0, 0, width, height, COMPRESSION_NONE );
    }
};

#endif // _MOVIE_H_
