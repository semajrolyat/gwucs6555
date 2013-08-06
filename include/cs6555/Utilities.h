/*-----------------------THE GEORGE WASHINGTON UNIVERSITY-----------------------
James Taylor                                                         jrt@gwu.edu

Utilities class

------------------------------------------------------------------------------*/

#ifndef _UTILITIES_H_
#define _UTILITIES_H_

#include <GL/glut.h>
#include <tiffio.h>

class Utilities {
public:
    static int writetiff(const char *f_name, const char *description, int x, int y, int width, int height, int compression) {
        TIFF *file;
        GLubyte *image, *p;
        int i;

        file = TIFFOpen(f_name, "w");
        if (file == NULL) {
            return 1;
        }
        //image = (GLubyte *) malloc(width * height * sizeof(GLubyte) * 3);
        image = new GLubyte[width * height * sizeof(GLubyte) * 3];
        glPixelStorei(GL_PACK_ALIGNMENT, 1);

        glReadPixels(x, y, width, height, GL_RGB, GL_UNSIGNED_BYTE, image);
        TIFFSetField(file, TIFFTAG_IMAGEWIDTH, (uint32) width);
        TIFFSetField(file, TIFFTAG_IMAGELENGTH, (uint32) height);
        TIFFSetField(file, TIFFTAG_BITSPERSAMPLE, 8);
        TIFFSetField(file, TIFFTAG_COMPRESSION, compression);
        TIFFSetField(file, TIFFTAG_PHOTOMETRIC, PHOTOMETRIC_RGB);
        TIFFSetField(file, TIFFTAG_SAMPLESPERPIXEL, 3);
        TIFFSetField(file, TIFFTAG_PLANARCONFIG, PLANARCONFIG_CONTIG);
        TIFFSetField(file, TIFFTAG_ROWSPERSTRIP, 1);
        TIFFSetField(file, TIFFTAG_IMAGEDESCRIPTION, description);
        p = image;
        for (i = height - 1; i >= 0; i--) {
            if (TIFFWriteScanline(file, p, i, 0) < 0) {
                //free(image);
                delete [] image;
                TIFFClose(file);
                return 1;
            }
            p += width * sizeof(GLubyte) * 3;
        }
        TIFFClose(file);
        return 0;
    }

};

#endif // _UTILITIES_H_
