#include <GL/glut.h>
#include <stdio.h>

//#include <stdlib.h>

static int framename = 0;
static int angle=0;
int Width;
int Height;
unsigned char *Pixels;
char filename[20];

#include <tiffio.h>

int writetiff(char *f_name, char *description,
  int x, int y, int width, int height, int compression)
{
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

void init(void) 
{
	
   glClearColor (0.0, 0.0, 0.0, 0.0);
   glClearDepth (1.0);
   glShadeModel (GL_SMOOTH);
}

void display(void)
{
	glClear (GL_COLOR_BUFFER_BIT);
	glEnable(GL_LIGHTING);
	glEnable(GL_DEPTH_TEST);

   	GLfloat LightAmbient[] =  { 0.4f, 0.4f, 0.4f, 1.0f};
	GLfloat LightDiffuse[] =  { 0.3f, 0.3f, 0.3f, 1.0f};
	GLfloat LightSpecular[] = { 0.4f, 0.4f, 0.4f, 1.0f};
	GLfloat LightPosition[] = { 5.0f, 5.0f, 5.0f, 1.0f}; 
	
	glClearColor(0.0,0.0,0.0,0.0);
	glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
	
	glLightfv(GL_LIGHT0, GL_AMBIENT, LightAmbient);
	glLightfv(GL_LIGHT0, GL_DIFFUSE, LightDiffuse);
	glLightfv(GL_LIGHT0, GL_SPECULAR, LightSpecular);
	glLightfv(GL_LIGHT0, GL_POSITION, LightPosition);
	glEnable(GL_LIGHT0);
	glShadeModel(GL_SMOOTH);

	GLfloat material_Ka[]   = { 0.11f, 0.06f, 0.11f, 1.0f};
	GLfloat material_Kd[]   = { 0.43f, 0.47f, 0.54f, 1.0f};
	GLfloat material_Ks[]   = { 0.33f, 0.33f, 0.52f, 1.0f};
	GLfloat material_Ke[]   = { 0.1f, 0.0f, 0.1f, 1.0f};
	GLfloat material_Se   = 10;
	
	glPushMatrix();
	glRotated (angle, 0.0, 1.0, 0.0);
	glMaterialfv(GL_FRONT, GL_AMBIENT, material_Ka);
	glMaterialfv(GL_FRONT, GL_DIFFUSE, material_Kd);
	glMaterialfv(GL_FRONT, GL_SPECULAR, material_Ks);
	glMaterialfv(GL_FRONT, GL_EMISSION, material_Ke);
	glMaterialf(GL_FRONT, GL_SHININESS, material_Se);
	glutSolidTeapot(1.0);
	glPopMatrix();

	glutSwapBuffers();
	
	sprintf(filename, "test%.04d.tif", framename);
	printf("%s\n",filename);
	writetiff(filename, "movie", 0, 0, Width, Height, COMPRESSION_NONE);

	
}

void reshape (int w, int h)
{
	glViewport (0, 0, (GLsizei) w, (GLsizei) h); 
	glMatrixMode (GL_PROJECTION);
	glLoadIdentity ();
	gluPerspective(65.0, (GLfloat) w/(GLfloat) h, 1.0, 20.0);
	glMatrixMode(GL_MODELVIEW);
	glLoadIdentity();
	glTranslatef (0.0, 0.0, -5.0);
	
	Width = w;
	Height = h;

}

void keyboard (unsigned char key, int x, int y)
{
   switch (key) {
  
	  case 27:
         //exit(0);
         break;

      default:
         break;
   }
}

void idle(void)
{
	angle = (angle+5)%360;
	framename++;
	glutPostRedisplay();	

}


int main(int argc, char** argv)
{
   glutInit(&argc, argv);
   glutInitDisplayMode (GLUT_DOUBLE | GLUT_RGB |GLUT_DEPTH);
   glutInitWindowSize (600, 600); 
   glutInitWindowPosition (100, 100);
   glutCreateWindow (argv[0]);
   init ();
   glutDisplayFunc(display); 
   glutReshapeFunc(reshape);
   glutKeyboardFunc(keyboard);
   glutIdleFunc(idle);
   glutMainLoop();
   return 0;
}

