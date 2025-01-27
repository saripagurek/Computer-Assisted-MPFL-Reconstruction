/* showstl.cpp
 */


#include "headers.h"

#include "persp.h"


perspWindow *w;


int main( int argc, char **argv )

{
  if (argc < 2) {
    cerr << "Usage: " << argv[0] << " filename [filename]*" << endl;
    exit(1);
  }

  // Set up windows

  glutInit( &argc, argv );
  glutInitDisplayMode( GLUT_DOUBLE | GLUT_RGBA | GLUT_DEPTH );

  w = new perspWindow( argc, argv, 100, 100, 600, 600, "femur" );

  // Run OpenGL

  glutMainLoop();
}
