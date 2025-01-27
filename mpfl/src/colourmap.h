// colourmap.h
//
// Define a colourmap texture


#ifndef COLOURMAP_H
#define COLOURMAP_H


#include "headers.h"


#define MAP_SIZE 1394


class Colourmap {

  static GLubyte map[3*MAP_SIZE];

 public:
  
  GLuint textureID;
  
  Colourmap() {
    setupTexture();
  };

  vec3 colour( float f ) {
    int i = 3 * floor(f * (MAP_SIZE-1));
    return vec3( map[i+0]/255.0, map[i+1]/255.0, map[i+2]/255.0 );
  }

  vec4 colour4( float f ) {
    int i = 3 * floor(f * (MAP_SIZE-1));
    return vec4( map[i+0]/255.0, map[i+1]/255.0, map[i+2]/255.0, 1.0 );
  }

  void setupTexture();
};


#endif
