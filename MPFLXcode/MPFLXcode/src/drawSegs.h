// drawSegs.h
//
// Create a class instance:
//
//    Segs *segs = new Segs();
//
// Use it:
//
//    segs->drawOneSeg( tail, head, MVP );


#ifndef DRAW_SEGS_H
#define DRAW_SEGS_H

#include "headers.h"
#include "gpuProgram.h"


class Segs {

  static const char *fragmentShader;
  static const char *vertexShader;

  GPUProgram *setupShaders();

  GPUProgram *gpuProg;
  
 public:

  Segs() { 
    gpuProg = setupShaders();
  };

  // Main function
  
  void drawSegs( GLuint primitiveType, vec3 *pts, vec4 *colours, vec3 *norms, int nPts, mat4 &MV, mat4 &MVP, vec3 lightDir );

  // Variant: single colour
  
  void drawSegs( GLuint primitiveType, vec3 *pts, vec4 colour, vec3 *norms, int nPts, mat4 &MV, mat4 &MVP, vec3 lightDir ) {

    vec4 *colours = new vec4[ nPts ];
    for (int i=0; i<nPts; i++)
      colours[i] = colour;

    drawSegs( primitiveType, pts, colours, norms, nPts, MV, MVP, lightDir );    

    delete[] colours;
  }
};

#endif
