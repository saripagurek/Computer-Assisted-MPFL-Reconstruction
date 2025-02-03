// tube.h


#ifndef TUBE_H
#define TUBE_H


#include "linalg.h"
#include "seq.h"
#include "gpuProgram.h"


#define TUBE_RADIAL_COUNT 32
#define TUBE_LINEAR_COUNT 16

class Tube {

  int nVerts;
  int nFaces;
  
  vec3 *vertexBuffer;
  vec3 *normalBuffer;

  GLuint vertexBufferID;
  GLuint normalBufferID;

  GLuint VAO; 

  void setupVAO();

 public:

  Tube() {
    setupVAO();
  }

  ~Tube() {}

  void draw( vec3 a, vec3 b, vec3 c, vec3 d, float r );
};

#endif
