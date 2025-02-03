// strokefont.h


#ifndef STROKEFONT_H
#define STROKEFONT_H


#include "headers.h"
#include "gpuProgram.h"
#include <string>


typedef enum { LEFT, CENTRE, RIGHT } Alignment;


class StrokeFont {

  static const char *fontVertexShader;
  static const char *fontFragmentShader;

  GPUProgram *gpuProg;

 public:

  StrokeFont() {
    gpuProg = new GPUProgram();
    gpuProg->init( fontVertexShader, fontFragmentShader, "strokefont" );
  }

  void drawStrokeString( string str, float x, float y, float height, float theta, Alignment alignment, vec3 colour );
  void drawStrokeString( string str, float x, float y, float height, float theta, Alignment alignment, vec3 colour, float aspect );
};


#endif
