// graph.h
//
// Draw a graph in a subwindow


#include "headers.h"
#include "seq.h"


#define TEXT_HEIGHT 0.03  // in [-1,1]x[-1,1] window

#define TICK_LENGTH 7 // in pixels 


// One line on the graph

class Trace {

 public:
  
  char *name;
  vec4 colour;
  seq<vec2> points;

  Trace() {}

  Trace( char *n, vec4 c ) {
    name = strdup(n);
    colour = c;
  }
};



// Graph of many traces
//
// Graph lies between lowerLeft and upperRight in [-1,1]x[-1,1] window

class Graph {

  vec3 ll, ur;
  char *horizName, *vertName;

  char *hFormat, *vFormat; // printf format for axis labels
  int hTicks, vTicks; // number of ticks

 public:

  float hMin, hMax, vMin, vMax;  // data bounds

  seq<Trace> traces;

  Graph() {}
  
  Graph( vec2 lowerLeft, vec2 upperRight, char *hName, char *vName, float _hMin, float _hMax, float _vMin, float _vMax, char *_hFormat, char *_vFormat, int _hTicks, int _vTicks ) {

    ll = vec3( lowerLeft.x,  lowerLeft.y,  0 );
    ur = vec3( upperRight.x, upperRight.y, 0 );

    horizName = strdup(hName);
    vertName = strdup(vName);

    hMin = _hMin;
    hMax = _hMax;
    vMin = _vMin;
    vMax = _vMax;

    hFormat = strdup( _hFormat );
    vFormat = strdup( _vFormat );

    hTicks = _hTicks;
    vTicks = _vTicks;
  }

  void draw( float xPos, bool keepVScale );
};
