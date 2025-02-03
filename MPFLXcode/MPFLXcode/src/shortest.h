// shortest.h
//
// for finding shortest paths



#ifndef SHORTEST

#define SHORTEST


#include "headers.h"
#include "seq.h"
#include "linalg.h"


class ShortestPath {

public:

  seq<vec3>  points;
  float      len;
  float      angle;

  ShortestPath() {
    points = seq<vec3>();
    len = 0;
    angle = MAXFLOAT;
  }

  ShortestPath( int capturedPtIndex ) {
    findShortestPath( capturedPtIndex );
  }

  ~ShortestPath() {
  }

  ShortestPath( vec3 femurPt, vec3 patellaPt, float &angle ) {
    findShortestPath( femurPt, patellaPt, angle );
  }

  void findShortestPath( vec3 femurPt, vec3 patellaPt, float &lastAngle );
  void findShortestPath( int capturedPtIndex );
  void tightenPath();
  void draw( vec4 &colour, mat4 &WCS_to_VCS, mat4 &WCS_to_CCS, vec3 &lightDirVCS );
  void findConvexHull2D( vec2 *pts, int n, vec2 *hullPts, int &nHull );
  float turn( vec2 a, vec2 b, vec2 c );
};


#endif
