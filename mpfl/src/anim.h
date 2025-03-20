// anim.h

#ifndef ANIM_H
#define ANIM_H


#include "main.h"
#include "separator.h"
#include "headers.h"
#include "shortest.h"


class Anim {

 public:

  int currentStep;

  seq<mat4>          transforms;
  seq<Separator*>    separators;
  seq<ShortestPath**> allShortestPaths;  // allShortestPaths[ <step> ][ <captured point> ] is a "ShortestPath" instance

  STL *patellaObj;
  STL *femurObj;

  char *statePath;
  char *stateDir;
  char *objFile0;
  char *objFile1;

  float initAvgDist;

  Anim() {
    currentStep = 0;
    patellaObj = NULL;
    initAvgDist = -1;
  }
  
  Anim( char *path ) {
    currentStep = 0;
    patellaObj = NULL;
    initAvgDist = -1;
    statePath = strdup(path);
    stateDir = getContainingDir(path);
  }

  void findTransform( mat4 &T, float distance, vec3 overallRotationAxis, vec3 overallRotationCentre, float correctionProportion );
  void advance( float distance );
  float directionWeight( float lambda1, float lambda2, float dist );

  bool stepForward() { return step(1); }
  bool stepBackward() { return step(-1); }
  void refresh() { step(0); }

  void recordShortestPaths( ShortestPath **shortestPaths, int frameIndex );
  void saveState();
  void loadState();
  void saveView();
  void loadView();
  void run( int dir );
  void recomputeShortestPaths();
  void setPatellaObj();
  void deleteCurrentStep();
  void computeShortestPathsForGrid();

  double calculateDistance(const mat4 &transform1, const mat4 &transform2);
  
  bool step( int i );
  
  char *getContainingDir( char *path ) {    // Find directory containing 'path'
    
    char *p = strrchr( path, '/' );
    if (p == NULL)
      p = strrchr( path, '\\' ); // in case of Windows

    if (p == NULL)
      return strdup( "." );
    else {
      *p = '\0';
      return strdup(path);
    }
  }

};

#endif
