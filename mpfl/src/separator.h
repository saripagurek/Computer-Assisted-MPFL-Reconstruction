// separator.h

#ifndef SEPARATOR_H
#define SEPARATOR_H


#include "headers.h"
#include "stl.h"
#include "priority.h"


class Separator {

  int readline_from_pipe( int pipe, char *buff, int buffSize );

 public:

  mat4 objToWorldTransform;

  seq<vec3>   endpoints;
  seq<vec3>   skeletonPoints;
  seq<float>  skeletonDists;
  seq<vec3>   skeletonNorms;

  vec3 *curvature1 = NULL;
  vec3 *curvature2 = NULL;
  float *lambda1 = NULL;
  float *lambda2 = NULL;
  float scaleMinDist = MAXFLOAT;
  float scaleMaxDist = MAXFLOAT;

  seq<int> *adjPts;

  STL *stl;          // stores midway surface
  int selectedAdjPt;
  vec3 c1, c2;	     // principal curvature directions at selectedAdjPt
  vec3 norm;

  seq<vec3> closestPts; // for debugging
  seq<vec3> closestNorms; // for debugging

  Separator() {
    stl = NULL;
    adjPts = NULL;
    curvature1 = NULL;
    curvature2 = NULL;
    lambda1 = NULL;
    lambda2 = NULL;
    scaleMinDist = MAXFLOAT;
    scaleMaxDist = MAXFLOAT;
  }

  Separator( seq<STL*> &objs, int maxNumClosestPts, float maxClosestPtsDist ) {
    stl = NULL;
    adjPts = NULL;
    curvature1 = NULL;
    curvature2 = NULL;
    lambda1 = NULL;
    lambda2 = NULL;
    scaleMinDist = MAXFLOAT;
    scaleMaxDist = MAXFLOAT;

    compute( objs, maxNumClosestPts, maxClosestPtsDist, false );
  };

  Separator( Separator &sep ) {

    objToWorldTransform = sep.objToWorldTransform;
    
    endpoints      = sep.endpoints;
    skeletonPoints = sep.skeletonPoints;
    skeletonDists  = sep.skeletonDists;
    skeletonNorms  = sep.skeletonNorms;

    curvature1 = new vec3[ skeletonPoints.size() ];
    curvature2 = new vec3[ skeletonPoints.size() ];
    lambda1    = new float[ skeletonPoints.size() ];
    lambda2    = new float[ skeletonPoints.size() ];

    for (int i=0; i<skeletonPoints.size(); i++) {
      curvature1[i] = sep.curvature1[i];
      curvature2[i] = sep.curvature2[i];
      lambda1[i]    = sep.lambda1[i];
      lambda2[i]    = sep.lambda2[i];
    }

    scaleMinDist = sep.scaleMinDist;
    scaleMaxDist = sep.scaleMaxDist;
    
    adjPts = NULL; // cheating

    if (sep.stl == NULL)
      stl = NULL;
    else
      stl = new STL( *sep.stl );
    
    selectedAdjPt = sep.selectedAdjPt;
    c1 = sep.c1;
    c2 = sep.c2;
    norm = sep.norm;

    closestPts = sep.closestPts;
    closestNorms = sep.closestNorms;
  }

  void compute( seq<STL*> &objs, int maxNumClosestPts, float maxClosestPtsDist, bool force );

  void buildSTL();

  void draw( mat4 &MV, mat4 &MVP, vec3 &lightDir );
  void selectPoint( vec3 start, vec3 dir, int maxNumClosestPts, float maxClosestPtsDist );
  void findPrincipalCurvatures( float &lambda1, float &lambda2, vec3 &c1, vec3 &c2, int ptIndex, int maxNumClosestPts, float maxClosestPtDist );
  void drawSelectedPoint( mat4 &MV, mat4 &MVP, vec3 &lightDir, int maxNumClosestPts, float maxClosestPtDist );
  void findClosestPoints( seq<int> &results, int ptIndex, float maxDist, int maxNumPts );
  vec3 findNorm( int ptIndex, int maxNumClosestPts, float maxClosestPtsDist );

  void save( int currentStep );
  bool load( int currentStep );
  bool deleteSeparatorFile( int step );
  void shiftSeparatorFile( int from, int to );
};

#endif
