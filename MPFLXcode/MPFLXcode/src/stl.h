#ifndef _STL_H
#define	_STL_H


#include "linalg.h"
#include "gpuProgram.h"


class STLTriangle {
  
public:

    vec3 normal;
    vec3 point1;
    vec3 point2;
    vec3 point3;
    vec3 centroid;

    float d1, d2, d3;

    bool send;

    STLTriangle() {}

    STLTriangle( vec3 n, vec3 p1, vec3 p2, vec3 p3 ) {

      point1 = p1;
      point2 = p2;
      point3 = p3;

      d1 = d2 = d3 = 0;

      /* normal = n.normalize(); */
      normal = ((p2-p1) ^ (p3-p1)).normalize();

      centroid = 0.333333 * (p1 + p2 + p3);
    }

    STLTriangle( vec3 n, vec3 p1, vec3 p2, vec3 p3, float dist1, float dist2, float dist3 ) {

      point1 = p1;
      point2 = p2;
      point3 = p3;

      d1 = dist1;
      d2 = dist2;
      d3 = dist3;

      normal = n.normalize();

      centroid = 0.333333 * (p1 + p2 + p3);
    }

    bool rayInt( vec3 rayStart, vec3 rayDir, vec3 &intPoint, vec3 &intNorm, float &intParam );

    float pointDist( vec3 p );
    vec3 closestPoint( vec3 p );
};


class STL {

  GLuint VAO;
  GLuint VBO;

 public:

  seq<STLTriangle*> tris;

  mat4 objToWorldTransform;
  vec3 centre;
  float radius;

  STL() {}
  
  STL( const char *filename ) {
    read( filename );
    setupVAO();
    objToWorldTransform = identity4();
  }

  STL( const STL & source ) { // copy constructor

    tris = source.tris; // copy sequence of triangles

    objToWorldTransform = source.objToWorldTransform;
    centre = source.centre;
    radius = source.radius;
    VAO = source.VAO;
    VBO = source.VBO;
  }

  void read( const char * fileName );
  void setupVAO();
  void draw();

  bool findFirstInt( vec3 rayStart, vec3 rayDir, vec3 &intPoint, vec3 &intNorm, float &intParam );

  vec3 projectOntoSurface( vec3 p, vec3 *norm );
};


#endif
