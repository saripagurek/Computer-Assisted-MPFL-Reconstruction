// sphere.h


#ifndef SPHERE_H
#define SPHERE_H


#include "linalg.h"
#include "seq.h"
#include "gpuProgram.h"


// icosahedron vertices (taken from Jon Leech http://www.cs.unc.edu/~jon)

#define sphere_tau (0.8506508084)      /* t=(1+sqrt(5))/2, tau=t/sqrt(1+t^2)  */
#define sphere_one (0.5257311121)      /* sphere_one=1/sqrt(1+t^2) , unit sphere     */

#define NUM_VERTS 12
#define NUM_FACES 20


class SphereFace {
 public:
  unsigned int v[3];
  SphereFace() {}
  SphereFace( int v0, int v1, int v2 ) {
    v[0] = v0; v[1] = v1; v[2] = v2;
  }
};


class Sphere {

  static const char *fragmentShader;
  static const char *vertexShader;

  GPUProgram *setupShaders();

  GPUProgram *gpuProg;

  public:

  Sphere() {
    centre = vec3(0,0,0);
    radius = 1;
    buildSphere( 2 );
    gpuProg = setupShaders();
  }

  Sphere( vec3 c, float r ) {
    centre = c;
    radius = r;
    buildSphere( 2 );
    gpuProg = setupShaders();
  }

  // A sphere with 0 levels is a truncated icosahedron.  Each
  // additional level refines the previous level by converting each
  // triangular face into four triangular faces, placing all face
  // vertices at distance 1 from the origin.
  
  void buildSphere( int numLevels ) {

    for (int i=0; i<NUM_VERTS; i++)
      verts.add( icosahedronVerts[i] );

    for (int i=0; i<verts.size(); i++)
      verts[i] = verts[i].normalize();

    for (int i=0; i<NUM_FACES; i++)
      faces.add( SphereFace( icosahedronFaces[i][0],
                             icosahedronFaces[i][1],
                             icosahedronFaces[i][2] ) );

    for (int i=0; i<numLevels; i++)
      refine();

    //gpu.init( vertShader, fragShader, "in sphere.h" );

    setupVAO();
  };

  ~Sphere() {}

  bool rayInt( vec3 rayStart, vec3 rayDir, int objPartIndex, float maxParam,
	       vec3 &intPoint, vec3 &intNorm, vec3 &intTexCoords, float &intParam, int &intPartIndex );

  void input( istream &stream );
  void output( ostream &stream ) const;

  vec3 textureColour( vec3 &p, int objPartIndex, float &alpha, vec3 &texCoords );

  void renderGL( mat4 &MV, mat4 &MVP, vec3 &lightDir, vec4 colour );

 private:

  vec3   centre;
  float  radius;

  seq<vec3>       verts;
  seq<SphereFace> faces;
  GLuint          VAO; 
  
  GPUProgram      gpu;

  void refine();
  void setupVAO();

  static vec3 icosahedronVerts[NUM_VERTS];
  static int icosahedronFaces[NUM_FACES][3];
};

#endif
