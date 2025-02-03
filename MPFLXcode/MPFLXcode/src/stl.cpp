// stl.cpp


#include "headers.h"
#include "linalg.h"
#include "stl.h"

#include <fstream>
#include <cstdlib>
#include <cstdio>
#include <cstring>
#include <iostream>
#include <iomanip>
#include <math.h>


// Read STL file

void STL::read( const char *fileName )

{
  char oneline[255];
  float t1, t2, t3;

  char arg1[100], arg2[20];

  FILE *in;

  //cout << "Reading " << fileName << endl;
  
  in = fopen(fileName, "r");
  int count = 0;
  vec3 n,p1,p2,p3;

  // First line: ASCII or RAW?

  fgets(oneline, 6, in);

  bool isAscii = (strncasecmp( oneline, "solid", 5 ) == 0);

  if (isAscii) {

    // Read ASCII STL

    while (!feof(in)) {

      fgets(oneline, 255, in);
      sscanf(oneline, "%s", arg1);

      if (strcasecmp(arg1, "facet") == 0) /* Is this a normal ?? */ {

	sscanf(oneline, "%s %s %f %f %f", arg1, arg2, &t1, &t2, &t3);
	n = vec3(t1,t2,t3);

      } else if (strcasecmp(arg1, "vertex") == 0) /* Is it a vertex ?  */ {
      
	sscanf(oneline, "%s %f %f %f", arg1, &t1, &t2, &t3);
	p1 = vec3(t1,t2,t3);

	fgets(oneline, 255, in); /* Next two lines vertices also!  */
	sscanf(oneline, "%s %f %f %f", arg1, &t1, &t2, &t3);
	p2 = vec3(t1,t2,t3);

	fgets(oneline, 255, in);
	sscanf(oneline, "%s %f %f %f", arg1, &t1, &t2, &t3);
	p3 = vec3(t1,t2,t3);

      } else if (strcasecmp(arg1, "endfacet") == 0)
      
	tris.add( new STLTriangle(n,p1,p2,p3) );
    }

  } else {

    // Read RAW STL

    if (sizeof(unsigned int) != 4) {
      cerr << "Expected unsigned int to be 4 bytes, but it is " << sizeof( unsigned int ) << ".  Fix this." << endl;
      exit(1);
    }

    if (sizeof(short unsigned int) != 2) {
      cerr << "Expected short unsigned int to be 2 bytes, but it is " << sizeof( short unsigned int ) << ".  Fix this." << endl;
      exit(1);
    }

    if (sizeof(float) != 4) {
      cerr << "Expected float to be 4 bytes, but it is " << sizeof( float ) << ".  Fix this." << endl;
      exit(1);
    }

    // Skip 80-byte header

    int count=0;

    while (oneline[count] != '\0') // already read in above
      count++;

    while (count<80) {
      fgetc(in);
      count++;
    }
    
    unsigned int numTriangles;

    fread( &numTriangles, sizeof(numTriangles), 1, in );

    // Read the triangles

    for (unsigned int i=0; i<numTriangles; i++) {

      vec3 n, p1, p2, p3;
      short unsigned int nAttr;

      fread( &n.x,  sizeof(float), 3, in );
      fread( &p1.x, sizeof(float), 3, in );
      fread( &p2.x, sizeof(float), 3, in );
      fread( &p3.x, sizeof(float), 3, in );
      fread( &nAttr, sizeof(short unsigned int), 1, in );

      tris.add( new STLTriangle( n, p1, p2, p3 ) );
    }
  }

  fclose(in);

  // Find object centre and radius

  vec3 sum(0,0,0);
  
  for (int i=0; i<tris.size(); i++)
    sum = sum + tris[i]->centroid;

  centre = (1/(float)tris.size()) * sum;

  radius = 0;
  for (int i=0; i<tris.size(); i++) {
    float r = (tris[i]->point1 - centre).length();
    if (r > radius)
      radius = r;
    r = (tris[i]->point2 - centre).length();
    if (r > radius)
      radius = r;
    r = (tris[i]->point3 - centre).length();
    if (r > radius)
      radius = r;
  }

  //cout << tris.size() << " triangles, centre " << centre << ", radius " << radius << endl;
}



// Compute plane/ray intersection, and then the local coordinates to
// see whether the intersection point is inside.

bool STLTriangle::rayInt( vec3 rayStart, vec3 rayDir, vec3 & intPoint, vec3 & intNorm, float & intParam )

{
  float param;
  vec3 point, lc;


  // Compute ray/plane intersection

  float dn = rayDir * normal;

  if (fabs(dn) < 0.0001) 	// *** CHANGED TO ALLOW INTERSECTION FROM BEHIND TRIANGLE ***
    return false;		// ray is parallel to plane

  float dist = point1 * normal;

  param = (dist - rayStart*normal) / dn;
  if (param < 0)
    return false;		// plane is behind starting point

  point = rayStart + param * rayDir;

  // Compute barycentric coords

  float totalArea = ((point2-point1) ^ (point3-point1)) * normal;

  float u = (((point3-point2) ^ (point - point2)) * normal) / totalArea;

  float v = (((point1-point3) ^ (point - point3)) * normal) / totalArea;

  // Reject if outside triangle

  if (u < 0 || v < 0 || u + v > 1)
    return false;

  // Return int point and normal and parameter

  intParam = param;
  intNorm = normal;
  intPoint = point;

  return true;
}



// Set up the VAO

void STL::setupVAO()

{
  // Start setting up a VAO
    
  glGenVertexArrays( 1, &VAO );
  glBindVertexArray( VAO );

  // Define the VBO
    
  glGenBuffers( 1, &VBO );
  glBindBuffer( GL_ARRAY_BUFFER, VBO );

  glVertexAttribPointer( 0, 3, GL_FLOAT, GL_FALSE, 0, 0 ); // attrib 0: three floats for a position
  glEnableVertexAttribArray( 0 );

  glVertexAttribPointer( 1, 3, GL_FLOAT, GL_FALSE, 0, (void *) ( 3*tris.size()*sizeof(vec3)) ); // attrib 1: three floats for a normal
  glEnableVertexAttribArray( 1 );

  glVertexAttribPointer( 2, 1, GL_FLOAT, GL_FALSE, 0, (void *) ( 6*tris.size()*sizeof(vec3)) ); // attrib 2: one float for distance
  glEnableVertexAttribArray( 2 );

  // Set up buffer for position and normal data

  vec3 *buff = new vec3[ 7*tris.size() ];

  vec3 *p = buff;
  
  for (int i=0; i<tris.size(); i++) {
    *p++ = tris[i]->point1;
    *p++ = tris[i]->point2;
    *p++ = tris[i]->point3;
  }

  for (int i=0; i<tris.size(); i++) {
    *p++ = tris[i]->normal;
    *p++ = tris[i]->normal;
    *p++ = tris[i]->normal;
  }

  for (int i=0; i<tris.size(); i++)
    *p++ = vec3( tris[i]->d1, tris[i]->d2, tris[i]->d3 );

  glBufferData( GL_ARRAY_BUFFER, 7*tris.size() * sizeof(vec3), buff, GL_STATIC_DRAW );

  // Done

  glBindVertexArray( 0 );
  glBindBuffer( GL_ARRAY_BUFFER, 0 );
  glBindBuffer( GL_ELEMENT_ARRAY_BUFFER, 0 );

  delete [] buff;
}



// Draw

void STL::draw()

{
  glBindVertexArray( VAO );
  glDrawArrays( GL_TRIANGLES, 0, 3*tris.size() );
  glBindVertexArray( 0 );
}


bool STL::findFirstInt( vec3 rayStart, vec3 rayDir, vec3 &intPoint, vec3 &intNorm, float &intParam )

{
  // Move ray into coordinate system of object

  mat4 invT = objToWorldTransform.inverse(); 

  rayStart = (invT * vec4(rayStart,1)).toVec3(); // w=1 for position
  rayDir   = (invT * vec4(rayDir,  0)).toVec3(); // w=0 for direction

  // Test all triangles

  vec3  minIntPoint, minIntNorm;
  float minIntParam = MAXFLOAT;

  for (int i=0; i<tris.size(); i++) {
    vec3 intPoint, intNorm;
    float intParam;
    if (tris[i]->rayInt( rayStart, rayDir, intPoint, intNorm, intParam ) && intParam < minIntParam) {
	minIntPoint = intPoint;
	minIntNorm  = intNorm;
	minIntParam = intParam;
      }
  }

  if (minIntParam == MAXFLOAT) 
    return false;

  intPoint = (objToWorldTransform * vec4(minIntPoint)).toVec3();

  vec4 n(minIntNorm);
  n.w = 0; // since this is a direction
  intNorm = (objToWorldTransform * n).toVec3();

  intParam = minIntParam;

  return true;
}



float STLTriangle::pointDist( vec3 p )

{
  return (p - closestPoint(p)).length();
}



vec3 STLTriangle::closestPoint( vec3 p )

{
  // find barycentric coordinates
  
  mat2 M;

  vec3 a = point2-point1;
  vec3 b = point3-point1;
  vec3 x = p-point1;

  M.rows[0] = { a*a, a*b };
  M.rows[1] = { b*a, b*b };

  if (M.rows[0][0] < 1e-6 || M.rows[1][1] < 1e-6 || fabs(M.rows[0][0] - M.rows[0][1]) < 1e-6 || fabs(M.rows[1][0] - M.rows[1][1]) < 1e-6)
    return vec3(MAXFLOAT,MAXFLOAT,MAXFLOAT);

  vec2 bary = M.inverse() * vec2( a*x, b*x );

  // inside triangle?
  
  if (bary[0] >= 0 && bary[1] >= 0 && bary[0] + bary[1] <= 1)
    return point1 + bary[0] * a + bary[1] * b;

  // closest to which edge?

  vec3 h,t;
  
  if (bary[0] < 0) {
    h = point1; // 3-1 edge
    t = point3;
  } if (bary[1] < 0) {
    h = point2; // 1-2 edge
    t = point1;
  } else {
    h = point3; // 2-3 edge
    t = point2;
  }

  float proj = ((p-t) * (h-t)) / ((h-t) * (h-t));

  if (proj<0)
    return t;
  else if (proj>1)
    return h;
  else
    return t + proj * (h-t);
}



vec3 STL::projectOntoSurface( vec3 p, vec3 *norm )

{
  // Test all triangles

  int   minTri;
  float minDist = MAXFLOAT;

  for (int i=0; i<tris.size(); i++) {
    float d = tris[i]->pointDist( p );
    if (d < minDist) {
      minDist = d;
      minTri = i;
    }
  }

  if (minDist == MAXFLOAT) {
    cerr << "STL::projectOntoSurface() could not find closest triangle to point " << p << endl;
    return p;
  }

  if (norm != NULL)
    *norm = tris[minTri]->normal;
  
  return tris[minTri]->closestPoint( p );
}

