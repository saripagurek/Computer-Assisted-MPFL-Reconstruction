// shortest.cpp


#include "main.h"
#include "shortest.h"


// Find the shortest path from capturedPoints[0] to capturedPoints[i]
//
// Rotate a plane around an axis between the two points.  At each
// position of the plane, intersect the plane with the objects to get
// a set of points.  Build the 2D hull of these points.  Find the
// shorter of the two paths on the hull between capturedPoints[0] to
// capturedPoints[i].
//
// Repeat for a number of angles to find the shortest path.


const int INIT_NUM_PLANE_ANGLES_TO_TEST = 180;
const int SUBSEQUENT_NUM_PLANE_ANGLES_TO_TEST = 10;
const float SUBSEQUENT_ANGLE_RANGE_TO_TEST = 10.0;


ShortestPath * findShortestPath( int capturedPtIndex );
void findConvexHull2D( vec2 *pts, int n, vec2 *hullPts, int &nHull );



// Order points lexicographically

#define LEX_SMALLER(a,b) (a.x < b.x || (a.x == b.x && (a.y < b.y || a.y == b.y && a.z < b.z)))

#define MIN(a,b) ((a) < (b) ? (a) : (b))



// capturedObjects[] and capturedPoints[] are assumed to hold
// the femoral point at [0] and the patellar point at [capturedPtIndex].
//
// The captured points are stored in the coordinate system of their own object.
//
// anim->transforms[0] is the obj-to-world transform of the patella in its initial (angle = 0) position.
//
// The epicondylar points are in the WCS.


void ShortestPath::findShortestPath( int capturedPtIndex )

{
  findShortestPath( capturedPoints[0], capturedPoints[capturedPtIndex], capturedPointLastAngle[capturedPtIndex] );
}



void ShortestPath::findShortestPath( vec3 femurPt, vec3 patellaPt, float &lastAngle )

{
  // Femoral (cp0) and patellar (cp) points in WCS
  
  vec3 cp0    = (objs[0]->objToWorldTransform * vec4(femurPt,1)).toVec3(); // femoral point
  vec3 cp     = (objs[1]->objToWorldTransform * vec4(patellaPt,1)).toVec3(); // patellar point in current position
  vec3 cpInit = (anim->transforms[0] * vec4(patellaPt,1)).toVec3(); // patellar point in initial position

  // Find angle around epicondylar axis relative to first position

  vec3 epiAxis = (leftEpicondyle - rightEpicondyle).normalize(); // epicondylar axis

  vec3 x = ((cpInit-leftEpicondyle) - ((cpInit-leftEpicondyle)*epiAxis)*epiAxis).normalize(); // direction to initial patellar point, but perpendicular to epiAxis
  vec3 y = (epiAxis ^ x).normalize();  // downward

  vec3 v1 = ((cp-leftEpicondyle) - ((cpInit-leftEpicondyle)*epiAxis)*epiAxis).normalize(); // direction to current patellar point, but perpendicular to epiAxis

  angle = atan2( v1*y, v1*x ) * 180.0 / M_PI; // 'angle' is instance var

  // Test many angles

  float minAngle = 0;
  len = MAXFLOAT;
  seq<vec3> *minPath = NULL;

  float startAngle, endAngle, angleStep;

  if (lastAngle == MAXFLOAT) {
    startAngle = 0;
    endAngle = 2*M_PI;
    angleStep = M_PI/(float) INIT_NUM_PLANE_ANGLES_TO_TEST;
  } else {
    startAngle = lastAngle - SUBSEQUENT_ANGLE_RANGE_TO_TEST/2;
    endAngle   = lastAngle + SUBSEQUENT_ANGLE_RANGE_TO_TEST/2;
    angleStep  = SUBSEQUENT_ANGLE_RANGE_TO_TEST / (float)SUBSEQUENT_NUM_PLANE_ANGLES_TO_TEST;
  }

  for (float theta=startAngle; theta < endAngle; theta += angleStep) {
    
    // Coordinate system in WCS
    
    vec3 z = (cp - cp0).normalize();
    vec3 y = z.perp2();
    vec3 x = z.perp1();

    // Plane normal in WCS
    
    vec3 n = cos(theta) * x + sin(theta) * y;

    seq<vec3> intPts;

    for (int i=0; i<objs.size(); i++) { // for each object

      // Move things into this object's local coordinate system

      mat4 T    = objs[i]->objToWorldTransform;
      mat4 Tinv = T.inverse();
      
      vec3 cp_local  = (Tinv * vec4(cp ,1)).toVec3();
      vec3 cp0_local = (Tinv * vec4(cp0,1)).toVec3();

      vec3 x_local   = (Tinv * vec4(y,  0)).toVec3(); 
      vec3 y_local   = (Tinv * vec4(x,  0)).toVec3();
      vec3 z_local   = (Tinv * vec4(z,  0)).toVec3();

      vec3 n_local   = (Tinv * vec4(n,  0)).toVec3();

      // Bounds of projections onto z axis between cp and cp0

      float minZ = cp_local  * z_local;
      float maxZ = cp0_local * z_local;
      
      if (minZ > maxZ) {
	float t = minZ; minZ = maxZ; maxZ = t; 
      }

      // Plane equation n_local dot x = d
    
      float d = cp0_local * n_local;

      // Check all triangle edges for plane crossing
      
      for (int j=0; j<objs[i]->tris.size(); j++) { // for each object triangle

	STLTriangle *tri = objs[i]->tris[j];
	vec3 pts[3] = { tri->point1, tri->point2, tri->point3 };

	for (int k=0; k<3; k++) { // for each triangle edge

	  vec3 a = pts[k];
	  vec3 b = pts[(k+1)%3];
	  if (LEX_SMALLER(a,b)) { // don't do both directions of an edge from different triangles

	    float denom = n_local * (b-a);
	    if (fabs(denom) > 0.0001) { // edge is not parallel to plane

	      float t = (d - n_local * a) / denom;
	      if (t >=0 && t <= 1) { // edge crosses plane

		vec3 pt = a + t * (b-a);
		if (pt * z_local > minZ && pt * z_local < maxZ) // int point is between cp and cp0

		  intPts.add( (T * vec4( pt, 1 )).toVec3() ); // record int point in world coordinate system
	      }
	    }
	  }
	}
      }
    }
    
    // Move points into 2D <z,t> system where t is perpendicular to z
    // and to the plane normal, n.

    vec3 t = (z ^ n).normalize();
    
    vec2 pts2[ intPts.size() + 2 ];

    pts2[0] = vec2(0,0);  // add cp0

    pts2[1].x = (cp - cp0) * z; // add cp
    pts2[1].y = (cp - cp0) * t;
      
    for (int i=0; i<intPts.size(); i++) {
      pts2[i+2].x = (intPts[i] - cp0) * z;
      pts2[i+2].y = (intPts[i] - cp0) * t;
    }

    // Compute convex hull

    vec2 hullPts[ intPts.size()+2 ];
    int  nHull;

    findConvexHull2D( pts2, intPts.size()+2, hullPts, nHull );

    // Move points back to 3D

    vec3 pts3[ nHull ];
    for (int i=0; i<nHull; i++)
      pts3[i] = cp0 + hullPts[i].x * z + hullPts[i].y * t;

    // Find indices of cp0 and cp on the hull (these points might not
    // have exactly the same coordinates as before due to transformations)

    int idx1, idx2;

    float minDist = MAXFLOAT;
    
    for (int i=0; i<nHull; i++) {
      float d = (pts3[i] - cp0).squaredLength();
      if (d < minDist) {
	minDist = d;
	idx1 = i;
      }
    }

    minDist = MAXFLOAT;

    for (int i=0; i<nHull; i++) {
      float d = (pts3[i] - cp).squaredLength();
      if (d < minDist) {
	minDist = d;
	idx2 = i;
      }
    }

    if (idx1 == idx2)
      continue; // no hull
    
    // Find distances idx1 -> idx2 and idx2 -> idx1

    float dist1to2 = 0;    
    float dist2to1 = 0;

    for (int i=idx1; i != idx2; i=(i+1)%nHull)
      dist1to2 += (pts3[ (i+1)%nHull ] - pts3[ i ]).length();

    for (int i=idx2; i != idx1; i=(i+1)%nHull)
      dist2to1 += (pts3[ (i+1)%nHull ] - pts3[ i ]).length();

    // Update shortest path over all rotations

    float pathMinDist = MIN(dist1to2,dist2to1);

    if (pathMinDist < len) {

      len = pathMinDist;
      minAngle = theta;

      if (minPath != NULL)
	delete minPath;

      points.clear();

      // Copy shortest path into 'minPath'

      if (dist1to2 < dist2to1) {  // from 1 to 2

	for (int i=idx1; i != idx2; i=(i+1)%nHull)
	  points.add( pts3[i] );
	points.add( pts3[idx2] );

      } else {  // from 2 to 1

	for (int i=idx2; i != idx1; i=(i+1)%nHull)
	  points.add( pts3[i] );
	points.add( pts3[idx1] );
      }
    }
  }

  lastAngle = minAngle;

  tightenPath();
}


// Graham Scan algorithm for convex hull


#ifdef MACOS
int compareCHpts( void *origin, const void *p1, const void *p2 )
#else
int compareCHpts( const void *p1, const void *p2, void *origin )
#endif
  
{
  float angle1 = atan2f( ((vec2*) p1)->y - ((vec2*) origin)->y, ((vec2*) p1)->x - ((vec2*) origin)->x );
  float angle2 = atan2f( ((vec2*) p2)->y - ((vec2*) origin)->y, ((vec2*) p2)->x - ((vec2*) origin)->x );

  if (angle1 < angle2)
    return -1;
  else if (angle1 > angle2)
    return +1;
  else
    return 0;
}



// Return > 0 for left turn a,b,c or < 0 for right turn
 
float ShortestPath::turn( vec2 a, vec2 b, vec2 c )

{
  return a.x * (b.y - c.y) - b.x * (a.y - c.y) + c.x * (a.y - b.y);
}
 

  
void ShortestPath::findConvexHull2D( vec2 *pts, int n, vec2 *hullPts, int &nHull )

{
  // Put leftmost point into pts[0]
  
  // for (int i=1; i<n; i++)
  //   if (pts[i].x < pts[0].x || pts[i].x == pts[0].x && pts[i].y < pts[0].y) {
  //     vec2 t = pts[0];
  //     pts[0] = pts[i];
  //     pts[i] = t;
  //   }

  // Order other points radially around pts[0]

#ifdef MACOS
  qsort_r( &pts[1], n-1, sizeof(vec2), &pts[0], compareCHpts );
#else
  qsort_r( &pts[1], n-1, sizeof(vec2), compareCHpts, &pts[0] );
#endif
  
  // Build hull

  hullPts[0] = pts[0];
  hullPts[1] = pts[1];
  hullPts[2] = pts[2];
  nHull = 3;

  for (int i=3; i<n; i++) {

    while (nHull > 2 && turn( hullPts[nHull-2], hullPts[nHull-1], pts[i]) <= 0) // right turn
      nHull--;

    hullPts[nHull] = pts[i];
    nHull++;
  }

  // Finally, check that last point added doesn't cause a right-hand turn with very first point

  while (nHull > 2 && turn( hullPts[nHull-2], hullPts[nHull-1], hullPts[0]) <= 0) // right turn
    nHull--;
}
 
  
// Tighten this so-called "shortest" path


void ShortestPath::tightenPath()

{
}


void ShortestPath::draw( vec4 &colour, mat4 &WCS_to_VCS, mat4 &WCS_to_CCS, vec3 &lightDirVCS )
  
{
  for (int j=0; j<points.size()-1; j++) {

    // end points
	
    vec3 p0 = points[j];
    vec3 p1 = points[j+1];

    // common "almost perpendiculars" to two segments at each end point
	
    vec3 x0;
    if (j == 0)
      x0 = (p1-p0).perp1().normalize();
    else {
      x0 = (points[j-1] - p0).normalize() + (p1-p0).normalize();
      if (x0.length() < 0.0001)
	x0 = (p1-p0).perp1().normalize();
      else
	x0 = x0.normalize();
    }
	
    vec3 x1;
    if (j == points.size()-2)
      x1 = (p1-p0).perp1().normalize();
    else {
      x1 = (points[j+2] - p1).normalize() + (p0-p1).normalize();
      if (x1.length() < 0.0001)
	x1 = (p1-p0).perp1().normalize();
      else
	x1 = x1.normalize();
    }

    if (x0 * x1 < 0) // (otherwise directions around circle might be different at two ends)
      x0 = -1 * x0;

    // other perpendiculars to x and axis

    vec3 y0 = (x0 ^ (p1-p0)).normalize();
    vec3 y1 = (x1 ^ (p1-p0)).normalize();

    // Build a cylinder

    const int NUM_CYL_FACES = 50;
    const float CYL_RADIUS = 0.5;
	  
    vec3 ps[ 2*(NUM_CYL_FACES+1) ];
    vec3 ns[ 2*(NUM_CYL_FACES+1) ];

    for (int i=0; i<NUM_CYL_FACES+1; i++) {
      float theta = i * (2*M_PI / (float) NUM_CYL_FACES);

      ns[2*i  ] = cos(theta)*x0 + sin(theta)*y0;
      ns[2*i+1] = cos(theta)*x1 + sin(theta)*y1;

      ps[2*i  ] = p0 + CYL_RADIUS * ns[2*i  ];
      ps[2*i+1] = p1 + CYL_RADIUS * ns[2*i+1];
    }

    segs->drawSegs( GL_TRIANGLE_STRIP, &ps[0], colour, &ns[0], 2*(NUM_CYL_FACES+1), WCS_to_VCS, WCS_to_CCS, lightDirVCS );
  }
}
