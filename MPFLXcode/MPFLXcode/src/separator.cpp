// separator.cpp

#include "separator.h"
#include "main.h"
#include "priority.cpp"  // necessary to show implementation to compiler

#include <unistd.h>
#include <iostream>
#include <thread>
#include <mutex>
#include <sys/time.h>
#include <sys/types.h>
#include <sys/stat.h>

#include <armadillo>  // for SVD


#define ADJ_PT_MAX_DIST_FRACTION 0.75   // only accept adjacencies that are closer than this fraction of the max dist between the objects

#define ADJ_PT_MAX_ASPECT_RATIO  3.00   // only accept adjacencies on Delaunay triangle with max/min edge length less than this

#define MAX_PT_TO_PT_DIST MAXFLOAT

#define MIN(a,b) ((a) < (b) ? (a) : (b))
#define MAX(a,b) ((a) > (b) ? (a) : (b))
	  


void Separator::compute( seq<STL*> &objs, int maxNumClosestPts, float maxClosestPtsDist, bool force )

{
  // try getting a cached separator
  
  if (!force && load( anim->currentStep ))
    return;

#if 1

  // Gather points

  //cout << "Gathering points" << endl;

  int obj0 = (objs[0]->tris.size() < objs[1]->tris.size() ? 0 : 1); // obj0 = index of object with fewer triangles
  
  STL &m0 = *objs[obj0];     // m0 = object with fewer triangles
  STL &m1 = *objs[1-obj0];   // m1 = object with more triangles

  endpoints.clear();
  skeletonPoints.clear();
  skeletonDists.clear();

  // Find centroids
  
  vec3 centroid0(0,0,0);
  for (int i=0; i<m0.tris.size(); i++)
    centroid0 = centroid0 + m0.tris[i]->centroid;
  centroid0 = (1.0/(float)m0.tris.size()) * centroid0;

  vec3 centroid1(0,0,0);
  for (int i=0; i<m1.tris.size(); i++)
    centroid1 = centroid1 + m1.tris[i]->centroid;
  centroid1 = (1.0/(float)m1.tris.size()) * centroid1;

  // Keep only triangles pointing to centroid of other object

  seq<vec3> m0triCentroids;
  seq<vec3> m0triNormals;
  seq<vec3> m1triCentroids;
  seq<vec3> m1triNormals;

  vec3 m1_centroid_in_m0_system = (m0.objToWorldTransform.inverse() * m1.objToWorldTransform * vec4(centroid1,1)).toVec3();

  for (int i=0; i<m0.tris.size(); i++)
    if (m0.tris[i]->normal * (m0.tris[i]->centroid - m1_centroid_in_m0_system) < 0) {
      m0triCentroids.add( (m0.objToWorldTransform * vec4 (m0.tris[i]->centroid, 1 )).toVec3() );
      m0triNormals.add(   (m0.objToWorldTransform * vec4( m0.tris[i]->normal,   0 )).toVec3() );
    }

  vec3 m0_centroid_in_m1_system = (m1.objToWorldTransform.inverse() * m0.objToWorldTransform * vec4(centroid0,1)).toVec3();

  for (int i=0; i<m1.tris.size(); i++)
    if (m1.tris[i]->normal * (m1.tris[i]->centroid - m0_centroid_in_m1_system) < 0) {
      m1triCentroids.add( (m1.objToWorldTransform * vec4 (m1.tris[i]->centroid, 1 )).toVec3() );
      m1triNormals.add(   (m1.objToWorldTransform * vec4( m1.tris[i]->normal,   0 )).toVec3() );
    }

  float minDot = cos( interobjectMaxAngle * M_PI/180.0 ); // 20 degrees

  // Run a bunch of threads to do this

  int numCPUs = thread::hardware_concurrency();

  if (numCPUs == 0) // == 0 if unknown
    numCPUs = 1;

  // Find pairs of points on m0 and m1 such that the segment between
  // the points is within 10 degrees (i.e. interobjectMaxAngle) of
  // the surface normal at each point.  Consider only triangle
  // centroids.

  int numItems = m0triCentroids.size();
  int itemsPerThread = ceil( numItems / (float) numCPUs );

  int N = numCPUs * itemsPerThread;

  bool filled[ N ];
  for (int i=0; i<N; i++)
    filled[i] = false;

  vec3  allEndpoints[ 2*N ];
  vec3  allSkeletonPoints[ N ];
  float allSkeletonDists[ N ];

  // mutex mut;
  
  auto f = [&]( int k ) 
    {
      // Determine range of points that this thread will handle
      
      int start = k * itemsPerThread;
      int end   = (k+1) * itemsPerThread;

      if (start >= numItems)
	return;
      
      if (end > numItems)
	end = numItems;

      // Handle this range of points

      for (int i=start; i<end; i++) {

	if (k == 0 && i % 100 == 0) { // thread 0 output progress updates
	  // cout << "\rFind closest points " << (end-i)/100 << "   "; 
	  // cout.flush();
	}

	// Find closest point on m1 to this m0 point
	
	vec3 &v0 = m0triCentroids[i];
	vec3 &n0 = m0triNormals[i];

	float minDist = MAX_PT_TO_PT_DIST;
	int   minJ;
	vec3  minDiff;
	vec3  minN1;

	for (int j=0; j<m1triCentroids.size(); j++) {

	  vec3 &v1 = m1triCentroids[j];
	  vec3 &n1 = m1triNormals[j];

	  vec3 diff = v1 - v0; // direction from m0 to m1
	  float dist = diff.length();
	  
	  if (dist < minDist && diff * n0 > minDot*dist && diff * n1 < -minDot*dist) {
	    // direction points above m0 surface and negative direction points above m1 surface
	    minDist = dist;
	    minJ = j;
	    minDiff = diff;
	    minN1 = n1;
	  }
	}

	if (minDist != MAX_PT_TO_PT_DIST) { // &&
	  //minDiff * n0    >  minDot*minDist &&
	  //minDiff * minN1 < -minDot*minDist) {

	  vec3 &v1 = m1triCentroids[minJ];

	  allEndpoints[2*i] = v0;
	  allEndpoints[2*i+1] = v1;
	  
	  allSkeletonPoints[i] = 0.5*(v0+v1);
	  allSkeletonDists[i] = (v1-v0).length();

	  filled[i] = true;
	  
	  // lock_guard<mutex> guard(mut);

	  // MUTEX start

	  // endpoints.add( v0 );
	  // endpoints.add( v1 );
	  // skeletonPoints.add( 0.5*(v0+v1) );
	  // skeletonDists.add( (v1-v0).length() );

	  // MUTEX end
	}
      }
    };


  // Run threads and wait for completion
  
  thread threads[numCPUs];
  
  for (int k=0; k<numCPUs; k++)
    threads[k] = thread( f, k );

  for (int k=0; k<numCPUs; k++)
    threads[k].join();

  // cout << "\r                                \r";
  // cout.flush();

  // Copy results

  for (int i=0; i<N; i++)
    if (filled[i]) {
      endpoints.add( allEndpoints[2*i] );
      endpoints.add( allEndpoints[2*i+1] );
      skeletonPoints.add( allSkeletonPoints[i] );
      skeletonDists.add( allSkeletonDists[i] );
    }

#elif 1

  // Test points: half a cylinder

  float minZ = -1;
  float maxZ = +1;
  float radius = 0.5;
  
  skeletonPoints.clear();
  skeletonDists.clear();
  endpoints.clear();
  
  for (float z=minZ; z<maxZ; z += (maxZ-minZ)/100.0)
    for (float theta=0; theta<M_PI/2.0; theta+=M_PI/2.0/50.0) {
      skeletonPoints.add( vec3( radius*cos(theta), radius*sin(theta), z ) );
      skeletonDists.add( 1 ); // dummy value
      endpoints.add( vec3( 0, 0, z ) );
      endpoints.add( vec3( 2*radius*cos(theta), 2*radius*sin(theta), z ) );
    }

#else

  // Test points: saddle

  skeletonPoints.clear();
  skeletonDists.clear();
  endpoints.clear();
  
  for (float x=-1; x<1; x+= 0.025)
    for (float y=-1; y<1; y+= 0.025) {
      vec3 pt( x, y, 0.5*(x*x-y*y) );
      vec3 grad = vec3( 1, 1, x-y );
      vec3 n = (vec3( 1, 0, x ) ^ vec3( 0, 1, -y )).normalize();
      skeletonPoints.add( pt );
      skeletonDists.add( 1 ); // dummy value
      endpoints.add( pt + n );
      endpoints.add( pt - n );
    }

#endif

  // Normalize dists in [0,1]

  scaleMinDist = MAXFLOAT;
  scaleMaxDist = 0;

  for (int i=0; i<skeletonDists.size(); i++) {
    if (skeletonDists[i] < scaleMinDist) 
      scaleMinDist = skeletonDists[i];
    if (skeletonDists[i] > scaleMaxDist)
      scaleMaxDist = skeletonDists[i];
  }

  if (scaleMinDist == scaleMaxDist) // just in case
    scaleMaxDist = scaleMinDist + 1;

  // add a bit on each end of scale
    
  // float diff = scaleMaxDist - scaleMinDist;

  // scaleMinDist = scaleMinDist - 0.2 * diff;
  // if (scaleMinDist < 0)
  //   scaleMinDist = 0;
    
  // scaleMaxDist = scaleMaxDist + 0.2 * diff;
  
  for (int i=0; i<skeletonDists.size(); i++)
    skeletonDists[i] = (skeletonDists[i] - scaleMinDist) / (scaleMaxDist - scaleMinDist);

  if (scaleMinDist < globalScaleMinDist)
    globalScaleMinDist = scaleMinDist;
  if (scaleMaxDist > globalScaleMaxDist) {
    globalScaleMaxDist = scaleMaxDist;
    if (globalScaleMaxDist > MAX_GLOBAL_SCALE_MAX)
      globalScaleMaxDist = MAX_GLOBAL_SCALE_MAX;
  }
  
  // Send points to '/usr/bin/qdelaunay' and get response

  // child process communication from https://stackoverflow.com/questions/41495219/c-bidirectional-pipe-stuck-trying-to-read-from-child-process-in-a-loop
  
  int parent_to_child[2];
  int child_to_parent[2];

  pipe(parent_to_child);
  pipe(child_to_parent);

  // cout << "\rBuild Delaunay "; cout.flush();
  
  int childPID = fork();

  if (childPID == 0) { // child

    close(parent_to_child[1]); // Close the writing end of the incoming pipe
    close(child_to_parent[0]); // Close the reading end of the outgoing pipe

    dup2(parent_to_child[0], STDIN_FILENO);  // replace stdin with incoming pipe
    dup2(child_to_parent[1], STDOUT_FILENO); // replace stdout with outgoing pipe

    // exec qvoronoi
    
    char *filename = "/usr/bin/qdelaunay";
    char *argv[]   = { filename, "Qt", "Fv", NULL };
    char *env[]    = { NULL };

    execve( filename, argv, env ); // doesn't return

    // child doesn't get to here
  }

  // parent

  close( parent_to_child[0] ); // Close the reading end of the outgoing pipe.
  close( child_to_parent[1] ); // Close the writing side of the incoming pipe.

  // Write the points

  int numPts = skeletonPoints.size();

  //cout << "Sending " << numPts << " points" << endl;
  
#define BUFF_SIZE 100000
  char buff[ BUFF_SIZE ];

  // output "3D"

  sprintf( buff, "3\n" );
  write( parent_to_child[1], buff, strlen(buff) );

  // output number of points

  sprintf( buff, "%d\n", numPts );
  write( parent_to_child[1], buff, strlen(buff) );

  // Output the points

  for (int i=0; i<numPts; i++) {
    sprintf( buff, "%f %f %f\n", skeletonPoints[i].x, skeletonPoints[i].y, skeletonPoints[i].z );
    write( parent_to_child[1], buff, strlen(buff) );
  }

  close( parent_to_child[1] );

  // Get response

  //cout << "Now waiting for response" << endl;

  // Find max distance for chosen pairs ... use as scale

  float maxDist = 0;
  for (int i=0; i<endpoints.size(); i+=2) {
    float dist = (endpoints[i] - endpoints[i+1]).length();
    if (dist > maxDist)
      maxDist = dist;
  }

  maxDist *= ADJ_PT_MAX_DIST_FRACTION;  // only accept adjacencies that are closer than this fraction of the max dist between the objects

  // get number of Delaunay tetrahedra

  int numRegions;

  readline_from_pipe( child_to_parent[0], buff, BUFF_SIZE );
  sscanf( buff, "%d", &numRegions );

  //cout << "Reading " << numRegions << " Delaunay regions" << endl;

  // Point adjacencies

  if (adjPts != NULL)
    delete [] adjPts;
  
  adjPts = new seq<int>[ numPts ];  // each point stores its adjacent points

  // Debugging:
  
  //selectedAdjPt = 288;
  //lookAt = skeletonPoints[ selectedAdjPt ];
  
  // STL to show triangulation
  
  if (stl != NULL)
    delete stl;
  
  stl = new STL();

  for (int i=0; i<numRegions; i++) {

    readline_from_pipe( child_to_parent[0], buff, BUFF_SIZE );    // numVerts siteIndex1 siteIndex2 ...

    int numVerts;
    sscanf( buff, "%d", &numVerts ); // should always be 4 vertices, as each "region" is a tetrahedron

    // Read the indices of this region
    
    int indices[numVerts];

    char *p = buff;

    for (int j=0; j<numVerts; j++) {

      while (*p != ' ' && *p != '\n' && *p != '\0') // skip to next point index
	p++;
      p++;

      sscanf( p, "%d", &indices[j] );
    }

    // Store point adjacencies between all pairs of points on the tetrahedron

    for (int j=0; j<numVerts-2; j++)
      for (int k=j+1; k<numVerts-1; k++)
	for (int l=k+1; l<numVerts; l++) {

	  // Check that the j-k-l triangle isn't too large
	  
	  vec3 &vj = skeletonPoints[indices[j]];
	  vec3 &vk = skeletonPoints[indices[k]];
	  vec3 &vl = skeletonPoints[indices[l]];

	  float lenJK = (vj-vk).length();
	  float lenJL = (vj-vl).length();
	  float lenKL = (vk-vl).length();

	  float minLen = MIN( lenJK, MIN( lenJL, lenKL ) );
	  float maxLen = MAX( lenJK, MAX( lenJL, lenKL ) );

	  if (maxLen / minLen < ADJ_PT_MAX_ASPECT_RATIO && maxLen < maxDist) { // edge aspect ratio < 3, edge length < 0.75 * (max inter-object dist)

	    // Add vertex adjacencies, unless already existing
	    
	    if (adjPts[indices[j]].findIndex( indices[k] ) == -1) {
	      adjPts[indices[j]].add( indices[k] );
	      adjPts[indices[k]].add( indices[j] );
	    }

	    if (adjPts[indices[j]].findIndex( indices[l] ) == -1) {
	      adjPts[indices[j]].add( indices[l] );
	      adjPts[indices[l]].add( indices[j] );
	    }

	    if (adjPts[indices[k]].findIndex( indices[l] ) == -1) {
	      adjPts[indices[k]].add( indices[l] );
	      adjPts[indices[l]].add( indices[k] );
	    }

	    // Add face to STL.  (This will result in front- and back-facing copies of the same triangle from face-adjacent tetrahedra.)
	    
	    vec3 n = ((vk-vj) ^ (vl-vj)).normalize();

	    float dist1 = (endpoints[indices[j]*2] - endpoints[indices[j]*2+1]).length(); // dist between surfaces, but going through this point
	    float dist2 = (endpoints[indices[k]*2] - endpoints[indices[k]*2+1]).length();
	    float dist3 = (endpoints[indices[l]*2] - endpoints[indices[l]*2+1]).length();

	    stl->tris.add( new STLTriangle( n, vj, vk, vl, dist1, dist2, dist3 ) );
	  }
	}
  }

  close( child_to_parent[0] );

  stl->setupVAO();
  stl->objToWorldTransform = identity4();

  objToWorldTransform = identity4();

  // Compute normal at each point as that for the least-square plane in the neighbourhood

  skeletonNorms.clear();

  for (int i=0; i<skeletonPoints.size(); i++)
    skeletonNorms.add( findNorm( i, maxNumClosestPts, maxClosestPtsDist ) );

  // Debugging
  
  // int nanCount = 0;
  // for (int i=0; i<skeletonPoints.size(); i++) {
  //   if (skeletonPoints[i] != skeletonPoints[i]) {
  //     cout << "NAN point: " << skeletonPoints[i] << endl;
  //     nanCount++;
  //   }
  //   if (skeletonNorms[i] != skeletonNorms[i]) {
  //     cout << "NAN norm: " << skeletonNorms[i] << endl;
  //     nanCount++;
  //   }
  // }

  // if (nanCount > 0) {
  //   cout << "NaN found among skeleton points." << endl;
  //   exit(1);
  // }

  // Compute curvatures at each point

  //cout << "Finding curvatures" << endl;
  
  if (curvature1 != NULL) {
    delete [] curvature1;
    delete [] curvature2;
    delete [] lambda1;
    delete [] lambda2;
  }
    
  curvature1 = new vec3[ skeletonPoints.size() ];
  curvature2 = new vec3[ skeletonPoints.size() ];
  lambda1   = new float[ skeletonPoints.size() ];
  lambda2   = new float[ skeletonPoints.size() ];
  
  for (int i=0; i<skeletonPoints.size(); i++) {
    // cout << "\rCompute curvature " << skeletonPoints.size()-i << "     "; 
    // cout.flush();
    findPrincipalCurvatures( lambda1[i], lambda2[i], curvature1[i], curvature2[i], i, maxNumClosestPts, maxClosestPtsDist );
  }

  // cout << "\r                                \r";
  // cout.flush();

  // Store in file

  // cout << "Save new separator as " << anim->currentStep << endl;
  
  save( anim->currentStep );

  // Store with cached separators
  
  while (anim->currentStep >= allSeparators.size())
    allSeparators.add( new Separator() );

  // cout << "Add new separator to 'allSeparators' as " << anim->currentStep << endl;

  allSeparators[ anim->currentStep ] = new Separator( *separator );
}

  


vec3 Separator::findNorm( int ptIndex, int maxNumClosestPts, float maxClosestPtsDist )

{
  seq<int> ptIndices;
    
  findClosestPoints( ptIndices, ptIndex, maxClosestPtsDist, maxNumClosestPts );

  int n = ptIndices.size();

  if (n < 3) {
    // cout << "."; cout.flush();
    findClosestPoints( ptIndices, ptIndex, 2*maxClosestPtsDist, 2*maxNumClosestPts );
    n = ptIndices.size();
  }

  if (n < 3) 
    return vec3(0,0,1);

  // Find mean of points
  
  float mean[3] = {0,0,0};

  for (int i=0; i<n; i++) {
    vec3 &v = skeletonPoints[ ptIndices[i] ];
    for (int j=0; j<3; j++)
      mean[j] = mean[j] + v[j];
  }

  for (int j=0; j<3; j++)
    mean[j] = mean[j] / (float) n;

  // Compute SVD of matrix of mean-centred points in rows, then use last column of V as the normal

  arma::fmat M(n,3);
    
  for (int i=0; i<n; i++) {
    vec3 &v = skeletonPoints[ ptIndices[i] ];
    for (int j=0; j<3; j++)
      M(i,j) = v[j] - mean[j];
  }

  arma::fmat U, V;
  arma::fvec s;

  bool success = arma::svd_econ( U, s, V, M, "right", "std" );

  if (!success) {
    cerr << "SVD failed" << endl;
    exit(1);
  }
      
  return vec3( V(0,2), V(1,2), V(2,2) ).normalize();
}



int Separator::readline_from_pipe( int pipe, char *buff, int buffSize )

{
  int i = 0;

  while (true) {

    char c;
    int count = read( pipe, &c, 1 );
    if (count < 1)
      break;
    
    buff[i++] = c;

    if (c == '\n')
      break;

    if (i == buffSize) {
      cerr << "readline_from_pipe(): buffer overflow at " << buffSize << " characters." << endl;
      exit(1);
    }
  }

  buff[i] = '\0';

  return i;
}
 

void Separator::buildSTL()

{
#if 0
  if (stl != NULL)
    delete stl;
  
  stl = new STL();

  int numEmptyRidges = 0;
  
  for (int i=0; i<numRidges; i++) {

    seq<int> &r = ridges[i];

    if (r.size() < 3) {
      numEmptyRidges++;
      continue;
    }

    // Reject max-edge-len/min-edge-len is too large
    
    vec3 centroid(0,0,0);
    for (int j=0; j<r.size(); j++)
      centroid = centroid + verts[r[j]];
    centroid = (1 / (float) r.size()) * centroid;

    float radiusSq = 0;
    for (int j=0; j<r.size(); j++) {
      float rSq = (verts[r[j]] - centroid).squaredLength();
      if (rSq > radiusSq)
	radiusSq = rSq;
    }

    if (sqrt(radiusSq) > 0.75)
      continue;

    vec3 &v0 = verts[ r[0] ];	// might be better to use three "spread out" vertices
    vec3 &v1 = verts[ r[1] ];
    vec3 &v2 = verts[ r[2] ];
    
    vec3 n = ((v1-v0) ^ (v2-v0)).normalize();

    for (int j=1; j<r.size()-1; j++)
      stl->tris.add( new STLTriangle( n, verts[r[0]], verts[r[j]], verts[r[j+1]], dists[r[0]], dists[r[j]], dists[r[j+1]] ) );
  }

  if (numEmptyRidges > 0)
    cerr << numEmptyRidges << " ridges had fewer than three vertices." << endl;

  stl->setupVAO();
  stl->objToWorldTransform = identity4();

  objToWorldTransform = identity4();
#endif
}


// Find the index of the skeletonPoint closest to the line "start + t dir"
//
// Compute and store its principal curvatures.


void Separator::selectPoint( vec3 start, vec3 dir, int maxNumClosestPts, float maxClosestPtsDist )

{
  dir = dir.normalize();

  selectedAdjPt = -1;

  float minDist = MAXFLOAT;

  for (int i=0; i<skeletonPoints.size(); i++) {
    float dist = ((skeletonPoints[i] - start) - ((skeletonPoints[i] - start) * dir) * dir).length(); // point-to-line distance
    if (dist < minDist) {
      minDist = dist;
      selectedAdjPt = i;
    }
  }

  // cout << "selectedAdjPt = " << selectedAdjPt << endl;
  // if (selectedAdjPt != -1 && endpoints.size() > 0)
  //   cout << "  dist = " << skeletonDists[selectedAdjPt] << " (" << (endpoints[selectedAdjPt/2] - endpoints[selectedAdjPt/2+1]).length() << ")" << endl;
}


// Find the two principal curvature vectors, c1 and c2.  The first of
// c1,c2 has the larger curvature


void Separator::findPrincipalCurvatures( float &lambda1, float &lambda2, vec3 &c1, vec3 &c2, int ptIndex, int maxNumClosestPts, float maxClosestPtDist )
  
{
  // Find closest points
  
  seq<int> ptIndices;
  
  findClosestPoints( ptIndices, ptIndex, maxClosestPtDist, maxNumClosestPts );

  int n = ptIndices.size();
  
  vec3 pts[n];
  vec3 norms[n];

  // Arrange norms of adjacent points to all point in same direction as the centre point (at 'ptIndex')

  // closestPts.clear();   // debugging
  // closestNorms.clear();

  vec3 ptNorm = skeletonNorms[ ptIndices[0] ];
  
  for (int i=0; i<n; i++) {

    int idx = ptIndices[i];

    pts[i] = skeletonPoints[idx];

    if (skeletonNorms[idx] * ptNorm > 0)
      norms[i] = skeletonNorms[idx];
    else
      norms[i] = -1 * skeletonNorms[idx];

    // closestPts.add( pts[i] ); // instance var, for debugging
    // closestNorms.add( norms[i] );
  }

  // Calculate axes of curvature

  vec3 w = ptNorm;
  vec3 u = w.perp1();
  vec3 v = w.perp2();

  if ((u ^ v) * w < 0) { // swap if necessary to get right-handed <u,v,w> system
    vec3 t = u;
    u = v;
    v = t;
  }

  // Find each pair of points in 'ptIndices' that are adjacent in the Voronoi diagram
  //

  // ****** PERHAPS JUST USE ALL PAIRS FROM 'ptIndices' ???
  // ****** PERHAPS JUST USE ALL PAIRS FROM 'ptIndices' ???
  // ****** PERHAPS JUST USE ALL PAIRS FROM 'ptIndices' ???
  // ****** PERHAPS JUST USE ALL PAIRS FROM 'ptIndices' ???
  // ****** PERHAPS JUST USE ALL PAIRS FROM 'ptIndices' ???
  // ****** PERHAPS JUST USE ALL PAIRS FROM 'ptIndices' ???
  

  struct Pair { int i0, i1; };
  seq< struct Pair > pairs;

  for (int i=0; i<n; i++) {

    int index0 =  ptIndices[i];

    if (skeletonPoints[index0] != skeletonPoints[index0]) {// NaN
      cout << "Found NaN0 at " << index0 << ": " << pts[index0] << endl;
      continue;
    }

    struct Pair p;
    p.i0 = i;
    
    for (int j=0; j<adjPts[index0].size(); j++) { // search through points adjacent to this pts[i]
      int index1 = adjPts[index0][j];

      int k;
      if (index0 < index1 && // Avoid duplicates as (index0,index1) and (index1,index0) will both be processed
	  (k = ptIndices.findIndex( index1 )) != -1) { // is in the neighbourhood of adjacent points

	// Find index in pts/norms arrays

	p.i1 = k;
	pairs.add( p );
      }
    }
  }

  // In the <u,v,w> system, solve to find "del_u N" and "del_v N"

  int nSamples = pairs.size();

  // arma::fmat A(nSamples,2);
  // arma::fmat b(nSamples,2);

  arma::fmat AA(2*nSamples,3);
  arma::fmat bb(2*nSamples,1);

  // int kk=0;
  int k=0;
  
  for (int i=0; i<pairs.size(); i++) {

    vec3 &p0 = pts[ pairs[i].i0 ];
    vec3 &p1 = pts[ pairs[i].i1 ];

    vec3 &n0 = norms[ pairs[i].i0 ];
    vec3 &n1 = norms[ pairs[i].i1 ];

    vec3 deltaP = p1 - p0;
    vec3 deltaN = n1 - n0;

    // Method 1
    
    AA(k,0) = deltaP * u;
    AA(k,1) = deltaP * v;
    AA(k,2) = 0;
    bb(k,0) = deltaN * u;
    k++;

    AA(k,0) = 0;
    AA(k,1) = deltaP * u;
    AA(k,2) = deltaP * v;
    bb(k,0) = deltaN * v;
    k++;

    // Method 2
    
    // A(kk,0) = deltaP * u;
    // A(kk,1) = deltaP * v;
    // b(kk,0) = deltaN * u;
    // b(kk,1) = deltaN * v;
    // kk++;
  }

  // Method 1
  
  arma::fmat xx = arma::pinv(AA) * bb; // find least-squares solution with pseudoinverse

  arma::fmat XX(2,2);

  XX(0,0) = xx(0);
  XX(0,1) = xx(1);
  XX(1,0) = xx(1);
  XX(1,1) = xx(2);

  float residualNorm2 = arma::norm( AA * xx - bb );
  // cout << "XX =" << endl << XX << endl;

  // Find the principal curvature directions (= eigenvectors/values of the 2x2 matrix in <u,v>)

  vec2 e1, e2;

  {
    float trace = XX(0,0) + XX(1,1);
    float det = (XX(0,0) * XX(1,1)) - (XX(0,1) * XX(1,0));

    lambda1 = trace/2.0 + sqrt( trace*trace/4.0 - det );
    lambda2 = trace/2.0 - sqrt( trace*trace/4.0 - det );
  }

  if (fabs(XX(1,0)) > 0.0001) {
    e1 = vec2( lambda1 - XX(1,1), XX(1,0) );
    e2 = vec2( lambda2 - XX(1,1), XX(1,0) );
  // } else if (fabs(XX(0,1)) > 0.0001) { // why, since XX(1,0) == XX(0,1) ????
  //   e1 = vec2( XX(0,1), lambda1 - XX(0,0) );
  //   e2 = vec2( XX(0,1), lambda2 - XX(0,0) );
  } else {
    e1 = vec2(1,0);
    e2 = vec2(0,1);
  }

  // Method 2

  // arma::fmat x = arma::pinv(A) * b; // find least-squares solution with pseudoinverse

  // arma::fmat X(2,2);
  // X(0,0) = x(0);
  // X(0,1) = x(1);
  // X(1,0) = x(1);
  // X(1,1) = x(2);

  // // float residualNorm = arma::norm( A * x - b );
  // // cout << "x =" << endl << x << endl;
  
  // // Find the principal curvature directions (= eigenvectors/values of the 2x2 matrix in <u,v>)

  // {
  //   float trace = XX(0,0) + XX(1,1);
  //   float det = (XX(0,0) * XX(1,1)) - (XX(0,1) * XX(1,0));

  //   lambda1 = trace/2.0 + sqrt( trace*trace/4.0 - det );
  //   lambda2 = trace/2.0 - sqrt( trace*trace/4.0 - det );
  // }

  // if (fabs(XX(1,0)) > 0.0001) {
  //   e1 = vec2( lambda1 - XX(1,1), XX(1,0) );
  //   e2 = vec2( lambda2 - XX(1,1), XX(1,0) );
  // } else if (fabs(XX(0,1)) > 0.0001) {
  //   e1 = vec2( XX(0,1), lambda1 - XX(0,0) );
  //   e2 = vec2( XX(0,1), lambda2 - XX(0,0) );
  // } else {
  //   e1 = vec2(1,0);
  //   e2 = vec2(0,1);
  // }

  // if (ptIndex == selectedAdjPt) {
  //   cout << "\r" << lambda1 << " | " << c1 << "        " << lambda2 << " | " << c2 << "   "; cout.flush();
  // }

  // Find curvature directions
  
  c1 = (e1[0]*u + e1[1]*v).normalize();
  c2 = (e2[0]*u + e2[1]*v).normalize();

  // HACK: Negate lambda's so that sign(lambda)*normal is the
  // direction toward the centre of curvature.  For some reason, it's
  // the opposite with the lambdas computed above.

  lambda1 = -lambda1;
  lambda2 = -lambda2;

  // Put principal curvature in first position
  
  if (fabs(lambda1) < fabs(lambda2)) {
    vec3 t = c1; c1 = c2; c2 = t;
    float tt = lambda1; lambda1 = lambda2; lambda2 = tt;
  }

  // cout << "Principal curvatures: residual norm = " << residualNorm2 << ", lambda 1,2 = " << lambda1 << " " << lambda2 << endl;
}



void Separator::draw( mat4 &MV, mat4 &MVP, vec3 &lightDir ) 

{
  glBlendFunc( GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA );
  glEnable( GL_BLEND );
  glPointSize( 4.0 );

  vec4 colours[ skeletonPoints.size() ];
  for (int i=0; i<skeletonPoints.size(); i++) {

    float colourFrac = 1 - ((skeletonDists[i]*(scaleMaxDist-scaleMinDist)+scaleMinDist) - globalScaleMinDist) / (globalScaleMaxDist - globalScaleMinDist);

    if (colourFrac < 0 || colourFrac > 1)
      colours[i] = vec4(0.1,0.1,0.1,1);
    else
      colours[i] = colourmap->colour4( colourFrac );
  }

  segs->drawSegs( GL_POINTS, &skeletonPoints[0], colours, NULL, skeletonPoints.size(), MV, MVP, lightDir );

  // if (stl != NULL) {
  //   glPolygonMode( GL_FRONT_AND_BACK, GL_POINT );
  //   stl->draw();
  //   glPolygonMode( GL_FRONT_AND_BACK, GL_FILL );
  // }

  glPointSize( 1.0 );
  glDisable( GL_BLEND );
}


#define SPHERE_COLOUR vec4(1,1,1,1)


void Separator::drawSelectedPoint( mat4 &MV, mat4 &MVP, vec3 &lightDir, int maxNumClosestPts, float maxClosestPtDist )

{
  // re-calculate for debugging
  
  // findPrincipalCurvatures( lambda1[selectedAdjPt], lambda2[selectedAdjPt], curvature1[selectedAdjPt], curvature2[selectedAdjPt], 
  // 			   selectedAdjPt, maxNumClosestPts, maxClosestPtsDist );

  if (showCurvatureCentres) { // key 'c'

    // show centres of curvature
  
    int n = skeletonPoints.size();

    vec3 ps[ 2*n ];
    vec4 cs[ 2*n ];

    vec4 normColour(0,0,0,0.2);

    for (int i=0; i<n; i++) {

      vec3 &p = skeletonPoints[i];
      vec3 &n = skeletonNorms[i];

      ps[2*i  ] = p + (1/lambda1[i]) * n;
      ps[2*i+1] = p + (1/lambda2[i]) * n;

      cs[2*i  ] = normColour;
      cs[2*i+1] = normColour;

      // float meanCurvature = 0.5*(1/lambda1[i]+1/lambda2[i]);

      // ps[i] = p + meanCurvature * n;
      // cs[i] = normColour;
    }

    glBlendFunc( GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA );
    glEnable( GL_BLEND );
    glPointSize( 3.0 );
    segs->drawSegs( GL_POINTS, &ps[0], &cs[0], NULL, 2*n, MV, MVP, lightDir );
    // segs->drawSegs( GL_POINTS, &ps[0], &cs[0], NULL, n, MV, MVP, lightDir );
    glPointSize( 1.0 );
    glDisable( GL_BLEND );
  }

  if (showNormals) { // key 'n'

    // extend normals
  
    int n = skeletonPoints.size();

    vec3 ps[ 2*n ];
    vec4 cs[ 2*n ];

    vec4 normColour(0,0,0,0.05);

    const float normalScale = 200;

    for (int i=0; i<n; i++) {

      vec3 &p = skeletonPoints[i];
      vec3 &n = skeletonNorms[i];

      ps[2*i  ] = p - normalScale * n;
      ps[2*i+1] = p + normalScale * n;

      cs[2*i  ] = normColour;
      cs[2*i+1] = normColour;
    }

    glBlendFunc( GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA );
    glEnable( GL_BLEND );
    segs->drawSegs( GL_LINES, &ps[0], &cs[0], NULL, 2*n, MV, MVP, lightDir );
    glDisable( GL_BLEND );
  }

  if (showPrincipalDirections) { // key 'p'

    // ----------------------------------------------------------------
    // ----------------------------------------------------------------
    // ----------------------------------------------------------------

    mat4 sphereScale = scale( 0.02, 0.02, 0.02 );
  
    // Draw principal curvature directions and normal
  
    vec3 p[6];
    vec4 c[6];

    float dirScale = 0.5*(1/lambda1[ selectedAdjPt ]+1/lambda2[ selectedAdjPt ]); // was 2

    p[0] = skeletonPoints[ selectedAdjPt ]; 
    p[1] = p[0] + 3*dirScale*curvature1[selectedAdjPt];
    c[0] = vec4(1,0,0,1);
    c[1] = vec4(1,0,0,1);

    p[2] = p[0]; 
    p[3] = p[0] + 2*dirScale*curvature2[selectedAdjPt];
    c[2] = vec4(0,1,0,1);
    c[3] = vec4(0,1,0,1);

    vec3 &norm = skeletonNorms[ selectedAdjPt ];
  
    p[4] = p[0]; 
    p[5] = p[0] + dirScale*norm;
    c[4] = vec4(0,0,1,1);
    c[5] = vec4(0,0,1,1);

    glLineWidth( 3.0 );
    segs->drawSegs( GL_LINES, &p[0], &c[0], NULL, 6, MV, MVP, lightDir );
    glLineWidth( 1.0 );

    // Draw point

    mat4 sphere_MV = MV * translate( p[0] ) * sphereScale;
    mat4 sphere_MVP = MVP * translate( p[0] ) * sphereScale;
    sphere->renderGL( sphere_MV, sphere_MVP, lightDir, SPHERE_COLOUR );
    
    // Draw osculating circles

    const int NUM_CIRCLE_PTS = 100;
    
    vec3 circ_p[NUM_CIRCLE_PTS];
    vec4 circ_c[NUM_CIRCLE_PTS];

    vec3  centre = skeletonPoints[ selectedAdjPt ] + (1/(float)lambda1[selectedAdjPt]) * skeletonNorms[ selectedAdjPt ];

    sphere_MV = MV * translate( centre ) * sphereScale;
    sphere_MVP = MVP * translate( centre ) * sphereScale;
    sphere->renderGL( sphere_MV, sphere_MVP, lightDir, SPHERE_COLOUR );

    float radius = 1/(float)lambda1[selectedAdjPt];
    vec3  y      = skeletonNorms[selectedAdjPt];
    vec3  x      = curvature1[selectedAdjPt];

    for (int i=0; i<NUM_CIRCLE_PTS; i++) {
      float sine = sin( i/(float)NUM_CIRCLE_PTS * 2*M_PI );
      float cosine = cos( i/(float)NUM_CIRCLE_PTS * 2*M_PI );
      circ_p[i] = centre + radius * (cosine*x + sine*y);
      circ_c[i] = vec4(1,0,0,1);
    }

    segs->drawSegs( GL_LINE_LOOP, &circ_p[0], &circ_c[0], NULL, NUM_CIRCLE_PTS, MV, MVP, lightDir );
    
    centre = skeletonPoints[ selectedAdjPt ] + (1/(float)lambda2[selectedAdjPt]) * skeletonNorms[ selectedAdjPt ];

    sphere_MV = MV * translate( centre ) * sphereScale;
    sphere_MVP = MVP * translate( centre ) * sphereScale;
    sphere->renderGL( sphere_MV, sphere_MVP, lightDir, SPHERE_COLOUR );

    radius = 1/(float)lambda2[selectedAdjPt];
    x      = curvature2[selectedAdjPt];

    for (int i=0; i<NUM_CIRCLE_PTS; i++) {
      float sine = sin( i/(float)NUM_CIRCLE_PTS * 2*M_PI );
      float cosine = cos( i/(float)NUM_CIRCLE_PTS * 2*M_PI );
      circ_p[i] = centre + radius * (cosine*x + sine*y);
      circ_c[i] = vec4(0,1,0,1);
    }
    
    segs->drawSegs( GL_LINE_LOOP, &circ_p[0], &circ_c[0], NULL, NUM_CIRCLE_PTS, MV, MVP, lightDir );
  }

  if (showNormals || showPrincipalDirections) {  // key 'n' or 'p'

    // Also draw the closest points, which were used to compute the normal and principal directions
    
    seq<int> ptIndices;
    findClosestPoints( ptIndices, selectedAdjPt, maxClosestPtDist, maxNumClosestPts );

    mat4 smallSphereScale = scale( 0.01, 0.01, 0.01 );
    
    for (int i=0; i<ptIndices.size(); i++) {
      mat4 sphere_MV = MV * translate( skeletonPoints[ ptIndices[i] ] ) * smallSphereScale;
      mat4 sphere_MVP = MVP * translate( skeletonPoints[ ptIndices[i] ] ) * smallSphereScale;
      sphere->renderGL( sphere_MV, sphere_MVP, lightDir, SPHERE_COLOUR );
    }

    // Also draw normals on the closest points
    //
    // Reorient normals as they were when the curvatures were calculated
    
    vec3 ptNorm = skeletonNorms[ ptIndices[0] ];
    vec3 pts[2];
    vec4 cs[2] = { vec4( 0.2, 0.2, 0.2, 1 ), vec4( 0.2, 0.2, 0.2, 1 ) };
    
    for (int i=0; i<ptIndices.size(); i++) {
      int idx = ptIndices[i];
      pts[0] = skeletonPoints[idx];

      vec3 n;
      if (skeletonNorms[idx] * ptNorm > 0)
	n = skeletonNorms[idx];
      else
	n = -1 * skeletonNorms[idx];

      pts[1] = pts[0] + n;

      segs->drawSegs( GL_LINES, &pts[0], &cs[0], NULL, 2, MV, MVP, lightDir );
    }
  }
}



// Find the points closest to skeletonPoints[ptIndex], within distance
// maxDist and with a maximum of maxNumPts.


#define MIN_TO_MAX(x) (-(x))


void Separator::findClosestPoints( seq<int> &results, int ptIndex, float maxDist, int maxNumPts )

{
  priority_queue_custom<int> pq;

  pq.makeEmpty();  // max-queue
  
  seq<int> processedIndices;	// not very efficient, but it's okay for so few (<100) points
  
  // Put first point adjacencies into queue

  vec3 centre = skeletonPoints[ptIndex];

  pq.add( ptIndex, MIN_TO_MAX(0) ); // change max-queue into min-queue by flipping priorities
  processedIndices.add( ptIndex );

  results.clear();

  // Add increasingly-distant points

  while (!pq.empty() && results.size() < maxNumPts) {

    if (MIN_TO_MAX( pq.max_priority() ) > maxDist)
      break; // remaining points are too far

    int index = pq.remove_max();

    results.add( index );

    for (int j=0; j<adjPts[index].size(); j++) { // add points adjacent to this one
      int adjIndex = adjPts[index][j];
      if (processedIndices.findIndex( adjIndex ) == -1) {
	processedIndices.add( adjIndex );
	pq.add( adjIndex, MIN_TO_MAX( (skeletonPoints[adjIndex] - centre).length() ) );
      }
    }
  }
}



void Separator::save( int currentStep )

{
  // Build filename

  char filename[ strlen(anim->stateDir) + 12 ];
  sprintf( filename, "%s/sep%04d.txt", anim->stateDir, currentStep );

  // Open file
  
  ofstream out( filename, ios::out );

  if (!out) {
    cerr << "Failed to open " << filename << endl;
    return;
  }

  // Write to file

  out << scaleMinDist << endl
      << scaleMaxDist << endl
      << endl
      << skeletonPoints.size() << endl
      << endl;

  for (int i=0; i<skeletonPoints.size(); i++)
    out << skeletonPoints[i] << " "
        << skeletonNorms[i] << " "
	<< skeletonDists[i] << " "
	<< curvature1[i] << " "
	<< curvature2[i] << " "
	<< lambda1[i] << " "
	<< lambda2[i] << endl;
}


bool Separator::deleteSeparatorFile( int step )

{
  // Build filename

  char filename[ strlen(anim->stateDir) + 12 ];
  sprintf( filename, "%s/sep%04d.txt", anim->stateDir, step );

  // Test for existence

  struct stat statbuf;

  if (stat( filename, &statbuf ) == -1)
    return false;

  // Delete file

  unlink( filename );

  return true;
}



void Separator::shiftSeparatorFile( int from, int to )

{
  // Build filenames

  char fromFilename[ strlen(anim->stateDir) + 12 ];
  sprintf( fromFilename, "%s/sep%04d.txt", anim->stateDir, from );

  char toFilename[ strlen(anim->stateDir) + 12 ];
  sprintf( toFilename, "%s/sep%04d.txt", anim->stateDir, to );
  
  // Shift

  rename( fromFilename, toFilename );
}



bool Separator::load( int currentStep )

{
  // Check for cached separator

  if (currentStep < allSeparators.size())
    if (allSeparators[currentStep] != NULL) {
      separator = allSeparators[currentStep];
      //cout << "cached separator " << currentStep << " (" << allSeparators.size() << " separators)" << endl;
      return true;
    }

  // Check for file with separator
  
  // Build filename

  char filename[ strlen(anim->stateDir) + 12 ];
  sprintf( filename, "%s/sep%04d.txt", anim->stateDir, currentStep );

  // Open file
  
  ifstream in( filename, ios::in );

  if (!in) {
    //cout << "NO cached separator " << currentStep << " (" << allSeparators.size() << " separators)" << endl;
    return false;
  }

  // Reset this separator

  endpoints.clear();
  skeletonPoints.clear();
  skeletonNorms.clear();
  skeletonDists.clear();
  if (curvature1 != NULL)
    delete [] curvature1;
  if (curvature2 != NULL)
    delete [] curvature2;
  if (lambda1 != NULL)
    delete [] lambda1;
  if (lambda2 != NULL)
    delete [] lambda2;

  // Read from file

  int n;
  
  in >> scaleMinDist >> scaleMaxDist >> n;

  curvature1 = new vec3[n];
  curvature2 = new vec3[n];
  lambda1    = new float[n];
  lambda2    = new float[n];

  for (int i=0; i<n; i++) {
    vec3 p, n, c1, c2;
    float d, l1, l2;

    in >> p >> n >> d >> c1 >> c2 >> l1 >> l2;

    skeletonPoints.add(p);
    skeletonNorms.add(n);
    skeletonDists.add(d);
    curvature1[i] = c1;
    curvature2[i] = c2;
    lambda1[i] = l1;
    lambda2[i] = l2;
  }

  objToWorldTransform = identity4();

  // Store this separator
  
  while (currentStep >= allSeparators.size())
    allSeparators.add( new Separator() );

  allSeparators[currentStep] = new Separator( *separator );

  return true;
}

