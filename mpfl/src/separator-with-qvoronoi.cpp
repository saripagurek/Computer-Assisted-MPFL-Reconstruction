// separator.cpp

#include "separator.h"
#include "main.h"

#include <unistd.h>
#include <iostream>
#include <thread>
#include <mutex>

void Separator::compute( seq<STL*> &objs )

{
  // Gather points

  //cout << "Gathering points" << endl;

  int obj0 = (objs[0]->tris.size() < objs[1]->tris.size() ? 0 : 1);
  
  STL &m0 = *objs[obj0];     // object with fewer triangles
  STL &m1 = *objs[1-obj0];   // object with more triangles

  seq<vec3> m0points;
  seq<vec3> m1points;
  
  endpoints.clear();
  skeletonPoints.clear();

  float minDot = cos( 10.0 * M_PI/180.0 ); // 10 degrees

  for (int i=0; i<m1.tris.size(); i++) // clear flags on m1 (used to prevent duplicates)
    m1.tris[i]->send = false;

  // Run a bunch of threads to do this

  int numCPUs = thread::hardware_concurrency();

  if (numCPUs == 0) // == 0 if unknown
    numCPUs = 1;

  mutex mut;
  
  // Find pairs of points on m0 and m1 such that the segment betweent
  // the points is within 10 degrees of the surface normal at each
  // point.

  auto f = [&]( int k ) 
    {
      int numItems = m0.tris.size();
      int itemsPerThread = ceil( numItems / (float) numCPUs );

      int start = k * itemsPerThread;
      int end   = (k+1) * itemsPerThread;

      if (start >= numItems)
	return;
      
      if (end > numItems)
	end = numItems;

      for (int i=start; i<end; i++) {

	if (k == 0 && i % 100 == 0) {
	  cout << "\r" << (end-i)/100 << "   "; 
	  cout.flush();
	}

	vec3 &v0 = m0.tris[i]->centroid;
	vec3 &n0 = m0.tris[i]->normal;

	float minDist = MAXFLOAT;
	int   minJ;

	for (int j=0; j<m1.tris.size(); j++) {

	  vec3 &v1 = m1.tris[j]->centroid;
	  vec3 &n1 = m1.tris[j]->normal;

	  vec3 diff = v1 - v0; // direction from m0 to m1
	  float dist = diff.length();
	  
	  if (dist < minDist && diff * n0 > minDot*dist && diff * n1 < -minDot*dist) {
	    // direction points above m0 surface and negative direction points above m1 surface
	    minDist = dist;
	    minJ = j;
	  }
	}

	if (minDist != MAXFLOAT) {

	  lock_guard<mutex> guard(mut);

	  vec3 v1 = m1.tris[minJ]->centroid;

	  endpoints.add( v0 );
	  endpoints.add( v1 );
	  skeletonPoints.add( 0.5*(v0+v1) );

	  m0points.add( v0 );
	  if (!m1.tris[minJ]->send) {
	    m1points.add( v1 );
	    m1.tris[minJ]->send = true;
	  }
	}
      }
    };

  // Run threads and wait for completion
  
  thread threads[numCPUs];
  
  for (int k=0; k<numCPUs; k++)
    threads[k] = thread( f, k );

  for (int k=0; k<numCPUs; k++)
    threads[k].join();

  cout << "\r          \r"; cout.flush();

  // Send points to '/usr/bin/qvoronoi' and get response

  // child process communication from https://stackoverflow.com/questions/41495219/c-bidirectional-pipe-stuck-trying-to-read-from-child-process-in-a-loop
  
  int parent_to_child[2];
  int child_to_parent[2];

  pipe(parent_to_child);
  pipe(child_to_parent);

  int childPID = fork();

  if (childPID == 0) { // child

    close(parent_to_child[1]); // Close the writing end of the incoming pipe
    close(child_to_parent[0]); // Close the reading end of the outgoing pipe

    dup2(parent_to_child[0], STDIN_FILENO);  // replace stdin with incoming pipe
    dup2(child_to_parent[1], STDOUT_FILENO); // replace stdout with outgoing pipe

    // exec qvoronoi
    
    char *filename = "/usr/bin/qvoronoi";
    char *argv[]   = { filename, "o", "Fv", NULL };
    char *env[]    = { NULL };

    execve( filename, argv, env ); // doesn't return

    // child doesn't get to here
  }

  // parent

  close( parent_to_child[0] ); // Close the reading end of the outgoing pipe.
  close( child_to_parent[1] ); // Close the writing side of the incoming pipe.

  // Write the points

  cout << "Sending " << m0points.size() << " + " << m1points.size() << " points" << endl;
  
  #define BUFF_SIZE 100000
  char buff[ BUFF_SIZE ];

  // output "3D"

  sprintf( buff, "3\n" );
  write( parent_to_child[1], buff, strlen(buff) );

  // output number of points

  sprintf( buff, "%d\n", m0points.size() + m1points.size() );
  write( parent_to_child[1], buff, strlen(buff) );

  // Output the points

  for (int i=0; i<m0points.size(); i++) {
    sprintf( buff, "%f %f %f\n", m0points[i].x, m0points[i].y, m0points[i].z );
    write( parent_to_child[1], buff, strlen(buff) );
  }

  for (int i=0; i<m1points.size(); i++) {
    sprintf( buff, "%f %f %f\n", m1points[i].x, m1points[i].y, m1points[i].z );
    write( parent_to_child[1], buff, strlen(buff) );
  }

  close( parent_to_child[1] );

  int m0count = m0points.size();

  // Get response

  cout << "Now waiting for response" << endl;

  // get dimension (should be 3)

  readline_from_pipe( child_to_parent[0], buff, BUFF_SIZE );

  // get number of Voronoi vertices, facets, ridges

  int numFacets;

  readline_from_pipe( child_to_parent[0], buff, BUFF_SIZE );
  sscanf( buff, "%d %d", &numVerts, &numFacets ); // 'numVerts' is an instance var

  cout << "Reading " << numVerts << " vertices and " << numFacets << " facets" << endl;

  // get Voronoi vertices
  
  if (verts != NULL)
    delete [] verts;
  
  verts = new vec3[ numVerts ]; // 'verts' is an instance var

  if (dists != NULL)
    delete [] dists;

  dists = new float[ numVerts ];

  for (int i=0; i<numVerts; i++) {
    readline_from_pipe( child_to_parent[0], buff, BUFF_SIZE ); // one vertex
    sscanf( buff, "%f %f %f", &verts[i].x, &verts[i].y, &verts[i].z );

    // Find distance to model (is about the same to each model)

    float minSqDist = MAXFLOAT;
    for (int j=0; j<m0points.size(); j++) {
      float sqDist = (m0points[j] - verts[i]).squaredLength();
      if (sqDist < minSqDist)
	minSqDist = sqDist;
    }

    dists[i] = sqrt(minSqDist);
  }

  // Normalize distances to [0,1]

  float minDist = MAXFLOAT;
  float maxDist = 0;

  for (int i=0; i<numVerts; i++) {
    if (dists[i] < minDist)
      minDist = dists[i];
    if (dists[i] > maxDist)
      maxDist = dists[i];
  }

  for (int i=0; i<numVerts; i++)
    dists[i] = (dists[i] - minDist) / (maxDist - minDist);

  // Skip facets

  for (int i=0; i<numFacets; i++)
    readline_from_pipe( child_to_parent[0], buff, BUFF_SIZE ); // one facet

  // number of Voronoi ridges

  int totalNumRidges;
  
  readline_from_pipe( child_to_parent[0], buff, BUFF_SIZE );
  sscanf( buff, "%d", &totalNumRidges );

  cout << "Reading " << totalNumRidges << " ridges" << endl;

  // Ridges (i.e. separators between cells)

  if (ridges != NULL)
    delete [] ridges;
  
  ridges = new seq<int>[ totalNumRidges ]; // 'ridges' is an instance var

  numRidges = 0; // 'numRidges' is an instance var
  
  for (int i=0; i<totalNumRidges; i++) {

    readline_from_pipe( child_to_parent[0], buff, BUFF_SIZE );    // numPts siteIndex1 siteIndex2 voronoiPt1 voronoiPt2 voronoiPt3 ...

    int numPts, siteIndex1, siteIndex2;
    sscanf( buff, "%d %d %d", &numPts, &siteIndex1, &siteIndex2 );

    if (siteIndex1 == 0 || siteIndex2 == 0) // skip if the vertex-at-infinity is one of the sites
      continue;

    if (siteIndex1 <= m0count && siteIndex2 <= m0count ||
	siteIndex1 > m0count && siteIndex2 > m0count) // skip this ridge if it separates parts of the SAME object (NOTE: vertex 0 is point at infinity)
      continue;

    char *p = buff;		// skip two spaces to get past the three args above
    for (int i=0; i<2; i++) {
      while (*p != ' ')
	p++;
      p++;
    }

    seq<int> vorPtIndices;

    int ptIndex;
    for (int j=2; j<numPts; j++) {

      while (*p != ' ' && *p != '\n' && *p != '\0') // skip to next point index
	p++;
      p++;

      sscanf( p, "%d", &ptIndex );

      if (ptIndex == 0) // stop if found the point-at-infinity (which has index 0)
	break;
      
      vorPtIndices.add( ptIndex );
    }

    if (ptIndex > 0)
      ridges[ numRidges++ ] = vorPtIndices; // don't include ridges with a point-at-infinity
  }

  close( child_to_parent[0] );

  // Build the STL object for the midway surface

  buildSTL();

  cout << "Kept " << numRidges << " ridges" << endl;
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
}
