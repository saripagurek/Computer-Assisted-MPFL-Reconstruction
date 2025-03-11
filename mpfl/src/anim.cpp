// anim.cpp


#include "headers.h"
#include "main.h"
#include "anim.h"

#include <iomanip>
#include <armadillo>  // for SVD
#include <thread>
#include <mutex>
#include <cmath>


// Find a rotation + translation that moves the separator points in
// the direction least constrained by the curvature.  Each point moves
// the specified 'distance' in the minimum-curvature direction.  Of
// the positive and negative minimum-curvature directions, the point
// moves with respect to 'overallRotationCentre' in a
// right-hand-positive direction around the specified
// 'overallRotationAxis'
//
// Each point's movement has a weight depending on
// 
//   (a) the degree to which the curvature constrains movement, which
//       is | kmax - kmin |
//
//   (b) the proximity of the point to the two surfaces (each point is
//       equidistant from the two surfaces)


#define CURVATURE_DIFF_WEIGHT 5

#define DISTANCE_WEIGHT 5
#define DISTANCE_EXPONENT 2

#define K_CORRECTION 1.5  // spring constant for distance correction


float Anim::directionWeight( float lambda1, float lambda2, float dist )

{
  return 
    CURVATURE_DIFF_WEIGHT * fabs(lambda1 - lambda2) + 
          DISTANCE_WEIGHT * pow( (1 - dist), DISTANCE_EXPONENT ); // distance is in [0,1] with 0 = min distance
}




void Anim::findTransform( mat4 &T, float distance, vec3 overallRotationAxis, vec3 overallRotationCentre )

{
  vec3 epiAxis = (leftEpicondyle - rightEpicondyle).normalize();
  
  int n = separator->skeletonPoints.size();

  // Find average distance before transform (in [0,1] using skeletonDists)

  float avgDist = 0;
  float avgDistSq = 0;

  for (int i=0; i<n; i++) {
    float d = separator->scaleMinDist + (separator->scaleMaxDist - separator->scaleMinDist) * separator->skeletonDists[i];

    avgDist   += d;
    avgDistSq += d*d;
  } 

  avgDist /= (float) n;
  avgDistSq /= (float) n;

  float avgDistStdev = sqrt( avgDistSq - avgDist*avgDist );

  // Will use the initial averge distance (initAvgDist) to find out how
  // much correction is required to get back to that distance.
  
  if (initAvgDist == -1)
    initAvgDist = avgDist;

  initAvgDist *= avgDistCorrection;

  float overallCorrection = initAvgDist - avgDist;

  // cout << "----------------" << endl
  //      << "overall correction " << overallCorrection << endl
  //      << "overall stdev      " << avgDistStdev << endl;

  // points move from P to Q
  
  vec3  P[n];  // source
  vec3  Q[n];  // dest
  float w[n];  // weight

  vec3 avgCorrection(0,0,0);
  
  for (int i=0; i<n; i++) {

    // Initial position
    
    P[i] = separator->skeletonPoints[i];

#if 0
    
    // Follow TANGENT in lower-curvature trough (i.e. "curvature2")
      
    vec3 offset = distance * separator->curvature2[i].normalize();

    Q[i] = P[i] + offset;

    // Check that this is in the correct rotation direction (according to sign of 'direction')
    
    if (distance * (((P[i] - overallRotationCentre) ^ (Q[i] - overallRotationCentre)) * overallRotationAxis) < 0) // right-hand-negative
      Q[i] = P[i] - offset;

    // Assign weight
    
    w[i] = directionWeight( separator->lambda1[i], separator->lambda2[i], separator->skeletonDists[i] );

#else
    
    // Follow CURVATURE in lower-curvature trough (i.e. "curvature2")

    if (fabs(separator->lambda2[i]) < 0.001) {

      Q[i] = P[i];
      w[i] = 0.0001;

    } else {

      if (distance == 0) {  // settle in place

	Q[i] = P[i];
	w[i] = 1.0;

      } else { // Move along groove
	
	vec3 centre = P[i] + (1/(float)separator->lambda2[i]) * separator->skeletonNorms[i];

	vec3 x = (P[i] - centre).normalize();  // = +/- skeletonNorms[i], depending on sign of lambda2[i]
	vec3 y = separator->curvature2[i].normalize();

	float radius = fabs( 1/(float)separator->lambda2[i] );

	float angle = distance / radius;   // angle in radians = distance along circumference of unit circle

	if (angle > M_PI/4)		// keep angles in [-45,+45] degrees
	  angle = M_PI/4;
	else if (angle < -M_PI/4)
	  angle = -M_PI/4;

	Q[i] = centre + radius * (cos(angle) * x + sin(angle) * y);

	// Check that this is in the correct rotation direction (according to sign of 'direction')
    
	if (distance * (((P[i] - overallRotationCentre) ^ (Q[i] - overallRotationCentre)) * overallRotationAxis) < 0) // right-hand-negative
	  Q[i] = centre + radius * (cos(-angle) * x + sin(-angle) * y);

	// Assign weight

	w[i] = directionWeight( separator->lambda1[i], separator->lambda2[i], separator->skeletonDists[i] );
      }

      // Add the distance-correction offset

      float thisDist = separator->scaleMinDist + (separator->scaleMaxDist - separator->scaleMinDist) * separator->skeletonDists[i];

      float correctionAmount = 0.5 * (initAvgDist-thisDist);

      vec3 pointToAxis = leftEpicondyle - separator->skeletonPoints[i];
      pointToAxis = pointToAxis - (pointToAxis * epiAxis) * epiAxis;
      
      vec3 correctionDir = (separator->skeletonNorms[i] * pointToAxis > 0) ? -1 * separator->skeletonNorms[i] : separator->skeletonNorms[i]; // normal direction to increase distance

      vec3 correction =  correctionAmount * correctionDir;

      Q[i] = Q[i] + correction;

      if (distance == 0 && thisDist > avgDist + 0.5*avgDistStdev)  // likely not in contact while settling into place
	w[0] = 0.001;

      avgCorrection = avgCorrection + correction;
    }
#endif
  }

  avgCorrection = (1/(float)n) * avgCorrection;

  // Apply Wahba's algorithm to find least-squares rotation from P to Q

  // First, centre both point clouds at origin

  vec3 Ptrans(0,0,0);
  vec3 Qtrans(0,0,0);

  for (int i=0; i<n; i++) {
    Ptrans = Ptrans + P[i];
    Qtrans = Qtrans + Q[i];
  }

  Ptrans = (1/(float)n) * Ptrans;
  Qtrans = (1/(float)n) * Qtrans;

  for (int i=0; i<n; i++) {
    P[i] = P[i] - Ptrans;
    Q[i] = Q[i] - Qtrans;
  }

  // Normalize the weights

  float sum = 0;
  for (int i=0; i<n; i++)
    sum += w[i];

  for (int i=0; i<n; i++)
    w[i] = w[i] / sum;

  // Build weighted cross-covariance matrix

  arma::fmat M(3,3);

  M.zeros();
  for (int i=0; i<n; i++)
    for (int r=0; r<3; r++)
      for (int c=0; c<3; c++)
	M(r,c) += w[i] * Q[i][r] * P[i][c];

  // Compute SVD
  
  arma::fmat U, V;
  arma::fvec s;

  bool success = arma::svd( U, s, V, M );

  if (!success) {
    cerr << "SVD failed" << endl;
    exit(1);
  }

  // Build rotation matrix

  arma::fvec sprime = { 1, 1, arma::det(U) * arma::det(V) };

  arma::fmat R = U * arma::diagmat(sprime) * V.t();

  // Build the translation
  
  arma::vec qt = { Qtrans[0], Qtrans[1], Qtrans[2] };
  arma::vec pt = { Ptrans[0], Ptrans[1], Ptrans[2] };

  arma::vec t = qt - R * pt;

  // copy into T

  for (int r=0; r<3; r++)
    for (int c=0; c<3; c++)
      T[r][c] = R(r,c);

  for (int c=0; c<3; c++)
    T[3][c] = 0;

  for (int r=0; r<3; r++)
    T[r][3] = t(r);
  
  T[3][3] = 1;

#if 0

  // Debugging output
  
  // Find movement magnitudes

  float transMag = (Ptrans - Qtrans).length(); // vec3( T[0][3], T[1][3], T[2][3] ).length();
  float rotMag = acos( 0.5 * ((T[0][0] + T[1][1] + T[2][2]) - 1) ) * 180 / M_PI;

  cout << "  correction " << fixed << showpoint << setprecision(3) << avgCorrection.length()
       << ", translate " << fixed << showpoint << setprecision(3) << transMag
       << ", rotate " << fixed << showpoint << setprecision(2) << rotMag << endl;
#endif
}


void Anim::advance( float distance )

{
  if (leftEpicondyle == vec3(0,0,0) || rightEpicondyle == vec3(0,0,0)) {
    cerr << "MUST SET EPICONDYLAR POINTS FIRST" << endl;
    return;
  }

  // cout << "advance " << distance << endl;

  // If forward, move to end of current transforms.  If backward, move
  // to start.

  if (distance > 0)
    currentStep = transforms.size()-1;
  else if (distance < 0)
    currentStep = 0;
  
  patellaObj->objToWorldTransform = transforms[currentStep];

  // If reversing (with currentStep == 0), shift all separator files
  // forward one to make room for a new file at position 0.

  if (distance < 0)
    for (int i=transforms.size(); i>0; i--)
      separator->shiftSeparatorFile( i-1, i );

  // calculate curvatures

  // separator->deleteSeparatorFile( currentStep ); // do not allow 'compute' to use cached separator

  // cout << "Move " << distance << " from step " << currentStep << endl;
  
  if (distance == 0)
    separator->compute( objs, maxNumClosestPts, maxClosestPtsDist, true );  // true = force recomputation
  else if  (!separator->load( currentStep ))
    separator->compute( objs, maxNumClosestPts, maxClosestPtsDist, false );

  // Find the transformation to advance by 'distance'
  
  mat4 T;
  
  findTransform( T, distance, (leftEpicondyle - rightEpicondyle).normalize(), 0.5 * (leftEpicondyle + rightEpicondyle) );

  // Apply to patella

  patellaObj->objToWorldTransform = T * patellaObj->objToWorldTransform;

  std::cout << "Before applying force matrices: " << std::endl;
  std::cout << "patellaObj->objToWorldTransform: " << patellaObj->objToWorldTransform << std::endl;


  // Get the force matrices from the springs
  mat4 quadForceMatrix = springQuadTendon->calculateForceMatrix();
  mat4 patellarForceMatrix = springPatellarTendon->calculateForceMatrix();
 
  // Apply the force matrices to the patella object
  patellaObj->objToWorldTransform = quadForceMatrix * patellaObj->objToWorldTransform;
  patellaObj->objToWorldTransform = patellarForceMatrix * patellaObj->objToWorldTransform;

  std::cout << "After applying force matrices: " << std::endl;
  std::cout << "patellaObj->objToWorldTransform: " << patellaObj->objToWorldTransform << std::endl;

  // Record in animation

  if (distance > 0) {
    transforms.add( patellaObj->objToWorldTransform );
    currentStep++;
  } else if (distance < 0) {
    transforms.shift(0);
    transforms[0] = patellaObj->objToWorldTransform;
  } else
    transforms[currentStep] = patellaObj->objToWorldTransform;
  
  // Remove old separator

  delete separator->stl;

  separator->stl = NULL;

  // Save state

  saveState();
}



bool Anim::step( int delta )

{
  bool move;

  if (delta == -1) {
    move = (currentStep > 0);
    if (move) currentStep--;
  } else if (delta == +1) {
    move = (currentStep < transforms.size()-1);
    if (move) currentStep++;
  }

  patellaObj->objToWorldTransform = transforms[currentStep];

  separator->load( currentStep );

  if (currentStep < allShortestPaths.size() && allShortestPaths[ currentStep ] != NULL)
    shortestPaths = allShortestPaths[ currentStep ];
  else
    for (int i=0; i<MAX_NUM_CAPTURED_POINTS; i++)
      shortestPaths[i] = NULL;

  return move;
}


// Helper function to calculate the Euclidean distance between two transformation matrices
double Anim::calculateDistance(const mat4 &transform1, const mat4 &transform2) {
    // Extract translation components (positions) from the transformation matrices
    vec3 pos1(transform1[0][3], transform1[1][3], transform1[2][3]);
    vec3 pos2(transform2[0][3], transform2[1][3], transform2[2][3]);

    // Calculate the Euclidean distance between the two positions
    double distance = std::sqrt(
        std::pow(pos2.x - pos1.x, 2) +
        std::pow(pos2.y - pos1.y, 2) +
        std::pow(pos2.z - pos1.z, 2)
    );

    return distance;
}



void Anim::saveState()

{
  // Open state file
  
  ofstream out( statePath, ios::out );

  if (!out) {
    cerr << "No state file " << statePath << endl;
    return;
  }

  // Write to state file

  out << objFile0 << endl
      << objFile1 << endl
      << endl
      << lookAt << endl
      << eyePosition << endl
      << upDir << endl
      << fovy << endl
      << endl
      << globalScaleMinDist << endl
      << globalScaleMaxDist << endl
      << initAvgDist << endl
      << endl
      << leftEpicondyle << endl
      << rightEpicondyle << endl
      << endl;

  if (globalScaleMaxDist > MAX_GLOBAL_SCALE_MAX)
    globalScaleMaxDist = MAX_GLOBAL_SCALE_MAX;

  // Captured points
  
  out << MAX_NUM_CAPTURED_POINTS << endl
      << endl;

  for (int i=0; i<MAX_NUM_CAPTURED_POINTS; i++)
    out << capturedObjects[i] << " " << capturedPoints[i] << endl;

  out << endl;

  // Transforms
  
  out << transforms.size() << endl 
      << endl;

  for (int i=0; i<transforms.size(); i++)
    out << transforms[i] << endl;

  // Individual shortest paths

  for (int i=0; i<transforms.size(); i++)

    for (int j=1; j<MAX_NUM_CAPTURED_POINTS; j++) {

      out << endl;
  
      if (i >= allShortestPaths.size() || allShortestPaths[i] == NULL || allShortestPaths[i][j] == NULL || allShortestPaths[i][j]->points.size() == 0) {
	out << 0 << endl
	    << 0 << endl
	    << 0 << endl;
      } else {
	out << allShortestPaths[i][j]->points.size() << endl
	    << allShortestPaths[i][j]->angle << endl
	    << allShortestPaths[i][j]->len << endl;
      
	for (int k=0; k<allShortestPaths[i][j]->points.size(); k++)
	  out << allShortestPaths[i][j]->points[k] << endl;
      }
    }

  // Grid points

  out << endl
      << gridPoints.size() << endl
      << endl;

  for (int i=0; i<gridPoints.size(); i++)
    out << gridPoints[i] << endl;

  out << endl;

  // Grid shortest paths

  out << gridPaths.size() << endl;  // number of grid points with paths
  
  for (int i=0; i<gridPaths.size(); i++) {

    out << endl
	<< gridPaths[i].size() << endl; // number of frames for this grid point

    for (int j=0; j<gridPaths[i].size(); j++) {

      out << gridPaths[i][j]->points.size() << endl
	  << gridPaths[i][j]->angle << endl
	  << gridPaths[i][j]->len << endl;
      
      for (int k=0; k<gridPaths[i][j]->points.size(); k++)
	out << gridPaths[i][j]->points[k] << endl;
    }
  }
}



void Anim::loadState()

{
  // Open state file
  
  ifstream in( statePath, ios::in );

  if (!in) {
    cerr << "No state file " << statePath << endl;
    return;
  }

  // Read from state file

  int numTransforms, numCapturedPoints;

  string f0, f1;

  in >> f0
     >> f1

     >> lookAt
     >> eyePosition
     >> upDir
     >> fovy

     >> globalScaleMinDist
     >> globalScaleMaxDist
     >> initAvgDist

     >> leftEpicondyle
     >> rightEpicondyle;

  if (globalScaleMaxDist > MAX_GLOBAL_SCALE_MAX)
    globalScaleMaxDist = MAX_GLOBAL_SCALE_MAX;

  // Read captured points
  
  in >> numCapturedPoints;

  if (numCapturedPoints > MAX_NUM_CAPTURED_POINTS)

    cerr << statePath << " has more selected points (" << numCapturedPoints << ") than the maximum permitted (" << MAX_NUM_CAPTURED_POINTS << ").  Recompile with a larger MAX_NUM_CAPTURED_POINTS." << endl;

  else {

    int i;
    
    for (i=0; i<numCapturedPoints; i++)
      in >> capturedObjects[i] >> capturedPoints[i];
    
    while (i < MAX_NUM_CAPTURED_POINTS)
      capturedPoints[ i++ ] = vec3(0,0,0);
  }

  // Read transforms

  in >> numTransforms;

  objFile0 = strdup( f0.c_str() );
  objFile1 = strdup( f1.c_str() );

  transforms.clear();
  
  for (int i=0; i<numTransforms; i++) {
    mat4 T;
    in >> T;
    transforms.add(T);
  }

  // Read shortest paths

  for (int i=0; i<transforms.size(); i++) {

    while (allShortestPaths.size() <= i)
      allShortestPaths.add( NULL );

    if (allShortestPaths[i] == NULL) {
      allShortestPaths[i] = new ShortestPath*[ MAX_NUM_CAPTURED_POINTS ];
      allShortestPaths[i][0] = NULL;
      for (int j=1; j<MAX_NUM_CAPTURED_POINTS; j++)
	allShortestPaths[i][j] = new ShortestPath();
    }
  
    for (int j=1; j<MAX_NUM_CAPTURED_POINTS; j++) {

      int numPoints;
      float angle;
      float len;

      in >> numPoints >> angle >> len;

      if (!in)
	break;
      
      allShortestPaths[i][j]->angle = angle;
      allShortestPaths[i][j]->len = len;
      allShortestPaths[i][j]->points.clear();

      for (int k=0; k<numPoints; k++) {
	vec3 pt;
	in >> pt;
	allShortestPaths[i][j]->points.add( pt );
      }
    }
  }

  if (numTransforms > 0)
    for (int i=0; i<MAX_NUM_CAPTURED_POINTS; i++)
      shortestPaths[i] = allShortestPaths[0][i];

  // Read grid points

  int nGridPoints;

  in >> nGridPoints;
  if (!in) // end of file (previous file format without grid points)
    nGridPoints = 0;

  gridPoints.clear();
  for (int i=0; i<nGridPoints; i++) {
    vec3 v;
    in >> v;
    gridPoints.add( v );
  }

  // Grid shortest paths

  int nPaths;

  in >> nPaths;
  if (!in)
    nPaths = 0;

  gridPaths.clear();

  for (int i=0; i<nPaths; i++) {

    gridPaths.add( seq<ShortestPath*>() );

    int nFrames;
    in >> nFrames;

    for (int j=0; j<nFrames; j++) {

      gridPaths[i].add( new ShortestPath() );

      int nPoints;
      float angle;
      float len;

      in >> nPoints >> angle >> len;

      gridPaths[i][j]->angle = angle;
      gridPaths[i][j]->len   = len;

      for (int k=0; k<nPoints; k++) {
	vec3 v;
	in >> v;
	gridPaths[i][j]->points.add( v );
      }
    }
  }

  computeGridPointQualities();

  // Load STL files

  char fn[ strlen(stateDir) + strlen(objFile0) + strlen(objFile1) + 2 ];

  sprintf( fn, "%s/%s", stateDir, objFile0 );
  objs.add( new STL(fn) );

  sprintf( fn, "%s/%s", stateDir, objFile1 );
  objs.add( new STL(fn) );

  setWorldRadius();

  setPatellaObj();

  if (numTransforms == 0)
    transforms.add( patellaObj->objToWorldTransform );
  else
    patellaObj->objToWorldTransform = transforms[0];

  // Load separators

  allSeparators = seq<Separator*>( transforms.size() );
  
  for (int i=0; i<transforms.size(); i++) {
    allSeparators.add( NULL );
    separator->load( i );
  }

  separator->load(0);
}


void Anim::setPatellaObj()

{
  if (patellaObj == NULL) {
    if (objs[0]->tris.size() < objs[1]->tris.size()) {
      patellaObj = objs[0];
      femurObj = objs[1];
    } else {
      patellaObj = objs[1];
      femurObj = objs[0];
    }
  }
}



void Anim::recordShortestPaths( ShortestPath **shortestPaths, int frameIndex )

{
  while (allShortestPaths.size() <= frameIndex)
    allShortestPaths.add( NULL );

  if (allShortestPaths[frameIndex] == NULL)
    allShortestPaths[frameIndex] = new ShortestPath*[ MAX_NUM_CAPTURED_POINTS ];
  
  for (int i=0; i<MAX_NUM_CAPTURED_POINTS; i++)
    allShortestPaths[frameIndex][i] = shortestPaths[i];

  saveState();
}



void Anim::saveView()

{
}


void Anim::loadView()

{
}



void Anim::run( int dir )

{
  if (transforms.size() == 0) {
    cout << "No transforms ... cannot run" << endl;
    return;
  }

  // currentStep = transforms.size() - 1;

  mat4 T = transforms[0] * transforms[currentStep].inverse();
  float rotation = acos( 0.5 * ((T[0][0] + T[1][1] + T[2][2]) - 1) ) * 180 / M_PI;
  //cout << "acos( " << ( 0.5 * ((T[0][0] + T[1][1] + T[2][2]) - 1) ) << ")" << endl;

  running = true;
  delPressed = false;

  while ( ((dir > 0 && rotation < 120) || dir < 0) && !delPressed ) {

    // cout << "angle " << fixed << showpoint << setprecision(2) << rotation << endl;

    for (int i=0; i<5; i++) {
      if (delPressed) {
	cout << "stop" << endl;
	break;
      }
      advance(0);
      display();
    }

    if (delPressed) {
      cout << "stop" << endl;
      break;
    }

    advance(dir);
    display();

    T = transforms[0] * transforms[currentStep].inverse();
    rotation = acos( 0.5 * ((T[0][0] + T[1][1] + T[2][2]) - 1) ) * 180 / M_PI;
    //cout << "acos( " << ( 0.5 * ((T[0][0] + T[1][1] + T[2][2]) - 1) ) << ")" << endl;
  }

  running = false;
  delPressed = false;

  // cout << "angle " << fixed << showpoint << setprecision(2) << rotation << endl;
}




void Anim::computeShortestPathsForGrid()

{
  if (transforms.size() == 0) {
    cout << "No transforms ... cannot recompute shortest paths" << endl;
    return;
  }

  if (gridPoints.size() == 0) {
    cout << "No grid points ... cannot recompute shortest paths" << endl;
    return;
  }

  if (capturedObjects[1] == -1) {
    cout << "No points on patella ... cannot recompute shortest paths" << endl;
    return;
  }

  // Clear
  
  gridPaths.clear();
  for (int i=0; i<gridPoints.size(); i++) {  // number of grid points
    gridPaths.add( seq<ShortestPath*>() );
    for (int j=0; j<transforms.size(); j++) // number of frames
      gridPaths[i].add( new ShortestPath() );
  }
		   
  // Run a bunch of threads to do this

  int numCPUs = thread::hardware_concurrency();

  if (numCPUs == 0) // == 0 if unknown
    numCPUs = 1;

  int numItems = gridPoints.size();
  int itemsPerThread = ceil( numItems / (float) numCPUs );

  cout << numCPUs << " CPUs, " << itemsPerThread << " items per thread" << endl;

  // Compute
    
  currentStep = 0;

  refresh();

  running = true;
  delPressed = false;
  
  float lastAngles[ gridPoints.size() ];
  for (int i=0; i<gridPoints.size(); i++)
    lastAngles[i] = MAXFLOAT;
  
  int frameIndex = 0;
  
  display();

  do { // only do first patellar point: capturedPoints[1]

    mutex mut;

    auto f = [&]( int k ) 
      {
	// Determine range of points that this thread will handle
      
	int start = k * itemsPerThread;
	int end   = (k+1) * itemsPerThread;

	if (start >= numItems)
	  return;
      
	if (end > numItems)
	  end = numItems;

	// {
	//   lock_guard<mutex> guard(mut);
	//   cout << "Thread " << k << ": " << start << "-" << end-1 << endl;
	//   cout.flush();
	// }

	for (int i=start; i<end; i++) {

	  gridPaths[i][frameIndex] = new ShortestPath( gridPoints[i], capturedPoints[1], lastAngles[i] );

	  // {
	  //   lock_guard<mutex> guard(mut);
	  //   cout << "Thread " << k << ": starting " << i << endl;
	  //   cout.flush();
	  // }
	}
      };

    // Run threads and wait for completion
  
    thread threads[numCPUs];
  
    for (int k=0; k<numCPUs; k++)
      threads[k] = thread( f, k );

    for (int k=0; k<numCPUs; k++) {
      threads[k].join();
      if (anim->currentStep == 0)
	display();
    }

    // Done

    frameIndex++;

    if (currentStep > 0)
      display();

  } while (stepForward() && !delPressed);	// advances currentStep

  saveState();

  running = false;
  delPressed = false;
}


void Anim::recomputeShortestPaths()

{
  if (transforms.size() == 0) {
    cout << "No transforms ... cannot recompute shortest paths" << endl;
    return;
  }

  if (capturedObjects[0] == -1) {
    cout << "No captured points ... cannot recompute shortest paths" << endl;
    return;
  }

  // Clear
  
  allShortestPaths.clear();
  
  // for (int i=0; i<transforms.size(); i++) {
  //   allShortestPaths.add( new ShortestPath*[ MAX_NUM_CAPTURED_POINTS ] );
  //   allShortestPaths[i][0] = NULL; // femur point
  // }
  
  // Compute
    
  currentStep = 0;

  refresh();

  running = true;
  delPressed = false;
  
  do {
    
    for (int i=1; i<MAX_NUM_CAPTURED_POINTS; i++)
      if (capturedObjects[i] != -1)
	shortestPaths[i] = new ShortestPath( i );
      else
	shortestPaths[i] = NULL;

    anim->recordShortestPaths( shortestPaths, currentStep );

    display();

  } while (stepForward() && !delPressed);	// advances currentStep

  running = false;
  delPressed = false;
}



void Anim::deleteCurrentStep()

{
  if (transforms.size() < 2)
    return;
  
  for (int i=currentStep; i < transforms.size() - 1; i++)
    separator->shiftSeparatorFile( i+1, i );

  if (currentStep < allSeparators.size())
    allSeparators.remove( currentStep );

  transforms.remove( currentStep );

  if (currentStep < allShortestPaths.size())
    allShortestPaths.remove( currentStep );

  if (currentStep >= transforms.size())
    currentStep = transforms.size()-1;

  separator = allSeparators[currentStep];
}
