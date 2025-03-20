// MPFL


#include "headers.h"
#include "linalg.h"
#include "stl.h"
#include "renderer.h"
#include "main.h"
#include "font.h"
#include "seq.h"
#include "axes.h"
#include "strokefont.h"
#include "sphere.h"
#include "tube.h"
#include "drawSegs.h"
#include "separator.h"
#include "anim.h"
#include "colourmap.h"
#include "priority.h"
#include "graph.h"
#include "shortest.h"
#include "spring.h"

#include <sstream>
#include <iomanip>
#include <sys/time.h>

char *objFile1;
char *objFile2;
char *dir;

GLFWwindow *window;
Renderer   *renderer;
Axes       *axes;
Sphere     *sphere;
Tube       *tube;
Segs       *segs;
Segs       *springSegs;
Anim       *anim;
Colourmap  *colourmap;
StrokeFont *strokeFont;
Graph      *graph;
Spring    *springQuadTendon;
Spring    *springPatellarTendon;

seq<Separator*> allSeparators;
Separator*      separator; // points to one from 'allSeparators' as the current separator

seq<STL*>   objs;

vec3 leftEpicondyle(0,0,0);
vec3 rightEpicondyle(0,0,0);
bool capturingLeftEpicondyle = false;
bool capturingRightEpicondyle = false;
bool showEpicondyles = false;

vec3 quadEndPoint1(0, 0, 0);
vec3 quadEndPoint2(0, 0, 0);
bool capturingQuadEndPoint1 = false;
bool capturingQuadEndPoint2 = false;

vec3 patellarEndPoint1(0, 0, 0);
vec3 patellarEndPoint2(0, 0, 0);
bool capturingPatellarEndPoint1 = false;
bool capturingPatellarEndPoint2 = false;

bool  capturingPoint = false;
int   capturedPointIndex;
bool  delPressed = false;
bool  running = false;

vec3  capturedPoints[ MAX_NUM_CAPTURED_POINTS ];
float capturedPointLastAngle[ MAX_NUM_CAPTURED_POINTS ];
int   capturedObjects[ MAX_NUM_CAPTURED_POINTS ];

ShortestPath **shortestPaths = new ShortestPath*[ MAX_NUM_CAPTURED_POINTS ];

vec4 capturedPointColours[ MAX_NUM_CAPTURED_POINTS ] = {
  vec4( 0.8, 0.8, 0.8, 1 ),
  vec4( 146/255.0, 193/255.0, 232/255.0, 1.0 ),
  vec4( 250/255.0, 170/255.0, 85/255.0, 1.0 ) ,
  vec4( 127/255.0, 219/255.0, 152/255.0, 1.0 )
};


vec4 randomColours[ NUM_RANDOM_COLOURS ] = {
  vec4( 230/255.0, 25/255.0, 75/255.0, 1 ),
  vec4( 60/255.0, 180/255.0, 75/255.0, 1 ),
  vec4( 255/255.0, 225/255.0, 25/255.0, 1 ),
  vec4( 0/255.0, 130/255.0, 200/255.0, 1 ),
  vec4( 245/255.0, 130/255.0, 48/255.0, 1 ),
  vec4( 70/255.0, 240/255.0, 240/255.0, 1 ),
  vec4( 240/255.0, 50/255.0, 230/255.0, 1 ),
  vec4( 250/255.0, 190/255.0, 212/255.0, 1 ),
  vec4( 0/255.0, 128/255.0, 128/255.0, 1 ),
  vec4( 220/255.0, 190/255.0, 255/255.0, 1 ),
  vec4( 170/255.0, 110/255.0, 40/255.0, 1 ),
  vec4( 255/255.0, 250/255.0, 200/255.0, 1 ),
  vec4( 128/255.0, 0/255.0, 0/255.0, 1 ),
  vec4( 170/255.0, 255/255.0, 195/255.0, 1 ),
  vec4( 0/255.0, 0/255.0, 128/255.0, 1 ),
  vec4( 128/255.0, 128/255.0, 128/255.0, 1 ),
  vec4( 255/255.0, 255/255.0, 255/255.0, 1 ),
  vec4( 0/255.0, 0/255.0, 0/255.0, 1 )
};


seq<vec3> edgesToDraw;
seq<vec4> edgesToDrawColours;
  

bool stopped = false;
bool showAxes = false;
bool showSkeleton = false;
bool showObjects = true;
bool showCastShadows = true;
bool showDistance = false; // was used in shader.  no longer used?
bool showMinDir = false;
bool useOriginalMethod = true;
bool showCurvatureCentres = false;
bool showNormals = false;
bool showPrincipalDirections = false;
bool showPatella = true;
bool showFemur = true;
bool showTraces = true;
bool showShortestPaths = true;
bool showCapturedPoints = true;
bool buildGrid = false;
bool startGridFormation = false;
bool measureIsometry = false;
bool showSprings = true;

seq<vec3> gridPoints;
seq<float> gridPointQuality; // first index is same as gridPoints index
seq< seq<ShortestPath*> > gridPaths; // first index is same as gridPoints index
int gridObject;
int leftToSettle = 0;
int highlightGridPoint = -1;

#define GRID_RADIUS 4
#define NUM_SETTLING_ITERATIONS 10

float *at = NULL;       // value at 60 degrees
float *maxBefore = NULL; // max value before 60 degrees
float *minBefore = NULL; // max value before 60 degrees
float *maxAfter = NULL; // max value after 60 degrees
float *minAfter = NULL; // max value after 60 degrees

float minDistRenderingRange = 0;
float maxDistRenderingRange = 0.001;

float interobjectMaxAngle = 40.0; // only accept pairings of points from different objects each within this angle of the two surface normals
float avgDistCorrection = 1.0; // factor by which avg dist changes with next patella transformation

float globalScaleMinDist = 1;
float globalScaleMaxDist = 10;

int   maxNumClosestPts = 20;
float maxClosestPtsDist = 1;

GLuint windowWidth = 1200;
GLuint windowHeight = 900;

GLFWcursor* rotationCursor;
GLFWcursor* translationCursor;

int count = 0;
int runAnim = 0;

#define TEXT_SIZE 0.03  // as a fraction of centre-to-top distance

const vec2 GRAPH_LL( -0.99, 0.5 );
const vec2 GRAPH_UR( -0.30, 0.99 );


// Viewpoint movement using the mouse

typedef enum { TRANSLATE, ROTATE } ModeType;
typedef enum { LEFT_BUTTON, MIDDLE_BUTTON, RIGHT_BUTTON, NO_BUTTON } ButtonType;

ModeType mode = ROTATE;		// translate or rotate mode
ButtonType button;		// which button is currently down
vec2 mouse;			// mouse position on button DOWN


// Drawing function

float fovy;
vec3 eyePosition;
vec3 upDir(0,1,0);
vec3 lookAt;
float worldRadius;
vec3 initEyeDir = vec3(0.7,0.3,1).normalize();

vec3 prevEyePosition, prevLookAt, prevUpDir;
vec3 targetEyePosition, targetLookAt, targetUpDir;
float targetFOVY, prevFOVY;

const float initEyeDistance = 10;


void centreView()

{
  // Look at centre

  lookAt = vec3(0,0,0);
  for (int i=0; i<objs.size(); i++)
    lookAt = lookAt + (objs[i]->objToWorldTransform * vec4( objs[i]->centre, 1 )).toVec3();
  lookAt = (1/(float)objs.size()) * lookAt;

  setWorldRadius();
    
  eyePosition = (initEyeDistance * worldRadius) * initEyeDir + lookAt;
  fovy = 1.1 * atan2( 1, initEyeDistance );
  vec3 t = upDir ^ initEyeDir;
  upDir = (initEyeDir ^ t).normalize();
}



void display()

{
  glClear( GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT );

  // WCS-to-VCS

  mat4 WCS_to_VCS = lookat( eyePosition, lookAt, upDir );

  // WCS-to-CCS

  float n = (eyePosition - lookAt).length() - worldRadius;
  float f = (eyePosition - lookAt).length() + worldRadius;

  mat4 WCS_to_CCS = perspective( fovy, windowWidth / (float) windowHeight, n, f ) * WCS_to_VCS;

  // Light direction up and above right shoulder (in ***WCS***)

  vec3 lightDir = (WCS_to_VCS.inverse() * vec4( 1, 1, 5, 0 )).toVec3().normalize();

  // Debugging edges

  if (edgesToDraw.size() > 0) {

    mat4 MV  = WCS_to_VCS * objs[0]->objToWorldTransform; // assume femur objs[0]
    mat4 MVP = WCS_to_CCS * objs[0]->objToWorldTransform;

    glLineWidth( 4 );
    segs->drawSegs( GL_LINES, &edgesToDraw[0], &edgesToDrawColours[0], NULL, edgesToDraw.size(), MV, MVP, lightDir );
    glLineWidth( 1 );
  }

  // Objects
  
  renderer->render( objs, separator->endpoints, separator->skeletonPoints, 
		    showSkeleton, showObjects, showCastShadows, showDistance, showSprings,
		    minDistRenderingRange, maxDistRenderingRange,
		    WCS_to_VCS, WCS_to_CCS, lightDir, window, false );

  // Draw the world axes

  if (showAxes && !renderer->debugOn()) {
    mat4 axesTransform;
    
    if (separator->adjPts != NULL)
      axesTransform = WCS_to_CCS * translate( separator->skeletonPoints[ separator->selectedAdjPt ] ) * scale( 10, 10, 10 );
    else
      axesTransform = WCS_to_CCS * scale( 10, 10, 10 );

    axes->draw( axesTransform, lightDir );
  }

  // Draw distance scale

  char buff[1000];

  if (showSkeleton && globalScaleMinDist != MAXFLOAT) {
    
    glDisable( GL_DEPTH_TEST );

    mat4 M = identity4();

    vec3 UR( 0.95, 0.95, 0 );
    vec3 LR( 0.95, 0.00, 0 );
    vec3 LL( 0.92, 0.00, 0 );
    vec3 UL( 0.92, 0.95, 0 );

    // Colours
    
    for (int i=0; i<MAP_SIZE; i++) {

      float f0 =   i   / (float) MAP_SIZE;
      float f1 = (i+1) / (float) MAP_SIZE;
      
      vec3 vs[4] = {
	(1-f0) * LR + f0 * UR,
	(1-f1) * LR + f1 * UR,
	(1-f1) * LL + f1 * UL,
	(1-f0) * LL + f0 * UL
      };

      vec4 c = colourmap->colour4( 1 - f0 );

      segs->drawSegs( GL_TRIANGLE_FAN, &vs[0], c, NULL, 4, M, M, lightDir );
    }
    
    // Box around scale
    
    vec3 corners[4] = { UR, LR, LL, UL };
    segs->drawSegs( GL_LINE_LOOP, &corners[0], vec4(0,0,0,1), NULL, 4, M, M, lightDir );

    // Scale labels

    float minLabel = ceil(globalScaleMinDist);
    float maxLabel = floor(globalScaleMaxDist);

    for (int val=minLabel; val<=maxLabel; val++) {

      // label
      
      float frac = (val - minLabel) / (maxLabel - minLabel);
      float y = (1-frac) * LL.y + frac * UL.y;

      sprintf( buff, "%d ", (int) val );
      string str3(buff);
      strokeFont->drawStrokeString( str3, LL.x, y-0.2*TEXT_SIZE, TEXT_SIZE, 0, RIGHT, vec3(1,0,0) );

      // tick

      vec3 vs[2] = { vec3( LL.x - (LR.x - LL.x) * 0.5, y, 0 ), vec3( LL.x , y, 0 ) };

      segs->drawSegs( GL_LINES, &vs[0], vec4(0,0,0,1), NULL, 2, M, M, lightDir );
    }
    
    glEnable( GL_DEPTH_TEST );
  }

  // Graph

  glDisable( GL_DEPTH_TEST );

  // Populate the traces from what's stored in 'anim'

  if (showTraces && gridPaths.size() > 0) {

    graph->traces.clear();

    // Grid traces
    
    for (int i=0; i<gridPaths.size(); i++) { // i = index of one grid point

      if (highlightGridPoint == i)
	graph->traces.add( Trace( "", vec3( 0, 0, 0 ) ) );
      else if (highlightGridPoint == -1)
	graph->traces.add( Trace( "", randomColours[i % NUM_RANDOM_COLOURS] ) );
      else
	graph->traces.add( Trace( "", vec3( 0.95, 0.95, 0.95 ) ) );

      for (int j=0; j<gridPaths[i].size(); j++) // j = index of frame
	graph->traces[i].points.add( vec2( gridPaths[i][j]->angle, gridPaths[i][j]->len ) );
    }

    if (anim->currentStep < gridPaths[0].size())
      graph->draw( gridPaths[0][anim->currentStep]->angle, highlightGridPoint != -1 );
    else
      graph->draw( 0, highlightGridPoint != -1 );

  } else if (showTraces && anim->allShortestPaths.size() > 0 && anim->allShortestPaths[0] != NULL && 
	     anim->allShortestPaths[0][1] != NULL && anim->allShortestPaths[0][1]->points.size() > 0) {

    graph->traces.clear();

    // Individual traces
    
    for (int i=1; i<MAX_NUM_CAPTURED_POINTS; i++) {
      
      graph->traces.add( Trace( "", capturedPointColours[i ] ) );

      for (int j=0; j<anim->allShortestPaths.size(); j++)
	if (anim->allShortestPaths[j][i] != NULL && anim->allShortestPaths[j][i]->len > 0)
	  graph->traces[i-1].points.add( vec2( anim->allShortestPaths[j][i]->angle, anim->allShortestPaths[j][i]->len ) );
    }

    if (anim->allShortestPaths.size() > anim->currentStep)
      if (anim->allShortestPaths[anim->currentStep] == NULL || anim->allShortestPaths[anim->currentStep][1] == NULL)
	graph->draw( 0, false );
      else
	graph->draw( anim->allShortestPaths[anim->currentStep][1]->angle, false );

  }

   
 
  // Output status messages

  mat4 T = anim->transforms[0] * anim->transforms[anim->currentStep].inverse();
  float rotation = acos( 0.5 * ((T[0][0] + T[1][1] + T[2][2]) - 1) ) * 180 / M_PI;

  // sprintf( buff, "%d/%d, angle %.1f", anim->currentStep+1, anim->transforms.size(), rotation );
  sprintf( buff, "%d/%d", anim->currentStep+1, anim->transforms.size() );

  string str2(buff);
  strokeFont->drawStrokeString( str2, -0.95, -0.95, TEXT_SIZE, 0, LEFT, vec3(1,0,1) );

  if (leftToSettle > 0) {
    sprintf( buff, "%d left to settle", leftToSettle );
    string str2(buff);
    strokeFont->drawStrokeString( str2, -0.5, 0.95, TEXT_SIZE, 0, LEFT, vec3(1,0,1) );
  }

  if (highlightGridPoint != -1) {

    if (measureIsometry)
      sprintf( buff, "0-120 variation: %.1f mm", maxBefore[highlightGridPoint]-minBefore[highlightGridPoint] );
    else
      sprintf( buff, "0-60 variation: %.1f mm", maxBefore[highlightGridPoint]-minBefore[highlightGridPoint] );

    string str2(buff);
    strokeFont->drawStrokeString( str2, -0.80, 0.45, TEXT_SIZE, 0, LEFT, vec3(1,0,1) );

    if (!measureIsometry && maxAfter[highlightGridPoint] > maxBefore[highlightGridPoint]) {
      sprintf( buff, "TIGHTER AFTER 60" );
      string str2(buff);
      strokeFont->drawStrokeString( str2, -0.80, 0.45-1.6*TEXT_SIZE, TEXT_SIZE, 0, LEFT, vec3(1,0,1) );
    }
  }

  glEnable( GL_DEPTH_TEST );

  glfwSwapBuffers( window );
  glfwPollEvents();  
}



// Handle windox size changes

void windowSizeCallback( GLFWwindow* window, int width, int height )

{
  windowWidth = width;
  windowHeight = height;
  glViewport(0, 0, width, height);
  renderer->reshape( width, height, window );
}



// Update the viewpoint angle upon idle

void updateState( float elapsedSeconds )

{
  return;
}


// GLFW Error callback

void GLFWErrorCallback( int error, const char* description )

{
  cerr << "Error " << error << ": " << description << endl;
  exit(1);
}



// Handle a key press


void keyCallback( GLFWwindow* window, int key, int scancode, int action, int mods )
  
{
  int n;
  
  if (action == GLFW_PRESS || action == GLFW_REPEAT) {
    switch (key) {

    case GLFW_KEY_ESCAPE:
      if (capturingPoint)
	capturingPoint = false;
      else {
	anim->saveState();
	exit(0);
      }

    case GLFW_KEY_DELETE: // erase current position

      if (running)

	delPressed = true;

      else if (capturingPoint) {

	if (capturedPointIndex == 0) {
	  capturedObjects[0] = -1;
	  capturedPoints[0] = vec3(0,0,0);
	} else {
	  for (int i=capturedPointIndex; i<MAX_NUM_CAPTURED_POINTS-1; i++) {
	    capturedObjects[i] = capturedObjects[i+1];
	    capturedPoints[i] = capturedPoints[i+1];
	  }
	  capturedObjects[MAX_NUM_CAPTURED_POINTS-1] = -1;
	  capturedPoints[MAX_NUM_CAPTURED_POINTS-1] = vec3(0,0,0);

	  for (int i=0; i<anim->transforms.size(); i++) {
	    for (int j=capturedPointIndex; j<MAX_NUM_CAPTURED_POINTS-1; j++)
	      anim->allShortestPaths[i][j] = anim->allShortestPaths[i][j+1];
	    anim->allShortestPaths[i][MAX_NUM_CAPTURED_POINTS-1] = NULL;
	  }
	}

	capturingPoint = false;
	anim->saveState();

      } else {

	anim->deleteCurrentStep();
	anim->refresh();
      }

      break;

    case GLFW_KEY_LEFT:
        fovy *= 0.9; // Zoom in
        if (fovy < 0.001)
            fovy = 0.001;
        anim->saveView();
        break;

    case GLFW_KEY_RIGHT:
        fovy *= 1.1; // Zoom out
        if (fovy > 135)
            fovy = 135;
        anim->saveView();
        break;

    case 'P':
      showPatella = !showPatella;
      break;

    case 'F':
      showFemur = !showFemur;
      break;

    case 'S':
      showSkeleton = !showSkeleton;
      break;

    case 'O':
      showObjects = !showObjects;
      break;

    case 'X':
      showCastShadows = !showCastShadows;
      break;

    case 'C':
      showCurvatureCentres = !showCurvatureCentres;
      break;
      
    case 'N':
      showNormals = !showNormals;
      break;
      
    case 'T':
      showTraces = !showTraces;
      break;
      
    case 'Z':
      showShortestPaths = !showShortestPaths;
      break;
      
    case 'Y':
      showCapturedPoints = !showCapturedPoints;
      break;
      
    case 'D':
      showPrincipalDirections = !showPrincipalDirections;
      break;
      
    case 'M':
      showMinDir = !showMinDir;
      break;

    case 'I':
      measureIsometry = !measureIsometry;
      computeGridPointQualities();
      break;

    case 'H':

      if (capturedObjects[0] != -1)

	if (mods & GLFW_MOD_SHIFT)
	  anim->recomputeShortestPaths();
	else {
	  for (int i=1; i<MAX_NUM_CAPTURED_POINTS; i++)
	    if (capturedObjects[i] != -1)
	      shortestPaths[i] = new ShortestPath( i );
	    else
	      shortestPaths[i] = NULL;
	  anim->recordShortestPaths( shortestPaths, anim->currentStep );
	}
      break;

    case 'G':
      if (mods & GLFW_MOD_SHIFT) {
	anim->computeShortestPathsForGrid();
	computeGridPointQualities();
      } else {
	buildGrid = true;
	gridPaths.clear();
	gridPoints.clear();
	gridPointQuality.clear();
	cout << "Place three grid points in triangle" << endl;
      }
      break;

    // case 'D':
    //   showDistance = !showDistance;
    //   break;

    case 'A': // advance
      if (mods & GLFW_MOD_SHIFT)
	anim->advance(0); // 0 = adjust patella in place so that it is equally supported in all directions
      else
	anim->advance(1); // advance patella downward

      display();

      break;

    case 'B': // backward
      anim->advance(-1); // advance patella upward
      break;

    case 'K':
      if (mods & GLFW_MOD_SHIFT)
	runAnim = -1;
      else
	runAnim = +1;
      break;

    case 'L':
      capturingLeftEpicondyle = true;
      showEpicondyles = true;
      break;

    case 'R':
      capturingRightEpicondyle = true;
      showEpicondyles = true;
      break;

    case 'Q':
      capturingQuadEndPoint1 = true;
      cout << "capturingQuadEndPoint1" << endl;
      break;

    case 'W':    
      capturingQuadEndPoint2 = true;
      cout << "capturingQuadEndPoint2" << endl;
      break;

    case 'J':
    capturingPatellarEndPoint1 = true;
    cout << "capturingPatellarEndPoint1" << endl;
    break;

    case 'U':    
    capturingPatellarEndPoint2 = true;
      cout << "capturingPatellarEndPoint2" << endl;
      break;

    case GLFW_KEY_DOWN:

      if (anim->stepForward())
	if (showPatella && !showFemur || showFemur && !showPatella) { // keep view fixed relative to visible patella

	  mat4 T = anim->transforms[ anim->currentStep ] * anim->transforms[ anim->currentStep - 1 ].inverse();
	  
	  eyePosition = (T * vec4(eyePosition,1)).toVec3();
	  upDir       = (T * vec4(upDir,0)).toVec3();
	  lookAt      = (T * vec4(lookAt,1)).toVec3();
	}
      
      if (capturedObjects[0] != -1) {
	for (int i=1; i<MAX_NUM_CAPTURED_POINTS; i++)
	  if (shortestPaths[i] == NULL || shortestPaths[i]->points.size() == 0)
	    if (capturedObjects[i] != -1)
	      shortestPaths[i] = new ShortestPath( i );
	    else
	      shortestPaths[i] = NULL;
	anim->recordShortestPaths( shortestPaths, anim->currentStep );
      }

      display();

      break;

    case GLFW_KEY_UP:

      if (anim->stepBackward())

	if (showPatella && !showFemur || showFemur && !showPatella) { // keep view fixed relative to visible patella

	  mat4 T = anim->transforms[ anim->currentStep ] * anim->transforms[ anim->currentStep + 1 ].inverse();
	  
	  eyePosition = (T * vec4(eyePosition,1)).toVec3();
	  upDir       = (T * vec4(upDir,0)).toVec3();
	  lookAt      = (T * vec4(lookAt,1)).toVec3();
	}
      
      if (capturedObjects[0] != -1) {
	for (int i=1; i<MAX_NUM_CAPTURED_POINTS; i++)
	  if (shortestPaths[i] == NULL || shortestPaths[i]->points.size() == 0)
	    if (capturedObjects[i] != -1)
	      shortestPaths[i] = new ShortestPath( i );
	    else
	      shortestPaths[i] = NULL;
	anim->recordShortestPaths( shortestPaths, anim->currentStep );
      }

      display();

      break;

    case '+':
    case '=':
      centreView();
      break;

    // case '+':
    // case '=':
    //   maxDistRenderingRange *= 1.1;
    //   break;

    // case '-':
    // case '_':
    //   maxDistRenderingRange /= 1.1;
    //   break;

    // case '>':
    // case '.':
    //   maxNumClosestPts += 1;
      
    //   for (int i=0; i<separator->skeletonPoints.size(); i++)
    // 	separator->findPrincipalCurvatures( lambda1[i], lambda2[i], curvature1[i], curvature2[i], i, maxNumClosestPts, maxClosestPtsDist );
    //   break;

    // case '<':
    // case ',':
    //   if (maxNumClosestPts > 1)
    // 	maxNumClosestPts -= 1;
    //   for (int i=0; i<separator->skeletonPoints.size(); i++)
    // 	separator->findPrincipalCurvatures( lambda1[i], lambda2[i], curvature1[i], curvature2[i], i, maxNumClosestPts, maxClosestPtsDist );
    //   break;

    // case 'E':
    //   cout << eyePosition << endl
    // 	   << lookAt << endl
    // 	   << upDir << endl
    // 	   << fovy*180/M_PI << endl;
    //   break;

    case 'E':
      showEpicondyles = !showEpicondyles;
      break;

    case ' ':
      if (mode == ROTATE)
	mode = TRANSLATE;
      else
	mode = ROTATE;
      glfwSetCursor( window, (mode == TRANSLATE ? translationCursor : rotationCursor) );
      break;

    case 'V':
      if (leftEpicondyle == vec3(0,0,0) || rightEpicondyle == vec3(0,0,0))
	cerr << "MUST SET EPICONDYLAR POINTS FIRST" << endl;
      else    
	separator->compute( objs, maxNumClosestPts, maxClosestPtsDist, (mods & GLFW_MOD_SHIFT) );  // force recomputation if SHIFT held down, other permit cache lookup
      break;

    // case '+':
    // case '=':
    //   count++;
    //   break;

    // case '-':
    // case '_':
    //   if (count > 0)
    // 	count--;
    //   break;

    case ']':
      interobjectMaxAngle++;
      cout << "interobjectMaxAngle = " << interobjectMaxAngle << endl;
      break;

    case '[':
      if (interobjectMaxAngle >= 1)
	interobjectMaxAngle--;
      cout << "interobjectMaxAngle = " << interobjectMaxAngle << endl;
      break;

    case '>':
    case '.':
      avgDistCorrection *= 1.1;
      cout << "avgDistCorrection = " << avgDistCorrection << endl;
      break;

    case '<':
    case ',':
      avgDistCorrection /= 1.1;
      cout << "avgDistCorrection = " << avgDistCorrection << endl;
      break;

    case '?':
    case '/':
      cout << "" << endl
	   << "l    capture left epicondyle" << endl
	   << "r    capture right epicondyle" << endl
	   << "e    show/hide epicondyles" << endl
	   << "0    capture femoral attachment point" << endl
	   << "1/2  capture one of patellar attachements points" << endl
	   << endl
	   << "     CTRL-click to select a point" << endl
	   << "y    show/hide captured points" << endl
	   << endl
	   << "v    recompute separator (use cached if possible)" << endl
	   << "V    recompute separator (force)" << endl
	
	   << "<    decrease max num closest points and recompute principal curvatures" << endl
	   << ">    increase max num closest points and recompute principal curvatures" << endl

	

	   << "A    adjust patella for equal support in all directions" << endl
	   << "a    advance patella (down)" << endl
	   << "b    backward patella (up)" << endl
	   << "DOWN step forward (down)" << endl
	   << "UP   step backward (up)" << endl
	   << "DEL  erase last position" << endl

	   << "h    compute shortest paths" << endl
	   << "z    show/hide shortest paths" << endl

	   << "k    run animation for 90 degrees" << endl

	   << "m    show/hide min direction" << endl
	   << "n    show/hide normals" << endl
	   << "c    show/hide curvature centres" << endl
	   << "d    show/hide principal directions" << endl
	   << "x    show/hide cast shadows" << endl
	   << "p    show/hide patella" << endl
	   << "f    show/hide femur" << endl
	   << "o    show/hide object display" << endl
	   << "t    show/hide traces" << endl
	   << "s    show/hide skeleton display" << endl
	   << "a    show/hide axes" << endl
	   << "d    show/hide distance heatmap" << endl
	   << "e    output eye parameters" << endl
	   << "+/-  inc/dec max distance rendering range" << endl;
    }

    // Check for digits 0, 1, 2, ...
    
    if (key - '0' >= 0 && key - '0' < MAX_NUM_CAPTURED_POINTS) {
      capturingPoint = true;
      capturedPointIndex = key - '0';
    }	
  }
}




// Find 2d mouse position on 3D arcball

vec3 arcballPos( vec2 pos )

{
  vec3 p(  pos.x/(float)windowWidth * 2 - 1, -(pos.y/(float)windowHeight * 2 - 1), 0 );
  float rr = p * p;
  if (rr <= 1)			// inside arcball
    p.z = sqrt( 1 - rr );
  else
    p = p.normalize();
  return p;
}


mat4 arcballTransform( vec2 pos, vec2 prevPos )

{
  vec3 p1 = arcballPos( pos );
  vec3 p0 = arcballPos( prevPos );

  float dot = p0 * p1;
  if (dot > 1) dot=1;
  float angle = acos( dot );

  vec3 axis = p0 ^ p1;

  return rotate( -1 * angle, axis );
}


// Mouse motion callback
//
// Only enabled when mouse button is down (which is done in mouseButtonCallback())

void mousePositionCallback( GLFWwindow* window, double x, double y )

{
  vec3 xdir, ydir, zdir;

  ydir = upDir.normalize();
  zdir = (eyePosition - lookAt).normalize();
  xdir = (ydir ^ zdir).normalize();

  if (mode == TRANSLATE) {

    if (glfwGetMouseButton( window, GLFW_MOUSE_BUTTON_LEFT ) == GLFW_PRESS) { // pan in x and y

      lookAt = lookAt + fovy * ((mouse.x-x) * xdir + (y-mouse.y) * ydir);
      eyePosition = eyePosition + fovy * ((mouse.x-x) * xdir + (y-mouse.y) * ydir);

    } else if (button == RIGHT_BUTTON) { // move in z

      lookAt = lookAt + (mouse.y-y)*0.2 * zdir;
      eyePosition = eyePosition + (mouse.y-y)*0.2 * zdir;
    }

    anim->saveView();

  } else { // mode == ROTATE

    if (glfwGetMouseButton( window, GLFW_MOUSE_BUTTON_RIGHT ) == GLFW_PRESS) { // zoom

      fovy *= 1.0 + 0.001*(mouse.y-y);
      if (fovy > 135)
	fovy = 135;
      if (fovy < 0.001)
	fovy = 0.001;

      anim->saveView();

    } else if (glfwGetMouseButton( window, GLFW_MOUSE_BUTTON_LEFT ) == GLFW_PRESS) { // rotate

      if (vec2(x,y) != mouse) {	// avoid NaN results with zero rotation and no rotation axis
	
	mat4 WCS_to_VCS = lookat( eyePosition, lookAt, upDir );
	mat4 VCS_to_WCS = WCS_to_VCS.inverse();
	mat4 T = VCS_to_WCS * arcballTransform( vec2(x,y), mouse ) * WCS_to_VCS;

	upDir = (T * vec4( upDir, 0 )).toVec3();

	vec3 eyeDir = eyePosition - lookAt;
	eyePosition = (T * vec4( eyeDir, 0)).toVec3() + lookAt;

	anim->saveView();
      }
    }
  }

  mouse.x = x;
  mouse.y = y;
}



void buildGridFromPoints()


{
  // make three points into equalateral triangle

  // find max inter-point distance
  
  float dists[3] = { (gridPoints[1] - gridPoints[0]).length(),
		     (gridPoints[2] - gridPoints[1]).length(),
		     (gridPoints[0] - gridPoints[2]).length() };

  for (int i=0; i<2; i++) // sort
    if (dists[i] > dists[i+1]) {
      float t = dists[i];
      dists[i] = dists[i+1];
      dists[i+1] = t;
    }

  float interpointDist = dists[2];

  // move points so that they are all 'interpointDist' apart
  
  vec3 x = (gridPoints[1] - gridPoints[0]).normalize();
  vec3 y = (((gridPoints[1] - gridPoints[0]) ^ (gridPoints[2] - gridPoints[0])) ^ x).normalize();

  gridPoints[1] = gridPoints[0] + interpointDist * x;
  gridPoints[2] = gridPoints[0] + 0.5 * interpointDist * x + 0.5*sqrt(3)*interpointDist * y;

  gridPoints[1] = objs[gridObject]->projectOntoSurface( gridPoints[1], NULL );
  gridPoints[2] = objs[gridObject]->projectOntoSurface( gridPoints[2], NULL );

  // Build a hex mesh from the three grid points

  vec3 v0 = gridPoints[0];
  vec3 d0 = gridPoints[1] - v0;
  vec3 d1 = gridPoints[2] - v0;

  gridPoints.clear();
  gridPointQuality.clear();

  int min = 1-GRID_RADIUS;
  int max = GRID_RADIUS;

  int dim = max-min+1;

  vec3 pts[dim][dim];
  vec3 norms[dim][dim];
  bool isPt[dim][dim];

  for (int i=0; i<dim; i++)
    for (int j=0; j<dim; j++)
      isPt[i][j] = false;
  
  for (int i=min; i<=max; i++)
    for (int j=min; j<=max; j++)
      if (min <= i+j && i+j <= max) { // in 1-neighbourhood
	pts[i-min][j-min] = objs[gridObject]->projectOntoSurface( v0 + i*d0 + j*d1, &norms[i-min][j-min] );
	isPt[i-min][j-min] = true;
      }

  // Settle mesh so that all points are equidistant

  for (int n=0; n<NUM_SETTLING_ITERATIONS; n++) {

    leftToSettle = NUM_SETTLING_ITERATIONS - 1 - n;

    edgesToDraw.clear(); // DEBUGGING
    edgesToDrawColours.clear(); // DEBUGGING
    
    vec3 deltas[dim][dim];
  
    for (int i=min; i<=max; i++) // for each (i,j) point on grid
      for (int j=min; j<=max; j++)
	if (isPt[i-min][j-min]) {

	  vec3 &c = pts[i-min][j-min]; // centre point
	
	  vec3 sum(0,0,0); // sum of offsets to each neighbour
	  int count = 0;

	  for (int ii=-1; ii<=1; ii++) // for each offset in the 1-neighbourhood
	    for (int jj=-1; jj<=1; jj++)
	      if (ii != jj) // is inside 1-neighbourhood?
		
		if (min <= i+ii && i+ii <= max && min <= j+jj && j+jj <= max && // is inside [min,max] bounds
		    isPt[i+ii-min][j+jj-min]) { // is inside original hexagonal set of points
		
		  vec3 &p = pts[i+ii-min][j+jj-min];
		
		  float thisDist = (p-c).length();
		  float diff = thisDist / interpointDist - 1;
		  sum = sum + (diff * (p-c));
		  count++;

		  // DEBUGGING

		  // float normDist = 2*diff + 0.5;  // was in [-0.5,-0.5], now [0,1]
		  // if (normDist < 0) normDist = 0;
		  // if (normDist > 1) normDist = 1;

		  // edgesToDraw.add( c );
		  // edgesToDraw.add( p );

		  // vec4 colour = colourmap->colour4( 1-normDist );

		  // edgesToDrawColours.add( colour );
		  // edgesToDrawColours.add( colour );
		}

	  deltas[i-min][j-min] = (1/(float)count) * sum;
	}


#define STEP 0.2
  
    for (int i=0; i<dim; i++)
      for (int j=0; j<dim; j++)
	if (isPt[i][j]) {

	  vec3 dir = deltas[i][j] - (deltas[i][j] * norms[i][j]) * norms[i][j];  // only change parallel to surface

	  pts[i][j] = pts[i][j] + STEP * dir;

	  if (n % 10 == 1) // only occasionally
	    pts[i][j] = objs[gridObject]->projectOntoSurface( pts[i][j], &norms[i][j] );
	}

    // DEBUGGING
  
    gridPoints.clear();
  
    for (int i=0; i<dim; i++)
      for (int j=0; j<dim; j++)
    	if (isPt[i][j])
    	  gridPoints.add( pts[i][j] );

    display();
  }

  leftToSettle = 0;

  // Set as the grid points
  
  gridPoints.clear();
  gridPointQuality.clear();
  
  for (int i=0; i<dim; i++)
    for (int j=0; j<dim; j++)
      if (isPt[i][j]) {
	gridPoints.add( pts[i][j] );
	gridPointQuality.add( -1 );
      }
}



void findPointUnderMouse(vec2 mousePos, bool ctrlKey, bool shiftKey) {
    // WCS-to-VCS
    mat4 WCS_to_VCS = lookat(eyePosition, lookAt, upDir);

    // WCS-to-CCS
    float n = (eyePosition - lookAt).length() - worldRadius;
    float f = (eyePosition - lookAt).length() + worldRadius;
    mat4 WCS_to_CCS = perspective(fovy, windowWidth / (float)windowHeight, n, f) * WCS_to_VCS;

    // Mouse in CCS
    vec4 mouseCCS(2 * mousePos.x / (float)windowWidth - 1,
                  2 * (1 - mousePos.y / (float)windowHeight) - 1,
                  0, 1);

    // Mouse in WCS
    vec3 mouseWCS = (WCS_to_CCS.inverse() * mouseCCS).toVec3();

    // Find point on model
    if (capturingLeftEpicondyle || capturingRightEpicondyle || capturingPoint || buildGrid || shiftKey || capturingQuadEndPoint1 || capturingQuadEndPoint2 || capturingPatellarEndPoint1 || capturingPatellarEndPoint2) {
        vec3 minIntPoint;
        float minIntParam = MAXFLOAT;
        int minObject = -1;

        for (int i = 0; i < objs.size(); i++) {
            vec3 intPoint, intNorm;
            float intParam;

            if (objs[i]->findFirstInt(eyePosition, mouseWCS - eyePosition, intPoint, intNorm, intParam) && intParam < minIntParam) {
                minIntPoint = intPoint;
                minIntParam = intParam;
                minObject = i;
            }
        }

        // If no intersection is found and capturing quad end points, use the mouseWCS position
        if (minIntParam == MAXFLOAT && (capturingQuadEndPoint1 || capturingPatellarEndPoint1)) {
            minIntPoint = mouseWCS;
        }

        if (minIntParam != MAXFLOAT || (capturingQuadEndPoint1 || capturingPatellarEndPoint1)) {
            if (capturingLeftEpicondyle) {
                leftEpicondyle = minIntPoint;
                capturingLeftEpicondyle = false;
                anim->saveState();
            } else if (capturingRightEpicondyle) {
                rightEpicondyle = minIntPoint;
                capturingRightEpicondyle = false;
                anim->saveState();
            } else if (capturingQuadEndPoint1) {
                quadEndPoint1 = minIntPoint;
                std::cout << "Captured quadEndPoint1: " << quadEndPoint1 << std::endl;
                capturingQuadEndPoint1 = false;
                anim->saveState();

                // Update the spring with the new quad end point 1
                springQuadTendon->reposition(quadEndPoint1, vec3(0, 0, 0));
            } else if (capturingQuadEndPoint2) {
              int patellaIndex = -1;
              for (int i = 0; i < objs.size(); i++) {
                  if (objs[i] == anim->patellaObj) {
                      patellaIndex = i;
                      break;
                  }
              }
              if (minObject == patellaIndex) {
                  // Transform the point to the local coordinate system of the patella object
                  quadEndPoint2 = (anim->patellaObj->objToWorldTransform.inverse() * vec4(minIntPoint, 1)).toVec3();
                  std::cout << "Captured quadEndPoint2: " << quadEndPoint2 << std::endl;
                  capturingQuadEndPoint2 = false;
                  anim->saveState();

                  // Update the spring with the new quad end point 2
                  springQuadTendon->reposition(vec3(0, 0, 0), quadEndPoint2);
              } else {
                  std::cout << "Quad end point 2 must be on the patella object" << std::endl;
              }
            } else if (capturingPoint) {
                // Store captured points in the coord system of their own object
                capturedPoints[capturedPointIndex] = (objs[minObject]->objToWorldTransform.inverse() * vec4(minIntPoint, 1)).toVec3();
                capturedObjects[capturedPointIndex] = minObject;
                capturingPoint = false;
                anim->saveState();
              } else if (capturingPatellarEndPoint1) {
                patellarEndPoint1 = minIntPoint;
                std::cout << "Captured patellarEndPoint1: " << patellarEndPoint1 << std::endl;
                capturingPatellarEndPoint1 = false;
                anim->saveState();

                // Update the spring with the new patellar end point 1
                springPatellarTendon->reposition(patellarEndPoint1, vec3(0, 0, 0));
            } else if (capturingPatellarEndPoint2) {
                int patellaIndex = -1;
                for (int i = 0; i < objs.size(); i++) {
                    if (objs[i] == anim->patellaObj) {
                        patellaIndex = i;
                        break;
                    }
                }
                if (minObject == patellaIndex) {
                    // Transform the point to the local coordinate system of the patella object
                    patellarEndPoint2 = (anim->patellaObj->objToWorldTransform.inverse() * vec4(minIntPoint, 1)).toVec3();
                    std::cout << "Captured patellarEndPoint2: " << patellarEndPoint2 << std::endl;
                    capturingPatellarEndPoint2 = false;
                    anim->saveState();

                    // Update the spring with the new patellar end point 2
                    springPatellarTendon->reposition(vec3(0, 0, 0), patellarEndPoint2);
                } else {
                    std::cout << "Patellar end point 2 must be on the patella object" << std::endl;
                }
            } else if (buildGrid) {
                if (gridPoints.size() == 1)
                    gridObject = minObject;
                if (gridObject == minObject) {
                    gridPoints.add((objs[minObject]->objToWorldTransform.inverse() * vec4(minIntPoint, 1)).toVec3());
                    gridPointQuality.add(-1);
                    if (gridPoints.size() > 2) {
                        buildGrid = false;
                        startGridFormation = true;
                    }
                }
            } else if (shiftKey) {
                // Find closest grid point
                float minDist = MAXFLOAT;
                int minGridPt;
                for (int i = 0; i < gridPoints.size(); i++) {
                    vec3 wcsGridPt = (objs[0]->objToWorldTransform * vec4(gridPoints[i], 1)).toVec3();
                    float dist = (wcsGridPt - minIntPoint).length();
                    if (dist < minDist) {
                        minDist = dist;
                        minGridPt = i;
                    }
                }

                if (minDist < 5)
                    highlightGridPoint = minGridPt;
                else
                    highlightGridPoint = -1;
            }
        }
    } else if (separator != NULL) {
        separator->selectPoint(eyePosition, mouseWCS - eyePosition, maxNumClosestPts, maxClosestPtsDist);
    }
}


// Mouse button callback

void mouseButtonCallback( GLFWwindow* window, int button, int action, int mods )

{
  double x, y;
  glfwGetCursorPos(window, &x, &y );

  if (action == GLFW_PRESS) {

    // store initial mouse position
      
    mouse.x = x;
    mouse.y = y;

    // enable mouse movement events
      
    glfwSetCursorPosCallback( window, mousePositionCallback );
      
  } else { // (action == GLFW_RELEASE)

    // disable mouse movement events
      
    glfwSetCursorPosCallback( window, NULL );

    // Handle clicks

    if (x == mouse.x && y == mouse.y) {

      if (mods & GLFW_MOD_CONTROL || mods & GLFW_MOD_SHIFT) { // CTRL-click: find closest point
	findPointUnderMouse( mouse, mods & GLFW_MOD_CONTROL, mods & GLFW_MOD_SHIFT );
	//lookAt = skeletonPoints[ separator->selectedAdjPt ];
      }
    }
  }
}



// Main program


int main( int argc, char **argv )

{
  // Need some filenames

  if (argc < 2 || argc > 3) {
    cerr << "Usage: " << argv[0] << " stl_file_1 stl_file_2" << endl
	 << "    or " << argv[0] << " state_file" << endl;
    exit(1);
  }

  // Set up GLFW

  glfwSetErrorCallback( GLFWErrorCallback );
  
  if (!glfwInit())
    return 1;

#ifdef MACOS
  glfwWindowHint( GLFW_CONTEXT_VERSION_MAJOR, 3 );
  glfwWindowHint( GLFW_CONTEXT_VERSION_MINOR, 2 );
  glfwWindowHint( GLFW_OPENGL_FORWARD_COMPAT, GL_TRUE );
  glfwWindowHint( GLFW_OPENGL_PROFILE, GLFW_OPENGL_CORE_PROFILE );
#else
  glfwWindowHint( GLFW_CLIENT_API, GLFW_OPENGL_ES_API );
  glfwWindowHint( GLFW_CONTEXT_VERSION_MAJOR, 3 );
  glfwWindowHint( GLFW_CONTEXT_VERSION_MINOR, 0 );
#endif

  // Set up the window

  window = glfwCreateWindow( windowWidth, windowHeight, "MPFL", NULL, NULL);
  
  if (!window) {
    glfwTerminate();
    cerr << "GLFW failed to create a window" << endl;

#ifdef MACOS
    const char *descrip;
    int code = glfwGetError( &descrip );
    cerr << "GLFW code:  " << code << endl;
    cerr << "GLFW error: " << descrip << endl;
#endif
    return 1;
  }

  glfwMakeContextCurrent( window );
  glfwSwapInterval( 1 );
  gladLoadGLLoader( (GLADloadproc) glfwGetProcAddress );

  glfwSetKeyCallback( window, keyCallback );
  glfwSetMouseButtonCallback( window, mouseButtonCallback );
  glfwSetWindowSizeCallback( window, windowSizeCallback );

  rotationCursor = glfwCreateStandardCursor( GLFW_ARROW_CURSOR );
  translationCursor = glfwCreateStandardCursor( GLFW_CROSSHAIR_CURSOR );

  glfwSetWindowPos( window, 7, 30 );

  // Init captured points
  
  for (int i=0; i<MAX_NUM_CAPTURED_POINTS; i++) {
    capturedObjects[i] = -1;
    shortestPaths[i] = NULL;
    capturedPointLastAngle[i] = MAXFLOAT;
  }

  // Setup up separator
  
  separator = new Separator();

  // Graph

  graph = new Graph( GRAPH_LL, GRAPH_UR, "angle", "length", 0, 120, 5, 15, "%.0f", "%.0f", 5, 5 );

  // Set up animator and world objects

  for (int i=0; i<MAX_NUM_CAPTURED_POINTS; i++)
    shortestPaths[i] = NULL;

  if (argc == 2) {

    anim = new Anim( argv[1] );	// state file
    anim->loadState();

    // centre femur at origin (patella is already translated in state file)

    mat4 T = translate( +1 * anim->femurObj->centre );
    anim->femurObj->objToWorldTransform   = T * anim->femurObj->objToWorldTransform;

  } else if (argc == 3) {

    anim = new Anim();
    anim->stateDir = anim->getContainingDir( strdup(argv[1]) );
    anim->statePath = new char[ strlen(anim->stateDir) + 11 ];
    sprintf( anim->statePath, "%s/state.txt", anim->stateDir );

    char *p;

    p = strrchr( argv[1], '/' );
    if (p == NULL)
      p = strrchr( argv[1], '\\' ); // in case of Windows
    if (p == NULL)
      anim->objFile0 = strdup( argv[1] );
    else
      anim->objFile0 = strdup( p+1 );

    p = strrchr( argv[2], '/' );
    if (p == NULL)
      p = strrchr( argv[2], '\\' );
    if (p == NULL)
      anim->objFile1 = strdup( argv[2] );
    else
      anim->objFile1 = strdup( p+1 );


    // Load objects
    
    for (int i=1; i<argc; i++)
      objs.add( new STL( argv[i] ) );

    anim->setPatellaObj(); // also sets 'femurObj'

    // centre femur at origin

    mat4 T = translate( +1 * anim->femurObj->centre );

    anim->patellaObj->objToWorldTransform = T * anim->patellaObj->objToWorldTransform;
    anim->femurObj->objToWorldTransform   = T * anim->femurObj->objToWorldTransform;

    centreView();
    
    // set initial 'anim' state

    anim->transforms.add( anim->patellaObj->objToWorldTransform );

    anim->saveState();
  }

  // Set up renderer

  // Example parameters for the Spring constructor
  double springConstant = 0.2;
  double dampingCoefficient = 0.95;
  double femur_X = -236.50, femur_Y =  -100.40, femur_Z =  205.92;
  double patella_X = -124.14, patella_Y = -67.45, patella_Z =  10.4;
  double tibia_X = -230.90, tibia_Y = -6.25, tibia_Z =  -32.25;

  springQuadTendon = new Spring(springConstant, dampingCoefficient, femur_X, femur_Y, femur_Z, patella_X, patella_Y, patella_Z, 0.1);
  springPatellarTendon = new Spring(springConstant, dampingCoefficient, tibia_X, tibia_Y, tibia_Z, patella_X, patella_Y, patella_Z, 0.1);

  axes      = new Axes();
  sphere    = new Sphere();
  tube      = new Tube();
  segs      = new Segs();
  springSegs = new Segs();
  colourmap = new Colourmap();
  renderer  = new Renderer( windowWidth, windowHeight, SHADER_DIR, window );

  // Set up fonts
  
  strokeFont = new StrokeFont();

  // Main loop

  anim->refresh();

  struct timeval prevTime, thisTime; // record the last rendering time
  gettimeofday( &prevTime, NULL );

  struct timeval springPrevTime, springThisTime; // record the last time for spring velocity calculation
  gettimeofday(&springPrevTime, NULL);

  glEnable( GL_DEPTH_TEST );

  // if (objs.size() > 1)
  //   separator->compute( objs, maxNumClosestPts, maxClosestPtsDist );

  while (!glfwWindowShouldClose( window )) {

    // Find elapsed time since last render

    gettimeofday( &thisTime, NULL );
    float elapsedSeconds = (thisTime.tv_sec + thisTime.tv_usec / 1000000.0) - (prevTime.tv_sec + prevTime.tv_usec / 1000000.0);
    prevTime = thisTime;

    // Update the world state

    updateState( elapsedSeconds );

    // Find elapsed time for spring velocity calculation
    gettimeofday(&springThisTime, NULL);
    float springElapsedSeconds = (springThisTime.tv_sec + springThisTime.tv_usec / 1000000.0) - (springPrevTime.tv_sec + springPrevTime.tv_usec / 1000000.0);
    springPrevTime = springThisTime;

    // Update the spring
    double distance = 1.0; // Example distance, replace with actual distance calculation

    springElapsedSeconds = springElapsedSeconds * 100.0;
    // std::cout << "springElapsedSeconds: " << springElapsedSeconds << std::endl;
    //springQuadTendon->update(springElapsedSeconds, distance);
    //springPatellarTendon->update(springElapsedSeconds, distance);

    // Clear, display, and check for events

    glClearColor( 1, 1, 1, 1 );
    glClear( GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT); // also clear depth buffer

    display();
    //glfwPollEvents();   // Done in display() so that other caller during runs can handle events

    if (runAnim != 0) {
      anim->run(runAnim);
      runAnim = 0;
    }

    if (startGridFormation) {
      buildGridFromPoints();
      startGridFormation = false;
    }
  }

  // Clean up

  glfwDestroyWindow( window );
  glfwTerminate();

  return 0;
}


void setWorldRadius()

{
  worldRadius = 0;
  for (int i=0; i<objs.size(); i++) {
    vec3 objCentre = (objs[i]->objToWorldTransform * vec4( objs[i]->centre, 1 )).toVec3();
    float radius = (objCentre - lookAt).length() + (objs[i]->objToWorldTransform.rows[0]).length() * objs[i]->radius;
    if (radius > worldRadius)
      worldRadius = radius;
  }
}



void computeGridPointQualities()

{
  float breakAngle;

  if (measureIsometry)
    breakAngle = MAXFLOAT;
  else
    breakAngle = 60;
  
  gridPointQuality.clear();

  int nPoints = gridPoints.size();

  minBefore = new float[ nPoints ]; // max value before 60 degrees
  maxBefore = new float[ nPoints ]; // max value before 60 degrees

  at        = new float[ nPoints ]; // value at 60 degrees

  maxAfter  = new float[ nPoints ]; // max value after 60 degrees
  minAfter  = new float[ nPoints ]; // max value after 60 degrees
  
  for (int i=0; i<nPoints; i++) 
    if (i < gridPaths.size()) {

      seq<ShortestPath*> &paths = gridPaths[i];

      // 0 - 60
    
      float min = MAXFLOAT;
      float max = -MAXFLOAT;

      int j = 0;
      for (;j<paths.size(); j++) {

	if (paths[j]->angle >= breakAngle)
	  break;

	if (paths[j]->len < min)
	  min = paths[j]->len;
	if (paths[j]->len > max)
	  max = paths[j]->len;
      }

      maxBefore[i] = max;
      minBefore[i] = min;

      if (j >= paths.size())
	continue;

      // At 60 (or close)

      at[i] = paths[j]->len;

      // 60+

      j++;
      if (j >= paths.size())
	continue;

      max = -MAXFLOAT;
      min = MAXFLOAT;
    
      for (;j<paths.size(); j++) {
	if (paths[j]->len < min)
	  min = paths[j]->len;
	if (paths[j]->len > max)
	  max = paths[j]->len;
      }
    
      maxAfter[i] = max;
      minAfter[i] = min;
    }

  // Gather ranges

  float before_min = MAXFLOAT;
  float before_max = -MAXFLOAT;

  float after_min = MAXFLOAT;
  float after_max = -MAXFLOAT;

  float before_diff_min = MAXFLOAT;
  float before_diff_max = -MAXFLOAT;

  float after_diff_min = MAXFLOAT;
  float after_diff_max = -MAXFLOAT;

  for (int i=0; i<nPoints; i++) {

    if (maxBefore[i] > before_max)
      before_max = maxBefore[i];

    if (minBefore[i] < before_min)
      before_min = minBefore[i];

    if (maxAfter[i] > after_max)
      after_max = maxAfter[i];

    if (minAfter[i] < after_min)
      after_min = minAfter[i];

    float before_diff = maxBefore[i] - minBefore[i];

    if (before_diff > before_diff_max)
      before_diff_max = before_diff;

    if (before_diff < before_diff_min)
      before_diff_min = before_diff;

    float after_diff = maxAfter[i] - minAfter[i];

    if (after_diff > after_diff_max)
      after_diff_max = after_diff;

    if (after_diff < after_diff_min)
      after_diff_min = after_diff;
  }

  if (before_max == before_min)
    before_max += 0.01;

  if (after_max == after_min)
    after_max += 0.01;

  if (before_diff_max == before_diff_min)
    before_diff_max += 0.01;

  if (after_diff_max == after_diff_min)
    after_diff_max += 0.01;

  // Evaluate

  for (int i=0; i<nPoints; i++) {

    float quality;

    if (measureIsometry) { // All measures are "before"

      float range = maxBefore[i] - minBefore[i];

      quality = (1 - (range - before_diff_min) / (before_diff_max - before_diff_min));

      // cout << "point " << std::setfill(' ') << std::setw(2) << i
      // 	   << fixed << setprecision(1)
      // 	   << ": range " << range
      // 	   << ", min " << before_diff_min
      // 	   << ", max " << before_diff_max
      // 	   << endl;

    } else {

      const float allowance = 2.0;
    
      if (maxAfter[i] > maxBefore[i] + allowance) // 0.0 if more than 2mm longer after breakAngle
	quality = 0.0;
      else if (maxAfter[i] > maxBefore[i]) // if longer after breakAngle, but by at most 2mm
	quality = (1-(maxAfter[i]-maxBefore[i])/allowance) * 0.1;
      else {
	float before_diff = maxBefore[i] - minBefore[i];
	quality = (1 - (before_diff - before_diff_min) / (before_diff_max - before_diff_min)) * 0.9 + 0.1;
      }

      // cout << "point " << std::setfill(' ') << std::setw(2) << i
      // 	   << fixed << setprecision(1)
      // 	   << ": quality " << quality
      // 	   << ", before " << minBefore[i] << " - " << maxBefore[i]
      // 	   << " (" << maxBefore[i] - minBefore[i] << ")"
      // 	   << ", after " << minAfter[i] << " - " << maxAfter[i]
      // 	   << " (" << maxAfter[i] - minAfter[i] << ")"
      // 	   << endl;

    }

    gridPointQuality.add( quality );

  }  

  // ISOMETRY
  //
  // min change over 0-60.  Not greater after 60.
  
  // for (int i=0; i<nPoints; i++) {

  //   float quality;
    
  //   quality = (minMax[i] - minMax_min) / (minMax_max - minMax_min);
      
  //   gridPointQuality.add( 1-quality );

  //   cout << "point " << std::setprecision(3) << i << ": quality " << quality << ", minmax before " << minMax[i] << ", at " << at[i] << ", max before " << maxBefore[i] << ", max after " << maxAfter[i] << endl;
  // }  

}

