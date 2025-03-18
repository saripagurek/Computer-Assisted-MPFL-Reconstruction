// main.h

#ifndef MAIN_H
#define MAIN_H

#define SHADER_DIR "../shaders/"
#define MAX_NUM_CAPTURED_POINTS 4
#define MAX_GLOBAL_SCALE_MAX 12

#include "seq.h"
#include "linalg.h"
#include "sphere.h"
#include "tube.h"
#include "drawSegs.h"
#include "separator.h"
#include "colourmap.h"
#include "stl.h"
#include "strokefont.h"
#include "shortest.h"
#include "spring.h"
#include "anim.h"

extern GLuint windowWidth, windowHeight;
extern GLFWwindow *window;
extern float factor;
extern float fovy;
extern vec3 eyePosition;
extern vec3 lookAt;
extern vec3 upDir;
extern float worldRadius;
extern bool showAxes;
extern bool useOriginalMethod;
extern bool showCurvatureCentres;
extern bool showNormals;
extern bool showPrincipalDirections;
extern bool showMinDir;
extern bool showEpicondyles;
extern bool showPatella;
extern bool showFemur;
extern bool showShortestPaths;
extern bool showCapturedPoints;
extern bool delPressed;
extern bool running;

extern StrokeFont *strokeFont;
extern float maxClosestPtsDist;
extern int   maxNumClosestPts;
extern float maxDistRenderingRange;
extern float interobjectMaxAngle;
extern float avgDistCorrection;

extern float globalScaleMinDist, globalScaleMaxDist;

extern vec3 capturedPoints[ MAX_NUM_CAPTURED_POINTS ];
extern int  capturedObjects[ MAX_NUM_CAPTURED_POINTS ];
extern vec4 capturedPointColours[ MAX_NUM_CAPTURED_POINTS ];
extern float capturedPointLastAngle[ MAX_NUM_CAPTURED_POINTS ];

extern ShortestPath **shortestPaths;

extern Sphere    *sphere;
extern Tube      *tube;
extern Segs      *segs;
extern Segs      *springSegs;
extern Anim      *anim;
extern Colourmap *colourmap;
extern Spring   *springQuadTendon;
extern Spring   *springPatellarTendon;

extern Separator *separator;
extern seq<Separator*> allSeparators;

extern seq<STL*> objs;

extern seq<vec3> gridPoints;
extern seq<float> gridPointQuality;
extern seq< seq<ShortestPath*> > gridPaths;
extern int highlightGridPoint;
extern vec3 leftEpicondyle, rightEpicondyle;
extern vec3 quadEndPoint1, quadEndPoint2;
extern vec3 patellarEndPoint1, patellarEndPoint2;

extern const int ablationCathIndex;

extern int count;

void glErrorReport( char *where );
void setWorldRadius();
void display();
void computeGridPointQualities();

#define NUM_RANDOM_COLOURS 18
extern vec4 randomColours[ NUM_RANDOM_COLOURS ];

#endif
