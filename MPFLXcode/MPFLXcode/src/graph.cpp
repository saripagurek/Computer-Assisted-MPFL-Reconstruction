// graph.cpp


#include "graph.h"
#include "main.h"


void Graph::draw( float xPos, bool keepVScale )

{
  // Window aspect

  int width, height;
  glfwGetWindowSize( window, &width, &height );

  float aspect = width / (float) height;

  float textWidth = TEXT_HEIGHT / aspect * 0.8;

  // Margins

  const float rightMargin = 0.5 * TEXT_HEIGHT / aspect;
  const float topMargin   = 0.5 * TEXT_HEIGHT;

  // Left margin has three spaces of width 0.5, one space of 1 for 'vertName', four spaces of 0.8 wide for tick labels

  const float leftMargin = (0.5*2 + 1 + 0.8*4) * TEXT_HEIGHT + rightMargin;

  // Bottom margin has three spaces of width 0.5, two of 1 for 'horizName' and tick labels

  const float bottomMargin = (0.5*2 + 2) * TEXT_HEIGHT + topMargin;

  // Graph bounds

  vec3 boundsLL = vec3( ll.x + leftMargin,  ll.y + bottomMargin, 0 );
  vec3 boundsLR = vec3( ur.x - rightMargin, ll.y + bottomMargin, 0 );
  vec3 boundsUR = vec3( ur.x - rightMargin, ur.y - topMargin,    0 );
  vec3 boundsUL = vec3( ll.x + leftMargin,  ur.y - topMargin,    0 );

  mat4 M = identity4();

  // DEBUG

  // vec3 ps[4] = { vec3( ll.x, ll.y, 0 ), vec3( ur.x, ll.y, 0 ), vec3( ur.x, ur.y, 0 ), vec3( ll.x, ur.y, 0 ) };

  // segs->drawSegs( GL_LINE_LOOP, &ps[0], vec3(0.8,0.8,0.8), NULL, 4, M, M, vec3(1,1,1) );

  // vec3 qs[4] = { boundsLL, boundsLR, boundsUR, boundsUL };

  // segs->drawSegs( GL_LINE_LOOP, &qs[0], vec3(0,0,0), NULL, 4, M, M, vec3(1,1,1) );

  // Axis labels

  string str1( vertName );
  strokeFont->drawStrokeString( str1, ll.x + TEXT_HEIGHT / aspect + rightMargin, 0.5*(boundsLL.y + boundsUL.y), TEXT_HEIGHT, M_PI/2.0, CENTRE, vec3(0,0,0), aspect );
  
  string str2( horizName );
  strokeFont->drawStrokeString( str2, 0.5*(boundsLL.x + boundsLR.x), ll.y + topMargin, TEXT_HEIGHT, 0, CENTRE, vec3(0,0,0) );

  // Set data bounds

  float minX = MAXFLOAT;
  float maxX = -MAXFLOAT;
  float minY = MAXFLOAT;
  float maxY = -MAXFLOAT;
    
  for (int i=0; i<traces.size(); i++)
    for (int j=0; j<traces[i].points.size(); j++) {
      float x = traces[i].points[j].x;
      float y = traces[i].points[j].y;
      if (x < minX) minX = x;
      if (x > maxX) maxX = x;
      if (y < minY) minY = y;
      if (y > maxY) maxY = y;
    }

  maxY = ceil( maxY + (maxY-minY) * 0.04 );
  minY = floor( minY - (maxY-minY) * 0.04 );

  maxX = ceil( maxX );
  minX = floor( minX );

  if (maxX > 120) maxX = 120;
  if (minX < 0) minX = 0;
  
  hMin = minX;
  hMax = maxX;

  if (!keepVScale) {
    vMin = minY;
    vMax = maxY;
  }
  
  // Ticks and axes

  vec3 ticks[ 2 * (hTicks + vTicks) + 4 ];
  int j = 0;

  for (int i=0; i<hTicks; i++) {

    // tick
    
    float x = boundsLL.x + (i/(float)(hTicks-1)) * (boundsLR.x - boundsLL.x);
    float y0 = boundsLL.y;
    float y1 = y0 - 2 * TICK_LENGTH / (float)height;

    ticks[j++] = vec3(x,y0,0);
    ticks[j++] = vec3(x,y1,0);

    // label

    char buff[1000];
    sprintf( buff, hFormat, hMin + (i/(float)(hTicks-1)) * (hMax-hMin) );
    string str( buff );

    strokeFont->drawStrokeString( str, x, ll.y + topMargin + 1.2 * TEXT_HEIGHT, TEXT_HEIGHT, 0, CENTRE, vec3(0,0,0) );
  }

  for (int i=0; i<vTicks; i++) {

    // tick
    
    float y = boundsLL.y + (i/(float)(vTicks-1)) * (boundsUL.y - boundsLL.y);
    float x0 = boundsLL.x;
    float x1 = x0 - 2 * TICK_LENGTH / (float)width;

    ticks[j++] = vec3(x0,y,0);
    ticks[j++] = vec3(x1,y,0);

    // label

    char buff[1000];
    sprintf( buff, vFormat, vMin + (i/(float)(vTicks-1)) * (vMax-vMin) );
    string str( buff );

    strokeFont->drawStrokeString( str, x1 - 0.5 * textWidth, y-0.3*TEXT_HEIGHT, TEXT_HEIGHT, 0, RIGHT, vec3(0,0,0) );
  }

  ticks[j++] = boundsLL; // horizontal axis
  ticks[j++] = boundsLR;

  ticks[j++] = boundsUL; // vertical axis
  ticks[j++] = boundsLL;

  segs->drawSegs( GL_LINES, &ticks[0], vec3(0,0,0), NULL, 2 * (hTicks + vTicks) + 4, M, M, vec3(1,1,1) );

  // Traces

  for (int i=0; i<traces.size(); i++) {

    vec3 tr[ traces[i].points.size() ];

    for (int j=0; j<traces[i].points.size(); j++)
      tr[j] = vec3( (traces[i].points[j].x - hMin) / (hMax-hMin) * (boundsUR.x-boundsLL.x) + boundsLL.x,
      		    (traces[i].points[j].y - vMin) / (vMax-vMin) * (boundsUR.y-boundsLL.y) + boundsLL.y,
      		    0 );
    
    segs->drawSegs( GL_LINE_STRIP, &tr[0], traces[i].colour, NULL, traces[i].points.size(), M, M, vec3(1,1,1) );
  }

  // Vertical line

  int n=0;
  for (int i=0; i<traces.size(); i++)
    if (traces[i].points.size() > n)
      n = traces[i].points.size();

  float vlineX = (xPos-hMin) / (hMax-hMin) * (boundsUR.x-boundsLL.x) + boundsLL.x;

  vec3 ls[2] = { vec3( vlineX, boundsLL.y, 0 ), vec3( vlineX, boundsUR.y, 0 ) };

  segs->drawSegs( GL_LINES, &ls[0], vec4( 0.8, 0.8, 0.8, 0 ), NULL, 2, M, M, vec3(1,1,1) );

  // Circles on vertical line for each trace
  
  const int numCircleDivisions = 100;
  vec3 cs[numCircleDivisions+2]; // generic circle of radius 1
  cs[0] = vec3(0,0,0);
  for (int i=0; i<numCircleDivisions; i++)
    cs[i+1] = vec3( cos(i/(float)numCircleDivisions*2*M_PI), sin(i/(float)numCircleDivisions*2*M_PI), 0 );
  cs[numCircleDivisions+1] = cs[0];

  const float circleRadius = 0.02 * (boundsUL.y - boundsLL.y); // 1% of graph height

  for (int i=0; i<traces.size(); i++) {

    int j=0; // find points[j] just beyond xPos
    while (j<traces[i].points.size() && traces[i].points[j].x < xPos)
      j++;

    // if (j < traces[i].points.size())
    //   cout << xPos << " " << traces[i].points[j].x << endl;
    
    if (j >= 0 && j < traces[i].points.size()) {

      float yPos;

      if (j == 0)
	yPos = traces[i].points[0].y;
      else
	yPos = traces[i].points[j-1].y + (xPos - traces[i].points[j-1].x) / (traces[i].points[j].x - traces[i].points[j-1].x) * (traces[i].points[j].y - traces[i].points[j-1].y);
    
      float vlineY = (yPos - vMin) / (vMax-vMin) * (boundsUR.y-boundsLL.y) + boundsLL.y;
      mat4 T = M * translate( vlineX, vlineY, 0 ) * scale(circleRadius,circleRadius*aspect,1);
      segs->drawSegs( GL_TRIANGLE_FAN, &cs[0], traces[i].colour, NULL, numCircleDivisions+2, T, T, vec3(1,1,1) );
    }
  }
}

