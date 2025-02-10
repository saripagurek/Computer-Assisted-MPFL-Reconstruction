// G-buffer renderer


#include "headers.h"
#include "renderer.h"
#include "main.h"
#include "spring.h"


#define EPICONDYLE_COLOUR vec4( 0.2, 0.8, 0.3, 1.0 )


// Render the scene in two passes for shadowing


void Renderer::render( seq<STL*> &objs, seq <vec3> &endpoints, seq <vec3> &skeletonPoints, 
		       bool showSkeleton, bool showObjects, bool showCastShadows, bool showDistance, bool showSprings,
		       float minDistRenderingRange, float maxDistRenderingRange,
		       mat4 &WCS_to_VCS, mat4 &WCS_to_CCS, 
		       vec3 &lightDir, GLFWwindow *window, bool renderOrthographic )

{
  gbuffer->BindForWriting();

  // Pass 1: Store shadow map

  pass1Prog->activate();

  gbuffer->BindTexture( SHADOW_GBUFFER, 0 ); // bind the shadow gbuffer to gbTextureUnit 0

  int activeDrawBuffers1[] = { SHADOW_GBUFFER };
  gbuffer->setDrawBuffers( 1, activeDrawBuffers1 );

  // WCS to light coordinate system (LCS)
  //
  // Rotate VCS so that lightDir lines up with -z.  Then apply
  // orthographic transform to define frustum boundaries.

  mat4 WCS_to_lightVCS = lookat( vec3(0,0,0) + 2*worldRadius*lightDir, vec3(0,0,0), vec3(0,1,0) );

  mat4 WCS_to_lightCCS
    = ortho( -worldRadius * windowWidth/windowHeight, worldRadius * windowWidth/windowHeight,
	     -worldRadius, worldRadius,
	     1, 3*worldRadius )
    * WCS_to_lightVCS;

  // Draw objects

  glClear( GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT );
  glEnable( GL_DEPTH_TEST );

  if (showObjects)
    for (int i=0; i<objs.size(); i++) 
      if (objs[i] == anim->patellaObj && showPatella || objs[i] != anim->patellaObj && showFemur) {
	mat4 OCS_to_lightCCS = WCS_to_lightCCS * objs[i]->objToWorldTransform;
	pass1Prog->setMat4( "OCS_to_lightCCS", OCS_to_lightCCS );
	objs[i]->draw();
      }

  // Draw midway surface

  if (showSkeleton && separator != NULL && separator->skeletonPoints.size() > 0) {
    mat4 OCS_to_lightCCS = WCS_to_lightCCS * separator->objToWorldTransform;
    pass1Prog->setMat4( "OCS_to_lightCCS", OCS_to_lightCCS );
  }

  // Render the springs
    if (showSprings) {
        spring->drawSpring(WCS_to_VCS, WCS_to_CCS, lightDir, vec4(0.8, 0.2, 0.2, 1.0));
    }

  // Draw any segments

  // if (endpoints.size() > 0)  {

  //   glPointSize( 2.0 );
  //   segs->drawSegs( GL_POINTS, &skeletonPoints[0], vec3(0.2,0.4,0.8), NULL, skeletonPoints.size(), WCS_to_VCS, WCS_to_CCS, lightDir );
  //   glPointSize( 1.0 );
  // }

  pass1Prog->deactivate();

  if (debug == 1) {
    gbuffer->DrawGBuffers( window );
    return;
  }

  // Pass 2: Render with shadows

  glBindFramebuffer( GL_DRAW_FRAMEBUFFER, 0 ); // fragment shader renders to main framebuffer now (= 0)

  pass2Prog->activate();

  vec4 kd(0.95, 0.7, 0.6,1.0);
  vec4 kdMidwaySurface(159/255.0, 229/255.0, 237/255.0, 0.5);

  vec3 Iin(1,1,1);	// light colour
  vec3 Ia = 0.3 * vec3(kd.x, kd.y, kd.z);
  vec3 ks = 0.1 * vec3(1,1,1);
  float shininess = 80;
    
  pass2Prog->setVec3( "Ia", Ia );
  pass2Prog->setVec4( "kd", kd );
  pass2Prog->setVec3( "ks", ks );
  pass2Prog->setFloat( "shininess", shininess );

  pass2Prog->setVec3( "lightDir", lightDir );
  pass2Prog->setVec3( "Iin", Iin );
  pass2Prog->setVec3( "eyePosition", eyePosition );
  pass2Prog->setMat4( "WCS_to_lightCCS", WCS_to_lightCCS );

  gbuffer->BindTexture( SHADOW_GBUFFER, 0 );
  pass2Prog->setInt( "shadowBuffer", BASE_GBUFFER_TEXTURE_UNIT + 0 ); // gbTextureUnit 0

  pass2Prog->setInt( "showCastShadows", showCastShadows );

  pass2Prog->setInt( "constantColour", 0 );  // use Phong

  glEnable( GL_DEPTH_TEST );

  glActiveTexture( GL_TEXTURE0 );
  glBindTexture( GL_TEXTURE_2D, colourmap->textureID );
  pass2Prog->setInt( "colourmapTexUnitID", 0 );
  pass2Prog->setInt( "texturing", true );

  // draw objects

  pass2Prog->setInt( "showDistance", false );

  if (showObjects)
    for (int i=0; i<objs.size(); i++)
      if (objs[i] == anim->patellaObj && showPatella || objs[i] != anim->patellaObj && showFemur) {
	pass2Prog->setMat4( "OCS_to_WCS", objs[i]->objToWorldTransform );
	mat4 OCS_to_CCS = (renderOrthographic ? WCS_to_lightCCS : WCS_to_CCS) * objs[i]->objToWorldTransform;
	pass2Prog->setMat4( "OCS_to_CCS", OCS_to_CCS );
	objs[i]->draw();
      }

  // Draw midway surface

  if (separator != NULL && separator->skeletonPoints.size() > 0) {

    pass2Prog->setInt( "showDistance", showDistance );
    pass2Prog->setFloat( "minDistRenderingRange", minDistRenderingRange );
    pass2Prog->setFloat( "maxDistRenderingRange", maxDistRenderingRange );

    mat4 OCS_to_WCS = separator->objToWorldTransform;

    mat4 OCS_to_VCS = (renderOrthographic ? WCS_to_VCS : WCS_to_lightVCS) * OCS_to_WCS;
    mat4 OCS_to_CCS = (renderOrthographic ? WCS_to_lightCCS : WCS_to_CCS) * OCS_to_WCS;
  
    pass2Prog->setMat4( "OCS_to_WCS", OCS_to_WCS );
    pass2Prog->setMat4( "OCS_to_CCS", OCS_to_CCS );
    pass2Prog->setVec4( "kd", kdMidwaySurface );
  
    if (showSkeleton && separator != NULL && separator->skeletonPoints.size() > 0)
      separator->draw( OCS_to_VCS, OCS_to_CCS, lightDir );

    if (separator != NULL && separator->selectedAdjPt != -1)
      separator->drawSelectedPoint( OCS_to_VCS, OCS_to_CCS, lightDir, maxNumClosestPts, maxClosestPtsDist );

    pass2Prog->setInt( "showDistance", false );

    // Draw any segments

    if (endpoints.size() > 0) {

#if 0
      glPointSize( 2.0 );
      segs->drawSegs( GL_POINTS, &skeletonPoints[0], vec3(0.2,0.4,0.8), NULL, skeletonPoints.size(), WCS_to_VCS, WCS_to_CCS, lightDir );
      glPointSize( 1.0 );
#endif

#if 0
      {    
	vec4 colours[ endpoints.size() ];
	for (int i=0; i<endpoints.size(); i++) {
	  colours[i] = colourmap->colour4( skeletonDists[i/2] );
	  colours[i][3] = 0.2;
	}
  
	segs->drawSegs( GL_LINES, &endpoints[0], colours, NULL, endpoints.size(), WCS_to_VCS, WCS_to_CCS, lightDir );
      }
#endif
    
#if 0
      vec3 *norms = new vec3[ 2*skeletonPoints.size() ];
      for (int i=0; i<skeletonPoints.size(); i++) {
	norms[2*i] = skeletonPoints[i];
	norms[2*i+1] = skeletonPoints[i] + (endpoints[2*i+1] - endpoints[2*i]).normalize();
      }
      segs->drawSegs( GL_LINES, &norms[0], vec3(0.2,0.4,0.8), NULL, 2*skeletonPoints.size(), WCS_to_VCS, WCS_to_CCS, lightDir );
#endif
    }

    // Show min-curvature directions

    if (showMinDir) {

      vec3 dirs[ 2*skeletonPoints.size() ];

      for (int i=0; i<skeletonPoints.size(); i++) {
	dirs[ 2*i+0 ] = skeletonPoints[i];
	dirs[ 2*i+1 ] = skeletonPoints[i] + 5* fabs(separator->lambda1[i] - separator->lambda2[i]) * separator->curvature2[i];
      }

      vec4 colours[ endpoints.size() ];
      for (int i=0; i<endpoints.size(); i++) {
	colours[i] = colourmap->colour4( separator->skeletonDists[i/2] );
	colours[i][3] = 0.2;
      }

      segs->drawSegs( GL_LINES, &dirs[0], colours, NULL, 2*separator->skeletonPoints.size(), WCS_to_VCS, WCS_to_CCS, lightDir );
    }
  }

  // Show epicondyles

  mat4 sphereScale = scale( 1,1,1 );

  if (showEpicondyles) {

    if (leftEpicondyle != vec3(0,0,0)) {
      mat4 sphere_MV  = WCS_to_VCS * translate( leftEpicondyle ) * sphereScale;
      mat4 sphere_MVP = WCS_to_CCS * translate( leftEpicondyle ) * sphereScale;
      sphere->renderGL( sphere_MV, sphere_MVP, lightDir, EPICONDYLE_COLOUR );
    }

    if (rightEpicondyle != vec3(0,0,0)) {
      mat4 sphere_MV  = WCS_to_VCS * translate( rightEpicondyle ) * sphereScale;
      mat4 sphere_MVP = WCS_to_CCS * translate( rightEpicondyle ) * sphereScale;
      sphere->renderGL( sphere_MV, sphere_MVP, lightDir, EPICONDYLE_COLOUR );
    }

    if (leftEpicondyle != vec3(0,0,0) && rightEpicondyle != vec3(0,0,0)) {    // draw line between

      vec3 vs[2] = { -0.1 * rightEpicondyle + 1.1 * leftEpicondyle,
		     1.1 * rightEpicondyle - 0.1 * leftEpicondyle };

      vec4 cs[2] = { vec4( 0.5, 0.5, 0.5, 1 ), vec4( 0.5, 0.5, 0.5, 1 ) };
    
      segs->drawSegs( GL_LINES, &vs[0], &cs[0], NULL, 2, WCS_to_VCS, WCS_to_CCS, lightDir );
    }
  }

  // Find light direction in VCS (used in sphere and cylinder shaders below)

  vec3 lightDirVCS = (WCS_to_VCS * vec4(lightDir,0)).toVec3();
  
  // Show captured points

  if (showCapturedPoints)

    for (int i=0; i<MAX_NUM_CAPTURED_POINTS; i++)
      if (capturedObjects[i] != -1) {
	vec3 p = (objs[capturedObjects[i]]->objToWorldTransform * vec4(capturedPoints[i],1)).toVec3();
	mat4 sphere_MV  = WCS_to_VCS * translate( p ) * sphereScale;
	mat4 sphere_MVP = WCS_to_CCS * translate( p ) * sphereScale;
	sphere->renderGL( sphere_MV, sphere_MVP, lightDirVCS, capturedPointColours[i] );
      }

  // Show grid points

  for (int i=0; i<gridPoints.size(); i++) {
    vec3 p = (objs[0]->objToWorldTransform * vec4(gridPoints[i],1)).toVec3();
    mat4 sphere_MV  = WCS_to_VCS * translate( p ) * sphereScale;
    mat4 sphere_MVP = WCS_to_CCS * translate( p ) * sphereScale;

    vec4 colour;
    if (i < gridPointQuality.size())
      colour = colourmap->colour4( gridPointQuality[i] );
    else
      colour = capturedPointColours[0];
    
    sphere->renderGL( sphere_MV, sphere_MVP, lightDirVCS, colour );
  }

  // Show shortest paths

  if (showShortestPaths) {

    // Grid points
    
    for (int i=0; i<gridPaths.size(); i++) {
      if (highlightGridPoint != -1 && highlightGridPoint != i)
	continue;
      if (anim->currentStep < gridPaths[i].size())
	gridPaths[i][anim->currentStep]->draw( randomColours[i%NUM_RANDOM_COLOURS], WCS_to_VCS, WCS_to_CCS, lightDirVCS );
    }

    // Individual points

    if (gridPaths.size() == 0)
      for (int i=1; i<MAX_NUM_CAPTURED_POINTS; i++)
	if (shortestPaths[i] != NULL)
	  shortestPaths[i]->draw( capturedPointColours[i], WCS_to_VCS, WCS_to_CCS, lightDirVCS );
  }

  // Render the springs
    if (showSprings) {
        spring->drawSpring(WCS_to_VCS, WCS_to_CCS, lightDirVCS, vec4(0.8, 0.2, 0.2, 1.0));
    }

  // Done

  pass2Prog->deactivate();
}
