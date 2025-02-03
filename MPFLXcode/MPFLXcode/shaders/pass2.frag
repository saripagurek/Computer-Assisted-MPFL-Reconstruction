// Pass 2 fragment shader
//
// Apply diffuse lighting to fragment.  Later do Phong lighting.
//
// Determine whether fragment is in shadow.  If so, reduce intensity to 50%.

#version 300 es
precision highp float;


uniform vec3      lightDir;	    // *direction* to light in WCS
uniform vec3      Iin;		    // (r,g,b) of light
uniform vec3      eyePosition;	    // *position* of eye in WCS
uniform mat4      WCS_to_lightCCS;  // transform from WCS to light's CCS
uniform sampler2D shadowBuffer;     // texture [0,1]x[0,1] of depth from light.  Values in [0,1].
uniform sampler2D texUnitID;        // object's texture (might not be provided)
uniform bool      texturing;        // =1 if object's texture is provided
uniform bool      showCastShadows;

uniform bool      showDistance;
uniform float     minDistance;
uniform float     maxDistance;
uniform sampler2D colourmapTexUnitID;

uniform vec3 ks;		    // we'll assume that these are constant on a primitive
uniform vec3 Ia;
uniform vec3 Ie;
uniform float shininess;

in vec4 colour;
in vec3 normal;
in vec3 wcsPosition;
in float distance;

out vec4 fragColour;


void main()

{
  // Calculate the position of this fragment in the light's CCS.

  vec4 ccsLightPos = WCS_to_lightCCS * vec4( wcsPosition, 1 );

  // Calculate the depth of this fragment in the light's CCS in the range [0,1]
  
  float fragDepth = 0.5 + 0.5 * ccsLightPos.z/ccsLightPos.w;

  // Determine the (x,y) coordinates of this fragment in the light's
  // CCS in the range [0,1]x[0,1].

  vec2 shadowTexCoords = 0.5 * (ccsLightPos.xy/ccsLightPos.w) + vec2( 0.5, 0.5 );

  // Look up the depth from the light in the shadowBuffer texture.

  float shadowDepth = texture( shadowBuffer, shadowTexCoords ).r;

  // Determine whether the fragment is in shadow.
  //
  // If results look bad, add a bit to the shadow texture depth to
  // prevent z-fighting.

  float shadowFactor;

  if (showCastShadows && fragDepth > shadowDepth+.01)
    shadowFactor = 0.5;
  else
    shadowFactor = 1.0;

  // Compute Phong illumination

  // compute diffuse reflection fraction

  vec3 N = normalize(normal);
  vec3 L = normalize(lightDir);
  vec3 V = normalize(eyePosition - wcsPosition);

  float NdotV = dot(N,V);

  bool isInside = false;

  if (NdotV < 0.0) {  // draw back-pointing faces, too
    N = -1.0 * N;
    NdotV = dot(N,V);
    isInside = true;
  }

  float NdotL = dot( N, L );
  float RdotV = 0.0;
  
  if (NdotL < 0.0) {

    NdotL = 0.0;
    RdotV = 0.0;

  } else { 

    // compute specular reflection fraction

    vec3 V = normalize(eyePosition - wcsPosition);
    vec3 R = (2.0 * dot( N, L )) * N - L; 

    RdotV = dot( R, V );
    if (RdotV < 0.0)
      RdotV = 0.0;
  }

  // Choose colour
  
  vec3 kd;

  if (showDistance) {

    float alpha = (distance - minDistance) / maxDistance;

    if (alpha < 0.01)
      alpha = 0.01;
    if (alpha > 0.99)
      alpha = 0.99;
    kd = texture( colourmapTexUnitID, vec2( alpha, 0 ) ).rgb;

  } else

    kd = colour.rgb;

  // Output the fragment colour, modified by the illumination model
  // and shadowing.

  vec3 Iout = ((NdotL * kd) + (pow(RdotV,shininess) * ks)) * Iin;

  fragColour = shadowFactor * vec4( Iout, colour.a );
}
