// Pass 2 vertex shader
//
// Output vertex colour, WCS normal, WCS postion, texture coordinates

#version 300 es
precision highp float;

uniform mat4 OCS_to_WCS;	// vertex OCS to WCS
uniform mat4 OCS_to_CCS;	// vertex OCS to eye CCS

layout (location=0) in vec3  vertPosition;
layout (location=1) in vec3  vertNormal;
layout (location=2) in float vertDistance;

out vec4 colour;		// vertex colour
out vec3 normal;		// vertex normal in WCS (for light fixed in WCS)
out vec3 wcsPosition;		// vertex position in WCS
out float distance;

uniform vec4 kd;


void main()

{
  gl_Position = OCS_to_CCS * vec4( vertPosition, 1 );

  colour = kd;

  distance = vertDistance;

  // calculate normal in WCS.  (Do not divide by w since this is a direction and w = 0.)

  vec4 n = OCS_to_WCS * vec4( vertNormal, 0.0 );
  normal = n.xyz;

  // Calculate position in WCS

  vec4 worldPos = OCS_to_WCS * vec4( vertPosition, 1.0 );
  wcsPosition = worldPos.xyz / worldPos.w;
}
