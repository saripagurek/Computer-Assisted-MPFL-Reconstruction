// strokefont.h
//
// Draw characters using FreeGlut's stroke characters
//
// The code from freeglut has be extracted and modified here so
// there's no dependence on freeglut.
//
// Draw 'string' at position (x,y) in viewing coordinates
// [-1,1]x[-1,1].  Scale the string to have height 'height'.
//
// We need 'transformLocation' to be the location at which to store
// the transformation matrix for the GPU shader.  This is found by the
// calling program using glGetUniformLocation().


#include "strokefont.h"
#include "fg_stroke.h" 


// Shaders for font rendering


const char *StrokeFont::fontVertexShader = R"XX(

  #version 300 es

  layout (location = 0) in vec4 position;
  uniform mat4 MVP;

  void main()

  {
     gl_Position = MVP * position;
  }

)XX";



const char *StrokeFont::fontFragmentShader = R"XX(

  #version 300 es

  uniform mediump vec3 colour;

  out mediump vec4 fragColour;

  void main()

  {
    fragColour = vec4( colour,1 );
  }

)XX";



void StrokeFont::drawStrokeString( string str, float x, float y, float height, float theta, Alignment alignment, vec3 colour )

{
  drawStrokeString( str, x, y, height, theta, alignment, colour, 1.0 );
}



void StrokeFont::drawStrokeString( string str, float x, float y, float height, float theta, Alignment alignment, vec3 colour, float aspect )

{
  gpuProg->activate();

  SFG_StrokeFont *font = &fgStrokeMonoRoman;
  
  float s = height / (float) font->Height; // scale of letters

  // Find total width of string

  float width = 0;
  for (unsigned int k=0; k<str.size(); k++)
    if (str[k] == '\n')
      width = 0;
    else {
      const SFG_StrokeChar *schar = font->Characters[ (unsigned char) str[k] ];
      width += schar->Right;
    }

  float xOffset;
  switch (alignment) {
  case LEFT:
    xOffset = 0; break;
  case CENTRE:
    xOffset = -0.5 * width; break;
  case RIGHT:
    xOffset = -width; break;
  }

  // Draw each letter

  float xPos = x;
  float initX = x;

  float vScale = cos(theta) + sin(theta)*aspect; // to account for aspect, BUT THIS DOESN'T WORK YET

  for (unsigned int k=0; k<str.size(); k++) 

    if (str[k] == '\n') {	// handle newline

      xPos = initX;
      y -= height * 1.2;

    } else {

      mat4 transform
	= translate( initX, y, 0 )
	* rotate( theta, vec3(0,0,1) )
	* translate( s*xOffset+(xPos-initX), 0, 0 )
	* scale( s*vScale, s, 1 );

      gpuProg->setMat4( "MVP", transform );
      gpuProg->setVec3( "colour", colour );
      
      // glutStrokeCharacter( font, str[k] );
      //
      // The following is extracted from FreeGlut source.

      const SFG_StrokeChar  *schar = font->Characters[ (unsigned char) str[k] ];
      const SFG_StrokeStrip *strip = schar->Strips;

      for (int i=0; i<schar->Number; i++, strip++) {

	// Create a VAO for this stroke

	GLuint VAO;
	glGenVertexArrays( 1, &VAO );
	glBindVertexArray( VAO );

	// Fill a buffer with the stroke's vertices

	float *verts = new float[strip->Number*2];

	for (int j=0; j<strip->Number; j++) {
	  verts[j*2+0] = strip->Vertices[ j ].X;
	  verts[j*2+1] = strip->Vertices[ j ].Y;
	}

	// Fill a VBO with the stroke's vertices

	GLuint VBO;
	glGenBuffers( 1, &VBO );
	glBindBuffer( GL_ARRAY_BUFFER, VBO );
	glBufferData( GL_ARRAY_BUFFER, strip->Number*2*sizeof(float), verts, GL_STATIC_DRAW );

	delete[] verts;

	// Draw the stroke

	glEnableVertexAttribArray( 0 );
	glVertexAttribPointer( 0, 2, GL_FLOAT, GL_FALSE, 0, 0 );

	glDrawArrays( GL_LINE_STRIP, 0, strip->Number );

	// Free everything

	glDisableVertexAttribArray( 0 );
	glDeleteBuffers( 1, &VBO );
	glDeleteVertexArrays( 1, &VAO );
      }

      // Move to next position

      xPos += s * schar->Right * vScale;
    }

  gpuProg->deactivate();
}
