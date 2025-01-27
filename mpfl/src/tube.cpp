// tube.cpp


#include "tube.h"


void Tube::setupVAO()

{
  // Set up buffers of vertices, normals, and face indices.  The
  // vertices and normals are calculated at drawing time.

  nVerts = TUBE_LINEAR_COUNT * TUBE_RADIAL_COUNT;

  vertexBuffer = new vec3[ nVerts ];
  normalBuffer = new vec3[ nVerts ];

  // Set up faces

  nFaces = 2 * TUBE_RADIAL_COUNT * (TUBE_LINEAR_COUNT - 1);

  GLuint *indexBuffer = new GLuint[ nFaces * 3 ];

  GLuint *i = indexBuffer;
  
  for (int z=0; z<TUBE_LINEAR_COUNT-1; z++)
    for (int r=0; r<TUBE_RADIAL_COUNT; r++) {

      // Two triangles for the face between
      //
      //    (r,z+1)---(r+1,z+1)
      //       |          |
      //       |          |
      //     (r,z)-----(r+1,z)
      //
      // Each radial slice, z, has TUBE_RADIAL_COUNT vertices.

      int rp1  = (r+1) % TUBE_RADIAL_COUNT;
      int zz   =   z   * TUBE_RADIAL_COUNT;  // = offset to slice z
      int zzp1 = (z+1) * TUBE_RADIAL_COUNT;  // = offset to slice z+1

      *i++ = r   + zz;
      *i++ = rp1 + zz;
      *i++ = r   + zzp1;

      *i++ = rp1 + zzp1;
      *i++ = r   + zzp1;
      *i++ = rp1 + zz;
    }

  // Create a VAO

  glGenVertexArrays( 1, &VAO );
  glBindVertexArray( VAO );

  // set up position and normal buffers, but don't provide data

  // vertex = attribute 0
  
  glGenBuffers( 1, &vertexBufferID );
  glBindBuffer( GL_ARRAY_BUFFER, vertexBufferID );
  glEnableVertexAttribArray( 0 );
  glVertexAttribPointer( 0, 3, GL_FLOAT, GL_FALSE, 0, 0 );

  // normal = attribute 1
  
  glGenBuffers( 1, &normalBufferID );
  glBindBuffer( GL_ARRAY_BUFFER, normalBufferID );
  glEnableVertexAttribArray( 1 );
  glVertexAttribPointer( 1, 3, GL_FLOAT, GL_FALSE, 0, 0 );

  // store faces

  GLuint indexBufferID;
  glGenBuffers( 1, &indexBufferID );
  glBindBuffer( GL_ELEMENT_ARRAY_BUFFER, indexBufferID );
  glBufferData( GL_ELEMENT_ARRAY_BUFFER, nFaces * 3 * sizeof(GLuint), indexBuffer, GL_STATIC_DRAW );

  // Clean up

  glBindVertexArray( 0 );
  glBindBuffer( GL_ARRAY_BUFFER, 0 );
  glBindBuffer( GL_ELEMENT_ARRAY_BUFFER, 0 );

  delete[] indexBuffer;
}


// Draw a tube with axis
//
//     a u^3 + b u^2 + c u + d
//
// for u in [0,1] and radius r.


void Tube::draw( vec3 a, vec3 b, vec3 c, vec3 d, float r )

{
  // Fill in vertex and normal buffers

  float u = 0;

  vec3 *vBuff = vertexBuffer;
  vec3 *nBuff = normalBuffer;

  vec3 nextCentre =  d + u * (c + u * (b + u * a));
  
  for (int i=0; i<TUBE_LINEAR_COUNT; i++) {

    // axis centre (this and next)

    vec3 thisCentre = nextCentre;

    u += 1.0 / (float) (TUBE_LINEAR_COUNT-1);

    nextCentre = d + u * (c + u * (b + u * a)); // (this will extend past the axis endpoint)

    // local coordinate system
    //
    // NOT GOOD: Need to use a Frenet frame to avoid snapping of the perp vectors.
    
    vec3 z = (nextCentre - thisCentre).normalize();
    vec3 x = z.perp1();
    vec3 y = z.perp2();

    // One ring around the axis
    
    float theta = 0;
    for (int j=0; j<TUBE_RADIAL_COUNT; j++) {

      vec3 norm = cos(theta) * x + sin(theta) * y;
      vec3 vert = thisCentre + r*norm;

      *vBuff++ = vert;
      *nBuff++ = norm;

      theta += 2*M_PI / (float) TUBE_RADIAL_COUNT;
    }
  }

  // Update vertex and normal buffers

  glBindBuffer( GL_ARRAY_BUFFER, vertexBufferID );
  glBufferData( GL_ARRAY_BUFFER, nVerts * 3 * sizeof(float), (GLfloat*) & (*vertexBuffer).x, GL_STREAM_DRAW );

  glBindBuffer( GL_ARRAY_BUFFER, normalBufferID );
  glBufferData( GL_ARRAY_BUFFER, nVerts * 3 * sizeof(float), (GLfloat*) & (*normalBuffer).x, GL_STREAM_DRAW );

  glBindBuffer( GL_ARRAY_BUFFER, 0 );

  // Draw
  
  glLineWidth( 3.0 );
  glBindVertexArray( VAO );
  glDrawElements( GL_TRIANGLES, nFaces * 3, GL_UNSIGNED_INT, 0 );
  glBindVertexArray( 0 );
  glLineWidth( 1.0 );
}

