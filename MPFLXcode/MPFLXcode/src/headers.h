// The standard headers included by all files


#ifndef HEADERS_H
#define HEADERS_H

#include "glad/include/glad/glad.h"
#include <GLFW/glfw3.h>

#include <sys/time.h>	// includes gettimeofday (to return current time)

#ifdef LINUX
  #include <unistd.h>		// includes usleep (to sleep for some time)
  #include <values.h>           // includes MAX_FLOAT
  #define sprintf_s sprintf
  #define _strdup strdup
  #define sscanf_s sscanf
  #define _getcwd getcwd
#endif

#ifdef _WIN32
  #include <direct.h>
  #define PATH_MAX 1000
  //#include <typeinfo>
  //#define M_PI 3.14159
  #define MAXFLOAT FLT_MAX
  //#define rint(x) floor((x)+0.5)
  #pragma warning(disable : 4244 4305 4996 4838)
#endif

#ifdef __APPLE_CC__
  #include <unistd.h>
  #define sprintf_s sprintf
  #define _strdup strdup
  #define sscanf_s sscanf
  #define _getcwd getcwd
#endif

#include <cstdlib>
#include <cstdio>
#include <cstring>
#include <iostream>
using namespace std;

#include <cmath>

#include "linalg.h"

#define randIn01() (rand() / (float) RAND_MAX)   // random number in [0,1]

#endif
