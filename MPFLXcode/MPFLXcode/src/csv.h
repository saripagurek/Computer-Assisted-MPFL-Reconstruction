// csv.h


#ifndef CSV_H
#define CSV_H


#include <cstdlib>


#define CSV_SEPARATOR ','


class CSV {

  bool convertNewlinesToHTML;

 public:

  int rows, cols;
  char ***data;
  char *fn;

  CSV( const char *filename ) {
    convertNewlinesToHTML = false;
    read( filename );
  }

  CSV( const char *filename, char *headers[] ) {
    convertNewlinesToHTML = false;
    read( filename );
    checkHeaders( filename, headers );
  }

  CSV( int r, int c ) {
    convertNewlinesToHTML = false;
    rows = r;
    cols = c;
    data = (char ***) malloc( rows * sizeof( char ** ) );
    for (int i=0; i<rows; i++) {
      data[i] = (char **) malloc( cols * sizeof( char * ) );
      for (int j=0; j<cols; j++)
	data[i][j] = "";
    }
  }

  ~CSV() {
    for (int i=0; i<rows; i++)
      free( data[i] );
    free( data );
  }

  void read( const char *filename );
  void write( const char *filename );

  void checkHeaders( const char *filename, char *headers[] );
};


#endif
