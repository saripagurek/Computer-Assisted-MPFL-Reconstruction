// csv.C

#include "seq.h"
#include "csv.h"
#include <cstdio>
#include <cstring>
#include <iostream>
using namespace std;

#define MAX_LINE_LEN 100000



// Read a CSV file

void CSV::read( const char *filename )

{
  char entry[MAX_LINE_LEN];

  FILE *f = fopen( filename, "r" );

  if (f == NULL) {
    cerr << "Could not open '" << filename << "'." << endl;
    return;
  }

  fn = strdup(filename);

  int row = 0;
  int maxCol = 0;

  seq< seq<char*> > lines;

  while (!feof(f)) {

    seq<char*> line;

    char c;
    int col = -1;

    do { // Handle one row

      /* Find first non-empty entry on this line */

      do {
	do
	  c = getc(f);
	while (!feof(f) && isspace(c) && c != '\r' && c != '\n');
	if (c == '\r')
	  c = getc(f);
	col++;
	if (c == CSV_SEPARATOR)
	  line.add( "" );
      } while (c == CSV_SEPARATOR);

      if (feof(f))
	break;

      bool insideQuotes = (c == '"');

      /* Get the entry */

      int i = 0;

      if (!insideQuotes)
	ungetc(c,f);

      do {
	c = getc(f);
	if (insideQuotes)

	  if (c == '\n' && convertNewlinesToHTML) {

	    entry[i++] = '<';
	    entry[i++] = 'b';
	    entry[i++] = 'r';
	    entry[i++] = '>';

	  } else if (c == '"') {

	    char c2 = getc(f);
	    if (c2 == '"') {
	      entry[i++] = '"';
	      c = '.';		// dummy value so as not to trigger end-of-entry detection
	    } else
	      ungetc(c2,f);

	  } else

	    entry[i++] = c;

	else // !insideQuotes

	  if (c != CSV_SEPARATOR && c != '\r' && c != '\n')
	    entry[i++] = c;

      } while (i < MAX_LINE_LEN-1 && !feof(f) && (!insideQuotes || c != '"') && (insideQuotes || (c != CSV_SEPARATOR && c != '\n' && c != '\r')));

      if (c == '\r')
	c = getc(f);

      entry[i] = '\0';

      if (i >= MAX_LINE_LEN) {
	fclose( f );
	cerr << "The data file has an entry in row " << row+1 << ", column " << col+1
	     << " of more than " << MAX_LINE_LEN-1 << " characters" << endl;
	return;
      }

      // Copy the entry

      line.add( strdup( entry ) );

      // Get the next char after the entry (unless the entry was ended with \n)

      if (insideQuotes)
	c = getc(f);

      if (c == '\r')
	c = getc(f);

    } while (!feof(f) && c != '\n' && c != '\r');

    lines.add( line );

    if (c == '\r')
      c = getc(f);

    if (col > maxCol)
      maxCol = col;
    
    if (!feof(f))
      row++;
  }

  fclose( f );

  rows = row;
  cols = maxCol+1;

  data = (char ***) malloc( rows * sizeof( char ** ) );
  for (int i=0; i<rows; i++) {
    data[i] = (char **) malloc( cols * sizeof( char * ) );
    for (int j=0; j<cols; j++)
      if (j < lines[i].size())
	data[i][j] = lines[i][j];
      else
	data[i][j] = "";
  }
}



// Write a CSV file

void CSV::write( const char *filename )

{
  FILE *f = fopen( filename, "w" );

  if (f == NULL) {
    cerr << "Could not open '" << filename << "'." << endl;
    return;
  }

  for (int i=0; i<rows; i++) {
    for (int j=0; j<cols; j++) {
      if (data[i][j][0] != '\0') {

	fputc( '"', f );

	for (char *p = data[i][j]; *p != '\0'; p++) {
	  fputc( *p, f );
	  if (*p == '"')
	    fputc( *p, f );	// double the quotes
	}

	fputc( '"', f );
      }

      if (j < cols-1)
	fputc( CSV_SEPARATOR, f );
    }

    fputc( '\n', f );
  }

  fclose( f );
}


// Verify that the column headers (on line 1) are identical to what
// are expected.

void CSV::checkHeaders( const char *filename, char *headers[] )

{
  bool ok = true;

  for (int i=0; headers[i][0] != '\0'; i++)
    if (strcmp( headers[i], data[0][i] ) != 0) {
      ok = false;
      cerr << "In " << filename << ", column " << i+1 << " header was exepected to be '" << headers[i]
	   << "', but was actually '" << data[0][i] << "'." << endl;
    }

  if (!ok)
    exit(1);
}
