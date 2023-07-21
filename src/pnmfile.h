/*
Copyright (C) 2006 Pedro Felzenszwalb

This program is free software; you can redistribute it and/or modify
it under the terms of the GNU General Public License as published by
the Free Software Foundation; either version 2 of the License, or
(at your option) any later version.

This program is distributed in the hope that it will be useful,
but WITHOUT ANY WARRANTY; without even the implied warranty of
MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
GNU General Public License for more details.

You should have received a copy of the GNU General Public License
along with this program; if not, write to the Free Software
Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307 USA
*/

/* basic image I/O */

#ifndef PNM_FILE_H
#define PNM_FILE_H

#include <cstdlib>
#include <climits>
#include <cstring>
#include <fstream>
#include "image.h"
#include "misc.h"

namespace felzenszwalb {


#define BUF_SIZE 256

class pnm_error { };

static void read_packed(unsigned char *data, int size, std::ifstream &f) {
  unsigned char c = 0;
  
  int bitshift = -1;
  for (int pos = 0; pos < size; pos++) {
    if (bitshift == -1) {
      c = f.get();
      bitshift = 7;
    }
    data[pos] = (c >> bitshift) & 1;
    bitshift--;
    }
}

static void write_packed(unsigned char *data, int size, std::ofstream &f) {
  unsigned char c = 0;
  
  int bitshift = 7;
  for (int pos = 0; pos < size; pos++) {
      c = c | (data[pos] << bitshift);
      bitshift--;
      if ((bitshift == -1) || (pos == size-1)) {
	f.put(c);
	bitshift = 7;
	c = 0;
      }
  }
}

/* read PNM field, skipping comments */ 
static void pnm_read(std::ifstream &file, char *buf) {
  char doc[BUF_SIZE];
  char c;
  
  file >> c;
  while (c == '#') {
    file.getline(doc, BUF_SIZE);
    file >> c;
  }
  file.putback(c);
  
  file.width(BUF_SIZE);
  file >> buf;
  file.ignore();
}

static image<uchar> *loadPBM(const char *name) {
  char buf[BUF_SIZE];
  
  /* read header */
  std::ifstream file(name, std::ios::in | std::ios::binary);
  pnm_read(file, buf);
  if (strncmp(buf, "P4", 2))
    throw pnm_error();
    
  pnm_read(file, buf);
  int width = atoi(buf);
  pnm_read(file, buf);
  int height = atoi(buf);
  
  /* read data */
  image<uchar> *im = new image<uchar>(