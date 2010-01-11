/*
    drawing bitmaps
    Copyright (C) 2009 Bob Mottram
    fuzzgun@gmail.com

    This program is free software: you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation, either version 3 of the License, or
    (at your option) any later version.

    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.

    You should have received a copy of the GNU General Public License
    along with this program.  If not, see <http://www.gnu.org/licenses/>.
*/

#ifndef BITMAP_H
#define BITMAP_H

#include <stdio.h>
#include <stdlib.h>
#include <fstream>
#include <iostream>
#include <string>
#include <cstring>
#include <assert.h>
using namespace std;

typedef struct BITMAPFILEHEADER
{
    unsigned long bfType;
    unsigned long bfSize;
    unsigned short bfReserved1;
    unsigned short bfReserved2;
    unsigned long bfOffBits;
} BITMAPFILEHEADER;

typedef struct BITMAPINFOHEADER
{
    //unsigned long biSize;
    unsigned long biWidth;
    unsigned long biHeight;
    unsigned short biPlanes;
    unsigned short biBitCount;
    unsigned long biCompression;
    unsigned long biSizeImage;
    unsigned long biXPelsPerMeter;
    unsigned long biYPelsPerMeter;
    unsigned long biClrUsed;
    unsigned long biClrImportant;
} BITMAPINFOHEADER;


class Bitmap
{
public:
    int Width, Height;

    //Initalization
    Bitmap();
    Bitmap(Bitmap &B);
    Bitmap(Bitmap &B, int x, int y, int Width, int Height);
    Bitmap(int Width, int Height);
    Bitmap(unsigned char *bmp, int Width, int Height, int BytesPerPixel);

    //Destruction
    ~Bitmap();

    //Memory
    void FreeMemory();
    void Allocate(int Width, int Height);
    Bitmap& operator = (const Bitmap &B);

    //Clearing Functions
    void Clear();                       //clears all pixels to black

    //File Functions
    void Save(const char *filename); //saves bitmap to filename in *.BMP format
    void SavePPM(const char *filename); //saves bitmap to filename in *.PPM format
    bool FromFile(std::string filename); //loads bitmap from filename in 24 or 8 bit *.BMP format

    unsigned char* Data;    // raw image data
    int bytes_per_pixel;
};

#endif
