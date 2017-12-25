/**
 * This file is part of the LePi Project:
 * https://github.com/cosmac/LePi
 *
 * MIT License
 *
 * Copyright (c) 2017 Andrei Claudiu Cosma
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 */

// LePi
#include <LeptonCommon.h>
#include <LeptonCamera.h>
#include <png.h>

// Third party
//#include <opencv2/core.hpp>
//#include <opencv2/highgui.hpp>

// C/C++
#include <iostream>
#include <unistd.h>
#include <chrono>

/**
 * @brief Sample app for streaming IR videos using LePi parallel interface
 */
int write_png_file(const char* file_name, std::vector<uint16_t>& frame );
int main()
{    
    // Open camera connection
    LeptonCamera cam;
    //        cam.sendCommand(AGC_EN, nullptr);
    //        sleep(1);
	    cam.sendCommand(REBOOT, nullptr);
        sleep(1);
	//	            cam.sendCommand(FFC, nullptr);
        sleep(1);
    cam.start();
    
    // Define frame
    std::vector<uint16_t> frame(cam.width() * cam.height());
    
    // Stream frames
    int frame_nb{0};


    while (true) {


	char key;
	std::cin >> key;
	if (key == 'q'){
	  break;
	}
	if (key == 'f'){
	  
	  // Frame request
	  if (cam.hasFrame()) {
	    cam.getFrameU16(frame);
	    ++frame_nb;

	    write_png_file("frame",frame);
	  }
	}
    }

    // Release sensor
    cam.stop();

    
    return EXIT_SUCCESS;
}



int write_png_file(const char* file_name, std::vector<uint16_t>& frame )
{
  /* create file */
  FILE *fp = fopen(file_name, "wb");
  if (!fp)
    {
      std:: cerr << "File open failed '" << file_name << "'" << std::endl;
      return -1;
    }


  /* initialize stuff */
  png_structp png_ptr;
  png_ptr = png_create_write_struct(PNG_LIBPNG_VER_STRING, NULL, NULL, NULL);

  if (!png_ptr)
    {
      std::cerr << "[write_png_file] png_create_write_struct failed" << std::endl;
      return -1;
    }


  png_infop info_ptr;
  info_ptr = png_create_info_struct(png_ptr);
  if (!info_ptr)
    {
      std::cerr << "[write_png_file] png_create_info_struct failed" << std::endl;
      return -1;
    }

  //  if (setjmp(png_jmpbuf(png_ptr)))
  //    abort_("[write_png_file] Error during init_io");

  png_init_io(png_ptr, fp);


  /* write header */
  //  if (setjmp(png_jmpbuf(png_ptr)))
  //    abort_("[write_png_file] Error during writing header");

  png_set_IHDR(png_ptr, info_ptr, 80, 60,
	       16, PNG_COLOR_TYPE_GRAY, PNG_INTERLACE_NONE,
	       PNG_COMPRESSION_TYPE_DEFAULT, PNG_FILTER_TYPE_DEFAULT);

  png_write_info(png_ptr, info_ptr);


  /* write bytes */
///  if (setjmp(png_jmpbuf(png_ptr)))
//    abort_("[write_png_file] Error during writing bytes");


  
  int row = 0;
  for (row = 0; row < 60; row++)
    {
      png_byte data[160];
      for (int i = 0; i < 80; ++i)
	{
	  uint16_t word = frame[row*80+i];
	  data[2*i] = word >> 8;
	  data[2*i+1] = word & 0xff;
	  
	}
      png_write_row(png_ptr, data);
      
    }
  //  png_write_image(png_ptr, row_pointers);


  /* end write */
  //  if (setjmp(png_jmpbuf(png_ptr)))
  //    abort_("[write_png_file] Error during end of write");

  png_write_end(png_ptr, NULL);

  /* cleanup heap allocation */
  //  for (y=0; y<height; y++)
  //    free(row_pointers[y]);
  //  free(row_pointers);

  fclose(fp);
}
