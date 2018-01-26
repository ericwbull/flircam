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

// Third party
//#include <opencv2/core.hpp>
//#include <opencv2/highgui.hpp>

// C/C++
#include <png.h>
#include <iostream>
#include <unistd.h>
#include <chrono>
#include <boost/filesystem.hpp>
#include <sstream>
#include <functional>
#include <fstream>
#include <array>
#include "ros/ros.h"
#include "std_msgs/String.h"
#include "std_msgs/UInt32.h"

int write_image_data_file(const char* file_name, const std::vector<uint16_t>& frame );
int read_image_data_file(const char* file_name, std::vector<uint16_t>& frame );


struct truncated_minus: std::binary_function<uint16_t, uint16_t, uint16_t>
{
  uint16_t operator()(uint16_t a, uint16_t b) const
  {
    if (b > a)
      {
	return 0;
      }
    else
      {
	return a - b;
      }
  }
};

struct stretch
{
  uint16_t min;
  uint16_t scalar;
  
  uint16_t operator()(uint16_t& pixel) const
  {
    pixel = (pixel - min) * scalar;
  }
};

class Normalized
{
public:
  uint16_t m_average;
  std::array<int16_t,80*60> m_data;
};

std::string GetImageFileName(unsigned int imageId)
{
  // folder number
  unsigned int majorId = imageId >> 16;

  // file number
  unsigned int minorId = imageId & 0xffff;

  std::ostringstream ossDir;
  std::ostringstream ossFilename;

  ossDir << "/tmp/flircam/" << majorId;
  ossFilename << "/tmp/flircam/" << majorId << '/' << minorId;

  std::string dirname = ossDir.str(); 
  std::string fname = ossFilename.str(); 

  //  std::cout << "dirname=" << dirname << std::endl;
  //  std::cout << "fname=" << fname << std::endl;
  return fname;
}

std::string GetBaselineFileName(unsigned int imageId)
{
  std::string imageFileName = GetImageFileName(imageId);
  return imageFileName + ".baseline";
}

void GetBaseline(unsigned int imageId, Normalized& baseline)
{
  // If there's no baseline file, then use the source imageId as the baseline.
  // Baseline file is /tmp/flircam/0/1.baseline
}

ros::Publisher g_pub;

void ProcessImage(const std_msgs::UInt32::ConstPtr& msg)
{

  unsigned int imageId = msg->data;

  std::cout << "process image " << imageId << std::endl; 

  Normalized baseline;
  GetBaseline(imageId, baseline);
  
  #if 0
    // subtract flat field
    std::transform(frame.begin(), frame.end(), s_flatfield.begin(), frame.begin(), truncated_minus());

    uint16_t minValue = 0;
    uint16_t maxValue = 0;

    auto iterPairMinMax = std::minmax_element(frame.begin(), frame.end());
    minValue = *iterPairMinMax.first;
    maxValue = *iterPairMinMax.second;

    stretch s;
    s.min = minValue;
    s.scalar = 65535 / (maxValue - minValue);
    std::for_each(frame.begin(), frame.end(), s); 
    //    write_png_file(fname.c_str(),frame);
  }
  else
    {
      auto iterMin = std::min_element(frame.begin(), frame.end());
      uint16_t minValue = *iterMin;
      
      for (auto& item: frame)
	{
	  item -= minValue;
	}
      
      std::cout << "save flat field minValue=" << minValue <<  std::endl;
      // Save as flat field
      s_flatfield = frame;
    }

#endif

// If heat difference found then publish heat detection message containing the imageId.
  if (false)
  {
    std_msgs::UInt32 pubMsg;
    pubMsg.data = imageId;
    g_pub.publish(pubMsg);
  }

}


/**
 * @brief Sample app for streaming IR videos using LePi parallel interface
 */
int main(int argc, char** argv)
{
  std::cout << "ImageProc hello"<< std::endl;
    
    ros::init(argc, argv, "ImageProc");
    ros::NodeHandle n;
    
    g_pub =  n.advertise<std_msgs::UInt32>("detection", 10);
    ros::Subscriber sub = n.subscribe<std_msgs::UInt32>("image_done", 10, ProcessImage);

    std::cout << "start ros spin" << std::endl;
    ros::spin();
    
    return EXIT_SUCCESS;
}


int write_image_data_file(const char* file_name, const std::vector<uint16_t>& frame )
{
  std::ofstream ofs(file_name, std::ios::binary);
  
  if (ofs.fail())
    {
      std:: cerr << "File open failed '" << file_name << "'" << std::endl;
      return -1;
    }

  int size = frame.size() * sizeof(uint16_t);
  //  ofs.write(&size, sizeof(int));
  // For the current application the size is always 80 x 60 pixels
  ofs.write(reinterpret_cast<const char*>(&frame[0]), size);

  return 0;
}

int read_image_data_file(const char* file_name, std::vector<uint16_t>& frame )
{
  std::ofstream ofs(file_name, std::ios::binary);
  
  if (ofs.fail())
    {
      std:: cerr << "File open failed '" << file_name << "'" << std::endl;
      return -1;
    }

  int size = frame.size() * sizeof(uint16_t);
  //  ofs.write(&size, sizeof(int));
  // For the current application the size is always 80 x 60 pixels
  ofs.write(reinterpret_cast<char*>(&frame[0]), size);

  return 0;
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

  static png_bytep s_row_ptrs[60];
  static png_byte s_data[60][160];

  static bool s_onceOnlyInit = true;
  if (s_onceOnlyInit)
    {
      s_onceOnlyInit = false;
      for (int i = 0; i < 60; i++){
	s_row_ptrs[i] = &s_data[i][0];
      }
  
    }  

  // swap bytes
  int row = 0;
  for (row = 0; row < 60; row++)
    {
      for (int col = 0; col < 80; ++col)
	{
	  uint16_t word = frame[row*80+col];
	  s_data[row][2*col] = word >> 8;
	  s_data[row][2*col+1] = word & 0xff;
	  
	}
      
    }
  png_write_image(png_ptr, s_row_ptrs);


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
