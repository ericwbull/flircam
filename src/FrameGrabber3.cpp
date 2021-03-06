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

// Third party
//#include <opencv2/core.hpp>
//#include <opencv2/highgui.hpp>

// C/C++
#include <pthread.h>
#include <iostream>
#include <unistd.h>
#include <chrono>
#include <sstream>
#include <functional>
#include <fstream>
#include "ros/ros.h"
#include "std_msgs/String.h"
#include "std_msgs/UInt32.h"
#include "flircam/ImageId.h"

#include "ImageUtil.h"

class WatchDog
{
public:

  pthread_t m_tid;
  int m_timeoutSeconds;
  bool m_terminateRequested = false;
  bool m_running = true;
  WatchDog(int timeoutSec)
  {
    m_timeoutSeconds = timeoutSec;
    int result = pthread_create(&m_tid, 0, WatchDog::callRun, this);
    if (result)
      {
	std::cout << "WatchDog thread create error " << result << std::endl;
	m_running = false;
      }
  }

      static void* callRun(void *arg) {return ((WatchDog*)arg)->run();}

  void* run()
  {
    timespec timeoutTime;
    timespec currentTime;
    timespec sleepTime;
    sleepTime.tv_sec = 0;
    sleepTime.tv_nsec = 10000000; // 10 ms
    
    clock_gettime(CLOCK_MONOTONIC_RAW,&currentTime);
    timeoutTime = currentTime;
    timeoutTime.tv_sec += m_timeoutSeconds;

    bool timeout = false;
    while (!timeout && !m_terminateRequested)
      {
	nanosleep(&sleepTime,0);
	clock_gettime(CLOCK_MONOTONIC_RAW,&currentTime);
	if (currentTime.tv_sec > timeoutTime.tv_sec)
	  {
	    timeout = true;
	  }
	if (currentTime.tv_sec == timeoutTime.tv_sec && currentTime.tv_nsec > timeoutTime.tv_nsec)
	  {
	    timeout = true;
	  }
      }
    if (timeout)
      {
	std::cout << "FrameGrabber2 self terminating" << std::endl;
	// Terminate the process
	exit(-1);
      }
  }

  void terminate()
  {
    if (m_running)
      {
	m_terminateRequested = true;
	pthread_join(m_tid, 0);
	m_running = false;
      }
  }

  ~WatchDog()
  {
    terminate();
  }
};

struct CamPub
{
  //  LeptonCamera* cam = nullptr;
    ros::Publisher pub;
  
  CamPub()
  {
  }
  
};


static ImageUtil::NormalizedFrame g_flatfield; // cam.width() * cam.height());

void SaveImage(const flircam::ImageId::ConstPtr& msg, CamPub& camPub)
{
	const flircam::ImageId& imageId = *msg;
	// If this function runs for more than 10 seconds, then the watchdog will terminate the process.
	//WatchDog watchDog(10);

	//    LeptonCamera& cam = *camPub.cam;
	static LePi lePi;
	static bool first = true;
	if (first) {
		if (!lePi.OpenConnection())
		{
			std::cout << "failed to open connectoin" << std::endl;
			exit(-1);
		}
		first = false;

		// Read flatfield from file
		// This is in case we restarted after a crash
		flircam::ImageId flatFieldImageId = imageId;
		flatFieldImageId.frameNumber = 0;
		ImageUtil::ReadBaseline(flatFieldImageId, g_flatfield);
	}

	LeptonType lp_type = lePi.GetType();
	LeptonCameraConfig lp_config(lp_type);

	unsigned int frameNum = imageId.frameNumber;
	std::cout << "frameNum=" << imageId.frameNumber << std::endl;
	if (frameNum == 99)
	{
		// Test what happens if get stuck here for a long time.
		sleep(100);
	}

	bool isFlatField = false;
	if (frameNum == 0)
	{
		isFlatField = true;
	}

	// Define frame
	static std::vector<uint16_t> frame(60 * 80); // cam.width() * cam.height());

	// If end collection, then tell the camera to shutdown.
	if (imageId.endCollection)
	{
		// Forward to ImageProc.  This is just the end of collectoin message, no image data was saved.
		flircam::ImageId pubMsg;
		pubMsg = imageId;
		camPub.pub.publish(pubMsg);
		return;
	}
	else
	{
		// Stream frames
		int frame_nb{ 0 };

		// Frame request
		lePi.GetFrame(&frame[0], FRAME_U16);
	}

	ImageUtil::NormalizedFrame nf(frame);

	if (!isFlatField)
	{
		if (g_flatfield.getNormalizedData().size() != frame.size())
		{
			std::cout << "No flatfield data" << std::endl;
		}
		else
		{
			// subtract flat field
			nf.add(g_flatfield);
		}

		// Output stretched image for inspection
		std::vector<uint16_t> stretched;
		nf.getStretchedData(stretched);
		ImageUtil::WritePGM(imageId, stretched, "stretched");

		if (ImageUtil::WriteImage(imageId, nf))
		{
			flircam::ImageId pubMsg;
			pubMsg = imageId;
			camPub.pub.publish(pubMsg);
		}
	}
	else
	{
		std::cout << "Flatfield" << std::endl;

		// Flatfield
		double average = nf.getAverage();

		nf.add(-average);
		nf.invert();

		std::cout << "save flat field average value=" << average << std::endl;

		// Save as flat field
		g_flatfield = nf;

		flircam::ImageId flatFieldImageId = imageId;
		flatFieldImageId.frameNumber = 0;
		ImageUtil::WriteBaseline(flatFieldImageId, g_flatfield);
	}
}


/**
 * @brief Sample app for streaming IR videos using LePi parallel interface
 */

 // Flatfield correction:
 //   Take image of flat field target.
 //   Take average of all flatfield pixels.
 //   Take difference from average for each pixel.  This is the flatfield correction frame.
 //   Subtract the flatfield correction frame from each image.
 //
 // For each image frame, we take the average of all pixels.  Monitor this average for statistical anomoly.
 // For each image frame, we take the min and max of all pixels.  Monitor these values for statistical anomoly.
 //
 // Translate each pixel value to it's difference from the average -- call this the normalized image. (We did the same to create the flatfield correction frame).
 // The detection algorithm operates on this normalized image, which has the average value subtracted out, and pixels can have negative values.
 // The FrameGrabber component does all of the above.

int main(int argc, char** argv)
{
    std::cout << "FrameGrabber3 hello"<< std::endl;
    CamPub camPub;

    ros::init(argc, argv, "FrameGrabber3");
  
    ros::NodeHandle n;


    auto SaveImageBoundFunction = boost::bind(SaveImage,_1, std::ref(camPub));

    camPub.pub = n.advertise<flircam::ImageId>("image_done", 10);
    ros::Subscriber saveImagesub = n.subscribe<flircam::ImageId>("save_image", 10, SaveImageBoundFunction);


    std::cout << "start ros spin" << std::endl;
    ros::spin();


    return EXIT_SUCCESS;
}


