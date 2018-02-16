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
#include <sstream>
#include <functional>
#include <fstream>
#include "ros/ros.h"
#include "std_msgs/String.h"
#include "std_msgs/UInt32.h"

#include "ImageUtil.h"

struct CamPub
{
  LeptonCamera* cam = nullptr;
    ros::Publisher pub;

  CamPub()
  {
    cam = new LeptonCamera();
    //    cam->sendCommand(RESET, nullptr);
    //    sleep(1);
  }
  void start()
  {
    cam->start();
  }    
  void stop()
  {
    cam->stop();
  }    
  void restart()
  {
    std::cout << "stopping..." << std::endl;
    stop();
    std::cout << "stopped" << std::endl;
    //    delete cam;
    //    cam = new LeptonCamera();
    std::cout << "sending reset" << std::endl;
    int count = 10;
    while (false == cam->sendCommand(RESET, nullptr))
      {
	std::cout << "reset failed " << count << std::endl;
	
	if (count-- == 0)
	  {
	    std::cout << "! FrameGrabber self-terminating" << std::endl;
	    exit(-1);
	  }
      }
    
    std::cout << "reset done" << std::endl;
    
    //    sleep(1);
    start();
    std::cout << "started" << std::endl;
  }    
  
};

std::vector<uint16_t> g_flatfield(80 * 60);

void SaveImage(const std_msgs::UInt32::ConstPtr& msg, CamPub& camPub)
{

    LeptonCamera& cam = *camPub.cam;
    unsigned int imageId = msg->data;

    bool saveToFile = true;
    if (imageId == 0)
    {
        saveToFile = false;
    }


    // Define frame
    std::vector<uint16_t> frame(cam.width() * cam.height());



    // Stream frames
    int frame_nb{0};

    // Frame request
    if (cam.hasFrame()) {
        cam.getFrameU16(frame);
        ++frame_nb;
    }
    else
    {
        std::cout << "no frame" << std::endl;
	camPub.restart();
	//	cam.sendCommand(
        return;
    }



    if (saveToFile)
    {
        // subtract flat field
        std::transform(frame.begin(), frame.end(), g_flatfield.begin(), frame.begin(), ImageUtil::truncated_minus());

        uint16_t minValue = 0;
        uint16_t maxValue = 0;

        auto iterPairMinMax = std::minmax_element(frame.begin(), frame.end());
        minValue = *iterPairMinMax.first;
        maxValue = *iterPairMinMax.second;

        // Output stretched image for inspection
        std::vector<uint16_t> stretched = frame;
        ImageUtil::stretch s;
        s.min = minValue;
        s.scalar = 65535 / (maxValue - minValue);
        std::for_each(stretched.begin(), stretched.end(), s);
        ImageUtil::WritePGM(imageId, stretched, "stretched");

        if (ImageUtil::WriteImage(imageId,frame))
        {
            std_msgs::UInt32 pubMsg;
            pubMsg.data = imageId;
            camPub.pub.publish(pubMsg);
        }
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
        g_flatfield = frame;
	ImageUtil::WriteImage(0,g_flatfield);
    }
}


/**
 * @brief Sample app for streaming IR videos using LePi parallel interface
 */
int main(int argc, char** argv)
{
    std::cout << "FrameGrabber hello"<< std::endl;

    // Read flatfield file
    ImageUtil::ReadImage(0, g_flatfield);
    
    // Open camera connection
    CamPub camPub;
    camPub.start();

    ros::init(argc, argv, "FrameGrabber");
    ros::NodeHandle n;

    auto SaveImageBoundFunction = boost::bind(SaveImage,_1, std::ref(camPub));

    camPub.pub = n.advertise<std_msgs::UInt32>("image_done", 10);
    ros::Subscriber sub = n.subscribe<std_msgs::UInt32>("save_image", 10, SaveImageBoundFunction);

    std::cout << "start ros spin" << std::endl;
    ros::spin();


    // Release sensor
    camPub.stop();


    return EXIT_SUCCESS;
}


