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
    LeptonCamera cam;
    ros::Publisher pub;
};


void SaveImage(const std_msgs::UInt32::ConstPtr& msg, CamPub& camPub)
{

    LeptonCamera& cam = camPub.cam;
    unsigned int imageId = msg->data;

    bool saveToFile = true;
    if (imageId == 0)
    {
        saveToFile = false;
    }


    // Define frame
    std::vector<uint16_t> frame(cam.width() * cam.height());

    static std::vector<uint16_t> s_flatfield(cam.width() * cam.height());


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
        return;
    }



    if (saveToFile)
    {
        // subtract flat field
        std::transform(frame.begin(), frame.end(), s_flatfield.begin(), frame.begin(), ImageUtil::truncated_minus());

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
        s_flatfield = frame;
    }
}


/**
 * @brief Sample app for streaming IR videos using LePi parallel interface
 */
int main(int argc, char** argv)
{
    std::cout << "FrameGrabber hello"<< std::endl;
    // Open camera connection
    CamPub camPub;
    camPub.cam.sendCommand(REBOOT, nullptr);
    //    sleep(1);
    //    cam.sendCommand(FFC, nullptr);
    sleep(1);
    camPub.cam.start();

    ros::init(argc, argv, "FrameGrabber");
    ros::NodeHandle n;

    auto SaveImageBoundFunction = boost::bind(SaveImage,_1, std::ref(camPub));

    camPub.pub = n.advertise<std_msgs::UInt32>("image_done", 10);
    ros::Subscriber sub = n.subscribe<std_msgs::UInt32>("save_image", 10, SaveImageBoundFunction);

    std::cout << "start ros spin" << std::endl;
    ros::spin();


    // Release sensor
    camPub.cam.stop();


    return EXIT_SUCCESS;
}


