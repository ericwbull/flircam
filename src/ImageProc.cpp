#include "ImageUtil.h"

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
#include "flircam/Detection.h"
#include "flircam/Configure.h"

ros::Publisher g_pubDetection;

uint16_t g_pixelChangeThreshold = 0;
const int HIST_SIZE=10;

// Signal strength is weighted sum of signal elements.
boost::array<uint32_t, HIST_SIZE> g_signalHistogramWeight;

// Signal detection if strength exceeds threshold.
uint32_t g_detectionThreshold = 0;

// Safe (no detection) if less than threshold. Independent of detectionThreshold.
// Note: Can force a baseline update by setting the safe threshold very high.
uint32_t g_safeThreshold = 0;


inline int getPixel(int x, int y, const std::vector<uint16_t>& data)
{
    int i = y * 80 + x;
    //  std::cout << "i=" << std::dec << i << " " << data[i] << std::endl;

    if (data[i] != 1 && data[i] != 0)
    {
        std::cout << "!!baddata x=" << x << " y=" << y << " data[" << i << "]=" << data[i] << std::endl;
        return 0;
    }
    return data[i];
}

inline int sumAdjacentPixels(int x, int y, const std::vector<uint16_t>& data)
{
    int sum = 0;
    sum += getPixel(x-1,y-1,data);
    sum += getPixel(x,y-1,data);
    sum += getPixel(x+1,y-1,data);
    sum += getPixel(x-1,y,data);
    sum += getPixel(x,y,data);
    sum += getPixel(x+1,y,data);
    sum += getPixel(x-1,y+1,data);
    sum += getPixel(x,y+1,data);
    sum += getPixel(x+1,y+1,data);
    return sum;
}

// returns signal strength
// modifies image so the bit map is returned to the caller
uint32_t DetectChanges(const std::vector<uint16_t>& baseline, const std::vector<uint16_t>& image, boost::array<uint32_t, HIST_SIZE>& histogram, std::vector<uint16_t>& detectionMap)
{
    // compare and threshold
    detectionMap.resize(image.size());
    std::transform(image.begin(), image.end(), baseline.begin(), detectionMap.begin(), ImageUtil::subtract_and_threshold(g_pixelChangeThreshold));

    // image is now an array of 0 or 1.

    histogram.fill(0);

    // Calculate histgram
    for (int y = 1 ; y < 59; y++)
    {
        for (int x = 1; x < 79; x++)
        {
            int signal = sumAdjacentPixels(x,y, detectionMap);
            histogram[signal] += 1;
        }
    }

    // Calculate sigal strenght
    uint32_t strength = 0;
    for (int i = 0; i < histogram.size(); ++i)
    {
        strength += g_signalHistogramWeight[i] * histogram[i];
    }

    return strength;
}

void Configure(const flircam::Configure::ConstPtr& msg)
{
    std::cout << "configure" << std::endl;
    g_pixelChangeThreshold = msg->pixelChangeThreshold;
    g_signalHistogramWeight = msg->signalHistogramWeight;
    g_detectionThreshold = msg->detectionThreshold;
    g_safeThreshold = msg->safeThreshold;

    std::cout << "pixel change " << g_pixelChangeThreshold << std::endl;
    std::cout << "detection " << g_detectionThreshold << std::endl;
    std::cout << "safe " << g_safeThreshold << std::endl;
    std::cout << "hist weight ";
    for(auto h: g_signalHistogramWeight)
      {
	std::cout << h << ' ';
      }
    std::cout << std::endl;
}

void ProcessImage(const std_msgs::UInt32::ConstPtr& msg)
{

    unsigned int imageId = msg->data;

    std::cout << "process image " << imageId << std::endl;

    std::vector<uint16_t> image;
    std::vector<uint16_t> baseline;
    ImageUtil::ReadImage(imageId, image);

    bool updateBaseline = true;
    int strength = 0;
    if (false == ImageUtil::ReadBaseline(imageId, baseline))
    {
        updateBaseline = true;
    }
    else
    {
        // Write the subtraction result to file for inspection
        std::vector<uint16_t> diff;
        diff.resize(image.size());

        std::transform(image.begin(), image.end(), baseline.begin(), diff.begin(), ImageUtil::truncated_minus());
        ImageUtil::WritePGM(imageId, diff, "diff");


        boost::array<uint32_t, HIST_SIZE> histogram;
        std::vector<uint16_t> detectionMap;
        strength = DetectChanges(baseline, image, histogram, detectionMap);
        ImageUtil::WritePGM(imageId, baseline, "diff_baseline");

        // write the detection bitmap to file for inspection.
        ImageUtil::WritePBM(imageId, detectionMap, "detect");
	ImageUtil::WriteDetectionMap(imageId, detectionMap);

        //
        // Publish detection result always
        flircam::Detection detectMsg;

	detectMsg.imageId = imageId;
        detectMsg.detection = strength > g_detectionThreshold;
        detectMsg.safe = strength < g_safeThreshold;
        detectMsg.signalStrength = strength;
        detectMsg.signalHistogram = histogram;

        // If safe (not much change in the image), then update the baseline.
        if (detectMsg.safe)
        {
            updateBaseline = true;
        }

        g_pubDetection.publish(detectMsg);
    }


    if (updateBaseline)
    {
        ImageUtil::WriteBaseline(imageId, image);
        ImageUtil::WritePGM(imageId, image, "new_baseline");
    }

}


void configInit()
{
  g_pixelChangeThreshold = 100;
  g_detectionThreshold = 300;
  g_safeThreshold = 60;
  int weight = 0;
  for (auto& h : g_signalHistogramWeight)
    {
      h = weight++;
    }
}

/**
 * @brief Sample app for streaming IR videos using LePi parallel interface
 */
int main(int argc, char** argv)
{

  configInit();
    std::cout << "ImageProc hello"<< std::endl;

    ros::init(argc, argv, "ImageProc");
    ros::NodeHandle n;

    g_pubDetection =  n.advertise<flircam::Detection>("detection", 10);
    ros::Subscriber subImageDone = n.subscribe<std_msgs::UInt32>("image_done", 10, ProcessImage);
    ros::Subscriber subConfigure = n.subscribe<flircam::Configure>("configure", 10, Configure);

    std::cout << "start ros spin" << std::endl;
    ros::spin();

    return EXIT_SUCCESS;
}


