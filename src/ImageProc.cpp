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

//const int HIST_SIZE=10;

// Signal strength is weighted sum of signal elements.
//boost::array<uint32_t, HIST_SIZE> g_signalHistogramWeight;

// Signal detection if strength exceeds threshold.
//uint32_t g_detectionThreshold = 0;

// Safe (no detection) if less than threshold. Independent of detectionThreshold.
// Note: Can force a baseline update by setting the safe threshold very high.
//uint32_t g_safeThreshold = 0;

uint16_t g_pixelChangeThreshold; // 150 typical
uint8_t  g_neighborMaxValueSpan; // 0 to 2 typical
uint8_t  g_neighborDetectionSpan; // 0 to 2 typical
uint8_t  g_neighborDetectionCountThreshold; // 1 to 9 typical

inline bool validateData(const std::vector<uint16_t>& data, int minVal, int maxVal)
{
  int i = 0;
  for(auto d:data)
    {
      if (d > maxVal || d < minVal)
	{
	  std::cout << "! baddata d: " << d << " i:" << i << std::endl;
	  return false;
	}
      i++;
    }
  return true;
}

inline int getPixel(int x, int y, const std::vector<uint16_t>& data)
{
    int i = y * 80 + x;
    return data[i];
}

inline void setPixel(int x, int y, std::vector<uint16_t>& data, uint16_t value)
{
    int i = y * 80 + x;
    data[i] = value;
}

inline int sumAdjacentPixels(int x, int y, int span, const std::vector<uint16_t>& data)
{
    int sum = 0;
    
    for(int x1 = -span; x1 <= span; x1++)
      {
	if (x+x1 < 0 || x+x1 >= 80) continue;
	for(int y1 = -span; y1 <= span; y1++)
	  {
	    if (y+y1 < 0 || y+y1 >= 60) continue;
	    sum += getPixel(x+x1,y+y1,data);
	  }
      }
    return sum;
}


inline uint16_t getMaxValueAdjacentPixels(int x, int y, int span, const std::vector<uint16_t>& data)
{
    uint16_t maxVal = 0;
    for(int x1 = -span; x1 <= span; x1++)
      {
	if (x+x1 < 0 || x+x1 >= 80) continue;
	for(int y1 = -span; y1 <= span; y1++)
	  {
	    if (y+y1 < 0 || y+y1 >= 60) continue;
	    uint16_t pixelVal = getPixel(x+x1,y+y1,data);
	    if (maxVal < pixelVal)
	      {
		maxVal = pixelVal;
	      }
	  }
      }
    
    return maxVal;
}

// returns signal strength
// modifies image so the bit map is returned to the caller
int DetectChanges(const std::vector<uint16_t>& baseline, const std::vector<uint16_t>& image, std::vector<uint16_t>& detectionMap, std::vector<uint16_t>& filteredDetectionMap)
{
    // compare and threshold
    detectionMap.assign(image.size(),0);
    std::transform(image.begin(), image.end(), baseline.begin(), detectionMap.begin(), ImageUtil::subtract_and_threshold(g_pixelChangeThreshold));

    // detection map is now an array of 0 or 1.
    if (false == validateData(detectionMap, 0, 1))
      {
	return -1;
      }

    // Next filter the detection map based on neighborhood detection count.
    filteredDetectionMap.assign(image.size(),0);
    int detectionPixelCount = 0;
    for (int y = 0; y < 60; y++)
    {
        for (int x = 0; x < 80; x++)
        {
    	  int sum = sumAdjacentPixels(x,y,g_neighborDetectionSpan, detectionMap);
	  if (sum >= g_neighborDetectionCountThreshold)
	    {
	      setPixel(x,y,filteredDetectionMap,1);
	      detectionPixelCount++;
	    }
	}
    }

    return detectionPixelCount;
}

void Configure(const flircam::Configure::ConstPtr& msg)
{
    std::cout << "configure" << std::endl;
    g_pixelChangeThreshold = msg->pixelChangeThreshold;
    g_neighborMaxValueSpan = msg->maxNeighborSpan;
    g_neighborDetectionSpan = msg->neighborDetectionSpan;
    g_neighborDetectionCountThreshold = msg->neighborDetectionCountThreshold;

    //    g_signalHistogramWeight = msg->signalHistogramWeight;
    //    g_safeThreshold = msg->safeThreshold;

    std::cout << "pixelChangeThreshold: " << g_pixelChangeThreshold << std::endl;
    std::cout << "neighborMaxValueSpan: " << g_neighborMaxValueSpan << std::endl;
    std::cout << "neighborDetectionSpan: " << g_neighborDetectionSpan << std::endl;
    std::cout << "neighborDetectionCountThreshold: " << g_neighborDetectionCountThreshold << std::endl;
}

void UpdateBaseline(int imageId, const std::vector<uint16_t> image)
{
  // Apply maxVal of neighbors to create baseline from image

  std::vector<uint16_t> baseline;
  baseline.assign(image.size(), 0);
  for (int y = 0; y < 60; y++)
    {
      for (int x = 0; x < 80; x++)
        {
	  uint16_t maxVal = getMaxValueAdjacentPixels(x, y, g_neighborMaxValueSpan, image);
	  setPixel(x,y,baseline,maxVal);
	}
    }

  ImageUtil::WriteBaseline(imageId, baseline);
  ImageUtil::WritePGM(imageId, baseline, "new_baseline");
}

void ProcessImage(const std_msgs::UInt32::ConstPtr& msg)
{

    unsigned int imageId = msg->data;

    std::cout << "process image " << imageId << std::endl;

    std::vector<uint16_t> image;
    std::vector<uint16_t> baseline;
    ImageUtil::ReadImage(imageId, image);

    bool updateBaseline = false;
    int detectionCount = 0;
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


        std::vector<uint16_t> detectionMap;
        std::vector<uint16_t> filteredDetectionMap;
        detectionCount = DetectChanges(baseline, image, detectionMap, filteredDetectionMap);

	// -1 is error
	if (detectionCount >= 0)
	  {
	    ImageUtil::WritePGM(imageId, baseline, "diff_baseline");

            // write the detection bitmap to file for inspection.
	    ImageUtil::WritePBM(imageId, detectionMap, "detect");
	    ImageUtil::WritePBM(imageId, filteredDetectionMap, "filteredDetect");
	    ImageUtil::WriteDetectionMap(imageId, filteredDetectionMap);
	  }
	
        //
        // Publish detection result always
        flircam::Detection detectMsg;

	detectMsg.imageId = imageId;
        detectMsg.detection = detectionCount > 0;
        detectMsg.safe = detectionCount == 0;
        detectMsg.error = detectionCount < 0;
        detectMsg.detectionCount = detectionCount;
	//        detectMsg.signalHistogram = histogram;

        // If safe (not much change in the image), then update the baseline.
        if (detectMsg.safe)
        {
            updateBaseline = true;
        }

        g_pubDetection.publish(detectMsg);
    }


    if (updateBaseline)
    {
      UpdateBaseline(imageId, image);
    }

}


void configInit()
{
  g_pixelChangeThreshold = 150;
  g_neighborMaxValueSpan = 2;
  g_neighborDetectionSpan = 1;
  g_neighborDetectionCountThreshold = 6;
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


