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

// returns detection count
int DetectChanges(const std::vector<double>& sigma, std::vector<uint16_t>& detectionMap, std::vector<uint16_t>& filteredDetectionMap)
{
    // compare and threshold
    detectionMap.assign(sigma.size(),0);

    // sigma values more than some threshold flag as detection
    //    std::transform(sigma.begin(), sigma.end(), baseline.begin(), detectionMap.begin(), ImageUtil::subtract_and_threshold(g_pixelChangeThreshold));

    // detection map is now an array of 0 or 1.
    if (false == validateData(detectionMap, 0, 1))
      {
	return -1;
      }

    // Next filter the detection map based on neighborhood detection count.
    filteredDetectionMap.assign(sigma.size(),0);
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


void PublishDetection(const flircam::ImageId& imageId, int detectionCount)
{
  flircam::Detection detectMsg;

  detectMsg.imageId = imageId;
  detectMsg.detection = detectionCount > 0;
  detectMsg.safe = detectionCount == 0;
  detectMsg.error = detectionCount < 0;
  detectMsg.detectionCount = detectionCount;
  g_pubDetection.publish(detectMsg);
}

void ReceiveImageId(const flircam::ImageId::ConstPtr& msg)
{
    const flircam::ImageId& imageId = *msg;
    std::cout << "image id " << ImageUtil::ImageIdToString(imageId) << std::endl;

    if (imageId.endCollection)
      {
	// No data to process
	PublishDetection(imageId, 0);
	return;
      }
    else
      {
	ProcessImage(imageId);
      }
}

struct AverageAndVariance
{
  const double SIGMA_MAX = 100.0;
  const double SIGMA_MIN = -100.0;
  double m_average = 0;
  double m_variance = 0;

  /// Returns sigma deviation from standard for the given sample,
  /// and adds the sample to the statistical average and variance.
  ///@param[in] sample the sample
  ///@param[in] weight the weight given to the new sample
  inline double add(double sample, double weight)
  {
    // Calculate the sigma deviation from standard for this sample.
    double diff0 = sample - m_average;
    double stdDeviation = sqrt(m_variance);

    double sigma = diff0/stdDeviation;
    if (stdDeviation != 0)
      {
	sigma = diff0/stdDeviation;
      }
    else
      {
	if (diff0 == 0)
	  {
	    sigma = 0;
	  }
	else
	  {
	    if (diff0 > 0)
	      {
		sigma = SIGMA_MAX;
	      }
	    else
	      {
		sigma = SIGMA_MIN;
	      }
	  }
      }
    
    // Update average and variance
    m_average = (sample * weight) + m_average * (1-weight);
    double diff1 = sample - m_average;

    double sampleVariance = diff0 * diff1;
    double m_variance = sampleVariance * weight + m_variance * (1-weight);

    return sigma;
  }

};

struct ImageBaseline
{
  const double WEIGHT = 0.05;
  const int PIXEL_COUNT = 4800;
  int sampleCount = 0;

  struct Pixel
  {
    AverageAndVariance m_averageAndVariance;
  };

  std::array<Pixel, PIXEL_COUNT> m_pixels = {0};

  void AddImage(std::vector<uint16_t> data, std::array<double,PIXEL_COUNT>& sigma)
  {
    // First sample is weighted at 100%
    double weight = m_sampleCount? WEIGHT: 100.0;
    for(uint i = 0; i < PIXEL_COUNT; i++)
      {
	Pixel& pixel = m_pixels[i];
	sigma[i] = pixel.m_averageAndVariance.add(data[i],weight);
      }

    sampleCount += 1;
  }
};

void ProcessImage(const flircam::ImageId& imageId)
{
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
  // ----
  // The ImageProc component does the the detection algorithm.
  // 
  // Baseline data contains an exponential windowed moving average value of each pixel of the normalized image.
  // Baseline data contains an exponential windowed moving average value of the power terms of the variance equation applied to each pixel of the normalized image.
  // The square root of this is the standard deviation.
  //
  //  For each pixel, update the average and the variance data.  This is just two floating point (double precision) values per pixel.
  //
  // Image data is always added to the baseline, even if there are detections.  This means that if a detected object doesn't move around, then it will eventually get ignored.
  // 
  // A detection is a pixel value that deviates too far from the standard.
  // Create a detection bitmap. Further operations may be performed on the detection bitmap.
  //
  // 
    ImageBaseline baseline;
    
    std::vector<uint16_t> image;
    ImageUtil::ReadImage(imageId, image);

    bool first = false;
    if (false == ImageUtil::ReadBaseline(imageId, baseline))
    {
      // Create baseline for the first time.
      baseline = ImageBaseline();
      first = true;
    }

    std::array<double,ImageBaseline::PIXEL_COUNT> sigma;
    baseline.AddImage(image,sigma);

    int detectionCount = 0;
    if (!first)
      {
	// Create detection bitmap from sigma values that are more then some threshold.

	// Write sigma image to file.
	// Write detection bitmap to file.
	std::vector<uint16_t> detectionMap;
	std::vector<uint16_t> filteredDetectionMap;
	detectionCount = DetectChanges(sigma, detectionMap, filteredDetectionMap);
	
	// write the detection bitmap to file for inspection.  Detetions may be ignored if baseline sample count is below some threshold.
	ImageUtil::WritePBM(imageId, detectionMap, "detect");
	ImageUtil::WritePBM(imageId, filteredDetectionMap, "filteredDetect");
	ImageUtil::WriteDetectionMap(imageId, filteredDetectionMap);
      }

    //
    // Publish detection result always
    PublishDetection(imageId, detectionCount);
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
    ros::Subscriber subImageDone = n.subscribe<flircam::ImageId>("image_done", 10, ProcessImage);
    ros::Subscriber subConfigure = n.subscribe<flircam::Configure>("configure", 10, Configure);

    std::cout << "start ros spin" << std::endl;
    ros::spin();

    return EXIT_SUCCESS;
}


