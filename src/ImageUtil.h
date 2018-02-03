#ifndef ImageUtil_h_defined
#define ImageUtil_h_defined

#include <stdint.h>
#include <vector>
#include <functional>

namespace ImageUtil
{
bool WriteImage(unsigned int imageId, const std::vector<uint16_t>& frame );
bool ReadImage(unsigned int imageId,std::vector<uint16_t>& frame );
int write_png_file(const char* file_name, std::vector<uint16_t>& frame );
bool ReadBaseline(unsigned int imageId, std::vector<uint16_t>&baseline);
bool WriteBaseline(unsigned int imageId, const std::vector<uint16_t>&d);
bool WriteDetectionMap(unsigned int imageId, const std::vector<uint16_t>&d);
bool WritePBM(unsigned int imageId, const std::vector<uint16_t>&d, const char* ext);
bool WritePGM(unsigned int imageId, const std::vector<uint16_t>&d, const char* ext);

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

struct compare_and_threshold: std::binary_function<uint16_t, uint16_t, uint16_t>
{
  compare_and_threshold(uint16_t threshold)
    : m_threshold(threshold)
    {
    }
  uint16_t m_threshold;
  uint16_t operator()(uint16_t a, uint16_t b) const
  {
    if ((b > a) && (b - a > m_threshold))
      {
	return 1;
      }
    if ((a > b) && (a - b > m_threshold))
      {
	return 1;
      }

    return 0;
  }
};

 struct subtract_and_threshold: std::binary_function<uint16_t, uint16_t, uint16_t>
{
  subtract_and_threshold(uint16_t threshold)
    : m_threshold(threshold)
    {
    }
  uint16_t m_threshold;
  uint16_t operator()(uint16_t a, uint16_t b) const
  {
    // image (a) > baseline (b)
    if ((a > b) && (a - b > m_threshold))
      {
	return 1;
      }
    return 0;
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

};

#endif
