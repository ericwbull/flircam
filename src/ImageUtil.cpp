#include "ImageUtil.h"
#include <png.h>
#include <iostream>
#include <unistd.h>
#include <chrono>
#include <boost/filesystem.hpp>
#include <sstream>
#include <functional>
#include <fstream>
#include <array>
#include <iomanip>
#include <algorithm>
#include <cereal/archives/binary.hpp>

namespace ImageUtil
{

	const int IMAGE_SIZE_PIXELS = 60 * 80;
	const int IMAGE_SIZE_BYTES = IMAGE_SIZE_PIXELS * sizeof(uint16_t);

	bool ReadImageDataFromFile(const char* file_name, std::vector<uint16_t>& frame);
	bool WriteImageDataToPGMFile(const char* file_name, const std::vector<uint16_t>& frame);
	bool WriteImageDataToPBMFile(const char* file_name, const std::vector<uint16_t>& frame);


	template <typename T>
	bool WriteImageDataToFile(const char* file_name, const std::vector<T>& frame)
	{
		std::ofstream ofs(file_name, std::ios::binary);

		if (ofs.fail())
		{
			std::cerr << "File open failed '" << file_name << "'" << std::endl;
			return false;
		}

		int size = frame.size() * sizeof(T);
		//  ofs.write(&size, sizeof(int));
		// For the current application the size is always 80 x 60 pixels
		ofs.write(reinterpret_cast<const char*>(&frame[0]), size);

		return true;
	}

	std::string GetImageFileName(const flircam::ImageId& imageId)
	{
		unsigned int folderNumber = imageId.collectionNumber;
		unsigned int frameNumber = imageId.frameNumber;
		unsigned int serialNumber = imageId.serialNumber;

		std::ostringstream ossDir;
		std::ostringstream ossFilename;

		ossDir << "/tmp/flircam/" << folderNumber;
		ossFilename << "/tmp/flircam/" << folderNumber << '/' << frameNumber << '.' << serialNumber;

		std::string dirname = ossDir.str();
		std::string fname = ossFilename.str();

		boost::filesystem::create_directories(dirname.c_str());

		//  std::cout << "dirname=" << dirname << std::endl;
		//  std::cout << "fname=" << fname << std::endl;
		return fname;
	}

	std::string ImageIdToString(const flircam::ImageId& imageId)
	{
		return GetImageFileName(imageId);
	}


	std::string GetBaselineFileName(const flircam::ImageId& imageId)
	{
		// baseline is outside of the collection folder
		unsigned int frameNumber = imageId.frameNumber;

		std::ostringstream ossDir;
		std::ostringstream ossFilename;

		ossDir << "/tmp/flircam";
		ossFilename << "/tmp/flircam/" << frameNumber << ".baseline";

		std::string dirname = ossDir.str();
		std::string fname = ossFilename.str();

		boost::filesystem::create_directories(dirname.c_str());

		//  std::cout << "dirname=" << dirname << std::endl;
		//  std::cout << "fname=" << fname << std::endl;
		return fname;

	}
	std::string GetDetectionMapFileName(const flircam::ImageId& imageId)
	{
		std::string imageFileName = GetImageFileName(imageId);
		return imageFileName + ".detection";
	}


	bool ReadBaseline(const flircam::ImageId& imageId, std::vector<uint16_t>&baseline)
	{
		//  return false if there is not baseline.
		// Baseline file is /tmp/flircam/0/1.baseline
		return ReadImageDataFromFile(GetBaselineFileName(imageId).c_str(), baseline);
	}

	bool WriteImage(const flircam::ImageId& imageId, const std::vector<uint16_t>& d)
	{
		return WriteImageDataToFile(GetImageFileName(imageId).c_str(), d);
	}

	bool WriteImage(const flircam::ImageId& imageId, const NormalizedFrame& nf)
	{
		return nf.save(GetImageFileName(imageId).c_str());
	}

	bool WritePGM(const flircam::ImageId& imageId, const std::vector<uint16_t>& d, const char* ext)
	{
		std::string imageFileName = GetImageFileName(imageId);
		imageFileName += ".";
		imageFileName += ext;
		imageFileName += ".pgm";

		return WriteImageDataToPGMFile(imageFileName.c_str(), d);
	}
	bool WritePBM(const flircam::ImageId& imageId, const std::vector<uint16_t>& d, const char* ext)
	{
		std::string imageFileName = GetImageFileName(imageId);
		imageFileName += ".";
		imageFileName += ext;
		imageFileName += ".pbm";

		return WriteImageDataToPBMFile(imageFileName.c_str(), d);
	}

	bool ReadImage(const flircam::ImageId& imageId, std::vector<uint16_t>& d)
	{
		return ReadImageDataFromFile(GetImageFileName(imageId).c_str(), d);
	}

	bool ReadImage(const flircam::ImageId& imageId, NormalizedFrame& nf)
	{
		return nf.load(GetImageFileName(imageId).c_str());
	}

	bool WriteBaseline(const flircam::ImageId& imageId, const std::vector<uint16_t>&d)
	{
		return WriteImageDataToFile(GetBaselineFileName(imageId).c_str(), d);
	}

	bool ReadBaseline(const flircam::ImageId& imageId, NormalizedFrame& nf)
	{
		return nf.load(GetBaselineFileName(imageId).c_str());
	}

	bool WriteBaseline(const flircam::ImageId& imageId, const NormalizedFrame& nf)
	{
		return nf.save(GetBaselineFileName(imageId).c_str());
	}

	bool WriteDetectionMap(const flircam::ImageId& imageId, const std::vector<uint16_t>&d)
	{
		std::vector<uint8_t> bitmap(d.size() / 8);
		int byteNum = 0;
		int bitMask = 1;
		for (auto p : d)
		{
			if (p > 0)
			{
				bitmap[byteNum] |= bitMask;
			}

			if (bitMask == 128)
			{
				bitMask = 1;
				byteNum += 1;
			}
			else
			{
				bitMask <<= 1;
			}
		}
		return WriteImageDataToFile(GetDetectionMapFileName(imageId).c_str(), bitmap);
	}



	bool ReadImageDataFromFile(const char* file_name, std::vector<uint16_t>& frame)
	{
		std::ifstream ifs(file_name, std::ios::binary);

		if (ifs.fail())
		{
			std::cerr << "File open failed '" << file_name << "'" << std::endl;
			return false;
		}

		frame.resize(IMAGE_SIZE_PIXELS);
		//  ofs.write(&size, sizeof(int));
		// For the current application the size is always 80 x 60 pixels
		ifs.read(reinterpret_cast<char*>(&frame[0]), IMAGE_SIZE_BYTES);

		return true;
	}

	bool ReadBaseline(const flircam::ImageId& imageId, ImageStatistics& baseline)
	{
		return baseline.load(GetBaselineFileName(imageId).c_str());
	}

	bool WriteBaseline(const flircam::ImageId& imageId, const ImageStatistics& baseline)
	{
		return baseline.save(GetBaselineFileName(imageId).c_str());
	}

	bool WriteImageDataToPGMFile(const char* file_name, const std::vector<uint16_t>& frame)
	{
		std::ofstream ofs(file_name);

		if (ofs.fail())
		{
			std::cerr << "File open failed '" << file_name << "'" << std::endl;
			return false;
		}

		auto iterPairMinMax = std::minmax_element(frame.begin(), frame.end());
		uint16_t minValue = *iterPairMinMax.first;
		uint16_t maxValue = *iterPairMinMax.second;

		ofs << "P2" << std::endl
			<< "# " << file_name << std::endl
			<< "80 60" << std::endl
			<< std::dec << maxValue << std::endl
			<< std::endl;

		for (auto pixel : frame)
		{
			ofs << std::dec << pixel << std::endl;
		}

		return true;
	}

	bool WriteImageDataToPBMFile(const char* file_name, const std::vector<uint16_t>& frame)
	{
		std::ofstream ofs(file_name);

		if (ofs.fail())
		{
			std::cerr << "File open failed '" << file_name << "'" << std::endl;
			return false;
		}

		ofs << "P1" << std::endl
			<< "# " << file_name << std::endl
			<< "80 60" << std::endl;

		for (auto pixel : frame)
		{
			ofs << std::dec << pixel << std::endl;
		}

		return true;
	}



	int write_png_file(const char* file_name, std::vector<uint16_t>& frame)
	{
		/* create file */
		FILE *fp = fopen(file_name, "wb");
		if (!fp)
		{
			std::cerr << "File open failed '" << file_name << "'" << std::endl;
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
			for (int i = 0; i < 60; i++) {
				s_row_ptrs[i] = &s_data[i][0];
			}

		}

		// swap bytes
		int row = 0;
		for (row = 0; row < 60; row++)
		{
			for (int col = 0; col < 80; ++col)
			{
				uint16_t word = frame[row * 80 + col];
				s_data[row][2 * col] = word >> 8;
				s_data[row][2 * col + 1] = word & 0xff;

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
	
	NormalizedFrame::NormalizedFrame()
	{
	}

	NormalizedFrame::NormalizedFrame(const std::vector<uint16_t>& frame)
	{
		int n = frame.size();
		m_frame.resize(n);
		int sum = std::accumulate(frame.begin(), frame.end(), 0);
		m_average = sum / n;
		std::transform(frame.begin(), frame.end(), m_frame.begin(), std::bind2nd(std::plus<int>(), m_average));
	}

	double 
		NormalizedFrame::getAverage() const
	{
		return m_average;
	}

	double 
		NormalizedFrame::getMin() const
	{
		return 0;
	}

	double 
		NormalizedFrame::getMax() const
	{
		return 0;
	}

	const std::vector<int>& 
		NormalizedFrame::getNormalizedData() const
	{
		return m_frame;
	}

	void 
		NormalizedFrame::getStretchedData(std::vector<uint16_t>& data) const
	{
		data.resize(m_frame.size());

		auto iterPairMinMax = std::minmax_element(m_frame.begin(), m_frame.end());
		int minValue = *iterPairMinMax.first;
		int maxValue = *iterPairMinMax.second;


		// Output stretched image for inspection
		std::vector<int> stretched = m_frame;
		ImageUtil::stretch<int> s;
		s.min = minValue;
		s.scalar = 255 / (maxValue - minValue);
		std::for_each(stretched.begin(), stretched.end(), s);

		std::copy(stretched.begin(), stretched.end(), data.begin());
	}

	void NormalizedFrame::deserializeFromStream(std::istream& is)
	{
		cereal::BinaryInputArchive iarchive(is);
		iarchive(*this);
	}

	void NormalizedFrame::serializeToStream(std::ostream& os) const
	{
		cereal::BinaryOutputArchive oarchive(os);
		oarchive(*this);
	}

	bool NormalizedFrame::load(const char* filename)
	{
		std::ifstream ifs(filename, std::ios::binary);
		deserializeFromStream(ifs);
		return !ifs.bad();
	}

	bool NormalizedFrame::save(const char* filename) const
	{
		std::ofstream ofs(filename, std::ios::binary);
		serializeToStream(ofs);
		return !ofs.bad();
	}

	// Invert the pixel values.
	void NormalizedFrame::invert()
	{
		for (int& d : m_frame)
		{
			d = -d;
		}
	}

	// Add x to all pixel values.  This changes only the average.
	// Use this to set average to zero. add(-GetAverage())
	void NormalizedFrame::add(double x)
	{
		m_average += x;
	}

	// Add the given frame to this frame
	void NormalizedFrame::add(const NormalizedFrame& data)
	{
		m_average += data.m_average;

		for (int i = 0; i < m_frame.size(); i++)
		{
			m_frame[i] += data.m_frame[i];
		}
	}

	void ImageStatistics::deserializeFromStream(std::istream& is)
	{
		cereal::BinaryInputArchive iarchive(is);
		iarchive(*this);
	}

	void ImageStatistics::serializeToStream(std::ostream& os) const
	{
		cereal::BinaryOutputArchive oarchive(os);
		oarchive(*this);
	}

	bool ImageStatistics::load(const char* filename)
	{
		std::ifstream ifs(filename, std::ios::binary);
		deserializeFromStream(ifs);
		return !ifs.bad();
	}

	bool ImageStatistics::save(const char* filename) const
	{
		std::ofstream ofs(filename, std::ios::binary);
		serializeToStream(ofs);
		return !ofs.bad();
	}
}
