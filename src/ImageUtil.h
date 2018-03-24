#ifndef ImageUtil_h_defined
#define ImageUtil_h_defined

#include <stdint.h>
#include <vector>
#include <functional>
#include <algorithm>
#include <flircam/ImageId.h>
#include <string>
#include <numeric>
#include <istream>
#include <cereal/types/vector.hpp>

namespace ImageUtil
{
	const double SIGMA_MAX = 100.0;
	const double SIGMA_MIN = -100.0;

	struct AverageAndVariance
	{
		double m_average;
		double m_variance;

		/// Returns sigma deviation from standard for the given sample,
		/// and adds the sample to the statistical average and variance.
		///@param[in] sample the sample
		///@param[in] weight the weight given to the new sample
		inline double add(double sample, double weight)
		{
			// Calculate the sigma deviation from standard for this sample.
			double diff0 = sample - m_average;
			double stdDeviation = sqrt(m_variance);

			double sigma;
			if (stdDeviation != 0)
			{
				sigma = diff0 / stdDeviation;
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
			m_average = (sample * weight) + m_average * (1 - weight);
			double diff1 = sample - m_average;

			double sampleVariance = diff0 * diff1;
		    m_variance = sampleVariance * weight + m_variance * (1 - weight);

			return sigma;
		}

	};

	struct ImageStatistics
	{
		static constexpr double WEIGHT = 0.05;

		int m_sampleCount = 0;
		
		struct Pixel
		{
			AverageAndVariance m_averageAndVariance;

			template<class Archive>
			void serialize(Archive& ar)
			{
				ar(m_averageAndVariance.m_average, m_averageAndVariance.m_variance);
			}
		};

		std::vector<Pixel> m_pixels;

		template<class Archive>
		void serialize(Archive& ar)
		{
			ar(m_sampleCount, m_pixels);
		}

		void addImage(const std::vector<int>& data, std::vector<double>& sigma)
		{
			if (m_pixels.size() == 0)
			{
				m_pixels.resize(data.size());
				std::fill(m_pixels.begin(), m_pixels.end(), Pixel());
			}

			sigma.resize(data.size());

			// First sample is weighted at 100%
			double weight = m_sampleCount ? WEIGHT : 1.0;
			for (uint i = 0; i < m_pixels.size(); i++)
			{
				Pixel& pixel = m_pixels[i];
				sigma[i] = pixel.m_averageAndVariance.add(data[i], weight);
			}

			m_sampleCount += 1;
		}

		bool save(const char* filename) const;
		bool load(const char* filename);

		void serializeToStream(std::ostream& os) const;
		void deserializeFromStream(std::istream& is);

		void getAverage(std::vector<double>& average) const;
		void getStandardDeviation(std::vector<double>& stddev) const;
	};


	class NormalizedFrame
	{
	public:
		NormalizedFrame();
		NormalizedFrame(const std::vector<uint16_t>& frame);
		bool save(const char* filename) const;
		bool load(const char* filename);

		const std::vector<int>& getNormalizedData() const;
		double getAverage() const;
		//double getMin() const;
		//double getMax() const;

		// Invert the pixel values.
		void invert();

		// Add x to all pixel values.  This changes only the average.
		// Use this to set average to zero. add(-GetAverage())
		void add(double x);

		// Add the given frame to this frame
		void add(const NormalizedFrame& data);

		// Get the data stretched to uint8_t
		void getStretchedData(std::vector<uint16_t>& frame) const;

	public:
		template<class Archive>
		void serialize(Archive& ar)
		{
			ar(m_frame, m_average);
		}

	private:
		// The pixel values, offset by subtracting the average value.
		std::vector<int> m_frame;

		// the average pixel value that was subtracted to create the normalized frame.
		double m_average;

		// the min pixel value
		//double m_min;

		// the max pixel value
		//double m_max;

		void serializeToStream(std::ostream& os) const;
		void deserializeFromStream(std::istream& is);
	};

	bool WriteImage(const flircam::ImageId&, const NormalizedFrame& frame);
	bool WriteImage(const flircam::ImageId&, const std::vector<uint16_t>& frame);
	bool ReadImage(const flircam::ImageId&, std::vector<uint16_t>& frame);
	bool ReadImage(const flircam::ImageId&, NormalizedFrame& frame);
	int write_png_file(const char* file_name, std::vector<uint16_t>& frame);
	bool ReadBaseline(const flircam::ImageId&, NormalizedFrame& baseline);
	bool ReadBaseline(const flircam::ImageId&, std::vector<uint16_t>& baseline);
	bool ReadBaseline(const flircam::ImageId&, ImageStatistics& baseline);
	bool WriteBaseline(const flircam::ImageId&, const std::vector<uint16_t>& d);
	bool WriteBaseline(const flircam::ImageId&, const NormalizedFrame& d);
	bool WriteBaseline(const flircam::ImageId&, const ImageStatistics& d);

	template <typename T>
	bool WriteDetectionMap(const flircam::ImageId&, const std::vector<T>&d);

	template <typename T>
	bool WritePBM(const flircam::ImageId&, const std::vector<T>&d, const char* ext);

	template <typename T>
	bool WritePGM(const flircam::ImageId&, const std::vector<T>&d, const char* ext);

	template <typename T>
	bool WritePGM8(const flircam::ImageId&, const std::vector<T>&d, const char* ext);

	std::string ImageIdToString(const flircam::ImageId&);

	struct truncated_minus : std::binary_function<uint16_t, uint16_t, uint16_t>
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

	struct compare_and_threshold : std::binary_function<uint16_t, uint16_t, uint16_t>
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

	struct subtract_and_threshold : std::binary_function<uint16_t, uint16_t, uint16_t>
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

	struct IsGreaterThan : std::unary_function<double, uint8_t>
	{
		double m_limit;

		IsGreaterThan(double limit)
		{
			m_limit = limit;
		}

		uint8_t operator()(double d) const
		{
			if (d > m_limit) return 1;
			return 0;
		}
	};

	struct LimitScaleOffset : std::unary_function<double,double>
	{
		double m_limit;
		double m_scale;
		double m_offset;

		double operator()(double d) const
		{
			if (d < -m_limit)
			{
				d = -m_limit;
			}
			if (d > m_limit)
			{
				d = m_limit;
			}

			return d * m_scale + m_offset;
		}

		LimitScaleOffset(double limit, double scale, double offset)
		{
			m_limit = limit;
			m_scale = scale;
			m_offset = offset;
		}

	};

	template<class T> 
	struct stretch
	{
		T min;
		double scalar;

		T operator()(T& pixel) const
		{
			pixel = static_cast<double>(pixel - min) * scalar;
		}
	};

};

#endif
