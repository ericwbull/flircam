#ifndef ImageUtil_h_defined
#define ImageUtil_h_defined

#include <stdint.h>
#include <vector>
#include <functional>
#include <flircam/ImageId.h>
#include <string>
#include <numeric>
#include <istream>
#include <cereal/types/vector.hpp>

namespace ImageUtil
{
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

			double sigma = diff0 / stdDeviation;
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
			double m_variance = sampleVariance * weight + m_variance * (1 - weight);

			return sigma;
		}

	};

	struct ImageBaseline
	{
		static constexpr double WEIGHT = 0.05;
		static constexpr int PIXEL_COUNT = 4800;

		int m_sampleCount = 0;

		struct Pixel
		{
			AverageAndVariance m_averageAndVariance;
		};

		std::vector<Pixel> m_pixels;

		void AddImage(const std::vector<uint16_t>& data, std::vector<double>& sigma)
		{
			sigma.resize(data.size());

			// First sample is weighted at 100%
			double weight = m_sampleCount ? WEIGHT : 100.0;
			for (uint i = 0; i < PIXEL_COUNT; i++)
			{
				Pixel& pixel = m_pixels[i];
				sigma[i] = pixel.m_averageAndVariance.add(data[i], weight);
			}

			m_sampleCount += 1;
		}
	};


	class NormalizedFrame
	{
	public:
		NormalizedFrame(const char* filename);
		NormalizedFrame(const std::vector<uint16_t>& frame);
		void SaveToFile(const char* filename) const;

	public:
		template<class Archive>
		void serialize(Archive& ar)
		{
			ar(m_frame, m_average, m_min, m_max);
		}

	private:
		std::vector<int> m_frame;
		int m_average;
		int m_min;
		int m_max;

		const std::vector<int>& GetNormalizedData() const;
		int GetAverage() const;



		void serializeToStream(std::ostream& os) const;
		void deserializeFromStream(std::istream& is);
	};

	bool WriteImage(const flircam::ImageId&, const std::vector<uint16_t>& frame);
	bool ReadImage(const flircam::ImageId&, std::vector<uint16_t>& frame);
	int write_png_file(const char* file_name, std::vector<uint16_t>& frame);
	bool ReadBaseline(const flircam::ImageId&, std::vector<uint16_t>&baseline);
	bool ReadBaseline(const flircam::ImageId&, ImageBaseline& baseline);
	bool WriteBaseline(const flircam::ImageId&, const std::vector<uint16_t>&d);
	bool WriteDetectionMap(const flircam::ImageId&, const std::vector<uint16_t>&d);
	bool WritePBM(const flircam::ImageId&, const std::vector<uint16_t>&d, const char* ext);
	bool WritePGM(const flircam::ImageId&, const std::vector<uint16_t>&d, const char* ext);
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
