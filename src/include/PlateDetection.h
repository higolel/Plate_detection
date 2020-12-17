#ifndef HYPERPR_PLATEDETECTION_H
#define HYPERPR_PLATEDETECTION_H

#include <PlateInfo.h>
#include <opencv2/opencv.hpp>
#include <vector>

namespace pr
{
	class PlateDetection
	{
		public:
			PlateDetection(std::string filename_cascade);
			PlateDetection();
			void LoadModel(std::string filename_cascade);
			void plateDetectionRough(cv::Mat InputImage,
					std::vector<pr::PlateInfo> &plateInfos,
					int min_w = 36, int max_w = 800);
		private:
			cv::CascadeClassifier cascade;
	};
} // namespace pr

#endif // HYPERPR_PLATEDETECTION_H
