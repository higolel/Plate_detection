#ifndef HYPERPR_FASTDESKEW_H
#define HYPERPR_FASTDESKEW_H

#include <math.h>
#include <opencv2/opencv.hpp>
namespace pr
{
	cv::Mat fastdeskew(cv::Mat skewImage, int blockSize);
} // namespace pr

#endif // HYPERPR_FASTDESKEW_H
