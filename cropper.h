#pragma once

#include <opencv2/opencv.hpp>

class Cropper {
public:
	void crop(cv::Mat src, cv::Mat& dest, cv::Size outSize, float theta, float phi, float fov);
	void crop(cv::Mat src, cv::Mat& dest, cv::Size outSize, cv::Point2f p, float fov);
	void crop(cv::Mat src, cv::Mat& dest, cv::Size outSize, cv::Point3f vecFront, float fov);
};