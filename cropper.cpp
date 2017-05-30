//
//  segmentation.cpp
//  
//
//  Created by 황일환 on 2016. 10. 23..
//  Modified byy 박채용 on 2017.05
//
//

#include "cropper.h"
#include <cmath>

void Cropper::crop(cv::Mat src, cv::Mat& dest, cv::Size outSize, float theta, float phi, float fov) {
	cv::Point3f vecFront = cv::Point3f(cos(theta), sin(theta), tan(phi));
	vecFront /= cv::norm(vecFront);

	crop(src, dest, outSize, vecFront, fov);
}

void Cropper::crop(cv::Mat src, cv::Mat& dest, cv::Size outSize, cv::Point2f p, float fov) {
	const float pi = 3.14159265;

	const float theta = p.x * pi + pi;
	const float phi = -p.y * pi / 2.0;
	cv::Point3f vecFront = cv::Point3f(cos(theta), sin(theta), tan(phi));
	vecFront /= cv::norm(vecFront);

	crop(src, dest, outSize, vecFront, fov);
}

void Cropper::crop(cv::Mat src, cv::Mat& dest, cv::Size outSize, cv::Point3f vecFront, float fov) {
	dest = cv::Mat::zeros(outSize, src.type());

	const float pi = 3.14159265;

	cv::Point3f vecPole = cv::Point3f(0.0f, 0.0f, 1.0f);
	cv::Point3f vecRight = vecFront.cross(vecPole);
	vecRight /= cv::norm(vecRight);
	cv::Point3f vecUp = vecRight.cross(vecFront);

	const float aspect = (float)outSize.width / (float)outSize.height;

	vecRight *= tan(fov / 2.0);
	vecUp *= tan(fov / 2.0) / aspect;

	for (size_t j = 0; j < outSize.height; j++) {
		for (size_t i = 0; i < outSize.width; i++) {
			float x = (float)((int)i - (int)(outSize.width / 2)) / (int)outSize.width * 2.0;
			float y = (float)((int)j - (int)(outSize.height / 2)) / (int)outSize.height * 2.0;

			cv::Point3f vecCur = vecFront - vecRight * x - vecUp * y;
			vecCur /= cv::norm(vecCur);

			float theta;
			if (vecCur.x == 0.0 && vecCur.y > 0.0)
				theta = pi / 2.0;
			else if (vecCur.x == 0.0 && vecCur.y < 0.0)
				theta = -pi / 2.0;
			else
				theta = atan(vecCur.y / vecCur.x) + (vecCur.x > 0 ? 0.0 : pi);

			float phi;
			phi = acos(vecCur.z);

			int sx = (int)(src.size().width * (theta / 2.0 / pi));
			int sy = (int)(src.size().height * (phi / pi));

			if (sy < 0) {
				sy *= -1;
				sx += (int)(src.size().width / 2);
			}
			if (sy >= (int)(src.size().height)) {
				sy = 2 * (int)(src.size().height) - sy - 1;
				sx += (int)(src.size().width / 2);
			}

			sx = (sx + (int)(src.size().width)) % (int)(src.size().width);

			src.row(sy).col(sx).copyTo(dest.row(j).col(i));
		}
	}
}
