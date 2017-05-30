#include <iostream>
#include <opencv2/opencv.hpp>
#include <cmath>
#include "segmentation.h"
#include "cropper.h"

//#include "depth_prediction.h"

using namespace std;
using namespace cv;

int main() {
	
	Mat src = imread("img\\360_5.jpg");
	Mat img_crop;
	Mat img_new;
	
	const float fovModel[] = {
		104.0 / 180.0 * CV_PI,
		84.0 / 180.0 * CV_PI,
		63.0 / 180.0 * CV_PI,
		47.0 / 180.0 * CV_PI,
		28.0 / 180.0 * CV_PI
	};

	const Size img_size = { src.size().height/3*2, src.size().height };
	const Size new_size = { img_size.width/2, img_size.height/2 };

	Point2f pnt = { 0.54f , 0.0f };
	// Check if everything was fine
	if (!src.data)
		return -1;
	
	// Show source image
	imshow("Source Image", src);
	
	Cropper crop;
	
	crop.crop(src, img_new, img_size, pnt, fovModel[0]);
	imshow("Crop Image", img_new);

	
	//resize(img_crop, img_new, new_size);
	//imshow("New Image", img_new);
	
	Segmentation seg;
	Mat img_seg = Mat::zeros(img_new.size().height, img_new.size().width, CV_8UC4);
	
	seg.init(img_new, sqrt(img_new.size().width * img_new.size().height), 0.0015f);
	seg.build();

	seg.output(img_seg);
	imshow("Segmentation Image", img_seg);
	
	Mat res;
	//resize(img_seg, res, { img_size.width, img_size.height});
	//imshow("Result Image", res);

	vector<int> compression_params;
	compression_params.push_back(CV_IMWRITE_PNG_COMPRESSION);
	compression_params.push_back(9);

	//imwrite("img\\out.png", img_seg, compression_params);
	
	waitKey(0);
	return 0;
}
