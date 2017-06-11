#pragma once
#include <windows.h>
#include <pxcsensemanager.h>

#include <conio.h>
#include <iostream>
#include <vector>

#include <opencv2/core.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/aruco.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/calib3d.hpp>
#include <ctime>

using namespace Intel::RealSense;

class CameraCalibration
{
	public:	
		CameraCalibration();
		~CameraCalibration();
		bool Calibrate();

	private:
		cv::Mat PXCImage2CVMat(PXCImage*, PXCImage::PixelFormat);
		static bool readDetectorParameters(std::string, cv::Ptr<cv::aruco::DetectorParameters>&);
		static bool saveCameraParams(const std::string&, cv::Size, float, int, const cv::Mat&, const cv::Mat&, double);

		std::string cameraParameter;
		std::string detectionParameter;
		std::string loadMarkerConfig;

		int markers_x;				// Number of markers in x direction
		int markers_y;				// Number of markers in y direction
		float marker_length;		// Marker side length(in pixels)
		float marker_separation;	// Separation between two consecutive markers in the grid (in pixels)

									/*
									dictionary: DICT_4X4_50=0, DICT_4X4_100=1, DICT_4X4_250=2,
									DICT_4X4_1000=3, DICT_5X5_50=4, DICT_5X5_100=5, DICT_5X5_250=6, DICT_5X5_1000=7,
									DICT_6X6_50=8, DICT_6X6_100=9, DICT_6X6_250=10, DICT_6X6_1000=11, DICT_7X7_50=12,
									DICT_7X7_100=13, DICT_7X7_250=14, DICT_7X7_1000=15, DICT_ARUCO_ORIGINAL = 16
									*/
		int dictionary_id;
		int calibrationFlags;
		float aspectRatio;
		bool refindStrategy;
};