#include "Calibration.h"

CameraCalibration::CameraCalibration()
{
	cameraParameter = "data/out_CameraParameters.yml";
	detectionParameter = "data/in_DetectorParameters";
	loadMarkerConfig = "data/in_MarkerConfiguration.yml";
}

CameraCalibration::~CameraCalibration()
{

}

cv::Mat CameraCalibration::PXCImage2CVMat(PXCImage *pxcImage, PXCImage::PixelFormat format)
{
	PXCImage::ImageData data;
	pxcImage->AcquireAccess(PXCImage::ACCESS_READ, format, &data);

	int width = pxcImage->QueryInfo().width;
	int height = pxcImage->QueryInfo().height;

	if (!format)
		format = pxcImage->QueryInfo().format;

	int type = 0;
	if (format == PXCImage::PIXEL_FORMAT_Y8)
		type = CV_8UC1;
	else if (format == PXCImage::PIXEL_FORMAT_RGB24)
		type = CV_8UC3;
	else if (format == PXCImage::PIXEL_FORMAT_DEPTH_F32)
		type = CV_32FC1;

	cv::Mat ocvImage = cv::Mat(cv::Size(width, height), type, data.planes[0]);

	pxcImage->ReleaseAccess(&data);

	return ocvImage;
}

bool CameraCalibration::readDetectorParameters(std::string filename, 
											   cv::Ptr<cv::aruco::DetectorParameters> &params)
{
	cv::FileStorage fs(filename, cv::FileStorage::READ);
	if (!fs.isOpened())
		return false;
	fs["adaptiveThreshWinSizeMin"] >> params->adaptiveThreshWinSizeMin;
	fs["adaptiveThreshWinSizeMax"] >> params->adaptiveThreshWinSizeMax;
	fs["adaptiveThreshWinSizeStep"] >> params->adaptiveThreshWinSizeStep;
	fs["adaptiveThreshConstant"] >> params->adaptiveThreshConstant;
	fs["minMarkerPerimeterRate"] >> params->minMarkerPerimeterRate;
	fs["maxMarkerPerimeterRate"] >> params->maxMarkerPerimeterRate;
	fs["polygonalApproxAccuracyRate"] >> params->polygonalApproxAccuracyRate;
	fs["minCornerDistanceRate"] >> params->minCornerDistanceRate;
	fs["minDistanceToBorder"] >> params->minDistanceToBorder;
	fs["minMarkerDistanceRate"] >> params->minMarkerDistanceRate;
	fs["doCornerRefinement"] >> params->doCornerRefinement;
	fs["cornerRefinementWinSize"] >> params->cornerRefinementWinSize;
	fs["cornerRefinementMaxIterations"] >> params->cornerRefinementMaxIterations;
	fs["cornerRefinementMinAccuracy"] >> params->cornerRefinementMinAccuracy;
	fs["markerBorderBits"] >> params->markerBorderBits;
	fs["perspectiveRemovePixelPerCell"] >> params->perspectiveRemovePixelPerCell;
	fs["perspectiveRemoveIgnoredMarginPerCell"] >> params->perspectiveRemoveIgnoredMarginPerCell;
	fs["maxErroneousBitsInBorderRate"] >> params->maxErroneousBitsInBorderRate;
	fs["minOtsuStdDev"] >> params->minOtsuStdDev;
	fs["errorCorrectionRate"] >> params->errorCorrectionRate;

	return true;
}

bool CameraCalibration::saveCameraParams(const std::string &filename, 
										 cv::Size imageSize, 
										 float aspectRatio, 
										 int flags, 
										 const cv::Mat &cameraMatrix, 
										 const cv::Mat &distCoeffs, 
										 double totalAvgErr)
{
	cv::FileStorage fs(filename, cv::FileStorage::WRITE);
	if (!fs.isOpened())
		return false;

	time_t tt;
	time(&tt);
	struct tm *t2 = localtime(&tt);
	char buf[1024];
	strftime(buf, sizeof(buf) - 1, "%c", t2);

	fs << "calibration_time" << buf;

	fs << "image_width" << imageSize.width;
	fs << "image_height" << imageSize.height;

	if (flags & cv::CALIB_FIX_ASPECT_RATIO) fs << "aspectRatio" << aspectRatio;

	if (flags != 0) {
		sprintf(buf, "flags: %s%s%s%s",
			flags & cv::CALIB_USE_INTRINSIC_GUESS ? "+use_intrinsic_guess" : "",
			flags & cv::CALIB_FIX_ASPECT_RATIO ? "+fix_aspectRatio" : "",
			flags & cv::CALIB_FIX_PRINCIPAL_POINT ? "+fix_principal_point" : "",
			flags & cv::CALIB_ZERO_TANGENT_DIST ? "+zero_tangent_dist" : "");
	}

	fs << "flags" << flags;

	fs << "camera_matrix" << cameraMatrix;
	fs << "distortion_coefficients" << distCoeffs;

	fs << "avg_reprojection_error" << totalAvgErr;

	return true;
}

bool CameraCalibration::Calibrate()
{
	// Get the detector parameters
	cv::Ptr<cv::aruco::DetectorParameters> detectorParams = cv::aruco::DetectorParameters::create();
	bool readable = readDetectorParameters("detector_params.yml", detectorParams);
	if (!readable)
	{
		std::cerr << "Invalid detector parameters file" << std::endl;
		return false;
	}

	// Load marker configuration
	cv::FileStorage fs(loadMarkerConfig, cv::FileStorage::READ);
	if (fs.isOpened())
	{
		fs["markersX"] >> markers_x;
		fs["markersX"] >> markers_y;
		fs["markersX"] >> marker_length;
		fs["markersX"] >> marker_separation;
		fs["markersX"] >> dictionary_id;
		fs["markersX"] >> margins;
		fs["markersX"] >> borderBits;
		fs["markersX"] >> showImage;
	}
	else
	{
		// Manual configuration
		markers_x = 9;
		markers_y = 8;
		marker_length = 0.04;
		marker_separation = 0.01;


		dictionary_id = cv::aruco::DICT_6X6_250;
		margins = 10;
		borderBits = 1;
		showImage = true;
	}
}