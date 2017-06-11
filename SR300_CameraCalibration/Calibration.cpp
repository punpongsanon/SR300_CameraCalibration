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
	bool readable = readDetectorParameters("data/in_DetectorParameters.yml", detectorParams);
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
		fs["markersY"] >> markers_y;
		fs["markerLength"] >> marker_length;
		fs["markerSeparation"] >> marker_separation;
		fs["dictionaryId"] >> dictionary_id;
	}
	else
	{
		// Manual configuration
		markers_x = 9;
		markers_y = 8;
		marker_length = 0.04;
		marker_separation = 0.01;
		dictionary_id = cv::aruco::DICT_6X6_250;
	}

	calibrationFlags = 0;
	aspectRatio = 1;
	refindStrategy = false;

	// Register markers
	cv::Ptr<cv::aruco::Dictionary> dictionary = cv::aruco::getPredefinedDictionary(cv::aruco::PREDEFINED_DICTIONARY_NAME(dictionary_id));

	// Create board object
	cv::Ptr<cv::aruco::GridBoard> gridboard = cv::aruco::GridBoard::create(markers_x, markers_y, marker_length, marker_separation, dictionary);
	cv::Ptr<cv::aruco::Board> board = gridboard.staticCast<cv::aruco::Board>();

	// Collected frames for calibration
	std::vector<std::vector<std::vector<cv::Point2f>>> allCorners;
	std::vector<std::vector<int>> allIds;
	cv::Size imageSize;

	PXCSession *session = PXCSession::CreateInstance();
	PXCSenseManager *sm = PXCSenseManager::CreateInstance();

	// Capture parameter (SR300)
	cv::Size frameSize = cv::Size(640, 480);
	float frameRate = 60;
	cv::Mat frameColor = cv::Mat::zeros(frameSize, CV_8UC3);

	sm->EnableStream(PXCCapture::STREAM_TYPE_COLOR, frameSize.width, frameSize.height, frameRate);
	sm->Init();

	while(1)
	{
		if (sm->AcquireFrame(true) < PXC_STATUS_NO_ERROR) break;

		PXCCapture::Sample *sample;
		sample = sm->QuerySample();

		if (sample->color)	frameColor = PXCImage2CVMat(sample->color, PXCImage::PIXEL_FORMAT_RGB24);

		std::vector<int> ids;
		std::vector<std::vector<cv::Point2f>> corners, rejected;

		// Detect markers
		cv::aruco::detectMarkers(frameColor, dictionary, corners, ids, detectorParams, rejected);

		// Refind strategy to detect more markers
		if (refindStrategy) cv::aruco::refineDetectedMarkers(frameColor, board, corners, ids, rejected);

		// draw results
		cv::Mat frameResult;
		frameColor.copyTo(frameResult);
		if (ids.size() > 0) cv::aruco::drawDetectedMarkers(frameResult, corners, ids);
		cv::putText(frameResult, "Press 'c' to add current frame. 'ESC' to finish and calibrate",
		cv::Point(10, 20), cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(255, 0, 0), 2);

		cv::imshow("Calibration - Input", frameResult);

		char key = (char)cv::waitKey(10);
		if (key == 27) break;
		if (key == 'c' && ids.size() > 0)
		{
			std::cout << "Frame captured" << std::endl;
			allCorners.push_back(corners);
			allIds.push_back(ids);
			imageSize = frameColor.size();
		}

		sm->ReleaseFrame();
	}

	if (allIds.size() < 1)
	{
		std::cerr << "Not enough captures for calibration" << std::endl;
		return 0;
	}

	// Calibration
	cv::Mat cameraMatrix, distCoeffs;
	std::vector<cv::Mat> rvecs, tvecs;
	double repError;

	if (calibrationFlags & cv::CALIB_FIX_ASPECT_RATIO) 
	{
		cameraMatrix = cv::Mat::eye(3, 3, CV_64F);
		cameraMatrix.at<double>(0, 0) = aspectRatio;
	}

	// Prepare data for calibration
	std::vector<std::vector<cv::Point2f>> allCornersConcatenated;
	std::vector<int> allIdsConcatenated;
	std::vector<int> markerCounterPerFrame;
	markerCounterPerFrame.reserve(allCorners.size());
	for (unsigned int i = 0; i < allCorners.size(); i++) 
	{
		markerCounterPerFrame.push_back((int)allCorners[i].size());
		for (unsigned int j = 0; j < allCorners[i].size(); j++) 
		{
			allCornersConcatenated.push_back(allCorners[i][j]);
			allIdsConcatenated.push_back(allIds[i][j]);
		}
	}

	// Calibrate camera
	repError = cv::aruco::calibrateCameraAruco(allCornersConcatenated, 
											   allIdsConcatenated,
											   markerCounterPerFrame, 
											   board, 
											   imageSize, 
											   cameraMatrix,
											   distCoeffs, 
											   rvecs, 
											   tvecs, 
											   calibrationFlags);

	bool isSave = saveCameraParams(cameraParameter, 
							       imageSize, 
								   aspectRatio, 
								   calibrationFlags, 
								   cameraMatrix,
								   distCoeffs, 
								   repError);

	if (!isSave) 
	{
		std::cerr << "Cannot save output file" << std::endl;
		return 0;
	}

	std::cout << "Rep Error: " << repError << std::endl;
	std::cout << "Calibration saved to " << cameraParameter << std::endl;
}