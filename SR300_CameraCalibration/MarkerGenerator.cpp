#include "MarkerGenerator.h"
#include <iostream>

MarkerGenerator::MarkerGenerator()
{
	loadMarkerConfig = "data/in_MarkerConfiguration.yml";
	result = "data/out_markerImage.jpeg";
}

MarkerGenerator::~MarkerGenerator()
{

}

bool MarkerGenerator::CreateMarker()
{
	// Read marker configuration
	cv::FileStorage fs(loadMarkerConfig, cv::FileStorage::READ);
	if (fs.isOpened())
	{
		fs["markersX"] >> markers_x;
		fs["markersY"] >> markers_y;
		fs["markerLength"] >> marker_length;
		fs["markerSeparation"] >> marker_separation;
		fs["dictionaryId"] >> dictionary_id;
		fs["margins"] >> margins;
		fs["borderBits"] >> borderBits;
		fs["showImage"] >> showImage;
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

	// Set image size
	cv::Size imageSize;
	imageSize.width = markers_x * (marker_length + marker_separation) - marker_separation + 2 * margins;
	imageSize.height = markers_y * (marker_length + marker_separation) - marker_separation + 2 * margins;
	std::cout << "[Create Marker] Image size: " << imageSize << std::endl;

	// Generate marker
	cv::Ptr<cv::aruco::Dictionary> dictionary = cv::aruco::getPredefinedDictionary(cv::aruco::PREDEFINED_DICTIONARY_NAME(dictionary_id));
	cv::Ptr<cv::aruco::GridBoard> board = cv::aruco::GridBoard::create(markers_x, markers_y, marker_length, marker_separation, dictionary);

	// Show created board
	cv::Mat boardImage;
	board->draw(imageSize, boardImage, margins, borderBits);

	// Write file
	std::vector<int> image_compression;
	image_compression.push_back(CV_IMWRITE_JPEG_QUALITY);
	image_compression.push_back(98);
	cv::imwrite(result, boardImage, image_compression);

	return true;
}