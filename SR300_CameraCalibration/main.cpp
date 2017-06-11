#include "MarkerGenerator.h"
#include "Calibration.h"

MarkerGenerator *markerGenerator;
CameraCalibration *cameraCalibration;

void main()
{
	// Initialize parameters
	markerGenerator = new MarkerGenerator();
	bool isCreateMarker = false;

	cameraCalibration = new CameraCalibration();
	bool isCalibrate = true;

	// Console
	int choice;
	std::cout << "---------------------------------------" << std::endl;
	std::cout << "CALIBRATION FOR SR300 using OPENCV 3.2" << std::endl;
	std::cout << "---------------------------------------" << std::endl;
	std::cout << "1. Create a marker board (aruco board)" << std::endl;
	std::cout << "2. Calibrate using an exist marker board" << std::endl;
	std::cout << "---------------------------------------" << std::endl;
	std::cin >> choice;

	// 1. Create Marker
	if (choice == 1) markerGenerator->CreateMarker();
	// 2. Camera Calibration
	if (choice == 2) cameraCalibration->Calibrate();

	std::cout << "Press any key to exit." << std::endl;

	cv::waitKey(0);
	return;
}