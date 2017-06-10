#include "MarkerGenerator.h"

MarkerGenerator *markerGenerator;

void main()
{
	// Initialize parameters
	markerGenerator = new MarkerGenerator();
	bool isCreateMarker = false;

	// 1. Create Marker
	if (isCreateMarker) bool success = markerGenerator->CreateMarker();
	
	// 2. Camera Calibration

}