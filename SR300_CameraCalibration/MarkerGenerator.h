#pragma once
#include <opencv2/highgui.hpp>
#include <opencv2/aruco.hpp>

class MarkerGenerator
{
	public:
		MarkerGenerator();
		~MarkerGenerator();

		bool CreateMarker();

	private:
		std::string loadMarkerConfig;
		std::string result;

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
		int margins;				// Margins size (in pixels). Default is marker separation 

		int borderBits;				// Number of bits in marker borders
		bool showImage;
};