/*
 * CameraGeometricCalibration.cpp
 *
 *  Created on: 12 nov. 2013
 *      Author: Jetze en Barend
 */

#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/calib3d/calib3d.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <iostream>
using namespace cv;
using namespace std;

// Displays a supplied image
int main(int argc, const char** argv) {

	VideoCapture camera;
	Mat webcamImage;
	camera.open(0);
	if (!camera.isOpened()) {
		cerr << "ERROR: Could not access the camera or video!" << endl;
		exit(1);
	}

	namedWindow("test", CV_WINDOW_AUTOSIZE);
	while (true) {
		camera >> webcamImage;
		if (webcamImage.empty()) {
			cerr << "ERROR: Couldn't grab a camera frame." << endl;
			exit(1);
		}
        
        // Vector
        vector<Point2f> pointBuf;

        // set boardSize
        Size boardSize;
        boardSize.width = 9;
        boardSize.height = 6;
        
		//rotate it on the x axis
		flip(webcamImage, webcamImage, 1);
		imshow("test", webcamImage);
        
        // Find a chessboard pattern on webcamfeed with size boardSize.
        bool found;
        found = findChessboardCorners( webcamImage, boardSize, pointBuf,
                                      CV_CALIB_CB_ADAPTIVE_THRESH | CV_CALIB_CB_FAST_CHECK | CV_CALIB_CB_NORMALIZE_IMAGE);
        
        if (found)
        {
            cout << "found" << endl;
        }
        
		if (27 == waitKey(33))
			break;
	}
	return 0;
}

