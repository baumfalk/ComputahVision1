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
#include <stdlib.h>

using namespace cv;
using namespace std;

bool takeSamples(Mat webcamImage,Size boardSize, vector<Point2f>& imagePoints, Vector<Point2f>& pointBuf);



// Displays a supplied image
int main(int argc, const char** argv) {
    
	VideoCapture camera;
	Mat webcamImage;
	camera.open(0);
	if (!camera.isOpened()) {
		cerr << "ERROR: Could not access the camera or video!" << endl;
		exit(1);
	}
    
    int imgCounter =0;
    
	namedWindow("test", CV_WINDOW_AUTOSIZE);
	while (true) {
		camera >> webcamImage;
		if (webcamImage.empty()) {
			cerr << "ERROR: Couldn't grab a camera frame." << endl;
			exit(1);
		}
        
        // Vectors
        vector<Point2f> pointBuf;
        vector<vector<Point2f> > imagePoints;
        

        
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
        
        if(found && imgCounter <25)
        {
            imagePoints.push_back(pointBuf);
            imgCounter++;
            cout << pointBuf << endl;
          

        }
        
        
		if (27 == waitKey(33))
			break;
	}
	return 0;
}

//bool takeSamples(Mat webcamImage,Size boardSize, vector<Point2f>& imagePoints, Vector<Point2f>& pointBuf)
//{
    
//    findChessboardCorners(webcamImage, boardSize, pointBuf);
//    imagePoints.push_back(pointBuf);
//    usleep(1);
    
//}
