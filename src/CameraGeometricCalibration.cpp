/*
 * CameraGeometricCalibration.cpp
 *
 *  Created on: 12 nov. 2013
 *      Author: Jetze en Barend
 */

#include "CameraGeometricCalibration.h"

/*
 * Initializes the class. Also create a window and shows a webcam picture in it.
 */
CameraGeometricCalibration::CameraGeometricCalibration(
		short numberOfSamplesNeeded, string windowName) {
	this->numberOfSamplesNeeded = numberOfSamplesNeeded;
	this->windowName = windowName;
	numberOfSamplesFound = 0;
	boardSize.width = 9; // TODO: make this dynamic.
	boardSize.height = 6;
	squareSize = 3;
	timestamp = clock();
	namedWindow(windowName, CV_WINDOW_AUTOSIZE);

	camera.open(0);
	if (!camera.isOpened()) {
		cerr << "ERROR: Could not access the camera or video!" << endl;
		exit(1);
	}

	takePicture();
	showPicture();
}

/*
 * Continuously check the webcam feed for occurrences of the chessboard until the
 * required number of samples is found.
 */
void CameraGeometricCalibration::takeSamples() {

	while (numberOfSamplesFound != numberOfSamplesNeeded && 27 != waitKey(33)) {
		takePicture();
		string msg = format("%d/%d", numberOfSamplesFound,
				numberOfSamplesNeeded);
		writeText(10, 20, msg);

		if (enoughTimeElapsed(15)) {
			chessBoardFound = findChessBoard();
			if (chessBoardFound) {
				numberOfSamplesFound++;
				/*Mat viewGray;
				 cvtColor(webcamImage, viewGray, CV_BGR2GRAY);
				 cornerSubPix( viewGray, pointBuf, Size(11,11),
				 Size(-1,-1), TermCriteria( CV_TERMCRIT_EPS+CV_TERMCRIT_ITER, 30, 0.1 ));*/
			}
			timestamp = clock();
		}
		showPicture();
	}
	chessBoardFound = false;
}

/*
 * Calibrate the camera using the retrieved samples,
 * i.e. find the intrinsic and extrinsic parameters.
 */
void CameraGeometricCalibration::calibrate() {
	//TODO: implement me
	vector<Point3f> tmp;
	chessBoardPoints3D.resize(1, tmp);
	calcChessBoardPositions3D(boardSize, squareSize, chessBoardPoints3D[0]);
	chessBoardPoints3D.resize(chessBoardPointList.size(),
			chessBoardPoints3D[0]);
	double rms = calibrateCamera(chessBoardPoints3D, chessBoardPointList,
			boardSize, cameraMatrix, distCoeffs, rvecs, tvecs,
			CV_CALIB_FIX_K4 | CV_CALIB_FIX_K5);

	cout << cameraMatrix << endl;
	cout << rms << endl;
}

//TODO: write comments
void CameraGeometricCalibration::calcChessBoardPositions3D(Size boardSize,
		float squareSize, vector<Point3f>& corners) {
	corners.clear();
	for (int i = 0; i < boardSize.height; ++i)
		for (int j = 0; j < boardSize.width; ++j)
			corners.push_back(
					Point3f(float(j * squareSize), float(i * squareSize), 0));
}

/*
 * Draw the axes of the chessboard on the origin as well as a cube.
 */
void CameraGeometricCalibration::drawAxesAndCube() {
	//TODO: implement me
	while (27 != waitKey(33)) {
		takePicture();

		chessBoardFound = findChessBoard();
		if (chessBoardFound) {
			Mat rvec;
			Mat rmat;
			Mat tvec;
			Mat extrinsic;
			//bool solvePnP(InputArray objectPoints, InputArray imagePoints, InputArray cameraMatrix, InputArray distCoeffs, OutputArray rvec, OutputArray tvec, bool useExtrinsicGuess=false, int flags=ITERATIVE )
			solvePnP(chessBoardPoints3D[0], pointBuf, cameraMatrix, distCoeffs,
					rvec, tvec);
			vector<Point3f> z(1);
			z[0] = Point3f(0, 0, 100);
			Mat zHomoG;

			convertPointsToHomogeneous(z, zHomoG);
			Rodrigues(rvec, rmat);
			hconcat(rmat, tvec, extrinsic);
			cout << "rmat " << rmat << " " <<  rmat.type() << endl;
			cout << "tvec " << tvec << endl;
			cout << "extrinsic " << extrinsic << " " << extrinsic.type()
					<< endl;
			cout << "zhomoG " << zHomoG << " " << zHomoG.type() << endl;

			Mat output = extrinsic * zHomoG;

			//Mat pt;
			//multiply(cameraMatrix,output,pt);
			//cout << pt << endl;
			//writeText(pt.data[0],pt.data[1],"TITS");
		}
		showPicture();
	}
}

/*
 * used to take a picture from the webcam. We rotate it around the x-axis so
 * that the window functions like a mirror. This the calibration easier,
 * since we are used to seeing ourself in a mirror and expect the window to also
 * behave in this way.
 */
void CameraGeometricCalibration::takePicture() {

	camera >> webcamImage;
	if (webcamImage.empty()) {
		cerr << "ERROR: Couldn't grab a camera frame." << endl;
		exit(1);
	}

	//rotate it on the x axis
	flip(webcamImage, webcamImage, 1);
}

/*
 * used to draw the picture taken from the webcam on the screen.
 * If a chessboard was found in the picture, we show it in the picture.
 */
void CameraGeometricCalibration::showPicture() {

	if (chessBoardFound) {
		drawChessboardCorners(webcamImage, boardSize, pointBuf, true);
	}
	imshow(windowName, webcamImage);
}

/*
 * used to draw a message on the window.
 */
void CameraGeometricCalibration::writeText(int x, int y, string msg) {
	putText(webcamImage, msg, Point(x, y), 1, 1, Scalar(0, 0, 255));
}

// used to find the chessboard in a picture and store the corners in a list.
bool CameraGeometricCalibration::findChessBoard() {
	bool result = findChessboardCorners(webcamImage, boardSize, pointBuf,
			CV_CALIB_CB_ADAPTIVE_THRESH | CV_CALIB_CB_FAST_CHECK
					| CV_CALIB_CB_NORMALIZE_IMAGE);

	if (result) {
		chessBoardPointList.push_back(pointBuf);
	}
	return result;

}
/*
 * used to check if enough time (in ms) have elapsed since the last
 * recognized chessboard. This is useful, since this allows the program to
 * take multiple pictures of the chessboard in different orientations, instead of
 * taking all pictures in a row.
 */
bool CameraGeometricCalibration::enoughTimeElapsed(double waitingTime) {
	return (clock() - timestamp > waitingTime * 1e-3 * CLOCKS_PER_SEC );
}
