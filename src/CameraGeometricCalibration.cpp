/*
 * CameraGeometricCalibration.cpp
 *
 *       Author: Jetze (3471055)
 *          and  Barend (3539784)
 *
 *  Some code was inspired by http://docs.opencv.org/doc/tutorials/calib3d/camera_calibration/camera_calibration.html,
 *  namely the code for enoughTimeElapsed, calculateReprojectionErrors and calcChessBoardPositions3D.
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
	done = false;
	boardSize.width = 9;
	boardSize.height = 6;
	squareSize = 5;
	timestamp = clock();
	namedWindow(windowName, CV_WINDOW_AUTOSIZE);

	camera.open(-1);
	if (!camera.isOpened()) {
		cerr << "ERROR: Could not access the camera or video!" << endl;
		exit(1);
	}

	takePicture();
	drawPicture();
}

/*
 * Continuously check the webcam feed for occurrences of the chessboard until the
 * required number of samples is found.
 */
void CameraGeometricCalibration::takeSamples() {

	while (numberOfSamplesFound != numberOfSamplesNeeded && !done) {
		char key = waitKey(33);
		if(key == 27 || key == 'q')
			done = true;
		takePicture();
		string msg = format("%d/%d", numberOfSamplesFound,
				numberOfSamplesNeeded);
		writeText(10, 20, msg);

		if (enoughTimeElapsed(100)) {
			chessBoardFound = findChessBoard();
			if (chessBoardFound) {
				numberOfSamplesFound++;

			}
			timestamp = clock();
		}
		drawPicture();
	}
	chessBoardFound = false;
}

/*
 * Calibrate the camera using the retrieved samples,
 * i.e. find the intrinsic and extrinsic parameters.
 */
void CameraGeometricCalibration::calibrate() {
	if(done)
		return;
	vector<Point3f> tmp;
	chessBoardPoints3D.resize(1, tmp);
	calcChessBoardPositions3D(boardSize, squareSize, chessBoardPoints3D[0]);
	chessBoardPoints3D.resize(chessBoardPointList.size(),
			chessBoardPoints3D[0]);
	double rms = calibrateCamera(chessBoardPoints3D, chessBoardPointList,
			boardSize, cameraMatrix, distCoeffs, rvecs, tvecs,
			CV_CALIB_FIX_K4 | CV_CALIB_FIX_K5);

	cout << "Camera matrix: " << cameraMatrix << endl;
	cout << "error: " << rms << endl;
	calculateReprojectionErrors();
}

/*
 * Undistort the image.
 * Calculate the rotation and translation vector.
 * Use that data to draw the 3 axes and a cube.
 */
void CameraGeometricCalibration::drawAxesAndCube() {
	while (!done) {
		char key = waitKey(33);
		if(key == 27 || key == 'q')
			done = true;
		takePicture();

		chessBoardFound = findChessBoard(false);
		if (chessBoardFound) {
			Mat rvec;
			Mat tvec;

			solvePnP(chessBoardPoints3D[0], pointBuf, cameraMatrix, distCoeffs,	rvec, tvec, false, CV_EPNP);
			// draw our own chessboard grid
			vector<Point2f> imagePoints;
			projectPoints(Mat(chessBoardPoints3D[0]), rvec, tvec, cameraMatrix,	distCoeffs, imagePoints);
			drawChessBoardGrid(imagePoints);


			// draw the axes
			vector<Point3f> axes;
			createAxes(axes, 20);
			projectPoints(Mat(axes), rvec, tvec, cameraMatrix, distCoeffs, imagePoints);
			drawAxes(imagePoints);

			// draw the cube
			createCube(axes, 10);
			projectPoints(Mat(axes), rvec, tvec, cameraMatrix, distCoeffs, imagePoints);
			drawCube(imagePoints);

		}
		drawPicture(false);
	}
}

/*
 * Draws the chessgrid using the transformed 3D coordinates instead of the
 * findChessBoardCorners 2D coordinates
 */
void CameraGeometricCalibration::drawChessBoardGrid(vector<Point2f>& imagePoints)
{
	for (int i = 0; i < (int)imagePoints.size() - 1; i++) {
		line(webcamImage, imagePoints[i], imagePoints[i + 1], Scalar(128, 0, 128));
	}
}

/*
 * Initialize the coordinates needed to draw the X, Y and Z axes.
 *
 */
void CameraGeometricCalibration::createAxes(vector<Point3f>& axes, int length) {
	axes.clear();
	axes.push_back(Point3f(0, 0, 0));
	axes.push_back(Point3f(length, 0, 0));
	axes.push_back(Point3f(0, length, 0));
	axes.push_back(Point3f(0, 0, -length));
}

/*
 * Draws axes in the 2d space
 */
void CameraGeometricCalibration::drawAxes(vector<Point2f>& imagePoints)
{
	line(webcamImage, imagePoints[0], imagePoints[1], Scalar(229, 0, 255), 2);
	line(webcamImage, imagePoints[0], imagePoints[2], Scalar(0, 255, 221), 2);
	line(webcamImage, imagePoints[0], imagePoints[3], Scalar(0, 51, 255), 2);
}

/*
 * Initialize the coordinates needed to draw a cube along the axes.
 *
 */
void CameraGeometricCalibration::createCube(vector<Point3f>& axes, int length) {
	axes.clear();
	axes.push_back(Point3f(0, 0, 0));

	axes.push_back(Point3f(length, 0, 0));
	axes.push_back(Point3f(0, length, 0));
	axes.push_back(Point3f(0, 0, -length));

	axes.push_back(Point3f(length, length, 0));
	axes.push_back(Point3f(0, length, -length));
	axes.push_back(Point3f(length, 0, -length));

	axes.push_back(Point3f(length, length, -length));
}

/*
 * Draws the cube in the 2d space
 */
void CameraGeometricCalibration::drawCube(vector<Point2f>& imagePoints)
{
	line(webcamImage, imagePoints[0], imagePoints[1], Scalar(255, 0, 0), 2);
	line(webcamImage, imagePoints[0], imagePoints[2], Scalar(255, 0, 0), 2);
	line(webcamImage, imagePoints[0], imagePoints[3], Scalar(255, 0, 0), 2);
	line(webcamImage, imagePoints[3], imagePoints[5], Scalar(255, 0, 0), 2);
	line(webcamImage, imagePoints[3], imagePoints[6], Scalar(255, 0, 0), 2);
	line(webcamImage, imagePoints[2], imagePoints[4], Scalar(255, 0, 0), 2);
	line(webcamImage, imagePoints[2], imagePoints[5], Scalar(255, 0, 0), 2);
	line(webcamImage, imagePoints[1], imagePoints[4], Scalar(255, 0, 0), 2);
	line(webcamImage, imagePoints[1], imagePoints[6], Scalar(255, 0, 0), 2);
	line(webcamImage, imagePoints[7], imagePoints[5], Scalar(255, 0, 0), 2);
	line(webcamImage, imagePoints[7], imagePoints[6], Scalar(255, 0, 0), 2);
	line(webcamImage, imagePoints[7], imagePoints[4], Scalar(255, 0, 0), 2);
}

/*
 * Calculate and print the reprojection error.
 *
 */
void CameraGeometricCalibration::calculateReprojectionErrors() {
	vector<Point2f> imagePoints2;
	int totalPoints = 0;
	double totalErr = 0, err;
	for (int i = 0; i < (int)chessBoardPointList.size(); i++) {
		projectPoints(Mat(chessBoardPoints3D[i]), rvecs[i], tvecs[i],
				cameraMatrix, distCoeffs, imagePoints2);
		err = norm(Mat(chessBoardPointList[i]), Mat(imagePoints2), CV_L2);
		int n = (int) chessBoardPointList[i].size();
		totalErr += err * err;
		totalPoints += n;
	}

	cout << "reprojection error:" << std::sqrt(totalErr / totalPoints) << endl;
}

/*
 *  Calculate the 3D coordinates for the chessboard (z-axis is 0).
 */
void CameraGeometricCalibration::calcChessBoardPositions3D(Size boardSize,
		float squareSize, vector<Point3f>& corners) {
	corners.clear();
	for (int i = 0; i < boardSize.height; ++i)
		for (int j = 0; j < boardSize.width; ++j)
			corners.push_back(
					Point3f(float(j * squareSize), float(i * squareSize), 0));
}

/*
 * used to take a picture from the webcam. We rotate it around the x-axis so
 * that the window functions like a mirror. This makes the calibration easier,
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
 * Draw the picture taken from the webcam on the screen.
 * If a chessboard was found in the picture, we show it in the picture.
 */
void CameraGeometricCalibration::drawPicture(bool drawChessBoard) {

	if (chessBoardFound && drawChessBoard) {
		drawChessboardCorners(webcamImage, boardSize, pointBuf, true);
	}
	imshow(windowName, webcamImage);
}

/*
 * Draw a message on the window.
 */
void CameraGeometricCalibration::writeText(int x, int y, string msg) {
	putText(webcamImage, msg, Point(x, y), 1, 1, Scalar(0, 0, 255));
}

/*
 * Find the chessboard in a picture and store the corners in a list.
 *
 */
bool CameraGeometricCalibration::findChessBoard(bool add) {
	bool result = findChessboardCorners(webcamImage, boardSize, pointBuf,
			CV_CALIB_CB_ADAPTIVE_THRESH | CV_CALIB_CB_FAST_CHECK
					| CV_CALIB_CB_NORMALIZE_IMAGE);

	if (result && add) {
		//Enhance the corners
		Mat viewGray;
		cvtColor(webcamImage, viewGray, CV_BGR2GRAY);
		cornerSubPix(viewGray, pointBuf, Size(11, 11), Size(-1, -1),
				TermCriteria(CV_TERMCRIT_EPS + CV_TERMCRIT_ITER, 30, 0.1));

		chessBoardPointList.push_back(pointBuf);
	}
	return result;

}
/*
 * Check if enough time (in ms) have elapsed since the last
 * recognized chessboard. This is useful, since this allows the program to
 * take multiple pictures of the chessboard in different orientations, instead of
 * taking all pictures in a row.
 */
bool CameraGeometricCalibration::enoughTimeElapsed(double waitingTime) {
	return (clock() - timestamp > waitingTime * 1e-3 * CLOCKS_PER_SEC);
}
