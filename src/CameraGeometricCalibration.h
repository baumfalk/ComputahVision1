/*
 * CameraGeometricCalibration.h
 *
 *       Author: Jetze (3471055)
 *          and  Barend (3539784)
 *
 */

#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/calib3d/calib3d.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <iostream>
#include <ctime>

using namespace cv;
using namespace std;

class CameraGeometricCalibration {
public:
	CameraGeometricCalibration(short numberOfSamplesNeeded = 40,
			string windowName = "ComputahVision1");
	void takeSamples();
	void calibrate();
	void drawAxesAndCube();

private:
	void takePicture();
	void writeText(int x, int y, string msg);
	bool findChessBoard(bool add = true);
	bool enoughTimeElapsed(double waitingTime);

	void calcChessBoardPositions3D(Size boardSize, float squareSize, vector<Point3f>& corners);
	void calculateReprojectionErrors();

	void createAxes(vector<Point3f>& axes, int length);
	void createCube(vector<Point3f>& axes, int length);
	void drawAxes(vector<Point2f>& imagePoints);
	void drawCube(vector<Point2f>& imagePoints);
	void drawChessBoardGrid(vector<Point2f>& imagePoints);
	void drawPicture(bool drawChessBoard = true);

	VideoCapture camera;
	Mat webcamImage;
	Mat cameraMatrix;
	Mat distCoeffs;

	bool chessBoardFound;
	bool done;
	short numberOfSamplesNeeded;
	short numberOfSamplesFound;
	double timestamp;
	float squareSize;
	string windowName;
	Size boardSize;

	vector<Mat> rvecs;
	vector<Mat> tvecs;
	vector<Point2f> pointBuf;
	vector<vector<Point2f> > chessBoardPointList;
	vector<vector<Point3f> > chessBoardPoints3D;
};
