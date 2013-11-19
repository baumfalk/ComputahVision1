#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/calib3d/calib3d.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <iostream>
#include <ctime>

using namespace cv;
using namespace std;

class CameraGeometricCalibration {
//TODO: restructure
public:
	CameraGeometricCalibration(short numberOfSamplesNeeded = 25,
			string windowName = "ComputahVision1");
	void takeSamples();
	void calibrate();
	void drawAxesAndCube();

private:
	void takePicture();
	void showPicture();
	void writeText(int x, int y, string msg);
	bool findChessBoard();
	bool enoughTimeElapsed(double waitingTime);
	void calcChessBoardPositions3D(Size boardSize, float squareSize,
			vector<Point3f>& corners);
	VideoCapture camera;
	Mat webcamImage;
	Mat cameraMatrix;
	Mat distCoeffs;

	short numberOfSamplesNeeded;
	short numberOfSamplesFound;
	string windowName;
	double timestamp;
	Size boardSize;
	float squareSize;
	bool chessBoardFound;

	vector<Mat> rvecs;
	vector<Mat> tvecs;
	vector<vector<Point3f> > chessBoardPoints3D;
	vector<Point2f> pointBuf;
	vector<vector<Point2f> > chessBoardPointList;

};
