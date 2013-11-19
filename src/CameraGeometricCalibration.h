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
	CameraGeometricCalibration(short numberOfSamplesNeeded = 25, string windowName = "ComputahVision1");
	void takeSamples();
	void calibrateCamera();
	void drawAxesAndCube();

private:
	void takePicture();
	void showPicture();
	void writeText(int x, int y, string msg);
	bool findChessBoard();
	bool enoughTimeElapsed(double waitingTime);

	VideoCapture camera;
	Mat webcamImage;
	short numberOfSamplesNeeded;
	short numberOfSamplesFound;
	string windowName;
	double timestamp;
	Size boardSize;
	bool chessBoardFound;
	vector<Point2f> pointBuf;
	vector<vector<Point2f> > chessBoardPointList;


};
