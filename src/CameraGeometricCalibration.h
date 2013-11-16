#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/calib3d/calib3d.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <iostream>


using namespace cv;
using namespace std;

class CameraGeometricCalibration {

public:
	void showShit();
	void openWebcam();
	void takeSamples();
	void calibrateCamera();
	void drawAxesAndCube();
};
