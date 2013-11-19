#include "CameraGeometricCalibration.h"

// Displays a supplied image
int main(int argc, const char** argv) {

	CameraGeometricCalibration test;
	test.takeSamples();
	test.calibrate();
	test.drawAxesAndCube();
	return 0;
}
