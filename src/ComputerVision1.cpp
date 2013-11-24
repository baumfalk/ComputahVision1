/*
 * ComputerVision1.cpp
 *
 *       Author: Jetze (3471055)
 *          and  Barend (3539784)
 *
 */

#include "CameraGeometricCalibration.h"

/*
 * Loads the calibration class and uses it.
 */
int main(int argc, const char** argv) {

	CameraGeometricCalibration cameraCalibrator;
	cameraCalibrator.takeSamples();
	cameraCalibrator.calibrate();
	cameraCalibrator.drawAxesAndCube();
	return 0;
}
