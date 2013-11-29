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

    int numberOfSamples = 50, timeBetweenSamples = 150;
    if(argc >= 3)
    {
        numberOfSamples = atoi(argv[1]);
        timeBetweenSamples = atoi(argv[2]);
        if(numberOfSamples == 0 || timeBetweenSamples == 0)
            cout << "Incorrect input: you need to give a number larger than 0";
    }
    else
        cout << "usage: ComputerVision1 [numberOfSamples=50, timeBetweenSamples=150]" << endl;
	CameraGeometricCalibration cameraCalibrator(numberOfSamples,timeBetweenSamples);
	cameraCalibrator.takeSamples();
	cameraCalibrator.calibrate();
	cameraCalibrator.drawAxesAndCube();
	return 0;
}
