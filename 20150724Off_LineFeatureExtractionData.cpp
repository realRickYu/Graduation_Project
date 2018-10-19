#pragma once
#include "C3DLaserSensorFeatureExtractionV20.0.h"
#include "C3DLaserSensorRSerialIOV11.0.h"
#include <sstream>
#include <string>
#include <iostream>

void main()
{
	C3DLaserSensorRSerialIO LaserSRIO;
	C3DLaserSensorFeatureExtraction LaserSRFeatrueExtraction;
	LaserSRFeatrueExtraction.LaserSensorIO.DefaultWorkingPath =  _getcwd(NULL,0);
	bool IsMeasureDone=LaserSRFeatrueExtraction.LaserScanSensor3DFeatrueExtraction();
}