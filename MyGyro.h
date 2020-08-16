/*
 *MyGyro.h
 *Created on 8/16/2020
 *Completed on ...
 *By S.H.
 *Relevant functions about the gyro module
 */

//First step: get the data from the module
//Second step: analyze the data

//This version does not include Kalman filter

#ifndef _MYGYRO_H_
#define _MYGYRO_H_

#include<math.h>
#include<Wire.h>

//The size of the data array from MPU
#define GYRO_ARRAY_SIZE 7
//The size of the outputArray
#define OUTPUT_ARRAY_SIZE 5
//Acceleration of gravity
#define g 9.8
//PI
#define PI 3.14159

int gyro[GYRO_ARRAY_SIZE];
int calib[GYRO_ARRAY_SIZE];
double output[OUTPUT_ARRAY_SIZE];

//Transform the value to the degree
//INPUT: The initial value of the angle
//OUTPUT: The degree value of the angle
double transToDeg(int value) {
	double omega = (500 * value) / 32768;
	return omega;
}

//Transform from rad to deg
//INPUT: The rad value of the angle
//OUTPUT: The degree value of the angle
double radToDeg(double rad) {
	double deg = (180 * rad) / PI;
	return deg;
}

//Transform the value to the acceleration
//INPUT: The initial value
//OUTPUT: The acceleration in reality
double transToAcceleration(int value) {
	double a = (2 * g * value) / (-32768);
	return a;
}

//Start up
//INPUT: NONE
//OUTPUT: NONE
//Notation: The function is suggested to be put in the “setup()” module
void MyGyroInitialize() {
	//Start up the wire
	Wire.begin();

	//Start up the MPU module
	Wire.beginTransmission(0x68);
	Wire.write(0x6B);
	Wire.write(0);
	Wire.endTransmission(true);

	//Set f=0
    Wire.beginTransmission(0x68);
    Wire.write(0x1c);
    Wire.requestFrom(0x68, 1, true);
    unsigned char accConf = Wire.read();
    accConf = ((accConf & 0xe7) | (0 << 3));
    Wire.write(accConf);
    Wire.endTransmission(true);

	//Set f=1
    Wire.beginTransmission(0x68);
    Wire.write(0x1b);
    Wire.requestFrom(0x68, 1, true);
    unsigned char accConf = Wire.read();
    accConf = ((accConf & 0xe7) | (1 << 3));
    Wire.write(accConf);
    Wire.endTransmission(true);

	//Start up the calibration
	calibInitialize(calib);
}

//Read the data(array) from the MPU
//INPUT: The address of the array to store the data 
//OUTPUT: NONE
void readData(int* arrayAddr) {
	Wire.beginTransmission(0x68);
	Wire.write(0x3B);
	Wire.requestFrom(0x68, 14, true);
	Wire.endTransmission(true);
	for (int i = 0; i < 7; i++) {
		arrayAddr[i] = Wire.read() << 8 | Wire.read();
	}
}

//Calibration initialization
//INPUT: The address of the array to store the calibration array
//OUTPUT: NONE
void calibInitialize(int* arrayAddr) {
	//Get 1000 arrays and add them all up
	int testData[7];
	int sumData[7];
	for (int i = 0; i < 1000; i++) {
		readData(testData);
		for (int j = 0; j < 7; j++) {
			sumData[i] += testData[i];
		}
	}

	//Get the average
	for (int k = 0; k < 7; k++) {
		arrayAddr[i] = (int)(sumData[i] / 1000);
	}
	arrayAddr[2] += 16384;
}

//Calibrate the data
//INPUT: The data array and the calibration array
//OUTPUT: NONE
void calibData(int* dataAddr, int* calibAddr) {
	int i;
	for (i = 0; i < 7; i++) {
		dataAddr[i] = dataAddr[i] - calibAddr[i];
	}
}

//Get the roll
//INPUT: The data
//OUTPUT: The roll angle
double getRoll(int* dataAddr) {
	double norm = sqrt(dataAddr[0] * dataAddr[0] + dataAddr[1] * dataAddr[1] + dadaAddr[2] * dadaAddr[2]);
	double normXZ = sqrt(dataAddr[0] * dataAddr[0]  + dataAddr[2] * dataAddr[2]);
	double cosine = normXZ / norm;
	double roll = acos(cosine);
	if (dataAddr[1] > 0) roll = -roll;
	return roll;
}

//Get the pitch
//INPUT: The data
//OUTPUT: The pitch angle
double getPitch(int* dataAddr) {
	double norm  = sqrt(dataAddr[0] * dataAddr[0] + dataAddr[1] * dataAddr[1] + dadaAddr[2] * dadaAddr[2]);
	double normYZ = sqrt(dataAddr[1] * dataAddr[1] + dataAddr[2] * dataAddr[2]);
	double cosine = normYZ / norm;
	double pitch = acos(cosine);
	if (dataAddr[0] < 0) pitch = -pitch;
	return pitch;
}

//Analyze the data
//INPUT: The data array and the final output
//OUTPUT: NONE
void analyzeData(int* dataAddr, int* outputAddr) {
	//Caculate the roll and the pitch
	double roll = getRoll(dataAddr);
	double pitch = getPitch(dataAddr);

	//Store the result in the array
	outputAddr[0] = transToAcceleration(dataAddr[0]);
	outputAddr[1] = transToAcceleration(dataAddr[1]);
	outputAddr[2] = transToAcceleration(dataAddr[2]);
	outputAddr[3] = radToDeg(roll);
	outputAddr[4] = radToDeg(pitch);
}

//The final 
//INPUT: NONE
//OUTPUT: outputArray
//Notation: This should be put in the “loop()” module
double* MYGYRO() {
	readData(gyro);
	calibData(gyro, calib);
	analyzeData(gyro, output);
	return output;
}
	
#endif //_MYGRRO_H_

/**************END OF THE FILE*******************/
