//sudo modprobe bcm2835-v4l2
// hcitool scan
//sudo rfcomm connect hci0 98:D3:21:F7:40:FE 1

#include <opencv2/core/core.hpp>
#include <opencv2/imgcodecs/imgcodecs.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/aruco.hpp>
#include <opencv2/calib3d/calib3d.hpp>
#include <opencv2/core/matx.hpp>
#include <wiringPi.h>
#include <wiringSerial.h>
#include <unistd.h>
#include <assert.h>

#include <sstream>
#include <iostream>
#include <fstream>
#include <string>
#include <stdlib.h>
#include <bits/stdc++.h>

using namespace std;
using namespace cv;

const float calibrationSquareDimensions = 0.02431f;
const float arucoSquareDimention = 0.1068f;
const Size chessboardDimensions = Size(9, 6);


void createArucoMarkers() {

	Mat outputMarker;
	Ptr<aruco::Dictionary> markerDictionary = aruco::getPredefinedDictionary(aruco::PREDEFINED_DICTIONARY_NAME::DICT_4X4_50);

	for (int i = 0; i < 50; i++) {
		aruco::drawMarker(markerDictionary, i, 500, outputMarker, 1);
		ostringstream convert;
		string imageName = "4x4Marker_";
		convert << imageName << i << ".jpg";
		imwrite(convert.str(), outputMarker);
	}
}

void createKnownBoardPositions(Size boardSize, float squareEdgeLength, vector<Point3f>& corners) {

	for (int i = 0; i < boardSize.height; i++) {
		for (int j = 0; j < boardSize.width; j++) {
			corners.push_back(Point3f(j*squareEdgeLength, i*squareEdgeLength, 0.0f));
		}
	}
}

void getChessboardCorners(vector<Mat> images, vector<vector<Point2f>>& allFoundCorners, bool showResults = false) {

	for (vector<Mat>::iterator iter = images.begin(); iter != images.end(); iter++) {
		vector<Point2f> pointBuf;	//point buffer
		bool found = findChessboardCorners(*iter, Size(9, 6), pointBuf, CALIB_CB_ADAPTIVE_THRESH | CALIB_CB_NORMALIZE_IMAGE);

		if (found) {
			allFoundCorners.push_back(pointBuf);
		}
		if (showResults) {
			drawChessboardCorners(*iter, Size(9, 5), pointBuf, found);
			imshow("Looking for corners", *iter);
			waitKey(0);
		}

	}

}

void cameraCalibration(vector<Mat> calibrationImages, Size boardSize, float squareEdgeLength, Mat& cameraMatrix, Mat& distortionCoefficients) {

	vector<vector<Point2f>> checkerboardImageSpacePoints;
	getChessboardCorners(calibrationImages, checkerboardImageSpacePoints, false);

	vector<vector<Point3f>> worldSpaceCornerPoints(1);

	createKnownBoardPositions(boardSize, squareEdgeLength, worldSpaceCornerPoints[0]);
	worldSpaceCornerPoints.resize(checkerboardImageSpacePoints.size(), worldSpaceCornerPoints[0]);

	vector<Mat> rVectors, tVectors;
	distortionCoefficients = Mat::zeros(8, 1, CV_64F);

	calibrateCamera(worldSpaceCornerPoints, checkerboardImageSpacePoints, boardSize, cameraMatrix, distortionCoefficients, rVectors, tVectors);

}

bool saveCameraCalibration(string name, Mat cameraMatrix, Mat distortionCoefficients) {
	ofstream outStream(name);
	if (outStream) {
		uint16_t rows = cameraMatrix.rows;
		uint16_t columns = cameraMatrix.cols;

		outStream << rows << endl;
		outStream << columns << endl;

		for (int r = 0; r < rows; r++) {
			for (int c = 0; c < columns; c++) {
				double value = cameraMatrix.at<double>(r, c);
				outStream << value << endl;
			}
			cout << "scc" << r << endl;
		}

		rows = distortionCoefficients.rows;
		columns = distortionCoefficients.cols;

		outStream << rows << endl;
		outStream << columns << endl;

		for (int r = 0; r < rows; r++) {
			for (int c = 0; c < columns; c++) {
				double value = cameraMatrix.at<double>(r, c);
				outStream << value << endl;
			}
			cout << "cc" << r << endl;
		}

		outStream.close();
		return true;

	}
	return false;
}

bool loadCameraCalibration(string name, Mat& cameraMatrix, Mat& distortionCoefficients) {

	ifstream inStream(name);
	if (inStream) {
		uint16_t rows;
		uint16_t columns;

		inStream >> rows;
		inStream >> columns;

		cameraMatrix = Mat(Size(rows, columns), CV_64F);

		for (int r = 0; r < rows; r++) {
			for (int c = 0; c < columns; c++) {
				double read = 0.0;
				inStream >> read;
				cameraMatrix.at<double>(r, c) = read;
				cout << cameraMatrix.at<double>(r, c) << "\n";
			}
		}
		//Distortion Coefficients
		inStream >> rows;
		inStream >> columns;

		distortionCoefficients = Mat::zeros(rows, columns, CV_64F);

		for (int r = 0; r < rows; r++) {
			for (int c = 0; c < columns; c++) {
				double read = 0.0;
				inStream >> read;
				distortionCoefficients.at<double>(r, c) = read;
				cout << distortionCoefficients.at<double>(r, c) << "\n";
			}
		}
		inStream.close();
		return true;
	}
	return false;

}

int startWebcamMonitoring(const Mat& cameraMatrix, const Mat& distortionCoefficients, float arucoSquareDimension) {

	Mat frame;
	double x,y,z,zx,zy,dx,dz;
	int perfectX = 200;     //desired X parameter of Marker's position
    int perfectZ = 200;     //desired distance to marker
	int zmax = 350;    //Z value of the nearest possible position of marker
	int zmin = 50;  //Z value of the farthest possible position of marker
	////////////////////////Bluetooth////////////////////////////

	char commandDirection;  //1- ahead, 2- stop
	char commandA[4];   //left motor
	char commandB[4];   //right motor
	int commandAi;
	int commandBi;
	int btCounter = 0;
	int serialDevice = 0;
    int wiringSetup;
    serialDevice = serialOpen("/dev/rfcomm0", 9600);
    if(serialDevice == -1){
        cout << "Unable to connect" << endl;
        return 1;
    }
    wiringSetup = wiringPiSetup();
    if(wiringSetup == -1){
        cout << "Unable to start wiringPi" << endl;
        return 1;
    }
    cout << "Monitoring started" << endl;
    //////////////////////Bluetooth end/////////////////////////////
	vector<int> markerIds;
	vector<vector<Point2f>>  markerCorners, rejectedCorners;
	aruco::DetectorParameters parameters;

	Ptr<aruco::Dictionary> markerDictionary = aruco::getPredefinedDictionary(aruco::PREDEFINED_DICTIONARY_NAME::DICT_4X4_50);
    VideoCapture vid = VideoCapture(0);

	if (!vid.isOpened()) {
		return -1;
	}
	vid >> frame;

	namedWindow("Webcam", WINDOW_AUTOSIZE);

	vector<Vec3d> rotationVectors, translationVectors;

	while (true) {
		if (!vid.read(frame))
			break;

		aruco::detectMarkers(frame, markerDictionary, markerCorners, markerIds);
		aruco::estimatePoseSingleMarkers(markerCorners, arucoSquareDimension, cameraMatrix, distortionCoefficients, rotationVectors, translationVectors);

		for (int i = 0; i < markerIds.size(); i++) {
			aruco::drawAxis(frame, cameraMatrix, distortionCoefficients, rotationVectors[i], translationVectors[i], arucoSquareDimention);
            //print location

            x = (markerCorners[i][0].x + markerCorners[i][1].x + markerCorners[i][2].x + markerCorners[i][3].x)/4 ;
            y = (markerCorners[i][0].y + markerCorners[i][1].y + markerCorners[i][2].y + markerCorners[i][3].y)/4 ;
            zx = (markerCorners[i][3].x - markerCorners[i][1].x) *480/640;
            zy = (markerCorners[i][3].y - markerCorners[i][1].y);
            z = zx + zy;
            cout << "X: " << x << " Z: " << z << endl;
            ////////////////SEND COMMANDS TO ARDUINO////////////////////////////
            btCounter++;
            if(btCounter == 10){    //As low as possible for bluetooth to send and receive data

                if(z > zmax || z < zmin) { //too close or too far, stop the robot
                        commandDirection = '2'; //Stop
                        sprintf(commandA, "%d", 0);
                        sprintf(commandB, "%d", 0);
                        cout << "CommandA: " << commandA << " CommandB: " << commandB << endl;
                }
                else {
                    dx = (perfectX - x)/570;    //640- resolution, 70- min, 570- max
                    dz = (perfectZ - z)/zmax;
                    commandDirection = '1';     //move forward
                    commandAi = 178*(1 + dz);   //128- 50% max speed of motor
                    commandBi = 178*(1 + dz);
                    commandAi = commandAi*(1 + dx/2);
                    commandBi = commandAi*(1 - dx/2);
                    if(commandAi < 100) commandAi = 100;  //min value for motor to work
                    if(commandBi < 100) commandBi = 100;  //min value for motor to work
                    cout << "dz: " << dz << " dx: " << dx << endl;
                    sprintf(commandA, "%d", commandAi);
                    sprintf(commandB, "%d", commandBi);
                    cout << "CommandA: " << commandA << " CommandB: " << commandB << endl;
                }

                serialPutchar(serialDevice, commandDirection);
                serialPutchar(serialDevice, 'a');
                serialPrintf(serialDevice, commandA);
                serialPutchar(serialDevice, 'b');
                serialPrintf(serialDevice, commandB);

                btCounter = 0;
                cout <<"-------------------Data Sent---------------------- " << endl;
            }

            //////////////////////END OF SENDING COMMANDS////////////////////////////////////

		}

		imshow("Webcam", frame);
		if (waitKey(30) >= 0) break;

	}

	return 1;

}

void cameraCalibrationProcess(Mat& cameraMatrix, Mat& distortionCoefficients) {
	Mat frame;
	Mat drawToFrame;

	vector<Mat> savedImages;

	vector<vector<Point2f>> markerCorners, rejectedCandidates;
    VideoCapture vid = VideoCapture(0);
    vid >> frame;

	if (!vid.isOpened()) {
		return;
	}

	int fps = 20;

	namedWindow("Webcam", WINDOW_AUTOSIZE);

	while (true) {
		if (!vid.read(frame))
			break;
		vector<Vec2f> foundPoints;
		bool found = false;

		found = findChessboardCorners(frame, chessboardDimensions, foundPoints, CALIB_CB_ADAPTIVE_THRESH | CALIB_CB_NORMALIZE_IMAGE);
		frame.copyTo(drawToFrame);
		drawChessboardCorners(drawToFrame, chessboardDimensions, foundPoints, found);
		if (found)
			imshow("Webcam", drawToFrame);
		else
			imshow("Webcam", frame);
		char character = waitKey(1000 / fps);

		switch (character) {
		case ' ':
			//saving image
			if (found) {
				Mat temp;
				frame.copyTo(temp);
				savedImages.push_back(temp);
				cout << "image saved" << savedImages.size() << endl;
			}
			break;
		case 's':
			//start calibration
			if (savedImages.size() > 15) {
                cout << "saving started" << endl;
				cameraCalibration(savedImages, chessboardDimensions, calibrationSquareDimensions, cameraMatrix, distortionCoefficients);
				saveCameraCalibration("CameraCalibrated5", cameraMatrix, distortionCoefficients);
				cout << "calibration finished" << endl;
			}
			break;
		case 27:
			//exit
			cout << "all done" << endl;
			return;
			break;
		}
	}
}
int main(int argv, char** argc) {

	Mat cameraMatrix = Mat::eye(3, 3, CV_64F);
	Mat distortionCoefficients;

	//cameraCalibrationProcess(cameraMatrix, distortionCoefficients);
	loadCameraCalibration("CameraCalibrated2", cameraMatrix, distortionCoefficients);

	startWebcamMonitoring(cameraMatrix, distortionCoefficients, arucoSquareDimention);

	return 0;
}
