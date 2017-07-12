#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/videoio.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/video.hpp>
#include <opencv2/core.hpp>
#include <opencv2/objdetect.hpp>

//C++
#include <iostream>
#include <sstream>
#include <math.h>

#include "camera.h"
#include <thread>

using namespace cv;
using namespace std;

void myThread(VideoCapture fluxCam) {
	//Mat binaryFrame;
	
	Mat frameThread;

	if (!fluxCam.read(frameThread)) {
		cerr << "Unable to read next frame." << endl;
		cerr << "Exiting..." << endl;
		exit(EXIT_FAILURE);
	}

	/*
	myCam->binairisation();
	/*
	myCam->calculBarycentre(binaryFrame);
	frameThread = myCam->tracking(frameThread);
	frameThread = myCam->drawContours(binaryFrame, frameThread);
	myCam->trajectory();
	myCam->fallDetection();
	
	cout << "dans le thread" << endl;
	myCam->affiche();
	//myCam->affiche(binaryFrame, "binary_Image", fluxCam);
	//imshow("thread frame" , frameThread);   */
}

int main() {
	
	//Mat frame;
	
	Mat frame; //current frame
	Mat binaryFrame; 
	char keyboard;
	Mat elementErode = getStructuringElement(MORPH_ELLIPSE,	Size(4,3), Point(0,0));
	Mat elementDilate = getStructuringElement(MORPH_ELLIPSE, Size(3,2),	Point(0,0));   
	camera * ipCam = new camera();	
	VideoCapture fluxCam =ipCam->getVideo();
	Ptr<BackgroundSubtractorMOG2> pMOG2; //MOG2 Background subtractor
	pMOG2 = createBackgroundSubtractorMOG2(700, 64, false); //MOG2 approach

	
	//create the capture object
	//VideoCapture capture("rtsp://admin:admin@10.35.127.245/live.sdp");
	VideoCapture capture("test_1.mp4");
	//VideoCapture capture(0);
	
	/*
	void detectAndDisplay(Mat frame);
	String bodyCascadeName = "haarcascade_fullbody.xml";
	//String eyes_cascade_name = "haarcascade_eye.xml";
	CascadeClassifier bodyCascade;
	CascadeClassifier upperBodyCascade;
	String windowName = "Capture - Face detection";  

	if (!bodyCascade.load(bodyCascadeName)) { 
		printf("--(!)Error loading face cascade\n"); 
		return -1; 
	}
	*/


	if (!capture.isOpened()) {
		//error in opening the video input
		cerr << "Unable to open video " << endl;
		exit(EXIT_FAILURE);
	}
		
	//read input data. ESC or 'q' for quitting
	keyboard = 0;

	while (keyboard != 'q' && keyboard != 27) {

		
		//read the current frame
		if (!fluxCam.read(frame)) {
			cerr << "Unable to read next frame." << endl;
			cerr << "Exiting..." << endl;
			exit(EXIT_FAILURE);
		}  

		
		thread processing(myThread, fluxCam);  
		
		binaryFrame = ipCam->binairisation(frame, binaryFrame, pMOG2, elementErode, elementDilate);
		ipCam->calculBarycentre(binaryFrame);
		frame = ipCam->tracking(frame); 
		frame = ipCam->drawContours(binaryFrame, frame);
		ipCam->trajectory();
		ipCam->fallDetection();
		
		ipCam->affiche(frame, "ip_cam", fluxCam);
		ipCam->affiche(binaryFrame, "binary_Image", fluxCam);  
	 
		keyboard = (char)waitKey(10);
		processing.join();
	}

	delete ipCam;
	
	//destroy GUI window
	destroyAllWindows();
	
	return EXIT_SUCCESS;
}

