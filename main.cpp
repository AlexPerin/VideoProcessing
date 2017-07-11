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
	//cout << "Launched by thread " << endl;
	Mat frame;
	if (!fluxCam.read(frame)) {
		cerr << "Unable to read next frame." << endl;
		cerr << "Exiting..." << endl;
		exit(EXIT_FAILURE);
	}
	

}

int main() {


	Mat frame; //current frame

	CvPoint barycentre;
	Mat binaryFrame; 
	char keyboard;
	Mat elementErode = getStructuringElement(MORPH_ELLIPSE,	Size(4,3), Point(0,0));
	Mat elementDilate = getStructuringElement(MORPH_ELLIPSE, Size(3,2),	Point(0,0));
	int nbFr = 0;
	camera * ipCam = new camera();	
	VideoCapture fluxCam =ipCam->getVideo();

	vector<Vec4i> hierarchy;
	vector<Vec3f> vecCircles;
	vector<Vec3f>::iterator itrCircles;

	Mat binaryImage;
	Ptr<BackgroundSubtractorMOG2> pMOG2; //MOG2 Background subtractor
	pMOG2 = createBackgroundSubtractorMOG2(700, 64, false); //MOG2 approach
	pMOG2->setNMixtures(3);
	
	int a=0;

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

	if (!fluxCam.isOpened()) {
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

		//resize(frame, frame, Size(640, 360), 0, 0, INTER_CUBIC);
		//resize(frame, frame, Size(960, 540), 0, 0, INTER_CUBIC);

		//frame = ipCam->detectAndDisplay(frame, body_cascade); 

		binaryFrame = ipCam->binairisation(frame, binaryImage, pMOG2, elementErode, elementDilate);
		barycentre = ipCam->calculBarycentre(binaryFrame);
		frame = ipCam->tracking(frame, barycentre); 
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

