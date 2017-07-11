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

using namespace cv;
using namespace std;

class camera
{
public:
	camera();
	~camera();
	VideoCapture getVideo();
	Mat binairisation(Mat image, Mat b_image, Ptr<BackgroundSubtractorMOG2> pMOG2, Mat erode, Mat dilate);
	CvPoint calculBarycentre(Mat image);
	Mat tracking(Mat image , CvPoint barycentre);
	void affiche(Mat image, string nameWindow, VideoCapture capture);
	int getPx();
	Mat detectAndDisplay(Mat frame, CascadeClassifier body_cascade);// , CascadeClassifier upper_body_cascade);
	Mat enclosingEllipse(Mat binaryFrame, Mat frame);
	void trajectory();
	void fallDetection();
	Mat drawContours(Mat binaryFrame, Mat frame);
private:

	CvPoint barycentre;
	int nbPixels;
	float x1, x2, y1, y2;
	float T[11] = {1,1,1 ,1 ,1 ,1 ,1 ,1 ,1 ,1 ,1};
	float theta;       // in degrees
	float total;
	float moy;
	float rectSizeRate;
	float theta2;


};


