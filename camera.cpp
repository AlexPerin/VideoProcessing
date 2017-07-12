#include "camera.h"



camera::camera()
{
	//elementErode = getStructuringElement(MORPH_ELLIPSE, Size(4, 3), Point(0, 0));
	//elementDilate = getStructuringElement(MORPH_ELLIPSE, Size(3, 2), Point(0, 0));
	//pMOG2 = createBackgroundSubtractorMOG2(700, 64, false); //MOG2 approach	
	nbPixels = 0;
	barycentre = (-1 , -1);
	x1 = 1;
	x2 = 1;
	y1 = 1;
	y2 = 1;
	theta = 0;      // in degrees
	total=0;
	rectSizeRate = 1;
	T[11] = { 1 };
}



camera::~camera() {
}

VideoCapture camera::getVideo() {

	//create the capture object
	VideoCapture capture("rtsp://admin:admin@10.35.127.245/live.sdp");
	//VideoCapture capture("test_1.mp4");
	//VideoCapture capture(0);
	
	return capture;

}

Mat camera::binairisation(Mat image, Mat b_image, Ptr<BackgroundSubtractorMOG2> pMOG2, Mat elementErode, Mat elementDilate) {
	pMOG2->apply(image, b_image, -1);
	erode(b_image, b_image, elementErode);
	dilate(b_image, b_image, elementDilate);
	return b_image;
}


void camera::calculBarycentre(Mat image) {
	int sommeX = 0;
	int sommeY = 0;
	nbPixels = 0;
	int largeur = image.cols;
	int hauteur = image.rows;
	int i = 0;
	int j = 0;
	for (i = 0; i<(hauteur); i++) {
		for (j = 0; j<(largeur); j++) {
			if (image.at<unsigned char>(i, j) == 255) {
				sommeX += j;
				sommeY += i;
				nbPixels++;
			}
		}
	}


	if (nbPixels > 300) {
		barycentre.x = (int)(sommeX / nbPixels);
		barycentre.y = (int)(sommeY / nbPixels);	
	}

	else {
		barycentre.x = -1;
		barycentre.y = -1;
	}
}

Mat camera::tracking(Mat image) {

	int objectNextStepX, objectNextStepY;
	CvPoint positionAct = barycentre;

	//s'il y a assez de pixels binairisé en blanc on calcul la prochaine position du cercle
 	if (nbPixels > 300) {

		// si le barycentre est hors de l'image on ne change pas sa position
		// sinon on change pas à pas la position de l'object vers la position désiriée
		if (abs(positionAct.x - barycentre.x) > 5) {
			objectNextStepX = max(5, min(100, abs(positionAct.x - barycentre.x) / 2));
			if ((positionAct.x - barycentre.x) < 0) {
				positionAct.x += objectNextStepX;
			}
			else {
				positionAct.x -= objectNextStepX;
			}
		}
		else if (abs(positionAct.y - barycentre.y) > 5) {
			objectNextStepY = max(5, min(100, abs(positionAct.y - barycentre.y) / 2));
			if ((positionAct.y - barycentre.y) < 0) {
				positionAct.y += objectNextStepY;
			}
			else {
				positionAct.y -= objectNextStepY;
			}
		}
		//cout << "X = " << barycentre.x << "y = " << barycentre.y << endl;
		circle(image, positionAct, 10, CV_RGB(255, 0, 0), -1);

	}
		
	else {
		positionAct.x = -1;
		positionAct.y = -1;
	}  

	return	image;

}


void camera::affiche(Mat image, string nameWindow, VideoCapture capture) {
	//get the frame number and write it on the current frame
	stringstream ss;
	rectangle(image, cv::Point(10, 2), cv::Point(100, 20),
		cv::Scalar(255, 255, 255), -1);
	ss << capture.get(CAP_PROP_POS_FRAMES);
	string frameNumberString = ss.str();
	putText(image, frameNumberString.c_str(), cv::Point(15, 15),
		FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(0, 0, 0));
	
	imshow(nameWindow, image);
}

int camera::getPx() {

	return nbPixels;
}


Mat camera::detectAndDisplay(Mat frame, CascadeClassifier body_cascade) { // CascadeClassifier upper_body_cascade) {

	vector<Rect> u_b;
	Mat frame_gray;
	cvtColor(frame, frame_gray, COLOR_BGR2GRAY);
	equalizeHist(frame_gray, frame_gray);

	//-- Detect faces
	body_cascade.detectMultiScale(frame_gray, u_b, 1.1, 3, 0 | CASCADE_SCALE_IMAGE, Size(30, 30));

	for (size_t i = 0; i < u_b.size(); i++) {
		Point center(u_b[i].x + u_b[i].width / 2, u_b[i].y + u_b[i].height / 2);
		ellipse(frame, center, Size(u_b[i].width / 2, u_b[i].height / 2), 0, 0, 360, Scalar(255, 0, 255), 4, 8, 0);
		Mat faceROI = frame_gray(u_b[i]);
/*		vector<Rect> eyes;

			//-- In each face, detect eyes
			upper_body_cascade.detectMultiScale(faceROI, eyes, 1.3, 2, 0 | CASCADE_SCALE_IMAGE, Size(30, 30));
			for (size_t j = 0; j < eyes.size(); j++)
			{
				Point eye_center(eyes[i].x + eyes[j].x + eyes[j].width / 2, eyes[i].y + eyes[j].y + eyes[j].height / 2);
				int radius = cvRound((eyes[j].width + eyes[j].height)*0.25);
				circle(frame, eye_center, radius, Scalar(255, 0, 0), 4, 8, 0);
			}  */
		} 

		return frame;
}

Mat camera::drawContours(Mat binaryFrame, Mat frame) {

	/* Detection de contours */
	vector<vector<Point> > contours;
	findContours(binaryFrame, contours, CV_RETR_EXTERNAL, CV_CHAIN_APPROX_NONE);

	/* Find the rotated rectangles and ellipses for each contour */
	vector<RotatedRect> minEllipse(contours.size());
	vector<RotatedRect> minRect(contours.size());
	float rectAngle = 0;
	float rectWidth, rectHeight = 0;
	for (int i = 0; i < contours.size(); i++)
	{
		if (contours[i].size() > 700)
		{
			minRect[i] = minAreaRect(Mat(contours[i]));
			minEllipse[i] = fitEllipse(Mat(contours[i]));
			theta = minEllipse[i].angle;
			rectAngle = minRect[i].angle*(-1);
			/* setting the correct width and height values */
			if ((minRect[i].size.width < minRect[i].size.height) && rectAngle<45) {
				rectWidth = minRect[i].size.width;
				rectHeight = minRect[i].size.height;
			}
			else if ((minRect[i].size.width < minRect[i].size.height) && rectAngle>45) {
				rectWidth = minRect[i].size.height;
				rectHeight = minRect[i].size.width;
			}
			else if ((minRect[i].size.width > minRect[i].size.height) && rectAngle>45) {
				rectWidth = minRect[i].size.height;
				rectHeight = minRect[i].size.width;
			}
			else if ((minRect[i].size.width > minRect[i].size.height) && rectAngle<45) {
				rectWidth = minRect[i].size.width;
				rectHeight = minRect[i].size.height;
			}
			rectSizeRate = rectWidth / rectHeight;
			//cout << "rectSizeRate = " << rectSizeRate << endl;
		}
	}

	/*Draw rotated rects + ellipses*/
	for (int i = 0; i < contours.size(); i++) {

		ellipse(frame, minEllipse[i], CV_RGB(121, 248, 248), 1, LINE_AA);
		Point2f rect_points[4]; minRect[i].points(rect_points);


		/*rotated rectangle*/

		for (int j = 0; j < 4; j++) {
			line(frame, rect_points[j], rect_points[(j + 1) % 4], CV_RGB(212, 115, 212), 2, 8);
		}
	}

	return frame;
}


void camera::trajectory() {
	x2 = x1;
	y2 = y1;
	x1=barycentre.x;
	y1=barycentre.y;
	if (barycentre.x != -1 && x1 != x2) {
		T[10] = /*(abs(x1 - x2) / (x1 - x2)) */  ((y1 - y2) / (x1 - x2));
		for (int l = 1; l < 11; l++) {
			T[l - 1] = T[l];
		}
	}
	
	for (int i = 0; i < 11; i++) {
		total = total + T[i];
	}
	
	total = total / 11;
	 
}

void camera::fallDetection() {
	
	/*
	cout << " theta = " << theta << endl;
	cout << " rate = " << rectSizeRate << endl;
	cout << "total = " << total << endl;  */
	
 	if ((total < -1.5)  && (theta <100 && theta > 80) && (rectSizeRate > 1)) {

		cout << "FALL !!!!!! \n" << endl;

	}
	
}

