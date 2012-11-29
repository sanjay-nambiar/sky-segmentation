
#include "ObstacleDetection.h"
#include "cv.h"
#include "highgui.h"


// main function

int main()
{
	CvCapture	*capture;
	IplImage*	frame;

	// initialize and recieve input data
	initGlobals();
	capture = cvCreateFileCapture("video1.avi");
	cvNamedWindow("Obstacle Detection");


	// perform all the operations
	while( true )
	{
		frame = cvQueryFrame(capture);
		if( !frame )
			break;
		avoidDirection(frame, findHorizon(frame));
		cvShowImage("Obstacle Detection", frame);
		if( output == true && flag == true)
		{
			cvSaveImage("Complete.jpg", frame);
			output = false;
		}
		if( cvWaitKey(1) == '1' )
			output = true;
	}
	cvWaitKey(0);

	// cleanup and exit
	cvDestroyWindow("Obstacle Detection");
	cvReleaseImage(&frame);
	cleanUp();

	return 0;
}
