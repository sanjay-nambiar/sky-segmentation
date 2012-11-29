
#include "ObstacleDetection.h"






// camera input dimensions
#define	FRAME_WIDTH			640
#define	FRAME_HEIGHT		480

bool			output = false, flag = false;
IplImage		*dst, *out, *temp;										//image variables for processing
IplImage		*tmp1, *tmp2;
IplImage		*Bimg, *Rimg, *Gimg, *mask, *merge;						//image variables for splitting, masking and merging
IplImage		*regions[9];											// 9 image regions through which the plane will pass

unsigned long	min_region_cost;										//	minimum cost of the region through which plane can pass
double			horizon_a,horizon_b, horizon_x0, horizon_y0;			// varibles for hough transform
float			*horizon_line;
float			horizon_rho, horizon_theta;
float			area, horizon_angle;									// obstacle area and horizon line angle

CvPoint			horizon_pt1, horizon_pt2, horizon_p1, horizon_p2;		//variables for horizon
CvPoint			poly[4], centroid;										// centroid of obstacles and fill poly polygon.
CvPoint			rod[4]={{320,0}, {374,0}, {367,370}, {323,370}};		// erasing the rod
CvPoint			final_pt[4] = {{0, FRAME_HEIGHT/2}, {FRAME_WIDTH-1, FRAME_HEIGHT/2}, {FRAME_WIDTH-1, FRAME_HEIGHT-1}, {0, FRAME_HEIGHT-1}};	//below horizon

CvRect			horizon_r;												// obstacle bounding rect



//	memory storage and seqence pointers for contours and hough transform
CvMemStorage	*horizon_storage = cvCreateMemStorage(0);
CvMemStorage	*contour_storage = cvCreateMemStorage(0);
CvSeq			*horizon_lines;
CvSeq			*contours, *ptr;



//	pointers for list traversal and cost for error functions and heading
unsigned char	*basePtr, *iterator;
unsigned long	current_cost = 0, cost = 0xFFFFFFFF;
unsigned int	count = 0, heading_region;







// allocate all the image variables

void initGlobals()
{
	dst = cvCreateImage(cvSize(FRAME_WIDTH,FRAME_HEIGHT), IPL_DEPTH_8U, 1);
	out = cvCreateImage(cvSize(FRAME_WIDTH,FRAME_HEIGHT), IPL_DEPTH_8U, 1);
	temp = cvCreateImage(cvSize(FRAME_WIDTH,FRAME_HEIGHT), IPL_DEPTH_8U, 1);

	tmp1 = cvCreateImage(cvSize(FRAME_WIDTH,FRAME_HEIGHT), IPL_DEPTH_8U, 1);
	tmp2 = cvCreateImage(cvSize(FRAME_WIDTH,FRAME_HEIGHT), IPL_DEPTH_8U, 1);

	merge = cvCreateImage(cvSize(FRAME_WIDTH,FRAME_HEIGHT), IPL_DEPTH_8U, 3);
	Bimg = cvCreateImage(cvSize(FRAME_WIDTH,FRAME_HEIGHT), IPL_DEPTH_8U, 1);
	Rimg = cvCreateImage(cvSize(FRAME_WIDTH,FRAME_HEIGHT), IPL_DEPTH_8U, 1);
	Gimg = cvCreateImage(cvSize(FRAME_WIDTH,FRAME_HEIGHT), IPL_DEPTH_8U, 1);
	mask = cvCreateImage(cvSize(FRAME_WIDTH,FRAME_HEIGHT), IPL_DEPTH_8U, 1);

	for(int i=0; i<9; i++)
		regions[i] = cvCreateImage(cvSize(FRAME_WIDTH/6,FRAME_HEIGHT/6), IPL_DEPTH_8U, 1);

}





//	segment sky and non sky - splits image into 3 channels, thresolds channel B and use it as a mask
//	use the created mask to multiply with all the 3 channels, and merge the image.

void skySegment(IplImage *frame)
{
	cvSplit(frame, Bimg, Gimg, Rimg, NULL);
	cvThreshold(Bimg, mask, 150, 255, CV_THRESH_BINARY_INV);
	cvMul(Bimg, mask, Bimg);
	cvMul(Gimg, mask, Gimg);
	cvMul(Rimg, mask, Rimg);
	cvMerge(Bimg, Gimg, Rimg, NULL, merge);
}








//	find the value of error function associated with a horizontal line and returns true if
//	current line is the best line so far.

bool selectHorizonLineByCost(IplImage* img, CvPoint p1, CvPoint p2)
{
	cvCopyImage(img, tmp1);
	cvCopyImage(img, tmp2);
	poly[0] = p1;
	poly[1] = p2;
	poly[2] = cvPoint(FRAME_WIDTH-1,FRAME_HEIGHT-1);
	poly[3] = cvPoint(0,FRAME_HEIGHT-1);
	cvFillConvexPoly(tmp1, poly, 4, CV_RGB(0,0,0));				// create image with below current horizon line removed
	poly[2] = cvPoint(FRAME_WIDTH-1, 0);
	poly[3] = cvPoint(0, 0);
	cvFillConvexPoly(tmp2, poly, 4, CV_RGB(255,255,255));		// create image with above current horizon line removed


	// find the total error function value
	current_cost = 0;
	basePtr = (unsigned char*)tmp1->imageData;
	for( int j=0; j<tmp1->height; basePtr += tmp1->widthStep, j++ )
	{
		iterator = basePtr;
		for( int i=0; i<tmp1->width; iterator++, i++ )
			if( *iterator == 255 )
				current_cost++;
	}

	basePtr = (unsigned char*)tmp2->imageData;
	for( int j=0; j < tmp2->height; basePtr += tmp2->widthStep, j++ )
	{
		iterator = basePtr;
		for( int i=0; i<tmp2->width; iterator++, i++ )
			if( *iterator == 0 )
				current_cost++;
	}
	if( current_cost < cost )				// check if it is the best so far and return true or false accordingly
		return true;
	else return false;
}



// Find the intersecting obstacles

CvPoint findIntersect(CvRect A, CvRect B)
{
	CvPoint delta = {0,0};
	if( B.x <= A.x + A.width && B.x + B.width >= A.x && B.y <= A.y + A.height && B.y + B.height >= A.y)
	{
		if( A.x > B.x )
			delta.x = B.x + B.width - A.x;
		else if( B.x + B.width > A.x + A.width )
			delta.x = B.x - A.x - A.width;
		else
		{
			delta.x = (( A.x + A.width )/2) - ((B.x + B.width)/2);
			if( delta.x >= 0 )
				delta.x = B.x + B.width - A.x;
			else
				delta.x = B.x - A.x - A.width;
		}

		if( A.y > B.y )
			delta.y = B.y + B.height - A.y;
		else if( B.y + B.height > A.y + A.height )
			delta.y = B.y - A.y - A.height;
		else
		{
			delta.y = (( A.y + A.height )/2) - ((B.y + B.height)/2);
			if( delta.y >= 0 )
				delta.y = B.y + B.height - A.y;
			else
				delta.y = B.y - A.y - A.width;
		}
	}
	return delta;
}




//	Finds horizon and detects obstacles and calculates heading.

CvPoint findHorizon(IplImage *img)
{
	// preprocessing the segmented image
	if( output == true && count%2 == 0)
		cvSaveImage("Original.jpg", img);
	skySegment(img);
	cvFillConvexPoly(merge, rod, 4, cvScalar(0,0,0));
	cvCvtColor(merge, dst, CV_RGB2GRAY);
	cvThreshold(dst, out, 100, 255, CV_THRESH_BINARY);
	cvErode(out, temp);
	cvDilate(temp, out);

	// find horizon every 4 frames
	if( count % 2 == 0 )
	{
		cvSmooth(out, temp, CV_GAUSSIAN);
		cvInRangeS(temp, cvScalar(50,0,0), cvScalar(150,0,0), temp);			// select only border pixels

		horizon_storage = cvCreateMemStorage(0);
		horizon_lines = cvHoughLines2( temp, horizon_storage, CV_HOUGH_STANDARD, 1, CV_PI/180, 100, 0, 0 );		// hough tranform to detect lines

		if( output == true )
		{
			cvSaveImage("Original.jpg", img);
				cvCopy(img, merge);
		}

		for(int i=0; i < MIN(horizon_lines->total,100); i++ )
		{
			horizon_line = (float*)cvGetSeqElem(horizon_lines,i);
			horizon_rho = horizon_line[0];
			horizon_theta = horizon_line[1];

			horizon_a = cos(horizon_theta);
			horizon_b = sin(horizon_theta);
			horizon_x0 = horizon_a*horizon_rho;
			horizon_y0 = horizon_b*horizon_rho;

			horizon_pt1.x = cvRound(horizon_x0 + 1000*(-horizon_b));
			horizon_pt1.y = cvRound(horizon_y0 + 1000*(horizon_a));
			horizon_pt2.x = cvRound(horizon_x0 - 1000*(-horizon_b));
			horizon_pt2.y = cvRound(horizon_y0 - 1000*(horizon_a));


			// check the current line's bounds and check if it is the best line so far using the error function
			if( horizon_pt1.x <= 0 && horizon_pt2.x >= FRAME_WIDTH-1 && horizon_pt1.y > 0 && horizon_pt2.y < FRAME_HEIGHT-1)
			{
				if( selectHorizonLineByCost(out, horizon_pt1, horizon_pt2) == true )
				{
					final_pt[0] = horizon_pt1;
					final_pt[1] = horizon_pt2;
				}
				if( output == true )
					cvLine( merge, horizon_pt1, horizon_pt2, CV_RGB(0, 255,0), 2);
			}
		}
		if( output == true )
		{
			cvSaveImage("Binary.jpg", out);
			cvSaveImage("Boundary.jpg", temp);
			cvSaveImage("Lines.jpg", merge);
			flag = true;
		}
	}
	count++;


	// find the obstacles and draw their bounding rects

	cvFillConvexPoly(out, final_pt, 4, cvScalar(0,0,0));
	contour_storage = cvCreateMemStorage(0);
	cvFindContours(out, contour_storage, &contours, sizeof(CvContour), CV_RETR_EXTERNAL, CV_CHAIN_APPROX_SIMPLE);
	CvPoint delta={0,0},temp;
	int obst_count = 0;

	for( ptr = contours; ptr != NULL; ptr = ptr->h_next )
	{
		area = (float)fabs(cvContourArea(ptr));
		if( area > 50 )
		{
			horizon_r = cvBoundingRect(ptr, 1);
			horizon_p1.x = horizon_r.x;
			horizon_p1.y = horizon_r.y;
			horizon_p2.x = horizon_r.x + horizon_r.width;
			horizon_p2.y = horizon_r.y + horizon_r.height;
			centroid.x = horizon_r.x + cvRound(horizon_r.width/2);
			centroid.y = horizon_r.y + cvRound(horizon_r.height/2);
			cvRectangle(img, horizon_p1, horizon_p2, CV_RGB(255,0,0));
			temp = findIntersect( cvRect((FRAME_WIDTH/4), (FRAME_HEIGHT/4-1),3*FRAME_WIDTH/4-2,(3*FRAME_HEIGHT/4-1)), horizon_r);
			delta.x += temp.x;
			delta.y += temp.y;
			obst_count++;
		}
	}
	cvRectangle(out, cvPoint((FRAME_WIDTH/4), (FRAME_HEIGHT/4-1)), cvPoint(3*FRAME_WIDTH/4-2,(3*FRAME_HEIGHT/4-1)), CV_RGB(255,255,255));
	cvLine(img,final_pt[0], final_pt[1], CV_RGB(0,255,0), 4);
	//horizon_angle = atan( (float)(final_pt[0].y - final_pt[1].y) / (final_pt[0].x - final_pt[1].x) );

	printf("Obstacles detected = %d\n", obst_count);
	if( delta.x > 0 || delta.y > 0)
	{
		printf("To avoid move ");
		if( delta.x > 0 )
			printf("%d units right", delta.x);
		else
			printf("%d units left", -(delta.x));
		if( delta.x != 0 )
			printf(" and ");
		if( delta.y > 0 )
			printf("%d units up", delta.y);
		else
			printf("%d units down", -(delta.y));
	}
	else
		printf("Stay on course.");
	printf("\n\n");

	return delta;
}




void avoidDirection(IplImage *img, CvPoint delta)
{
	if( delta.y > 0 )
	{
		cvLine(img, cvPoint(FRAME_WIDTH/2 , FRAME_HEIGHT-1), cvPoint(FRAME_WIDTH/2 , FRAME_HEIGHT-51), CV_RGB(255, 0,0),8);
		cvLine(img, cvPoint(FRAME_WIDTH/2-10 , FRAME_HEIGHT-51+10), cvPoint(FRAME_WIDTH/2 , FRAME_HEIGHT-51), CV_RGB(255, 0,0),8);
		cvLine(img, cvPoint(FRAME_WIDTH/2+10 , FRAME_HEIGHT-51+10), cvPoint(FRAME_WIDTH/2 , FRAME_HEIGHT-51), CV_RGB(255, 0,0),8);
	}
	else if( delta.y < 0 )
	{
		cvLine(img, cvPoint(FRAME_WIDTH/2 , FRAME_HEIGHT-1), cvPoint(FRAME_WIDTH/2 , FRAME_HEIGHT-51), CV_RGB(255, 0,0),8);
		cvLine(img, cvPoint(FRAME_WIDTH/2-10 , FRAME_HEIGHT-1-10), cvPoint(FRAME_WIDTH/2 , FRAME_HEIGHT-1), CV_RGB(255, 0,0),8);
		cvLine(img, cvPoint(FRAME_WIDTH/2+10 , FRAME_HEIGHT-1-10), cvPoint(FRAME_WIDTH/2 , FRAME_HEIGHT-1), CV_RGB(255, 0,0),8);
	}

	if( delta.x > 0 )
	{
		cvLine(img, cvPoint(FRAME_WIDTH/2+55 , FRAME_HEIGHT-25), cvPoint(FRAME_WIDTH/2+5, FRAME_HEIGHT-25), CV_RGB(255, 0,0),8);
		cvLine(img, cvPoint(FRAME_WIDTH/2+45 , FRAME_HEIGHT-35), cvPoint(FRAME_WIDTH/2+55, FRAME_HEIGHT-25), CV_RGB(255, 0,0),8);
		cvLine(img, cvPoint(FRAME_WIDTH/2+45 , FRAME_HEIGHT-15), cvPoint(FRAME_WIDTH/2+55, FRAME_HEIGHT-25), CV_RGB(255, 0,0),8);
	}
	else if( delta.x < 0 )
	{
		cvLine(img, cvPoint(FRAME_WIDTH/2-55 , FRAME_HEIGHT-25), cvPoint(FRAME_WIDTH/2-5, FRAME_HEIGHT-25), CV_RGB(255, 0,0),8);
		cvLine(img, cvPoint(FRAME_WIDTH/2-45 , FRAME_HEIGHT-35), cvPoint(FRAME_WIDTH/2-55, FRAME_HEIGHT-25), CV_RGB(255, 0,0),8);
		cvLine(img, cvPoint(FRAME_WIDTH/2-45 , FRAME_HEIGHT-15), cvPoint(FRAME_WIDTH/2-55, FRAME_HEIGHT-25), CV_RGB(255, 0,0),8);
	}
}






// cleanup all the image variables

void cleanUp()
{
	cvReleaseImage(&out);
	cvReleaseImage(&temp);
	cvReleaseImage(&dst);
	cvReleaseImage(&merge);
	cvReleaseImage(&mask);
	cvReleaseImage(&Bimg);
	cvReleaseImage(&Gimg);
	cvReleaseImage(&Rimg);
}

