#ifndef		OBSTACLE_DETECTION_H
#define		OBSTACLE_DETECTION_H



#include "highgui.h"
#include "cv.h"
#include "stdio.h"





void	initGlobals();
void	skySegment(IplImage *frame);
bool	selectHorizonLineByCost(IplImage* img, CvPoint p1, CvPoint p2);
CvPoint	findHorizon(IplImage *img);
void	avoidDirection(IplImage *img, CvPoint delta);
void	cleanUp();


extern bool  output;
extern bool  flag;

#endif