#ifndef EKFOA_H_
#define EKFOA_H_

#include <stdio.h> //sprintf

#include <iostream> //cout
#include <vector>   //vector
#include <opencv2/opencv.hpp> //opencv

//HOME DIR:
#include <unistd.h>
#include <sys/types.h>
#include <pwd.h> //getpwuid

#include "camera.hpp"
#include "kalman.hpp"
#include "motion_tracker_of.hpp"
#include "gui.hpp"

class EKFOA {
private:
	Camera cam;
	Kalman filter;
	cv::Mat frame;
	MotionTrackerOF motion_tracker;
public:
	EKFOA();
	void start();
	const Kalman & kalman_filter() const { return filter; }
};

#endif
