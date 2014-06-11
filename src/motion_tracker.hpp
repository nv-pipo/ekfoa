
#ifndef MOTION_TRACKER_H_
#define MOTION_TRACKER_H_

#include <string> //string
#include <iostream> //std::out

#include <opencv2/core/core.hpp> //circle, line, Point2f, putText
#include <opencv2/calib3d/calib3d.hpp> //findFundamentalMat
#include <opencv2/imgproc/imgproc.hpp> //cvtColor
#include <opencv2/video/tracking.hpp> //calcOpticalFlowPyrLK

#include <Eigen/Dense> //Matrix

#include <vector>

//MotionTracker interface:
class MotionTracker {
public:
	virtual std::string type() = 0;

	virtual void process(cv::Mat & input_2, Eigen::MatrixXd & features_added, std::vector<cv::Point2f> & features_tracked, std::vector<int> & features_removed) = 0;

	virtual ~MotionTracker(){}
};

#endif
