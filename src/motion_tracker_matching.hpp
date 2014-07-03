#ifndef MOTION_TRACKER_MATCHING_H_
#define MOTION_TRACKER_MATCHING_H_

#include "motion_tracker.hpp"

#include <opencv2/features2d/features2d.hpp> //cv::GoodFeaturesToTrackDetector, cv::FeatureDetector, cv::DescriptorExtractor, cv::DescriptorMatcher

class MotionTrackerMatching: public MotionTracker {
public:
	MotionTrackerMatching(int min_number_of_features_in_image, int distance_between_points);

	std::string type();

	void process(cv::Mat & input, std::vector<Features_extra> & features_extra, std::vector<cv::Point2f> & features_added);

	~MotionTrackerMatching(){}
private:

	int min_number_of_features_in_image_;
	int distance_between_points_;

	std::vector<cv::KeyPoint> keypoints_1, keypoints_2;
	cv::Mat descriptors_1, descriptors_2;
	std::vector<cv::DMatch> matches;

	cv::Ptr<cv::FeatureDetector> detector;
	cv::Ptr<cv::DescriptorExtractor> extractor;
	cv::Ptr<cv::DescriptorMatcher> matcher;

//
//	cv::Rect image_dimensions_;
//
//	// '1' refers to previous frame
//	// '2' refers to last received (input parameter) frame
//
//	cv::Mat input_1_gray_;
//	cv::Mat points_correctly_tracked_mask_; //Used to filter out currently used locations of the image, so that new features do not overlap with existing ones.
//
//	std::vector<cv::Point2f> points_tracked_1;
//	std::vector<cv::Point2f> points_tracked_2;
//	std::vector<cv::Point2f> points_tracked_1_reverse;
//
//	std::vector<uchar> status_of_; // status of tracked features (Optical Flow)
//	std::vector<uchar> status_of_reverse_; // status of tracked features (Optical Flow second)
//	std::vector<float> err;    // error of tracked features (Optical Flow)
//
//	bool accept_tracked_point(size_t i);
};

#endif
