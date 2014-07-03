#include "motion_tracker_matching.hpp"

std::string MotionTrackerMatching::type(){
	return std::string("Matching");
}

MotionTrackerMatching::MotionTrackerMatching(int min_number_of_features_in_image, int distance_between_points) : min_number_of_features_in_image_(min_number_of_features_in_image),
		distance_between_points_(distance_between_points){
//	detector = cv::FeatureDetector::create("ORB");;
	detector = new cv::OrbFeatureDetector(min_number_of_features_in_image_);
//	detector = new cv::SIFT(min_number_of_features_in_image_);
//	detector = cv::FeatureDetector::create("SIFT");;
//	detector = new cv::GoodFeaturesToTrackDetector(
//			min_number_of_features_in_image_, //int maxCorners=1000
//			0.01, ///double qualityLevel=0.01
//			distance_between_points_, //double minDistance
//			3, //int blockSize=3
//			true, //bool useHarrisDetector=false
//			0.04// double k=0.04
//	);

	// DESCRIPTOR (ORB)
//	extractor = new cv::OrbDescriptorExtractor;
	extractor = cv::DescriptorExtractor::create("ORB");
//	extractor = cv::DescriptorExtractor::create("SIFT");

	//Note: One of NORM_L1, NORM_L2, NORM_HAMMING, NORM_HAMMING2. L1 and L2 norms are preferable choices for SIFT and SURF descriptors, NORM_HAMMING should be used with ORB, BRISK and BRIEF, NORM_HAMMING2 should be used with ORB when WTA_K==3 or 4 (see ORB::ORB constructor description).
	matcher = new cv::BFMatcher(cv::NORM_L2);
}

void MotionTrackerMatching::process(cv::Mat & input, std::vector<Features_extra> & features_extra, std::vector<cv::Point2f> & features_added){
	// '1' refers to previous frame
	// '2' refers to last received (input parameter) frame

//	assert(features_extra.size() == points_tracked_1.size());

	double time;
	std::vector<cv::Point2f> features_tracked;

	cv::Mat inputGray;

	//Convert to Grayscale
	cv::cvtColor(input, inputGray, CV_RGB2GRAY);

	time = (double)cv::getTickCount();
	detector->detect(inputGray, keypoints_2);
	time = (double)cv::getTickCount() - time;
	std::cout << "Detector = " << time/((double)cvGetTickFrequency()*1000.) << "ms" << std::endl;

	time = (double)cv::getTickCount();
	extractor->compute(inputGray, keypoints_2, descriptors_2);
	time = (double)cv::getTickCount() - time;
	std::cout << "Extractor = " << time/((double)cvGetTickFrequency()*1000.) << "ms" << std::endl;

	//If have keypoints of a previous frame:
	if (keypoints_1.size()>0){

		time = (double)cv::getTickCount();
		matcher->match(descriptors_1, descriptors_2, matches);
		time = (double)cv::getTickCount() - time;
		std::cout << "Matcher = " << time/((double)cvGetTickFrequency()*1000.) << "ms" << std::endl;

		time = (double)cv::getTickCount();
		std::vector<cv::Point2f> points_1(matches.size());
		std::vector<cv::Point2f> points_2(matches.size());

		for(size_t i=0; i < matches.size() ; i++) {
			const cv::DMatch & match = matches[i];
			points_1[i] = keypoints_1[match.queryIdx].pt;
			points_2[i] = keypoints_2[match.trainIdx].pt;
		}

		std::vector<uchar> status_ransac;

		cv::findFundamentalMat(points_1, points_2, cv::FM_RANSAC, 0.1, 0.99, status_ransac);
		time = (double)cv::getTickCount() - time;
		std::cout << "RANSAC = " << time/((double)cvGetTickFrequency()*1000.) << "ms" << std::endl;


		cv::Scalar color;
		for(size_t i=0; i < points_1.size() ; i++) {
			color = cv::Scalar(0, 0, 255, 255);//red

			double x_diff = points_2[i].x - points_1[i].x;
			double y_diff = points_2[i].y - points_1[i].y;
//			if (status_ransac[i]){
			if (std::sqrt(x_diff*x_diff+y_diff+y_diff) < 6){
				//Could track it!, so add current position as sensed input:
//				features_extra[i+counter_removed_of].z(0) = p2[i].x;
//				features_extra[i+counter_removed_of].z(1) = p2[i].y;
//				features_extra[i+counter_removed_of].z_cv = p2[i];

				features_tracked.push_back(points_2[i]);

				//make sure new features are not above or too close to this feature:
//				cv::circle(points_correctly_tracked_mask_, p2[i], distance_between_points_, cv::Scalar(0), CV_FILLED);

				//color it in  the frame as green:
				color = cv::Scalar(0, 255, 0, 255);//green


				//Write the feature index next to it:
				std::stringstream text;
				text << features_tracked.size()-1;
				cv::Point2f text_start(points_2[i].x+5, points_2[i].y+5);
				cv::putText(input, text.str(), text_start, cv::FONT_HERSHEY_SIMPLEX, 0.5, color);


				//Draw circle at current position:
				cv::circle(input, points_2[i], 3, color, 1);

				//Draw line between start position and end position:
				cv::line(input,
						points_1[i],   // initial position
						points_2[i],   // new position
						color);
//			} else {
//				//Feature disappeared from image or was not correctly tracked, so mark it for deletion:
//				features_extra[i+counter_removed_of].is_valid = false;
			}



		}
		std::cout << "features_tracked.size(): " << features_tracked.size() << std::endl;
		std::cout << "points_1.size(): " << points_1.size() << std::endl;
	}

//		time = (double)cv::getTickCount() - time;
//		std::cout << "time OF = " << time/((double)cvGetTickFrequency()*1000.) << "ms" << std::endl;

//		std::vector<cv::Point2f> p1;
//		std::vector<cv::Point2f> p2;
//		std::vector<uchar> status_ransac;
//		std::vector<size_t> removed_of;
//
//		cv::findFundamentalMat(p1, p2, cv::FM_RANSAC, 0.5, 0.99, status_ransac);
//

//
//		size_t counter_removed_of=0;//counter_removed_of is keeps track of the number of features smaller than the current feature

//
//		//Finally, remember the correctly tracked points (for next call):
//		points_tracked_1 = features_tracked;
//	}
//
//	int num_new_features = min_number_of_features_in_image_ - features_tracked.size();
//
//	if (num_new_features > 0){
//		//TODO: Try to use some FAST heuristics to prefer unoccupied areas for new features...like a grid and one feature per square.
////		time = (double)cv::getTickCount();
//		cv::goodFeaturesToTrack(
//				input_2_gray,      // InputArray image
//				features_added, // OutputArray corners
//				num_new_features,  // int maxCorners - Number of points to detect
//				0.01,              // double qualityLevel=0.01 (larger is better quality)
//				distance_between_points_, // double minDistance=1
//				points_correctly_tracked_mask_,    // InputArray mask=noArray(). Where it should not look for new features
//				3,              // int blockSize=3
//				true,             // bool useHarrisDetector=false
//				0.04              // double k=0.04
//		);
//		//Add new points to the currently tracked features at the beginning:
//		points_tracked_1.insert(points_tracked_1.end(), features_added.begin(), features_added.end());
//
//		//Draw the newly added features in blue:
//		for (size_t i=0 ; i<features_added.size() ; i++){
//			cv::circle(input_2, features_added[i], 3, cv::Scalar(255,0,0), 1);
//			std::stringstream text;
//			text << features_tracked.size() + i;
//			cv::Point2f text_start(features_added[i].x+5, features_added[i].y+5);
//			cv::putText(input_2, text.str(), text_start, cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(255,0,0));
//		}

//		time = (double)cv::getTickCount() - time;
//		std::cout << "goodFeaturesToTrack = " << time/((double)cvGetTickFrequency()*1000.) << "ms" << std::endl;
//	}

	//Current keypoints/descriptors '_2' become next iteration's '_1':
	keypoints_1 = keypoints_2;
	descriptors_1 = descriptors_2;

}
