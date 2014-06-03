
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

int main(int argc, char** argv){


	cv::Mat frame;
	MotionTrackerOF motion_tracker(
			50, //min_number_of_features_in_image
			20  //distance_between_points
			);

//	cv::namedWindow("OF");
//	cv::moveWindow("OF", 1000, 100);
//	cv::namedWindow("MASK");
//	cv::moveWindow("MASK", 1000, 600);

	//Sequence path and initial image
//	std::string sequence_prefix = std::string(getpwuid(getuid())->pw_dir) + "/btsync/capture_samples/monoSLAM/ekfmonoslam/rawoutput";
	std::string sequence_prefix = std::string(getpwuid(getuid())->pw_dir) + "/btsync/capture_samples/monoSLAM/ardrone/heldIndoors2/img";
	int initIm = 0;
	int lastIm = 999;


	//Initialize the camera parameters
	Camera cam(
//			0.0112,	  		    //d
//			1.7945 / 0.0112,  //Cx
//			1.4433 / 0.0112,  //Cy
//			6.333e-2, //k1
//			1.390e-2, //k2
//			2.1735   			//f
			0.0112,	  		    //d
			342.5598894437880,  //Cx
			173.0343808455040,  //Cy
			0.0028450345588019, //k1
			0.0000000000000222, //k2
			2.1735   			//f
	);


	//Create a kalman filter with some initial initial velocity values and motion model parameters
	Kalman filter(
			0.0,   //v_0
			0.025, //std_v_0
			1e-15, //w_0
			0.025, //std_w_0
			0.007, //standar deviation for linear acceleration noise
			0.007, //standar deviation for angular acceleration noise
			1.0    //standar deviation for measurement noise
	);

	double time;

	char file_path[255]; // enough to hold all numbers up to 64-bits
	for (int step=initIm+1 ; step<lastIm ; step++){
		std::cout << "step: " << step << std::endl;

		//EKF prediction (state and measurement prediction)
		double delta_t = 1; //TODO: take time delta from timestamp of when the image was taken

		time = (double)cv::getTickCount();
		filter.predict_state_and_covariance(delta_t);
		time = (double)cv::getTickCount() - time;
		std::cout << "predict = " << time/((double)cvGetTickFrequency()*1000.) << "ms" << std::endl;

		//Sense environment (process frame)
		Eigen::MatrixXd features_new_uvds;
		std::vector<int> features_to_remove_sorted_idx;
		std::vector<Observation> features_observations;
//		sprintf(file_path, "%s%04d.pgm", sequence_prefix.c_str(), step);
		sprintf(file_path, "%s%03d.png", sequence_prefix.c_str(), step);
		frame = cv::imread(file_path, CV_LOAD_IMAGE_COLOR);   // Read the file

		time = (double)cv::getTickCount();
		motion_tracker.process(frame, features_new_uvds, features_observations, features_to_remove_sorted_idx);
		time = (double)cv::getTickCount() - time;
		std::cout << "tracker = " << time/((double)cvGetTickFrequency()*1000.) << "ms" << std::endl;
//		cv::imshow("OF", frame);
//		cv::waitKey(10);

		//Delete no longer seen features
		time = (double)cv::getTickCount();
		filter.delete_features( features_to_remove_sorted_idx );
		time = (double)cv::getTickCount() - time;
		std::cout << "delete = " << time/((double)cvGetTickFrequency()*1000.) << "ms" << std::endl;

		//EKF Update step
		time = (double)cv::getTickCount();
		filter.update(cam, features_observations);
		time = (double)cv::getTickCount() - time;
		std::cout << "update = " << time/((double)cvGetTickFrequency()*1000.) << "ms" << std::endl;

		//Add new features
		time = (double)cv::getTickCount();
		filter.add_features_inverse_depth( cam, features_new_uvds );
		time = (double)cv::getTickCount() - time;
		std::cout << "add_features = " << time/((double)cvGetTickFrequency()*1000.) << "ms" << std::endl;

		//PAUSE:
		std::cin.ignore(1);
	}

}
