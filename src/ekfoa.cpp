
#include "ekfoa.hpp"

EKFOA::EKFOA() :
cam(Camera(
		//Sample
		0.0112,	  		    //d
		1.7945 / 0.0112,  //Cx
		1.4433 / 0.0112,  //Cy
		6.333e-2, //k1
		1.390e-2, //k2
		2.1735   			//f

		//ARDRONE:
//		0.0112,	  		    //d
//		306.3494790390417392700328491628170013427734375,                    //Cx
//		186.7931235621226733201183378696441650390625,                       //Cy
//		0.01290423394388880151684162456149351783096790313720703125,         //k1
//		0.0004390594570202506897842187338909525351482443511486053466796875, //k2
//		2.1735   			//f
)),
filter(Kalman(
		0.0,   //v_0
		0.025, //std_v_0
		1e-15, //w_0
		0.025, //std_w_0
		0.007, //standar deviation for linear acceleration noise
		0.007, //standar deviation for angular acceleration noise
		1.0    //standar deviation for measurement noise
)),
motion_tracker(MotionTrackerOF(
		30, //min_number_of_features_in_image
		20  //distance_between_points
)) {

}

void EKFOA::start(){
	cv::Mat frame;

	double time;

	//Sequence path and initial image
	std::string sequence_prefix = std::string(getpwuid(getuid())->pw_dir) + "/btsync/capture_samples/monoSLAM/ekfmonoslam/rawoutput";
//	std::string sequence_prefix = std::string(getpwuid(getuid())->pw_dir) + "/btsync/capture_samples/monoSLAM/ardrone/heldIndoors2/img";
	int initIm = 90;
	int lastIm = 2000;

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
		Eigen::MatrixXd features_to_add;
		std::vector<int> features_to_remove;
		std::vector<cv::Point2f> features_tracked;
		sprintf(file_path, "%s%04d.pgm", sequence_prefix.c_str(), step);
//		sprintf(file_path, "%s%03d.png", sequence_prefix.c_str(), step);
		frame = cv::imread(file_path, CV_LOAD_IMAGE_COLOR);   // Read the file

		time = (double)cv::getTickCount();
		motion_tracker.process(frame, features_to_add, features_tracked, features_to_remove);
		//TODO: Why is optical flow returning points outside the image???
		time = (double)cv::getTickCount() - time;
		std::cout << "tracker = " << time/((double)cvGetTickFrequency()*1000.) << "ms" << std::endl;

		//Delete no longer seen features
		time = (double)cv::getTickCount();
		filter.delete_features( features_to_remove );
		time = (double)cv::getTickCount() - time;
		std::cout << "delete = " << time/((double)cvGetTickFrequency()*1000.) << "ms" << std::endl;


		//EKF Update step
		time = (double)cv::getTickCount();
		filter.update(cam, features_tracked);
		time = (double)cv::getTickCount() - time;
		std::cout << "update = " << time/((double)cvGetTickFrequency()*1000.) << "ms" << std::endl;

		//Add new features
		time = (double)cv::getTickCount();
		filter.add_features_inverse_depth( cam, features_to_add );
		time = (double)cv::getTickCount() - time;
		std::cout << "add_features = " << time/((double)cvGetTickFrequency()*1000.) << "ms" << std::endl;


		time = (double)cv::getTickCount();
		std::vector< std::pair<Point, size_t> > triangle_list;

		const Eigen::MatrixXd & p_k_k = filter.p_k_k();
		const Eigen::VectorXd & x_k_k = filter.x_k_k();

		for (int feature_idx=0 ; feature_idx<features_tracked.size() ; feature_idx++){
			int feature_depth_index = 13 + feature_idx*6 + 5;
			if (p_k_k(feature_depth_index, feature_depth_index) < 0.1 && x_k_k(feature_depth_index) > 0)//TODO: Do this nicely...using sigmas.
				triangle_list.push_back( std::make_pair( Point(features_tracked[feature_idx].x, features_tracked[feature_idx].y), feature_idx));
			if (x_k_k(feature_depth_index) < 0 ){
				std::cout << "darn!!! : idx=" << feature_depth_index << ", value=" << x_k_k(feature_depth_index) << std::endl;
				std::cin.ignore(1);
			}
		}

		Delaunay triangulation(triangle_list.begin(), triangle_list.end());
		cv::Scalar delaunay_color = cv::Scalar(255, 0, 0); //blue
		for(Delaunay::Finite_faces_iterator fit = triangulation.finite_faces_begin(); fit != triangulation.finite_faces_end(); ++fit) {
			const Delaunay::Face_handle & face = fit;
			//face->vertex(i)->info() = index of the point in the observation list.
			line(frame, features_tracked[face->vertex(0)->info()], features_tracked[face->vertex(1)->info()], delaunay_color, 1);
			line(frame, features_tracked[face->vertex(1)->info()], features_tracked[face->vertex(2)->info()], delaunay_color, 1);
			line(frame, features_tracked[face->vertex(2)->info()], features_tracked[face->vertex(0)->info()], delaunay_color, 1);
		}

		time = (double)cv::getTickCount() - time;
		std::cout << "delaunay = " << time/((double)cvGetTickFrequency()*1000.) << "ms" << std::endl;

		cv::Mat bigger;
		cv::resize(frame, bigger, cv::Size(frame.size().width*2, frame.size().height*2));

//		cv::imshow("bla", bigger);
		//Notify the gui of the new state:
		Gui::update_state_and_cov(filter.x_k_k(), filter.p_k_k(), frame, triangulation);

		//PAUSE:
		std::cin.ignore(1);
	}
}
