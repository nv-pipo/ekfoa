
#include <boost/thread.hpp>   // boost::thread

#include "ekfoa.hpp"
#include "gui.hpp"

#include <opencv2/highgui/highgui.hpp> //imread

void ekfoa(){
	EKFOA ekfoa;
	cv::Mat frame;
	//Sequence path and initial image
//	std::string sequence_prefix = std::string(getpwuid(getuid())->pw_dir) + "/btsync/capture_samples/monoSLAM/ekfmonoslam/rawoutput";
//	std::string sequence_prefix = std::string(getpwuid(getuid())->pw_dir) + "/btsync/capture_samples/monoSLAM/1394/downsample/img";
//	std::string sequence_prefix = std::string(getpwuid(getuid())->pw_dir) + "/btsync/capture_samples/monoSLAM/ardrone/wall/img";
	std::string sequence_prefix = std::string(getpwuid(getuid())->pw_dir) + "/btsync/capture_samples/monoSLAM/ardrone/rotation/img";
//	std::string sequence_prefix = std::string(getpwuid(getuid())->pw_dir) + "/btsync/capture_samples/monoSLAM/ardrone/corridor_1/img";
//	std::string sequence_prefix = std::string(getpwuid(getuid())->pw_dir) + "/btsync/capture_samples/monoSLAM/ardrone/fly_indoors/img";
	int initIm = 50;
	int lastIm = 408;
	double delta_t = 1; //TODO: take time delta from timestamp of when the image was taken

	char file_path[255]; // enough to hold all numbers up to 64-bits

	cv::namedWindow("Camera input", cv::WINDOW_AUTOSIZE );
	cv::moveWindow("Camera input", 1040, 0);

	Eigen::Matrix3d axes_orientation_and_confidence;
	std::list<Eigen::Vector3d> trajectory;

	std::vector<Point3d> XYZs[3]; // for positions of 'mu', 'close' and 'far'


	for (int step=initIm+1 ; step<lastIm ; step++){
		std::cout << "step: " << step << std::endl;

//		sprintf(file_path, "%s%04d.pgm", sequence_prefix.c_str(), step);
		sprintf(file_path, "%s%03d.png", sequence_prefix.c_str(), step);
		frame = cv::imread(file_path, CV_LOAD_IMAGE_COLOR);   // Read the file

		Delaunay triangulation;
		Point3d closest_point;
		Eigen::Vector3d position;
		//Add a space for the current position in the trajectory list:
		ekfoa.process(delta_t, frame, position, axes_orientation_and_confidence, XYZs, triangulation, closest_point);
		trajectory.push_back(position);

		//Show the processed frame:
		cv::imshow("Camera input", frame);

		Gui::update_draw_parameters(trajectory, axes_orientation_and_confidence, XYZs, triangulation, closest_point);
		//PAUSE:
		std::cin.ignore(1);
	}
}

int main(int argc, char** argv){
	//initialize the OpenGL gui:
	Gui::init();

	//Start a thread for the Extended Kalman Filter:
    boost::thread ekfoa_thread (ekfoa);

	bool keep_going = true;
    while (keep_going){
    	keep_going = Gui::redraw();
    }

    ekfoa_thread.join();

	Gui::release();
}

