#ifndef EKFOA_H_
#define EKFOA_H_

#include <stdio.h> //sprintf

#include <iostream> //cout
#include <vector>   //vector
#include <opencv2/highgui/highgui.hpp> //imread

//HOME DIR:
#include <unistd.h>
#include <sys/types.h>
#include <pwd.h> //getpwuid

#include "camera.hpp"
#include "kalman.hpp"
#include "motion_tracker_of.hpp"
#include "gui.hpp"

//Delaunay triangulation:
#include <CGAL/Exact_predicates_exact_constructions_kernel.h>
#include <CGAL/Delaunay_triangulation_2.h>
#include <CGAL/Triangulation_vertex_base_with_info_2.h>
typedef CGAL::Exact_predicates_exact_constructions_kernel         K;
typedef CGAL::Triangulation_vertex_base_with_info_2<size_t, K>    Vb;
typedef CGAL::Triangulation_data_structure_2<Vb>                  Tds;
typedef CGAL::Delaunay_triangulation_2<K, Tds>                    Delaunay;
typedef K::Point_2                                                Point2d;

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
