#ifndef EKFOA_H_
#define EKFOA_H_

#include <stdio.h> //sprintf

#include <utility>  // std::pair
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

#include <CGAL/Simple_cartesian.h>
#include <CGAL/AABB_tree.h>
#include <CGAL/AABB_traits.h>
#include <CGAL/AABB_triangle_primitive.h>
typedef CGAL::Simple_cartesian<double> K_surface;
typedef K_surface::FT FT;
typedef K_surface::Ray_3 Ray;
typedef K_surface::Line_3 Line;
typedef K_surface::Point_3 Point3d;
typedef K_surface::Triangle_3 Triangle;
typedef std::list<Triangle>::iterator Iterator;
typedef CGAL::AABB_triangle_primitive<K_surface, Iterator> Primitive;
typedef CGAL::AABB_traits<K_surface, Primitive> AABB_triangle_traits;
typedef CGAL::AABB_tree<AABB_triangle_traits> Tree;

class EKFOA {
private:
	Camera cam;
	Kalman filter;
	cv::Mat frame;
	MotionTrackerOF motion_tracker;
public:
	EKFOA();
	void process(cv::Mat & frame, const double delta_t);
	const Kalman & kalman_filter() const { return filter; }
};

#endif
