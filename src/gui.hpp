#ifndef GUI_H_
#define GUI_H_

#include <stdio.h>
#include <stdlib.h>
#include <math.h>

#include <iostream>
#include <list>

#include <GL/glu.h>
#include <GLFW/glfw3.h>

#include <opencv2/core/core.hpp> //Point2f

#include <vector>   //vector
#include <Eigen/Core>  //Eigen::Vector3d
#include <Eigen/Geometry> //Eigen::Quaternion

#include "feature.hpp"
#include "kalman.hpp"
#include "motion_model.hpp"
#include "opengl_utils/arcball.hpp"

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


class Gui {
public:
	static bool redraw();
	static void init();
	static void release();
	static void update_state_and_cov( const Eigen::Vector3d & camera_pos, const Eigen::Vector4d & camera_orientation, const std::vector<Point3d> & XYZs_mu, const std::vector<Point3d> & XYZs_close, const std::vector<Point3d> & XYZs_far, const Delaunay & triangulation, const Point3d & closest_point, const cv::Mat & frame_cv );

private:
	static Arcball arcball_;
	static GLfloat zoom_;
	static GLboolean is_rotating_;

	static Eigen::MatrixXd drone_points_;

	static GLuint frame_gl_;
	static cv::Mat frame_cv_;
	static double frame_ratio_;

	static GLboolean frame_changed_;

	static std::vector<Point3d> XYZs_mu_;
	static std::vector<Point3d> XYZs_close_;
	static std::vector<Point3d> XYZs_far_;
	static Point3d closest_point_;

	static Delaunay triangulation_;

	static GLFWwindow* window_;

	static std::vector<Eigen::Vector3d> trajectory;
	static Eigen::Vector4d camera_orientation_;


	static void draw_frame();
	static void draw_drone();
	static void draw_surface();
	static void error_callback(int error, const char* description);
	static void key_callback(GLFWwindow* window, int key, int scancode, int action, int mods);
	static void mouse_button_callback(GLFWwindow* window, int button, int action, int mods);
	static void cursor_position_callback(GLFWwindow* window, double x, double y);
	static void scroll_callback(GLFWwindow* window, double x, double y);
	static void framebuffer_size_callback(GLFWwindow* window, int width, int height);
};

#endif
