#ifndef GUI_H_
#define GUI_H_

#include <stdio.h>
#include <stdlib.h>
#include <math.h>

#include <GL/glu.h>
#include <GLFW/glfw3.h>

#include <opencv2/core/core.hpp> //Point2f

#include <vector>   //vector
#include <Eigen/Core>  //Eigen::Vector3d
#include <Eigen/Geometry> //Eigen::Quaternion

#include "feature.hpp"
#include "kalman.hpp"
#include "opengl_utils/arcball.hpp"

//Delaunay triangulation:
#include <CGAL/Exact_predicates_exact_constructions_kernel.h>
#include <CGAL/Delaunay_triangulation_2.h>
#include <CGAL/Triangulation_vertex_base_with_info_2.h>
typedef CGAL::Exact_predicates_exact_constructions_kernel         K;
typedef CGAL::Triangulation_vertex_base_with_info_2<size_t, K>    Vb;
typedef CGAL::Triangulation_data_structure_2<Vb>                  Tds;
typedef CGAL::Delaunay_triangulation_2<K, Tds>                    Delaunay;
typedef K::Point_2                                                Point;

class Gui {
public:
	static bool redraw();
	static void init();
	static void release();
	static void update_state_and_cov( const Eigen::VectorXd & x_k_k, const Eigen::MatrixXd & p_k_k, const cv::Mat & frame, const Delaunay & triangulation );

private:
	static Arcball arcball_;
	static GLfloat zoom_;
	static GLboolean is_rotating_;

	static Eigen::MatrixXd drone_points_;

	static GLuint frame_gl_;
	static cv::Mat frame_cv_;
	static double frame_ratio_;

	static GLboolean frame_changed_;

	static Eigen::VectorXd x_k_k_;
	static Eigen::MatrixXd p_k_k_;
	static Delaunay triangulation_;

	static GLFWwindow* window_;

	static std::vector<Eigen::Vector3d> trajectory;


	static void draw_frame();
	static void draw_drone();
	static void error_callback(int error, const char* description);
	static void key_callback(GLFWwindow* window, int key, int scancode, int action, int mods);
	static void mouse_button_callback(GLFWwindow* window, int button, int action, int mods);
	static void cursor_position_callback(GLFWwindow* window, double x, double y);
	static void scroll_callback(GLFWwindow* window, double x, double y);
	static void framebuffer_size_callback(GLFWwindow* window, int width, int height);
};

#endif
