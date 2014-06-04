#ifndef GUI_H_
#define GUI_H_

#include <stdio.h>
#include <stdlib.h>
#include <math.h>

#include <GL/glu.h>
#include <GLFW/glfw3.h>

#include <vector>   //vector
#include <Eigen/Core>  //Eigen::Vector3d
#include <Eigen/Geometry> //Eigen::Quaternion

#include "feature.hpp"
#include "kalman.hpp"
#include "opengl_utils/arcball.hpp"

class Gui {
public:
	static bool redraw();
	static void init();
	static void release();
	static void update_state_and_cov( const Eigen::VectorXd & x_k_k, const Eigen::MatrixXd & p_k_k );

private:
	static Arcball arcball_;
	static GLfloat zoom_;
	static GLboolean locked_;
	static GLboolean ready_;

	static Eigen::VectorXd x_k_k_;
	static Eigen::MatrixXd p_k_k_;

	static GLFWwindow* window_;

	static std::vector<Eigen::Vector3d> trajectory;


	static void error_callback(int error, const char* description);
	static void key_callback(GLFWwindow* window, int key, int scancode, int action, int mods);
	static void mouse_button_callback(GLFWwindow* window, int button, int action, int mods);
	static void cursor_position_callback(GLFWwindow* window, double x, double y);
	static void scroll_callback(GLFWwindow* window, double x, double y);
	static void framebuffer_size_callback(GLFWwindow* window, int width, int height);
};

#endif
