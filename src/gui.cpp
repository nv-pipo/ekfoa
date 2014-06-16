#include "gui.hpp"

boost::mutex Gui::lock_;

Arcball Gui::arcball_;
GLfloat Gui::zoom_ = 0.5f;
GLboolean Gui::is_rotating_ = GL_FALSE;

Eigen::MatrixXd Gui::drone_points_(3, 6); //(six vertex)

std::vector<Point3d> Gui::XYZs_mu_;
std::vector<Point3d> Gui::XYZs_close_;
std::vector<Point3d> Gui::XYZs_far_;
Point3d Gui::closest_point_;

Eigen::Vector4d Gui::camera_orientation_(1, 0, 0, 0);

Eigen::Matrix3d Gui::cov_rWC_;

Delaunay Gui::triangulation_;

GLFWwindow* Gui::window_;

std::vector<Eigen::Vector3d> Gui::trajectory;


//========================================================================
// Initialize Miscellaneous OpenGL state
//========================================================================

void Gui::init(void){
	int width, height;

	glfwSetErrorCallback(error_callback);

	if (!glfwInit())
		exit(EXIT_FAILURE);

	window_ = glfwCreateWindow(640, 600, "EKF Obstacle avoidance", NULL, NULL);
	if (!window_) {
		glfwTerminate();
		exit(EXIT_FAILURE);
	}
	glfwSetWindowPos(window_, 1040, 430);


	glfwSetKeyCallback(window_, key_callback);
	glfwSetFramebufferSizeCallback(window_, framebuffer_size_callback);
	glfwSetMouseButtonCallback(window_, mouse_button_callback);
	glfwSetCursorPosCallback(window_, cursor_position_callback);
	glfwSetScrollCallback(window_, scroll_callback);

	glfwMakeContextCurrent(window_);
	glfwSwapInterval(1);

	glfwGetFramebufferSize(window_, &width, &height);
	framebuffer_size_callback(window_, width, height);
    // Use Gouraud (smooth) shading
    glShadeModel(GL_SMOOTH);

    // Switch on the z-buffer
    glEnable(GL_DEPTH_TEST);

    glEnableClientState(GL_VERTEX_ARRAY);
    glEnableClientState(GL_COLOR_ARRAY);

    // Background color is white
    glClearColor(1, 1, 1, 0);

    //Limit the number of renderings to 60 per second
    glfwSwapInterval(1);

    //A pyramid of 16x9x20 (w x h x d). With its top at the origin and the base towards the 'Z' axis:
    drone_points_ << 0, -16,  16, 16, -16, -16,
    		         0,  -9,  -9,  9,   9,  -9,
    		         0,  20,  20, 20,  20,  20;

}



//========================================================================
// Print errors
//========================================================================

void Gui::error_callback(int error, const char* description){
    fprintf(stderr, "Error: %s\n", description);
}


//========================================================================
// Handle key strokes
//========================================================================

void Gui::key_callback(GLFWwindow* window, int key, int scancode, int action, int mods)
{
//    if (action != GLFW_PRESS)
//        return;
//
//    switch (key) {
//        case GLFW_KEY_ESCAPE:
//            glfwSetWindowShouldClose(window, GL_TRUE);
//            break;
//        default:
//            break;
//    }
}


//========================================================================
// Callback function for mouse button events
//========================================================================

void Gui::mouse_button_callback(GLFWwindow* window, int button, int action, int mods)
{
    if (button != GLFW_MOUSE_BUTTON_LEFT)
        return;

    if (action == GLFW_PRESS){
    	double x, y;
    	glfwGetCursorPos(window_, &x, &y);
    	arcball_.startRotation(x, y);
    	is_rotating_ = GL_TRUE;
    } else {
    	is_rotating_ = GL_FALSE;
    	arcball_.stopRotation();
    }
}


//========================================================================
// Callback function for cursor motion events
//========================================================================

void Gui::cursor_position_callback(GLFWwindow* window, double x, double y){
    if (is_rotating_)
    	arcball_.updateRotation(x, y);
}


//========================================================================
// Callback function for scroll events
//========================================================================

void Gui::scroll_callback(GLFWwindow* window, double x, double y){
    zoom_ -= (float) y / 4.f;
    if (zoom_ < 0)
        zoom_ = 0;
}


//========================================================================
// Callback function for framebuffer resize events
//========================================================================

void Gui::framebuffer_size_callback(GLFWwindow* window, int width, int height){
	std::cout << "framebuffer_size_callback()" << std::endl;
    double window_ratio = 1;

    if (height > 0)
        window_ratio = width / height;

    // Setup viewport
    glViewport(0, 0, width, height);

    // Setup rotator:
    arcball_.setWidthHeight(width, height);

    // Change to the projection matrix and set our viewing volume
    glMatrixMode(GL_PROJECTION);
    glLoadIdentity();
    gluPerspective(110.0, window_ratio, 0.1f, 1024.0);
}


void Gui::update_draw_parameters( const Eigen::Vector3d & camera_pos, const Eigen::Vector4d & camera_orientation,  const Eigen::Matrix3d & cov_rWC, const std::vector<Point3d> & XYZs_mu, const std::vector<Point3d> & XYZs_close, const std::vector<Point3d> & XYZs_far, const Delaunay & triangulation, const Point3d & closest_point ){
	//lock
	lock_.lock();

	XYZs_mu_ = XYZs_mu;
	XYZs_close_ = XYZs_close;
	XYZs_far_ = XYZs_far;

	closest_point_ = closest_point;

	camera_orientation_ = camera_orientation;

	triangulation_ = triangulation;
    trajectory.push_back(camera_pos);

    cov_rWC_ = cov_rWC;

    //unlock
    lock_.unlock();
}

//========================================================================
// redraw
//========================================================================

bool Gui::redraw(){
	if (glfwWindowShouldClose(window_))
	    	return false;

    // Clear the color and depth buffers
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

    // We don't want to modify the projection matrix
    glMatrixMode(GL_MODELVIEW);
    glLoadIdentity();


    //Lock
    lock_.lock();

    // Move back
    glTranslatef(0, 0, -zoom_);
    // Rotate the view
    glRotatef(180, 1, 0, 0);//TODO: integrate this rotation to the arcball.


    arcball_.applyRotationMatrix();

    //Draw "drone":
    draw_drone();

    //Draw surface:
    draw_surface();
    lock_.unlock();

    glfwSwapBuffers(window_);
    glfwPollEvents();
//    glfwWaitEvents();

    return true;
}

void Gui::draw_drone(){
	glPushMatrix();
    //The "drone"
    glScalef(0.01f, 0.01f, 0.01f);

    //TODO: draw shapes with OpenGL primitives: 'glDrawElements'. It will be faster.
    glColor3f(0.f, 0.f, 0.f);
    glPolygonMode(GL_FRONT_AND_BACK, GL_LINE);
    glLineWidth(2.0);
    glBegin(GL_TRIANGLE_FAN);
    for (int i=0 ; i< drone_points_.cols() ; i++){
    	const Eigen::Vector3d & point = drone_points_.col(i);
    	glVertex3f(point(0), point(1), point(2));
    }
    glEnd();
    glPopMatrix();

    //Draw trajectory:
    glPointSize(2.0);
    glColor3f(0.f, 0.f, 1.f);
    glBegin(GL_POINTS);
    for (size_t i=0 ; i<trajectory.size() ; i++){
    	//remember to cancel the translation of the rotation ( - trajectory.back() ):
    	glVertex3f(trajectory[i](0)-trajectory.back()(0), trajectory[i](1)-trajectory.back()(1), trajectory[i](2)-trajectory.back()(2));
    }
    glEnd();

    //Draw confidence of position. One line between origin and sigma*3 on each dimension:
	const double sigma_3_rWC_X = 3*std::sqrt(cov_rWC_(0, 0)); //3*sqrt(rWC_X_variance) = 3 * sigma_X
	const double sigma_3_rWC_Y = 3*std::sqrt(cov_rWC_(1, 1)); //3*sqrt(rWC_Y_variance) = 3 * sigma_Y
	const double sigma_3_rWC_Z = 3*std::sqrt(cov_rWC_(2, 2)); //3*sqrt(rWC_Z_variance) = 3 * sigma_Z

	glLineWidth(3.0);
	glBegin(GL_LINES);
	//Draw camera position uncertainty as a line between the 0 and 3*sigma of the mean in each axis. The mean is 0, because the drone is at the origin of the coordinate system:
	glColor3f(1, 0, 0);
	glVertex3f(             0, 0, 0);
	glVertex3f( sigma_3_rWC_X, 0, 0);
	glColor3f(0, 1, 0);
	glVertex3f(0,              0, 0);
	glVertex3f(0, -sigma_3_rWC_Y, 0);
	glColor3f(0, 0, 1);
	glVertex3f(0, 0,              0);
	glVertex3f(0, 0,  sigma_3_rWC_Z);
	glEnd();

}


void Gui::draw_surface(){
	//Since we want the drone static, the whole scene is moving. So have to recenter it to the initial center, that is substract the current drone position ( trajectory.back() )

	glPointSize(5.0);

	glColor3f(1.f, 0.f, 0.f);
	glBegin(GL_POINTS);
	glVertex3f(closest_point_.x(), closest_point_.y(), closest_point_.z());

	glEnd();

	//Points
	glPointSize(4.0);

	glColor3f(1.f, 0.f, 1.f);
	glBegin(GL_POINTS);
	//Draw each point mean estimated position:
	for (size_t i=0 ; i<XYZs_mu_.size() ; i++){
		glVertex3f(XYZs_mu_[i].x(), XYZs_mu_[i].y(), XYZs_mu_[i].z());
	}
	glEnd();

	//Inverse depth uncertainty:
	glLineWidth(1.0);
	glColor3f(0, 0, 0);
	glBegin(GL_LINES);
	//Draw each point depth uncertainty as a line between the -3*sigma and 3*sigma of the mean:
	for (size_t i=0 ; i<XYZs_mu_.size() ; i++){
		glVertex3f(XYZs_close_[i].x(), XYZs_close_[i].y(), XYZs_close_[i].z());
		glVertex3f(XYZs_far_[i].x(), XYZs_far_[i].y(), XYZs_far_[i].z());
	}
	glEnd();

	//surface:
	glColor3f(0, 1, 1);
	glPolygonMode(GL_FRONT_AND_BACK, GL_FILL);
	std::list<Triangle> surface_faces;
	glBegin(GL_TRIANGLES);
	for(Delaunay::Finite_faces_iterator fit = triangulation_.finite_faces_begin(); fit != triangulation_.finite_faces_end(); ++fit) {
		const Delaunay::Face_handle & face = fit;
		//face->vertex(i)->info() = index of the point in the observation list.
		//The surface is built with a pesimistic approach...the closest point of the 99.73% of the distribution mass
		glVertex3f(XYZs_close_[face->vertex(0)->info()].x(), XYZs_close_[face->vertex(0)->info()].y(), XYZs_close_[face->vertex(0)->info()].z());
		glVertex3f(XYZs_close_[face->vertex(1)->info()].x(), XYZs_close_[face->vertex(1)->info()].y(), XYZs_close_[face->vertex(1)->info()].z());
		glVertex3f(XYZs_close_[face->vertex(2)->info()].x(), XYZs_close_[face->vertex(2)->info()].y(), XYZs_close_[face->vertex(2)->info()].z());

	}
	glEnd();

	glLineWidth(2.0);
	glColor3f(0, 0, 0);
	glBegin(GL_LINES);
	//Draw the segment lines betweeen each two points in the surface:
	for(Delaunay::Finite_faces_iterator fit = triangulation_.finite_faces_begin(); fit != triangulation_.finite_faces_end(); ++fit) {
		const Delaunay::Face_handle & face = fit;
		//face->vertex(i)->info() = index of the point in the observation list.
		glVertex3f(XYZs_close_[face->vertex(0)->info()].x(), XYZs_close_[face->vertex(0)->info()].y(), XYZs_close_[face->vertex(0)->info()].z());
		glVertex3f(XYZs_close_[face->vertex(1)->info()].x(), XYZs_close_[face->vertex(1)->info()].y(), XYZs_close_[face->vertex(1)->info()].z());
		glVertex3f(XYZs_close_[face->vertex(1)->info()].x(), XYZs_close_[face->vertex(1)->info()].y(), XYZs_close_[face->vertex(1)->info()].z());
		glVertex3f(XYZs_close_[face->vertex(2)->info()].x(), XYZs_close_[face->vertex(2)->info()].y(), XYZs_close_[face->vertex(2)->info()].z());
		glVertex3f(XYZs_close_[face->vertex(2)->info()].x(), XYZs_close_[face->vertex(2)->info()].y(), XYZs_close_[face->vertex(2)->info()].z());
		glVertex3f(XYZs_close_[face->vertex(0)->info()].x(), XYZs_close_[face->vertex(0)->info()].y(), XYZs_close_[face->vertex(0)->info()].z());
	}

	glEnd();
}

void Gui::release(){
	glfwDestroyWindow(window_);
	glfwTerminate();
}
