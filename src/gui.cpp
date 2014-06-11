#include "gui.hpp"

Arcball Gui::arcball_;
GLfloat Gui::zoom_ = 3.f;
GLboolean Gui::is_rotating_ = GL_FALSE;

Eigen::MatrixXd Gui::drone_points_(3, 6); //(six vertex)

double Gui::frame_ratio_ = 0;
cv::Mat Gui::frame_cv_;
GLuint Gui::frame_gl_;
GLboolean Gui::frame_changed_ = GL_FALSE;

std::vector<Point3d> Gui::XYZs_mu_;
std::vector<Point3d> Gui::XYZs_close_;
std::vector<Point3d> Gui::XYZs_far_;
Point3d Gui::closest_point_;

Eigen::Vector4d Gui::camera_orientation_(1, 0, 0, 0);

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

	window_ = glfwCreateWindow(1300, 800, "EKF Obstacle avoidance", NULL, NULL);
	if (!window_) {
		glfwTerminate();
		exit(EXIT_FAILURE);
	}
	glfwSetWindowPos(window_, 280, 20);


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
    gluPerspective(90.0, window_ratio, 0.1f, 1024.0);
}


//TODO: make thread safe:
void Gui::update_state_and_cov( const Eigen::Vector3d & camera_pos, const Eigen::Vector4d & camera_orientation, const std::vector<Point3d> & XYZs_mu, const std::vector<Point3d> & XYZs_close, const std::vector<Point3d> & XYZs_far, const Delaunay & triangulation, const Point3d & closest_point, const cv::Mat & frame_cv ){
//	std::cout << "update" << std::endl;
	XYZs_mu_ = XYZs_mu;
	XYZs_close_ = XYZs_close;
	XYZs_far_ = XYZs_far;

	closest_point_ = closest_point;

	camera_orientation_ = camera_orientation;

	triangulation_ = triangulation;
    trajectory.push_back(camera_pos);

    frame_cv_ = frame_cv;
    frame_ratio_ = (double)frame_cv.cols/(double)frame_cv.rows;
    frame_changed_ = true;

//    redraw();//TODO: can't call it from the kalman filter thread...work around this
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


    //Draw frame:
    draw_frame();

    // Move back
    glTranslatef(0, 0, -zoom_);
    // Rotate the view
    glRotatef(180, 1, 0, 0);//TODO: integrate this rotation to the arcball.


    arcball_.applyRotationMatrix();

    //Draw "drone":
    draw_drone();


    //Draw surface:
    draw_surface();

    glfwSwapBuffers(window_);
    glfwPollEvents();
//    glfwWaitEvents();

    return true;
}

void Gui::draw_frame(){
	if (frame_changed_){
	    	//TODO: Can this go in the init?
	    	glGenTextures(1, &frame_gl_);
	    	glBindTexture(GL_TEXTURE_2D, frame_gl_);
	    	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_REPEAT);
	    	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_REPEAT);
	    	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
	    	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
	    	glPixelStorei(GL_UNPACK_ALIGNMENT, 1);

	    	glTexImage2D(GL_TEXTURE_2D, 0, GL_RGB, frame_cv_.cols, frame_cv_.rows, 0, GL_BGR, GL_UNSIGNED_BYTE, frame_cv_.data);

	    	frame_changed_ = false;//Synchronization of this variable is not too bad, only that an update could take an extra loop.
	    }

	    glEnable(GL_TEXTURE_2D);
	    glPushMatrix();
	    glTranslatef(-0.9, 0.9, -1);
	    glBindTexture(GL_TEXTURE_2D, frame_gl_); // choose the texture to use.
	    glColor3f(1, 1, 1);

	    glBegin(GL_QUADS);

	    double x = 1.0/2.0;  //width = 1/3 * width/width
	    double y = frame_ratio_/2.0; //height = 1/3 * height/width = ratio/3

	    glTexCoord2f(0.0f, 0.0f);glVertex3f(0,  0, 0);
	    glTexCoord2f(0.0f, 1.0f);glVertex3f(0, -y, 0);
	    glTexCoord2f(1.0f, 1.0f);glVertex3f(x, -y, 0);
	    glTexCoord2f(1.0f, 0.0f);glVertex3f(x,  0, 0);

	    glEnd();
	    glPopMatrix();
	    glDisable(GL_TEXTURE_2D);
}

void Gui::draw_drone(){
    //Draw trajectory:
    glPointSize(2.0);
    glColor3f(0.f, 0.f, 1.f);
    glBegin(GL_POINTS);
    for (size_t i=0 ; i<trajectory.size() ; i++){
    	glVertex3f(trajectory[i](0), trajectory[i](1), trajectory[i](2));
    }
    glEnd();

    //The "drone"
    glPushMatrix();

    if (trajectory.size()>0)
    	glTranslated(trajectory.back()(0), trajectory.back()(1), trajectory.back()(2));

    Eigen::Matrix3d R;
    MotionModel::quaternion_matrix(camera_orientation_, R);

    //rotate:
    Eigen::MatrixXd drone_points = R * drone_points_;

    glScalef(0.01f, 0.01f, 0.01f);

    //TODO: draw this with OpenGL primitives: 'glDrawElements'. It will be faster.
    glColor3f(0.f, 0.f, 1.f);
    glBegin(GL_POLYGON);
    for (int i=0 ; i< drone_points.cols() ; i++){
    	const Eigen::Vector3d & point = drone_points.col(i);
		glVertex3f(point(0), point(1), point(2));
    }
    glEnd();

    glColor3f(0.f, 0.f, 0.f);
    glLineWidth(2.0);
    glBegin(GL_LINES);
    const Eigen::Vector3d & origin = drone_points.col(0);
    for (int i=0 ; i< drone_points.cols() ; i++){
    	const Eigen::Vector3d & point = drone_points.col(i);
    	glVertex3f(origin(0), origin(1), origin(2));
    	glVertex3f(point(0), point(1), point(2));
    }
    glEnd();
    glPopMatrix();
}


void Gui::draw_surface(){

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
