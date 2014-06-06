#include "gui.hpp"

Arcball Gui::arcball_;
GLfloat Gui::zoom_ = 3.f;
GLboolean Gui::is_rotating_ = GL_FALSE;

Eigen::MatrixXd Gui::drone_points_(3, 6); //(six vertex)

double Gui::frame_ratio_ = 0;
cv::Mat Gui::frame_cv_;
GLuint Gui::frame_gl_;
GLboolean Gui::frame_changed_ = GL_FALSE;

Eigen::VectorXd Gui::x_k_k_;
Eigen::MatrixXd Gui::p_k_k_;
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

    glLineWidth(2.0);

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
void Gui::update_state_and_cov( const Eigen::VectorXd & x_k_k, const Eigen::MatrixXd & p_k_k, const cv::Mat & frame_cv, const Delaunay & triangulation ){
//	std::cout << "update" << std::endl;
	x_k_k_ = x_k_k;
	p_k_k_ = p_k_k;
	triangulation_ = triangulation;
    trajectory.push_back(x_k_k.segment(0, 3));

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

    if (p_k_k_.rows()>0){
    	//Draw "drone":
    	draw_drone();


    	//Draw points:
    	glPointSize(4.0);
    	glBegin(GL_POINTS);
    	glColor3f(1.f, 0.f, 1.f);
    	int num_features = (x_k_k_.rows() - 13) / 6;
    	int start_feature = 13;
    	std::vector<Eigen::Vector3d> XYZs;
    	for (int i=0 ; i<num_features ; i++){
    		const Eigen::VectorXd & yi = x_k_k_.segment(start_feature, 6);
    		Eigen::Vector3d XYZ;
    		Feature::compute_cartesian(yi, XYZ);
    		glVertex3f(XYZ(0), XYZ(1), XYZ(2));
    		//Remember the 3D points. Used later for adding the surface. (TODO: Make more sense of the indexes, here I'm adding the points sequentially, but the observation_list can be non sequential)
    		XYZs.push_back(XYZ);
    		start_feature += 6;
    	}
    	glEnd();

    	//    glVertexPointer(3, GL_FLOAT, sizeof(struct Vertex), vertex);
    	//    glColorPointer(3, GL_FLOAT, sizeof(struct Vertex), &vertex[0].r); // Pointer to the first color
    	//    glDrawElements(GL_QUADS, 4 * QUADNUM, GL_UNSIGNED_INT, quad);

    	//Draw triangles:
    	glBegin(GL_TRIANGLES);
    	glColor3f(0, 1, 1);
		for(Delaunay::Finite_faces_iterator fit = triangulation_.finite_faces_begin(); fit != triangulation_.finite_faces_end(); ++fit) {
			const Delaunay::Face_handle & face = fit;
			//face->vertex(i)->info() = index of the point in the observation list.
			XYZs[face->vertex(0)->info()];
			glVertex3f(XYZs[face->vertex(0)->info()](0), XYZs[face->vertex(0)->info()](1), XYZs[face->vertex(0)->info()](2));
			glVertex3f(XYZs[face->vertex(1)->info()](0), XYZs[face->vertex(1)->info()](1), XYZs[face->vertex(1)->info()](2));
			glVertex3f(XYZs[face->vertex(2)->info()](0), XYZs[face->vertex(2)->info()](1), XYZs[face->vertex(2)->info()](2));
		}
    	glEnd();

    	glBegin(GL_LINES);
    	glColor3f(0, 0, 0);
    	for(Delaunay::Finite_faces_iterator fit = triangulation_.finite_faces_begin(); fit != triangulation_.finite_faces_end(); ++fit) {
    		const Delaunay::Face_handle & face = fit;
    		//face->vertex(i)->info() = index of the point in the observation list.
    		glVertex3f(XYZs[face->vertex(0)->info()](0), XYZs[face->vertex(0)->info()](1), XYZs[face->vertex(0)->info()](2));
    		glVertex3f(XYZs[face->vertex(1)->info()](0), XYZs[face->vertex(1)->info()](1), XYZs[face->vertex(1)->info()](2));
    		glVertex3f(XYZs[face->vertex(1)->info()](0), XYZs[face->vertex(1)->info()](1), XYZs[face->vertex(1)->info()](2));
    		glVertex3f(XYZs[face->vertex(2)->info()](0), XYZs[face->vertex(2)->info()](1), XYZs[face->vertex(2)->info()](2));
    		glVertex3f(XYZs[face->vertex(2)->info()](0), XYZs[face->vertex(2)->info()](1), XYZs[face->vertex(2)->info()](2));
    		glVertex3f(XYZs[face->vertex(0)->info()](0), XYZs[face->vertex(0)->info()](1), XYZs[face->vertex(0)->info()](2));
    	}
    	glEnd();

    }

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

    glTranslated(x_k_k_(0), x_k_k_(1), x_k_k_(2));
//    glRotated(x_k_k_(3) * 180/M_PI, x_k_k_(4), x_k_k_(5), x_k_k_(6));

    Eigen::Quaterniond q(x_k_k_(3), x_k_k_(4), x_k_k_(5), x_k_k_(6));
    Eigen::MatrixXd R = q.toRotationMatrix();

    //rotate:
    Eigen::MatrixXd drone_points = R * drone_points_;

//    Eigen::MatrixXd drone_points_rotated = drone_points_;
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

void Gui::release(){
	glfwDestroyWindow(window_);
	glfwTerminate();
}
