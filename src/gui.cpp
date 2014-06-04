#include "gui.hpp"

Arcball Gui::arcball_;
GLfloat Gui::zoom_ = 2.f;
GLboolean Gui::locked_ = GL_FALSE;
GLboolean Gui::ready_ = GL_FALSE;

Eigen::VectorXd Gui::x_k_k_;
Eigen::MatrixXd Gui::p_k_k_;

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
//    glVertexPointer(3, GL_FLOAT, sizeof(struct Vertex), vertex);
//    glColorPointer(3, GL_FLOAT, sizeof(struct Vertex), &vertex[0].r); // Pointer to the first color

    glLineWidth(2.0);

    // Background color is black
    glClearColor(1, 1, 1, 0);

    //Limit the number of renderings to 60 per second
    glfwSwapInterval(1);

    ready_ = GL_TRUE;
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
    if (action != GLFW_PRESS)
        return;

    switch (key) {
        case GLFW_KEY_ESCAPE:
            glfwSetWindowShouldClose(window, GL_TRUE);
            break;
        default:
            break;
    }
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
    	locked_ = GL_TRUE;
    } else {
    	locked_ = GL_FALSE;
    	arcball_.stopRotation();
    }
}


//========================================================================
// Callback function for cursor motion events
//========================================================================

void Gui::cursor_position_callback(GLFWwindow* window, double x, double y){
    if (locked_)
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
    float ratio = 1.f;

    if (height > 0)
        ratio = (float) width / (float) height;

    // Setup viewport
    glViewport(0, 0, width, height);

    // Setup rotator:
    arcball_.setWidthHeight(width, height);

    // Change to the projection matrix and set our viewing volume
    glMatrixMode(GL_PROJECTION);
    glLoadIdentity();
    gluPerspective(120.0, ratio, 0.3f, 1024.0);
}


//TODO: make thread safe:
void Gui::update_state_and_cov( const Eigen::VectorXd & x_k_k, const Eigen::MatrixXd & p_k_k ){
//	std::cout << "update" << std::endl;
	x_k_k_ = x_k_k;
	p_k_k_ = p_k_k;
    trajectory.push_back(x_k_k.segment(0, 3));
//    redraw();
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

    // Move back
    glTranslatef(0.0, 0.0, -zoom_);
    // Rotate the view
    glRotatef(180, 1, 0, 0);//TODO: integrate this rotation to the arcball.
    arcball_.applyRotationMatrix();

    //Draw trajectory:
    glPointSize(2.0);
    glColor3f(0.f, 0.f, 1.f);
    glBegin(GL_POINTS);
    for (size_t i=0 ; i<trajectory.size() ; i++){
    	glVertex3f(trajectory[i](0), trajectory[i](1), trajectory[i](2));
    }
    glEnd();

    //Draw "drone":
    glPushMatrix();
    glTranslated(x_k_k_(0), x_k_k_(1), x_k_k_(2));
    glRotated(x_k_k_(3), x_k_k_(4), x_k_k_(5), x_k_k_(6));
    glScalef(0.1f, 0.1f, 0.1f);
    glBegin(GL_POLYGON);
    glVertex3f(0.0f, 0.0f, 0.0f);
    glVertex3f(-1.4f, -1.0f, 2.0f);
    glVertex3f( 1.4f, -1.0f, 2.0f);
    glVertex3f( 1.4f,  1.0f, 2.0f);
    glVertex3f(-1.4f,  1.0f, 2.0f);
    glVertex3f(-1.4f, -1.0f, 2.0f);
    glEnd();
    glColor3f(0.f, 0.f, 0.f);
    glBegin(GL_LINES);
    glVertex3f(0.0f, 0.0f, 0.0f);
    glVertex3f(-1.4f, -1.0f, 2.0f);
    glVertex3f(0.0f, 0.0f, 0.0f);
    glVertex3f( 1.4f, -1.0f, 2.0f);
    glVertex3f(0.0f, 0.0f, 0.0f);
    glVertex3f( 1.4f,  1.0f, 2.0f);
    glVertex3f(0.0f, 0.0f, 0.0f);
    glVertex3f(-1.4f,  1.0f, 2.0f);
    glVertex3f(0.0f, 0.0f, 0.0f);
    glVertex3f(-1.4f, -1.0f, 2.0f);
    glEnd();

    glPopMatrix();

    //Draw points:
    glPointSize(4.0);
    glBegin(GL_POINTS);
    glColor3f(1.f, 0.f, 1.f);
    int num_features = (x_k_k_.rows() - 13) / 6;
    int start_feature = 13;
    for (int i=0 ; i<num_features ; i++){
    	const Eigen::VectorXd & yi = x_k_k_.segment(start_feature, 6);
    	Eigen::Vector3d XYZ;
    	Feature::compute_cartesian(yi, XYZ);
    	glVertex3f(XYZ(0), XYZ(1), XYZ(2));
    	start_feature += 6;
    }
    glEnd();

//    glDrawElements(GL_QUADS, 4 * QUADNUM, GL_UNSIGNED_INT, quad);

    glfwSwapBuffers(window_);
    glfwPollEvents();
//    glfwWaitEvents();

    return true;
}

void Gui::release(){
	glfwDestroyWindow(window_);
	glfwTerminate();
}
