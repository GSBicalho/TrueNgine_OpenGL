#include <iostream>
#include <string>
#include <fstream>

#define _USE_MATH_DEFINES
#include <math.h>

// GLEW
#define GLEW_STATIC
#include <GL/glew.h>

// GLFW
#include <GLFW/glfw3.h>

// GLM
#include <glm/glm.hpp>
#include <glm/gtc/matrix_transform.hpp>
#include <glm/gtc/type_ptr.hpp>

// Other Libs
#include <SOIL/SOIL.h>

// Other includes
#include "Shader.h"
#include "Camera3D.h"
#include "CameraND.h"
#include "NMatrixOperations.h"

#include "json.h"

using json = nlohmann::json;

// Function prototypes
void key_callback(GLFWwindow* window, int key, int scancode, int action, int mode);
void scroll_callback(GLFWwindow* window, double xoffset, double yoffset);
void doMovement();
void mouse_callback(GLFWwindow* window, double xpos, double ypos);

// Window dimensions
const GLuint WIDTH = 800, HEIGHT = 600;

// Camera Variables
// The camera used to show the perspective in 3D space
Camera3D camera3D;
// The cameras used to show the perspective in ND space.
// The cameras are stored ordered by number of dimensions.
std::vector<CameraND> camerasND;
GLfloat lastX = WIDTH / 2;
GLfloat lastY = HEIGHT / 2;
bool firstMouse = true;

// FOURTH DIMENSION
float current_w = 0.0f;
float speed_w = 0.5f;

//Render stuff
unsigned int VBO, VAO, EBO;

//Keyboard
bool keys[1024];

//DeltaTime
GLfloat deltaTime = 0.0f;	// Time between current frame and last frame
GLfloat lastFrame = 0.0f;  	// Time of last frame

glm::vec4 wire_color = { 1, 0, 0, 1 };
glm::vec4 fill_color = { 1, 1, 1, 1 };
float wire_width = 2;

Eigen::MatrixXf vertices4d(16, 4);
float vertices[48];

const int N = 4;

inline double to_radians(double degrees) {
	return degrees / (180.0 / M_PI);
}

inline double to_degrees(double radians) {
	return radians * (180.0 / M_PI);
}

Eigen::MatrixXf GlmToEigen(glm::mat4 m) {
	Eigen::MatrixXf ans(4, 4);
	for (int i = 0; i < 4; i++) {
		for (int j = 0; j < 4; j++) {
			ans(i, j) = m[i][j];
		}
	}
	return ans;
}

Eigen::MatrixXf EigenToEigen(Eigen::MatrixXf m) {
	Eigen::MatrixXf ans(4, 4);
	for (int i = 0; i < 4; i++) {
		for (int j = 0; j < 4; j++) {
			ans(i, j) = m(i, j);
		}
	}
	return ans;
}

int test_matrixes() {
	Camera3D normalCamera(glm::vec3(0.0f, 0.0f, 3.0f));

	Eigen::VectorXf position = Eigen::VectorXf::Zero(3);
	Eigen::Vector3f up = Eigen::Vector3f::Zero();
	up(1) = 1.f;
	//CameraN dimensionalCamera = CameraN(3, position, up, HEIGHT, WIDTH);

	glm::mat4 testTranslateGlm;
	testTranslateGlm = glm::translate(testTranslateGlm, { 1,2,3 });

	Eigen::VectorXf toTranslate(3);
	toTranslate << 1, 2, 3;

	std::cout << "GLM translate\n";
	std::cout << GlmToEigen(testTranslateGlm) << "\n";
	std::cout << "Eigen translate\n";
	std::cout << translateMatrixN(3, toTranslate).transpose() << "\n\n";

	glm::mat4 testRotateGlm;
	testRotateGlm = glm::rotate(testRotateGlm, 0.3f, glm::vec3(1, 0, 0));
	std::cout << "GLM rotate\n";
	std::cout << GlmToEigen(testRotateGlm) << "\n";
	std::cout << "Eigen rotate\n";
	std::cout << rotateMatrixN(3, 1, 2, 0.3f).transpose() << "\n\n";

	glm::mat4 testPerspectiveGlm;
	testPerspectiveGlm = glm::perspective(45.f, (float)WIDTH / (float)HEIGHT, 0.1f, 1000.0f);
	std::cout << "GLM perspective\n";
	std::cout << GlmToEigen(testPerspectiveGlm) << "\n";
	std::cout << "Eigen perspective\n";
	std::cout << perspectiveMatrixN(3, 45.f, 0.1f, 1000.0f, (float)WIDTH / (float)HEIGHT).transpose() << "\n\n";

	glm::mat4 testLookatGlm;
	testLookatGlm = glm::lookAt(glm::vec3{ 0,0,0 }, glm::vec3{ 0.05,0.2,0.75 }, glm::vec3{ 0,1,0 });
	std::cout << "GLM LookAt\n";
	std::cout << GlmToEigen(testLookatGlm) << "\n";
	std::cout << "Eigen LookAt\n";
	Eigen::VectorXf toTest(3);
	toTest << 0.05f, 0.2f, 0.75f;
	std::cout << lookAtMatrixN(3, Eigen::VectorXf::Zero(3), toTest, up).transpose() << "\n\n";

	system("pause");
	return 0;
}

void updateVertexMatrix() {
	Eigen::VectorXf from(4);
	Eigen::VectorXf to(4);
	from << -4, 0, 0, 0;
	to << 0, 0, 0, 0;

	Eigen::MatrixXf ups(4, 2);
	ups << 0, 0, 1, 0, 0, 0, 0, 1;

	Eigen::MatrixXf newM = vertices4d;

	for (CameraND c : camerasND) {
		newM = projectPointsLosingDimension(vertices4d, c.GetViewProjectionModelMatrix());
	}

	for (int j = 0; j < 16; j++) {
		//Eigen::VectorXf point = projectPointLosingDimension(vertices4d.col(j), view4d);
		Eigen::VectorXf point = newM.col(j);
		vertices[j * 3 + 0] = point(0);
		vertices[j * 3 + 1] = point(1);
		vertices[j * 3 + 2] = point(2);
	}
}

void updateVBO() {
	glBindBuffer(GL_ARRAY_BUFFER, VBO);
	glBufferData(GL_ARRAY_BUFFER, sizeof(vertices), vertices, GL_DYNAMIC_DRAW);
	glBindBuffer(GL_ARRAY_BUFFER, 0);
}

// The MAIN function, from here we start the application and run the game loop
int main() {
	// Init GLFW
	glfwInit();
	// Set all the required options for GLFW
	glfwWindowHint(GLFW_CONTEXT_VERSION_MAJOR, 3);
	glfwWindowHint(GLFW_CONTEXT_VERSION_MINOR, 3);
	glfwWindowHint(GLFW_OPENGL_PROFILE, GLFW_OPENGL_CORE_PROFILE);
	glfwWindowHint(GLFW_RESIZABLE, GL_FALSE);

	// Create a GLFWwindow object that we can use for GLFW's functions
	GLFWwindow* window = glfwCreateWindow(WIDTH, HEIGHT, "TrueNgine", nullptr, nullptr);
	glfwMakeContextCurrent(window);

	// Set the required callback functions
	glfwSetKeyCallback(window, key_callback);
	glfwSetCursorPosCallback(window, mouse_callback);
	glfwSetScrollCallback(window, scroll_callback);

	//Setting Mouse
	glfwSetInputMode(window, GLFW_CURSOR, GLFW_CURSOR_DISABLED);

	// Set this to true so GLEW knows to use a modern approach to retrieving function pointers and extensions
	glewExperimental = GL_TRUE;
	// Initialize GLEW to setup the OpenGL Function pointers
	glewInit();

	// Define the viewport dimensions
	glViewport(0, 0, WIDTH, HEIGHT);
	glEnable(GL_DEPTH_TEST);
	glDepthFunc(GL_LESS);
	glDisable(GL_CULL_FACE);

	// Build and compile our shader program
	std::ofstream myfile;
	myfile.open("W:\\asd.txt");
	myfile << generateNDimensionalShader(4);
	myfile.close();

	//Shader ourShader(generateNDimensionalShader(4), "simple_color_shader.frag", false);
	Shader ourShader("simple_color_shader.vs", "simple_color_shader.frag");
	//Shader ourShader("line_shader.vs", "line_shader.frag");
	//Shader lightingShader("light_color_shader.vs", "light_color_shader.frag");

	Eigen::VectorXf position = Eigen::VectorXf::Zero(N);
	position(0) = -4.f;
	Eigen::VectorXf target = Eigen::VectorXf::Zero(N);
	target(0) = 4.f;

	//setting up 3d camera
	camera3D = Camera3D(glm::vec3(0.0f, 0.0f, 3.0f));

	//adding a 4D camera to our array of cameras
	camerasND.push_back(CameraND(4, position, target));

	float vertices4dMatrix[] = {
		0, 0, 0, 0,
		0, 0, 0, 1,
		0, 0, 1, 0,
		0, 0, 1, 1,

		0, 1, 0, 0,
		0, 1, 0, 1,
		0, 1, 1, 0,
		0, 1, 1, 1,

		1, 0, 0, 0,
		1, 0, 0, 1,
		1, 0, 1, 0,
		1, 0, 1, 1,

		1, 1, 0, 0,
		1, 1, 0, 1,
		1, 1, 1, 0,
		1, 1, 1, 1,
	};

	for (int i = 0; i < 16; i++) {
		for (int j = 0; j < 4; j++) {
			vertices4d(i, j) = vertices4dMatrix[i * 4 + j] * 2 - 1;
		}
	}
	vertices4d.transposeInPlace();

	updateVertexMatrix();


	unsigned int indices[] = {
		//Cube at 0
		0, 1,
		1, 3,
		3, 2,
		2, 0,

		4, 5,
		5, 7,
		7, 6,
		6, 4,

		0, 4,
		1, 5,
		2, 6,
		3, 7,

		//Cube at 1
		8,  9,
		9, 11,
		11, 10,
		10,  8,

		12, 13,
		13, 15,
		15, 14,
		14, 12,

		8, 12,
		9, 13,
		10, 14,
		11, 15,

		//Connections
		0,  8,
		1,  9,
		2, 10,
		3, 11,
		4, 12,
		5, 13,
		6, 14,
		7, 15,
	};

	std::cout << glGetString(GL_RENDERER) << std::endl;

	glGenVertexArrays(1, &VAO);
	glGenBuffers(1, &VBO);
	glGenBuffers(1, &EBO);

	glBindVertexArray(VAO);

	glBindBuffer(GL_ARRAY_BUFFER, VBO);
	glBufferData(GL_ARRAY_BUFFER, sizeof(vertices), vertices, GL_DYNAMIC_DRAW);

	std::cout << sizeof(vertices4d) << "\n";

	glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, EBO);
	glBufferData(GL_ELEMENT_ARRAY_BUFFER, sizeof(indices), indices, GL_STATIC_DRAW);

	// position attribute
	glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, 3 * sizeof(float), (void*)0);
	glEnableVertexAttribArray(0);

	// Game loop
	while (!glfwWindowShouldClose(window)) {
		// Set frame time
		GLfloat currentFrame = (float)glfwGetTime();
		deltaTime = currentFrame - lastFrame;
		lastFrame = currentFrame;

		// Check and call events
		glfwPollEvents();
		doMovement();

		// Clear the colorbuffer
		glClearColor(0.2f, 0.3f, 0.3f, 1.0f);
		glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

		// Draw our first triangle
		ourShader.Use();

		GLint inColor_uniform = glGetUniformLocation(ourShader.Program, "inColor");
		glUniform4fv(inColor_uniform, 1, glm::value_ptr(wire_color));

		// Get the dimensional uniform locations
		/*for(int i = N; i >= 3; i--){
		std::stringstream ss;
		ss << "m" << i;

		GLint viewLoc = glGetUniformLocation(ourShader.Program, ss.str().c_str());
		glUniformMatrix4fv(viewLoc, (i+1)*(i+1), GL_FALSE, camera.GetViewMatrix(i).transpose().data());
		}*/

		// Create camera transformation
		glm::mat4 view;
		view = camera3D.GetViewMatrix();

		glm::mat4 projection;
		projection = glm::perspective(camera3D.Zoom, (float)WIDTH / (float)HEIGHT, 0.1f, 1000.0f);

		glm::mat4 model;
		model = glm::translate(model, { 0,0,0 });
		// Get the uniform locations
		GLint modelLoc = glGetUniformLocation(ourShader.Program, "model");
		GLint viewLoc = glGetUniformLocation(ourShader.Program, "view");
		GLint projLoc = glGetUniformLocation(ourShader.Program, "projection");
		// Pass the matrices to the shader
		glUniformMatrix4fv(viewLoc, 1, GL_FALSE, glm::value_ptr(view));
		glUniformMatrix4fv(projLoc, 1, GL_FALSE, glm::value_ptr(projection));
		glUniformMatrix4fv(modelLoc, 1, GL_FALSE, glm::value_ptr(model));

		glBindVertexArray(VAO);

		updateVertexMatrix();
		updateVBO();

		glDrawElements(GL_LINES, 62, GL_UNSIGNED_INT, 0);

		glBindVertexArray(0);

		// Swap the buffers
		glfwSwapBuffers(window);
	}
	// Properly de-allocate all resources once they've outlived their purpose
	glDeleteVertexArrays(1, &VAO);
	glDeleteBuffers(1, &VBO);

	//glDeleteBuffers(1, &EBO);
	// Terminate GLFW, clearing any resources allocated by GLFW.
	glfwTerminate();

	return 0;
}

// Is called whenever a key is pressed/released via GLFW
void key_callback(GLFWwindow* window, int key, int scancode, int action, int mode) {
	if (key == GLFW_KEY_ESCAPE && action == GLFW_PRESS)
		glfwSetWindowShouldClose(window, GL_TRUE);
	if (key >= 0 && key < 1024) {
		if (action == GLFW_PRESS)
			keys[key] = true;
		else if (action == GLFW_RELEASE)
			keys[key] = false;
	}
}

double curr_angle_x = 0;
double curr_angle_y = 0;
double curr_angle_z = 0;
void doMovement() {
	// TODO: generalize this to N dimensions (or at least as many can fit in a keyboard)
	// Camera controls

	if (keys[GLFW_KEY_W])
		camera3D.ProcessKeyboard(FORWARD, deltaTime);
	if (keys[GLFW_KEY_S])
		camera3D.ProcessKeyboard(BACKWARD, deltaTime);
	if (keys[GLFW_KEY_A])
		camera3D.ProcessKeyboard(LEFT, deltaTime);
	if (keys[GLFW_KEY_D])
		camera3D.ProcessKeyboard(RIGHT, deltaTime);
	if (keys[GLFW_KEY_SPACE])
		camera3D.ProcessKeyboard(UP, deltaTime);
	if (keys[GLFW_KEY_LEFT_SHIFT])
		camera3D.ProcessKeyboard(DOWN, deltaTime);
	if (keys[GLFW_KEY_Q]) {
		vertices4d = rotateMatrixN(4, 0, 1, deltaTime) * vertices4d;
	}


	/*if (keys[GLFW_KEY_A]) {
	camera.ProcessKeyboard(0, false, deltaTime);
	}
	if (keys[GLFW_KEY_D]) {
	camera.ProcessKeyboard(0, true, deltaTime);
	}
	if (keys[GLFW_KEY_SPACE]) {
	camera.ProcessKeyboard(1, true, deltaTime);
	}
	if (keys[GLFW_KEY_LEFT_SHIFT]) {
	camera.ProcessKeyboard(1, false, deltaTime);
	}
	if (keys[GLFW_KEY_W]) {
	camera.ProcessKeyboard(2, true, deltaTime);
	}
	if (keys[GLFW_KEY_S]) {
	camera.ProcessKeyboard(2, false, deltaTime);
	}
	if (keys[GLFW_KEY_Q]) {
	camera.ProcessKeyboard(3, false, deltaTime);
	}
	if (keys[GLFW_KEY_E]) {
	camera.ProcessKeyboard(3, true, deltaTime);
	}*/
}

void mouse_callback(GLFWwindow* window, double xpos, double ypos) {
	if (firstMouse) {
		lastX = (float)xpos;
		lastY = (float)ypos;
		firstMouse = false;
	}

	GLfloat xoffset = (float)xpos - lastX;
	GLfloat yoffset = lastY - (float)ypos;  // Reversed since y-coordinates go from bottom to left

	lastX = (float)xpos;
	lastY = (float)ypos;

	camera3D.ProcessMouseMovement(xoffset, yoffset);
}

void scroll_callback(GLFWwindow* window, double xoffset, double yoffset) {
	//camera.ProcessMouseScroll((float)yoffset);
}