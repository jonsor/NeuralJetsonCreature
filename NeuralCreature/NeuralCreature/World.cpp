/**
World.cpp
Purpose: Main application file. Initializes all the libraries and sets up the world, then starts the main game loop.

@author Sjur Barndon, Jonas Sørsdal
@version 1.0 23.03.2017
*/

#include "stdafx.h"
#include "World.h"

#define GLEW_STATIC

//Prototypes
void key_callback(GLFWwindow* window, int key, int scancode, int action, int mode);
void key_callback(GLFWwindow* window, int key, int scancode, int action, int mode);
void scroll_callback(GLFWwindow* window, double xoffset, double yoffset);
void mouse_callback(GLFWwindow* window, double xpos, double ypos);


//Variables for camera, controls and time:

Camera camera(glm::vec3(0.0f, 3.0f, 10.0f));
bool keys[1024];
GLfloat lastX, lastY;
bool firstMouse = true;
bool applyImpulse = true;
bool motorImpulses = false;

GLfloat deltaTime = 0.0f;
GLfloat lastFrame = 0.0f;

//Leg control:
btScalar targetAngle;
btScalar targetAngleRightKnee;
btScalar targetAngleLeftKnee;
btScalar targetAngleAbdomen;

bool holdingO = false;
bool holdingP = false;
bool holdingR = false;
bool holdingT = false;
bool holdingY = false;
bool holdingI = false;

bool render = true;

//Light attributes
glm::vec3 lightPos(-30.0f, 20.0f, 0.0f);

/**
	Initializes all the libraries, sets up a window, and starts the main loop.
*/
void World::init() {
	//Create a GLFW Window
	GLFWwindow* window = rm.initWindow(WIDTH, HEIGHT);

	//Init glew and view
	rm.initGLEW(window);

	//Set key callbacks
	glfwSetKeyCallback(window, key_callback);
	glfwSetCursorPosCallback(window, mouse_callback);
	glfwSetScrollCallback(window, scroll_callback);

	//OpenGL options
	glfwSetInputMode(window, GLFW_CURSOR, GLFW_CURSOR_DISABLED);
	glEnable(GL_DEPTH_TEST);

	//Build and compile shader program
	Shader lightingShader("lightingShader.vs", "lightingShader.frag");

	//Init pysics 
	pm.initPhysics();

	//Light
	GLuint lightVAO, planeVAO;
	initPlaneAndLight(& lightVAO, & planeVAO);


	//std::cout << "start\n";
	//for(int i = 0; i < 1000; i++){
	//	Box * box = new Box({ 2, 1.6, 200 }, glm::vec3(0.1f, 0.1f, 0.9f), 1.f, 1.0f, 1.f, 0);
	//	delete box;
	//	
	//	std::this_thread::sleep_for(std::chrono::milliseconds(1));
	//}
	//Box * box = new Box({ 2, 1.6, 200 }, glm::vec3(0.1f, 0.1f, 0.9f), 1.f, 1.0f, 1.f, 0);
	////delete box;
	//std::cout << "finish\n";
	//std::this_thread::sleep_for(std::chrono::milliseconds(10000));
	//Start render loop
	renderLoop(window, planeVAO, lightVAO, lightingShader);

	// Properly de-allocate all resources once they've outlived their purpose
	glDeleteVertexArrays(1, &planeVAO);
	glDeleteBuffers(1, &planeVAO);
	glDeleteBuffers(1, &planeVAO);

	//pm.removeBody(groundRigidBody);
	//delete groundRigidBody->getMotionState();
	//delete groundRigidBody;
	//delete groundShape;

	//Terminate
	glfwTerminate();
	return;
}

/**
	Sets up a few render objects and starts the main game loop.

	@param window The applications GLFW render window.
	@param planeVAO Vertex Array Object for the ground plane.
	@param ligthVAO Vertex Array Object for the light source.
	@param lightingShader Shader object for the fragment shader and the vertex shader.
	@param cubes Vector of cubes to place in the world.
		
*/
void World::renderLoop(GLFWwindow* window, GLint planeVAO, GLint lightVAO, Shader lightingShader) {
	
	//Init hinge variables:
	targetAngle = 0.0f;
	targetAngleRightKnee = 0.0f;
	targetAngleLeftKnee = 0.0f;
	targetAngleAbdomen = 0.0f;

	bool isEnableMotor = true;
	
	//Create genetic algorithm
	// mutationRate, mutationChance(for every weight), crossoverProb
	GeneticAlgorithm ga(0.1, 0.05, 0.9, 50, 1, &pm);

	double fps = 0;
	float accumilatedTime = 0;
	bool simulate = false;
	int numLoops = 0;
	int showFPS = 0;
	std::vector<double> resultVec;
	resultVec.push_back(0.0);
	resultVec.push_back(0.0);
	GLfloat startTime = glfwGetTime();
	int frameCount = 0;
	int simTime = 600;
	int terminationFrame = 20;

	//START MAIN LOOP:
	while (!glfwWindowShouldClose(window)) {
		frameCount++;
		// Set frame time
		GLfloat currentFrame = glfwGetTime();
		deltaTime = currentFrame - lastFrame;
		lastFrame = currentFrame;
		accumilatedTime += deltaTime;
		if (accumilatedTime >= 1.0f) {
			
			fps++;
			accumilatedTime = 0;
		}

		if (currentFrame - startTime >= 1.0)
		{
			frameCount = 0;
		}

		showFPS++;

		// Check if any events have been activiated (key pressed, mouse moved etc.) and call corresponding response functions
		glfwPollEvents();
		doMovement();
		
		// Clear the colorbuffer
		glClearColor(0.2f, 0.3f, 0.3f, 1.0f);
		glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

		if (applyImpulse) {
			applyImpulse = false;
			if (simulate) {
				simulate = false;
			}
			else {
				simulate = true;
			}
		}

		//Use shader
		lightingShader.use();

		GLint objectColorLoc = glGetUniformLocation(lightingShader.program, "objectColor");
		GLint lightColorLoc = glGetUniformLocation(lightingShader.program, "lightColor");
		GLint lightPosLoc = glGetUniformLocation(lightingShader.program, "lightPos");
		GLint viewPosLoc = glGetUniformLocation(lightingShader.program, "viewPos");
		glUniform3f(objectColorLoc, 1.0f, 0.5f, 0.31f);
		glUniform3f(lightColorLoc, 1.0f, 1.0f, 1.0f);
		glUniform3f(lightPosLoc, lightPos.x, lightPos.y, lightPos.z);
		glUniform3f(viewPosLoc, camera.Position.x, camera.Position.y, camera.Position.z);

		// Create camera transformations
		glm::mat4 model;
		glm::mat4 view;
		glm::mat4 projection;
		view = camera.GetViewMatrix();
		projection = glm::perspective(camera.Zoom, (GLfloat)WIDTH / (GLfloat)HEIGHT, 0.1f, 1000.0f);

		// Get their uniform location
		GLint viewLoc = glGetUniformLocation(lightingShader.program, "view");
		GLint projLoc = glGetUniformLocation(lightingShader.program, "projection");
		
		// Pass them to the shaders
		glUniformMatrix4fv(viewLoc, 1, GL_FALSE, glm::value_ptr(view));
		
		// Note: currently we set the projection matrix each frame, but since the projection matrix rarely changes it's often best practice to set it outside the main loop only once.
		glUniformMatrix4fv(projLoc, 1, GL_FALSE, glm::value_ptr(projection));

		//GLint vertexColorLocation = glGetUniformLocation(lightingShader.program, "customColor");

		//DRAW PLANE
		//model = glm::rotate(model, 1.57079633f, glm::vec3(1.0f, 0.0f, 0.0f));
		GLint modelLoc = glGetUniformLocation(lightingShader.program, "model");
		glUniformMatrix4fv(modelLoc, 1, GL_FALSE, glm::value_ptr(model));
		glBindVertexArray(planeVAO);
		//glDrawElements(GL_TRIANGLES, 6, GL_UNSIGNED_INT, 0);
		glDrawArrays(GL_TRIANGLES, 0, 36);
		if(simulate) pm.update(1, 1);

		numLoops++;
		ga.updateCreatures(lightingShader, render, &pm);
		if (!ga.keepRunning() || numLoops >= 1500) {
			terminationFrame+= 10;
			GLfloat thisTime = glfwGetTime();
			std::cout << "Generation time: " << thisTime - startTime << std::endl;
			startTime = glfwGetTime();
			ga.createNewGeneration(&pm);
			std::cout << "NumLoops: " << numLoops << "\n";
			numLoops = 0;
		}
		
		//Swap the screen buffers
		glfwSwapBuffers(window);
	}
}

/**
	Alters the camera positions based on user input.
*/
void World::doMovement()
{
	// Camera controls
	if (keys[GLFW_KEY_W])
		camera.ProcessKeyboard(FORWARD, deltaTime);
	if (keys[GLFW_KEY_S])
		camera.ProcessKeyboard(BACKWARD, deltaTime);
	if (keys[GLFW_KEY_A])
		camera.ProcessKeyboard(LEFT, deltaTime);
	if (keys[GLFW_KEY_D])
		camera.ProcessKeyboard(RIGHT, deltaTime);
	if (keys[GLFW_KEY_SPACE])
		camera.ProcessKeyboard(UP, deltaTime);
	if (keys[GLFW_KEY_LEFT_CONTROL])
		camera.ProcessKeyboard(DOWN, deltaTime);
	if (keys[GLFW_KEY_LEFT_SHIFT])
		camera.ProcessKeyboard(BOOST, deltaTime);
}

/**
	Initializes the ground plane rendering (and the light source).

	@param lightVAO Pointer to the lights Vertex Array Object.
	@param planeVAO Pointer to the planes Vertex Array Objectm.
*/
void World::initPlaneAndLight(GLuint* lightVAO, GLuint* planeVAO)
{
	float s = 300.0f;
	GLfloat rectangleVertices[] = {
		s,  0.f, s,  // Top Right
		s, 0.f, -s,  // Bottom Right
		-s, 0.f, -s,  // Bottom Left
		-s,  0.f, s   // Top Left 
	};
	GLuint indices[] = {  // Note that we start from 0!
		0, 1, 3,  // First Triangle
		1, 2, 3   // Second Triangle
	};

	GLfloat BoxVertices[] = {
		-s,  0, -s,  0.0f,  1.0f,  0.0f,
		s,  0, -s,  0.0f,  1.0f,  0.0f,
		s,  0,  s,  0.0f,  1.0f,  0.0f,
		s,  0,  s,  0.0f,  1.0f,  0.0f,
		-s,  0,  s,  0.0f,  1.0f,  0.0f,
		-s,  0, -s,  0.0f,  1.0f,  0.0f
	};


	//PLANE STUFF
	//VBO = vertex buffer objects. Can store a large number of vertices in the GPU memory.
	//VAO = vertex array object. Stores vertex buffer objects so that they can be easily swapped during rendering.
	//EBO = element buffer objects. Used to add triangles together to for instance draw a rectangle(avoids overhead).

	GLuint planeVBO, planeEBO;
	glGenVertexArrays(1, planeVAO);
	glGenBuffers(1, &planeVBO);
	//glGenBuffers(1, &planeEBO);
	// Bind the Vertex Array Object first, then bind and set vertex buffer(s) and attribute pointer(s).
	glBindVertexArray(*planeVAO);

	glBindBuffer(GL_ARRAY_BUFFER, planeVBO);
	glBufferData(GL_ARRAY_BUFFER, sizeof(BoxVertices), BoxVertices, GL_STATIC_DRAW);

	//glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, planeEBO);
	//glBufferData(GL_ELEMENT_ARRAY_BUFFER, sizeof(indices), indices, GL_STATIC_DRAW);

	//Position attribute
	glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, 6 * sizeof(GLfloat), (GLvoid*)0);
	glEnableVertexAttribArray(0);

	//Color attribute
	glVertexAttribPointer(1, 3, GL_FLOAT, GL_FALSE, 6 * sizeof(GLfloat), (GLvoid*)(3 * sizeof(GLfloat)));
	glEnableVertexAttribArray(1);

	glBindBuffer(GL_ARRAY_BUFFER, 0); // Note that this is allowed, the call to glVertexAttribPointer registered VBO as the currently bound vertex buffer object so afterwards we can safely unbind

	glBindVertexArray(0); // Unbind VAO (it's always a good thing to unbind any buffer/array to prevent strange bugs), remember: do NOT unbind the EBO, keep it bound to this VAO

	glGenVertexArrays(1, lightVAO);
	glBindVertexArray(*lightVAO);
	// We only need to bind to the VBO (to link it with glVertexAttribPointer), no need to fill it; the VBO's data already contains all we need.
	glBindBuffer(GL_ARRAY_BUFFER, planeVBO);
	// Set the vertex attributes (only position data for the lamp))
	glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, 6 * sizeof(GLfloat), (GLvoid*)0); // Note that we skip over the normal vectors
	glEnableVertexAttribArray(0);
	glBindVertexArray(0);
}

/**
	Helper method to change target angle of hinges.
*/
void World::incrementTargetAngles()
{
	float factor = 0.2f;
	if (holdingO) {
		if (targetAngleRightKnee <= PI) {
			targetAngleRightKnee += factor;
		}
	}
	if (holdingP) {
		if (targetAngleRightKnee >= -0.1f) {
			targetAngleRightKnee -= factor;
		}
	}
	if (holdingR) {
		if (targetAngleLeftKnee <= PI) {
			targetAngleLeftKnee += factor;
		}
	}
	if (holdingT) {
		if (targetAngleLeftKnee >= -0.1f) {
			targetAngleLeftKnee -= factor;
		}
	}
	if (holdingY) {
		if (targetAngleAbdomen <= PI) {
			targetAngleAbdomen += factor;
		}
	}
	if (holdingI) {
		if (targetAngleAbdomen >= -PI) {
			targetAngleAbdomen -= factor;
		}
	}
	//std::cout << targetAngleLeftKnee << std::endl;
}

/**
	Called whenever a key is pressed or released via GLFW.

	@param window The applications GLFW render window.
	@param key What key that is being activated.
	@param scancode
	@param action What action to listen for, for instance PRESS or RELEASE.
	@param mode
*/
void key_callback(GLFWwindow* window, int key, int scancode, int action, int mode)
{
	if (key == GLFW_KEY_ESCAPE && action == GLFW_PRESS)
		glfwSetWindowShouldClose(window, GL_TRUE);
	if (key >= 0 && key < 1024)
	{
		if (action == GLFW_PRESS)
			keys[key] = true;
		else if (action == GLFW_RELEASE)
			keys[key] = false;
	}
	if (key == GLFW_KEY_F && action == GLFW_PRESS) {
		applyImpulse = true;
	}

	if (key == GLFW_KEY_G && action == GLFW_PRESS) {
		if (motorImpulses) {
			motorImpulses = false;
		}
		else {
			motorImpulses = true;
		}
	}

	if (key == GLFW_KEY_O && action == GLFW_PRESS) {
		holdingO = true;
	}
	if (key == GLFW_KEY_O && action == GLFW_RELEASE) {
		holdingO = false;
	}
	if (key == GLFW_KEY_P && action == GLFW_PRESS) {
		holdingP = true;
	}
	if (key == GLFW_KEY_P && action == GLFW_RELEASE) {
		holdingP = false;
	}
	if (key == GLFW_KEY_R && action == GLFW_PRESS) {
		holdingR = true;
	}
	if (key == GLFW_KEY_R && action == GLFW_RELEASE) {
		holdingR = false;
	}
	if (key == GLFW_KEY_T && action == GLFW_PRESS) {
		holdingT = true;
	}
	if (key == GLFW_KEY_T && action == GLFW_RELEASE) {
		holdingT = false;
	}
	if (key == GLFW_KEY_Y && action == GLFW_PRESS) {
		holdingY = true;
	}
	if (key == GLFW_KEY_Y && action == GLFW_RELEASE) {
		holdingY = false;
	}
	if (key == GLFW_KEY_I && action == GLFW_PRESS) {
		holdingI = true;
	}
	if (key == GLFW_KEY_I && action == GLFW_RELEASE) {
		holdingI = false;
	}

	if (key == GLFW_KEY_V && action == GLFW_PRESS) {
		if (render) {
			render = false;
		}
		else {
			render = true;
		}
	}
}

/**
	Called whenever a mouse movement is recognized.	

	@param window The applications GLFW render window.
	@param xpos The x position of the mouse pointer.
	@param ypos The y position of the mouse pointer.
*/
void mouse_callback(GLFWwindow* window, double xpos, double ypos)
{
	if (firstMouse)
	{
		lastX = xpos;
		lastY = ypos;
		firstMouse = false;
	}

	GLfloat xoffset = xpos - lastX;
	GLfloat yoffset = lastY - ypos;  // Reversed since y-coordinates go from bottom to left

	lastX = xpos;
	lastY = ypos;

	camera.ProcessMouseMovement(xoffset, yoffset);
}

/**
	Called whenver the mouse scroller is activated.

	@param window The applications GLFW render window.
	@param xoffset
	@param yoffset
*/
void scroll_callback(GLFWwindow* window, double xoffset, double yoffset)
{
	camera.ProcessMouseScroll(yoffset);
}



World::~World()
{
}
