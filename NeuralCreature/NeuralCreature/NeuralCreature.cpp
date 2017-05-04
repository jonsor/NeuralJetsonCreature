/**
NeuralCerature.cpp
Purpose: Main application file. Initializes all the libraries and sets up the world, then starts the main game loop.

@author Sjur Barndon, Jonas Sørsdal
@version 1.0 23.03.2017
*/

#include "stdafx.h"
#include "NeuralCreature.h"

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
bool applyImpulse = false;
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

//Light attributes
glm::vec3 lightPos(-30.0f, 20.0f, 0.0f);

/**
	Initializes all the libraries, sets up a window, and starts the main loop.
*/
void NeuralCreature::init() {
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

	GLuint lightVAO, planeVAO;
	initPlaneAndLight(& lightVAO, & planeVAO);

	//Populate the world with cubes:
	

	Cube cube1(glm::vec3(6.0f, 60.0f, 1.0f), glm::vec3(0.2f, 0.3f, 0.7f), 1.0f, 1.0f, 1.0f, 10);
	Cube cube2(glm::vec3(6.0f, 40.0f, 1.0f), glm::vec3(0.2f, 0.3f, 0.7f), 2.0f, 0.5f, 1.0f, 20);
	Cube cube3(glm::vec3(6.0f, 20.0f, 1.0f), glm::vec3(0.2f, 0.3f, 0.7f), 0.5f, 1.0f, 0.5f, 1);
	pm.addBody(cube1.getRigidBody());
	pm.addBody(cube2.getRigidBody());
	pm.addBody(cube3.getRigidBody());

	cube1.addHinge(glm::vec3(5.0f, 0.0f, 0.0f), glm::vec3(-5.0f, 0.0f, 0.0f), glm::vec3(0.0f, 1.0f, 0.0f), glm::vec3(0.0f, 1.0f, 0.0f), & cube2, true, & pm, "h1");

	std::vector<Cube> cubes;
	//cubes.push_back(cube1);
	//cubes.push_back(cube2);
	//cubes.push_back(cube3);

	//Plane
	btCollisionShape* groundShape = new btStaticPlaneShape(btVector3(0, 1, 0), 1);

	btDefaultMotionState* groundMotionState = new btDefaultMotionState(btTransform(btQuaternion(0, 0, 0, 1), btVector3(0, -1, 0)));
	btRigidBody::btRigidBodyConstructionInfo groundRigidBodyCI(0, groundMotionState, groundShape, btVector3(0, 0, 0));
	btRigidBody* groundRigidBody = new btRigidBody(groundRigidBodyCI);
	//groundRigidBody->setFriction(5.0f);
	pm.addBody(groundRigidBody, 2, 1);

	//Start render loop
	renderLoop(window, planeVAO, lightVAO, lightingShader, cubes);

	// Properly de-allocate all resources once they've outlived their purpose
	glDeleteVertexArrays(1, &planeVAO);
	glDeleteBuffers(1, &planeVAO);
	glDeleteBuffers(1, &planeVAO);

	pm.removeBody(groundRigidBody);
	delete groundRigidBody->getMotionState();
	delete groundRigidBody;

	//delete fallShape;
	delete groundShape;

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
void NeuralCreature::renderLoop(GLFWwindow* window, GLint planeVAO, GLint lightVAO, Shader lightingShader, std::vector<Cube> cubes) {
	
	//Init hinge variables:
	targetAngle = 0.0f;
	targetAngleRightKnee = 0.0f;
	targetAngleLeftKnee = 0.0f;
	targetAngleAbdomen = 0.0f;

	bool isEnableMotor = true;
	btScalar maxMotorImpulse = 20.0f; // 1.0f / 8.0f is about the minimum

	//Creates the creature
	//Ceature creature(&pm, glm::vec3(0.0f, 0.0f, 0.0f));
	//Spider spider(&pm);
	
	//Create genetic algorithm
	GeneticAlgorithm ga(0.03, 0.1, 10, 1, &pm);

/*
	Cube lightPosMarker(lightPos, glm::vec3(0.2f, 0.3f, 0.7f), 0.5f, 0.5f, 0.5f, 10);
	pm.addBody(lightPosMarker.getRigidBody());

	Cube fallCube(glm::vec3(0.0f, 1.0f, 2.0f), glm::vec3(0.2f, 0.3f, 0.7f), 1.0f, 1.0f, 1.0f, 5);
	pm.addBody(fallCube.getRigidBody());
*/
	Cube cameraCollisionBox(glm::vec3(0.0f, 1.0f, 2.0f), glm::vec3(0.2f, 0.3f, 0.7f), 1.0f, 1.0f, 1.0f, 5);
	pm.addBody(cameraCollisionBox.getRigidBody());

	float fps = 0;
	float accumilatedTime = 0;
	bool simulate = false;
	int numLoops = 0;
	std::vector<double> resultVec;
	resultVec.push_back(0.0);
	resultVec.push_back(0.0);

	//START MAIN LOOP:
	while (!glfwWindowShouldClose(window)) {
		fps++;
		// Set frame time
		GLfloat currentFrame = glfwGetTime();
		deltaTime = currentFrame - lastFrame;
		lastFrame = currentFrame;
		accumilatedTime += deltaTime;
		if (accumilatedTime > 0.2f) {
			//std::cout << "fps: " << fps << std::endl;
			fps = 0;
			accumilatedTime = 0;
			//double time = sin(0.000002 * std::chrono::system_clock::now().time_since_epoch().count() );
			//std::cout << time << std::endl;
		}
		// Check if any events have been activiated (key pressed, mouse moved etc.) and call corresponding response functions
		glfwPollEvents();
		doMovement();
		
		// Clear the colorbuffer
		glClearColor(0.2f, 0.3f, 0.3f, 1.0f);
		glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

		if (applyImpulse) {
			//fallCube.getRigidBody()->activate();
			//fallCube.getRigidBody()->applyImpulse(btVector3(5, 10, 3), btVector3(1, 0, 0));
			applyImpulse = false;
			//float rand = std::rand() % 360;
			//targetAngle = glm::radians(rand);
			//std::cout << targetAngle << std::endl;
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
		//glUniform4f(vertexColorLocation, 0.2f, 0.5f, 0.3f, 1.0f);
		model = glm::rotate(model, 1.57079633f, glm::vec3(1.0f, 0.0f, 0.0f));
		GLint modelLoc = glGetUniformLocation(lightingShader.program, "model");
		glUniformMatrix4fv(modelLoc, 1, GL_FALSE, glm::value_ptr(model));
		glBindVertexArray(planeVAO);
		glDrawElements(GL_TRIANGLES, 6, GL_UNSIGNED_INT, 0);

		if(simulate) pm.update(deltaTime, 1);

		//Camera box
		cameraCollisionBox.setPosition(glm::vec3(camera.Position.x, camera.Position.y, camera.Position.z));
		btDefaultMotionState* ms = new btDefaultMotionState(btTransform(btQuaternion(0, 0, 0, 1), btVector3(camera.Position.x, camera.Position.y, camera.Position.z)));
		cameraCollisionBox.getRigidBody()->setMotionState(ms);
		cameraCollisionBox.updatePhysics();

		/*
		spider.updatePhysics();
		spider.render(lightingShader);

		lightPosMarker.render(lightingShader);

		fallCube.updatePhysics();
		fallCube.render(lightingShader);
		*/
		for (int i = 0; i < cubes.size(); i++) {
			cubes[i].updatePhysics();
			cubes[i].render(lightingShader);

		}

		//Update and render Creature
		//creature.activate();
		//creature.updatePhysics();
		//creature.render(lightingShader);

		ga.updateCreatures(lightingShader);
		if (numLoops >= 400) {
			ga.createNewGeneration();
			numLoops = 0;
		}
		numLoops++;
		std::cout << numLoops << std::endl;
		
		////Neural Network
		//std::vector<int> topology{ 6, 10, 7, 6 };
		//NeuralNetwork neuralNet(topology);
		//std::vector<double> inputAngles = creature.getAllAngles();
		//neuralNet.forward(inputAngles);

		////Update motor impulses
		//neuralNet.getResults(resultVec);
		//creature.setMaxMotorImpulses(maxMotorImpulse);
		//creature.setAllTargetVelocities(resultVec);

		//double mU = 0.0;
		//std::cout << resultVec[0]- mU << "    " << resultVec[1]- mU << "    " << resultVec[2]- mU << "    " << resultVec[3]- mU << std::endl;
		////std::cout << creature.getRightThigh()->getHinge("rightKnee")->getHingeAngle() << std::endl;
		//numLoops = 1;
		//
		//creature.getCenterPosition();
		//numLoops++;

		//std::cout << "right: " << creature.getRelativePosition(creature.getRightThigh()).x << " left: " << creature.getRelativePosition(creature.getLeftThigh()).x << std::endl;
	/*
		if (motorImpulses) {
			//Update creature's hinge motors: 
			//creature.getChest()->getHinge("abdomen")->enableMotor(isEnableMotor);
			
			creature.getHips()->getHinge("leftHip")->enableMotor(isEnableMotor);
			creature.getHips()->getHinge("rightHip")->enableMotor(isEnableMotor);
			creature.getRightThigh()->getHinge("rightKnee")->enableMotor(isEnableMotor);
			creature.getLeftThigh()->getHinge("leftKnee")->enableMotor(isEnableMotor);

			//creature.getChest()->getHinge("abdomen")->setMaxMotorImpulse(maxMotorImpulse);
			creature.getHips()->getHinge("leftHip")->setMaxMotorImpulse(maxMotorImpulse);
			creature.getHips()->getHinge("rightHip")->setMaxMotorImpulse(maxMotorImpulse);
			creature.getRightThigh()->getHinge("rightKnee")->setMaxMotorImpulse(maxMotorImpulse);
			creature.getLeftThigh()->getHinge("leftKnee")->setMaxMotorImpulse(maxMotorImpulse);

			incrementTargetAngles();

			//creature.getChest()->getHinge("abdomen")->setMotorTarget(targetAngleAbdomen, deltaTime);
			creature.getHips()->getHinge("leftHip")->setMotorTargetVelocity(targetAngle);
			creature.getHips()->getHinge("rightHip")->setMotorTargetVelocity(targetAngle);
			creature.getRightThigh()->getHinge("rightKnee")->setMotorTargetVelocity(targetAngleRightKnee);
			creature.getLeftThigh()->getHinge("leftKnee")->setMotorTargetVelocity(targetAngleLeftKnee);
		
		}
		*/
		//Swap the screen buffers
		glfwSwapBuffers(window);

	}

	//pm.removeBody(fallCube.getRigidBody());
	//delete fallCube.getRigidBody()->getMotionState();
	//delete fallCube.getRigidBody();

	
}

/**
	Alters the camera positions based on user input.
*/
void NeuralCreature::doMovement()
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
void NeuralCreature::initPlaneAndLight(GLuint* lightVAO, GLuint* planeVAO)
{
	float s = 50.0f;
	GLfloat rectangleVertices[] = {
		s,  s, 0.0f,  // Top Right
		s, -s, 0.0f,  // Bottom Right
		-s, -s, 0.0f,  // Bottom Left
		-s,  s, 0.0f   // Top Left 
	};
	GLuint indices[] = {  // Note that we start from 0!
		0, 1, 3,  // First Triangle
		1, 2, 3   // Second Triangle
	};

	//PLANE STUFF
	//VBO = vertex buffer objects. Can store a large number of vertices in the GPU memory.
	//VAO = vertex array object. Stores vertex buffer objects so that they can be easily swapped during rendering.
	//EBO = element buffer objects. Used to add triangles together to for instance draw a rectangle(avoids overhead).
	GLuint planeVBO, planeEBO;
	glGenVertexArrays(1, planeVAO);
	glGenBuffers(1, &planeVBO);
	glGenBuffers(1, &planeEBO);
	// Bind the Vertex Array Object first, then bind and set vertex buffer(s) and attribute pointer(s).
	glBindVertexArray(*planeVAO);

	glBindBuffer(GL_ARRAY_BUFFER, planeVBO);
	glBufferData(GL_ARRAY_BUFFER, sizeof(rectangleVertices), rectangleVertices, GL_STATIC_DRAW);

	glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, planeEBO);
	glBufferData(GL_ELEMENT_ARRAY_BUFFER, sizeof(indices), indices, GL_STATIC_DRAW);

	//Position attribute
	glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, 3 * sizeof(GLfloat), (GLvoid*)0);
	glEnableVertexAttribArray(0);

	//Color attribute
	glVertexAttribPointer(1, 3, GL_FLOAT, GL_FALSE, 3 * sizeof(GLfloat), (GLvoid*)(3 * sizeof(GLfloat)));
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
void NeuralCreature::incrementTargetAngles()
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



NeuralCreature::~NeuralCreature()
{
}
