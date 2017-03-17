#include "stdafx.h"
#include "OpenGLTutorial.h"
#define GLEW_STATIC

//Prototypes
void key_callback(GLFWwindow* window, int key, int scancode, int action, int mode);
void key_callback(GLFWwindow* window, int key, int scancode, int action, int mode);
void scroll_callback(GLFWwindow* window, double xoffset, double yoffset);
void mouse_callback(GLFWwindow* window, double xpos, double ypos);

//TODO Fiks disse slik at de kan brukes i headeren. 
//Vanskelig siden callback funksjonene ikke funker hvis de er klassemetoder OpenGLTutorial::key_callback f.eks
// Camera
Camera camera(glm::vec3(0.0f, 3.0f, 10.0f));

bool keys[1024];
GLfloat lastX, lastY;
bool firstMouse = true;

GLfloat deltaTime = 0.0f;
GLfloat lastFrame = 0.0f;
// Light attributes
glm::vec3 lightPos(-21.2f, 5.0f, 2.0f);
// x y z

bool applyImpulse = false;


//https://learnopengl.com/#!Getting-started/Hello-Triangle

OpenGLTutorial::OpenGLTutorial()
{
}

void OpenGLTutorial::init() {
	glfwInit();
	glfwWindowHint(GLFW_CONTEXT_VERSION_MAJOR, 3);
	glfwWindowHint(GLFW_CONTEXT_VERSION_MINOR, 3);
	glfwWindowHint(GLFW_OPENGL_PROFILE, GLFW_OPENGL_CORE_PROFILE);
	glfwWindowHint(GLFW_RESIZABLE, GL_FALSE);
	glfwWindowHint(GLFW_SAMPLES, 4); // 4x antialiasing
	glfwSwapInterval(1);
	GLFWwindow* window = glfwCreateWindow(WIDTH, HEIGHT, "OPENGL TUTORIAL", nullptr, nullptr);

	if (window == nullptr) {
		std::cout << "Failed to create GLFW window" << std::endl;
		glfwTerminate();
		return;
	}
	glfwMakeContextCurrent(window);

	//Set key callbacks
	glfwSetKeyCallback(window, key_callback);
	glfwSetCursorPosCallback(window, mouse_callback);
	glfwSetScrollCallback(window, scroll_callback);

	// Options
	glfwSetInputMode(window, GLFW_CURSOR, GLFW_CURSOR_DISABLED);

	//init glew 
	initGLEW();

	//Init view
	initView(window);
	// Setup some OpenGL options

	glEnable(GL_DEPTH_TEST);
	//Build anc compile shader program
	Shader lightingShader("lightingShader.vs", "lightingShader.frag");

	//Build and compile shaders
	//GLint shaderProgram = initShaders();

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
	GLuint planeVBO, planeVAO, planeEBO;
	glGenVertexArrays(1, &planeVAO);
	glGenBuffers(1, &planeVBO);
	glGenBuffers(1, &planeEBO);
	// Bind the Vertex Array Object first, then bind and set vertex buffer(s) and attribute pointer(s).
	glBindVertexArray(planeVAO);

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

	GLuint lightVAO;
	glGenVertexArrays(1, &lightVAO);
	glBindVertexArray(lightVAO);
	// We only need to bind to the VBO (to link it with glVertexAttribPointer), no need to fill it; the VBO's data already contains all we need.
	glBindBuffer(GL_ARRAY_BUFFER, planeVBO);
	// Set the vertex attributes (only position data for the lamp))
	glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, 6 * sizeof(GLfloat), (GLvoid*)0); // Note that we skip over the normal vectors
	glEnableVertexAttribArray(0);
	glBindVertexArray(0);

	//Start render loop
	Cube cube1(glm::vec3(0.0f, 1.0f, 2.0f), glm::vec3(0.2f, 0.3f, 0.7f), 1.0f, 1.0f, 1.0f);
	Cube cube2(glm::vec3(5.0f, 2.0f, 2.0f), glm::vec3(0.2f, 0.3f, 0.7f), 2.0f, 0.5f, 1.0f);
	Cube cube3(glm::vec3(-3.0f, 1.0f, 4.0f), glm::vec3(0.2f, 0.3f, 0.7f), 0.5f, 1.0f, 0.5f);
	std::vector<Cube> cubes;
	cubes.push_back(cube1);
	cubes.push_back(cube2);
	cubes.push_back(cube3);
	renderLoop(window, planeVAO, lightVAO, lightingShader, cubes);

	// Properly de-allocate all resources once they've outlived their purpose
	glDeleteVertexArrays(1, &planeVAO);
	glDeleteBuffers(1, &planeVAO);
	glDeleteBuffers(1, &planeVAO);

	//Terminate
	glfwTerminate();
	return;
}

void OpenGLTutorial::initGLEW() {
	glewExperimental = GL_TRUE;
	if (glewInit() != GLEW_OK) {
		std::cout << "Failed to initialize GLEW" << std::endl;
		return;
	}
}

void OpenGLTutorial::initView(GLFWwindow* window)
{
	int width, height;
	glfwGetFramebufferSize(window, &width, &height);
	glViewport(0, 0, width, height);
}
btScalar targetAngle;


void OpenGLTutorial::renderLoop(GLFWwindow* window, GLint planeVAO, GLint lightVAO, Shader lightingShader, std::vector<Cube> cubes) {
	PhysicsManager pysMan;
	pysMan.initPhysics();

	btCollisionShape* groundShape = new btStaticPlaneShape(btVector3(0, 1, 0), 1);

	btDefaultMotionState* groundMotionState = new btDefaultMotionState(btTransform(btQuaternion(0, 0, 0, 1), btVector3(0, -1, 0)));
	btRigidBody::btRigidBodyConstructionInfo groundRigidBodyCI(0, groundMotionState, groundShape, btVector3(0, 0, 0));
	btRigidBody* groundRigidBody = new btRigidBody(groundRigidBodyCI);
	//groundRigidBody->setFriction(5.0f);
	pysMan.addBody(groundRigidBody);

	//Hinge stuff
	//btScalar targetAngle;
	btCollisionShape* rightCollisionShape;
	btCollisionShape* middleCollisionShape;
	btCollisionShape* leftCollisionShape;
	btRigidBody* rightRigidBody;
	btRigidBody* middleRigidBody;
	btRigidBody* leftRigidBody;
	btHingeConstraint* rightHingeConstraint;
	btHingeConstraint* leftHingeConstraint;

	//Init hinge stuff
	targetAngle = 0.0f;
	btScalar oldAngle = 0.0f;
	// create collision shapes
	const btVector3 rightBoxHalfExtents(0.5f, 0.5f, 0.2f);
	rightCollisionShape = new btBoxShape(rightBoxHalfExtents);
	Cube hinge1(glm::vec3(5.0f, 1.0f, 2.0f), glm::vec3(0.4f, 0.8f, 0.3f), 0.5f, 0.5f, 0.2f);

	const btVector3 middleBoxHalfExtents(0.5f, 0.5f, 0.2f);
	middleCollisionShape = new btBoxShape(middleBoxHalfExtents);
	Cube hinge2(glm::vec3(8.0f, 1.0f, 2.0f), glm::vec3(0.7f, 0.3f, 0.4f), 0.5f, 0.5f, 0.2f);

	const btVector3 leftBoxHalfExtents(0.5f, 0.5f, 0.2f);
	leftCollisionShape = new btBoxShape(leftBoxHalfExtents);
	Cube hinge3(glm::vec3(8.0f, 1.0f, 2.0f), glm::vec3(0.1f, 0.3f, 0.4f), 0.5f, 0.5f, 0.2f);

	// create right rigid body
	const btScalar rightMass = 10.0f;
	btTransform rightTransform;
	rightTransform.setIdentity();
	//x z y
	const btVector3 rightOrigin(4.0f, 0.5f, 5.0f);
	rightTransform.setOrigin(rightOrigin);
	rightRigidBody = createRigidBody(rightCollisionShape, rightMass, rightTransform);
	pysMan.addBody(rightRigidBody);

	// create middle rigid body
	const btScalar middleMass = 1.0f;
	btTransform middleTransform;
	middleTransform.setIdentity();
	const btVector3 middleOrigin(4.0f, 0.5f, 5.0f);
	middleTransform.setOrigin(middleOrigin);
	middleRigidBody = createRigidBody(middleCollisionShape, middleMass, middleTransform);
	pysMan.addBody(middleRigidBody);

	// create left rigid body
	const btScalar leftMass = 10.0f;
	btTransform leftTransform;
	leftTransform.setIdentity();
	const btVector3 leftOrigin(4.5f, 0.5f, 5.0f);
	leftTransform.setOrigin(leftOrigin);
	leftRigidBody = createRigidBody(leftCollisionShape, leftMass, leftTransform);
	pysMan.addBody(leftRigidBody);


	// create right hinge constraint
	const btVector3 pivotInA(1.0f, 0.0f, 0.0f);
	const btVector3 pivotInB(-1.0f, 0.0f, 0.0f);
	btVector3 axisInA(0.0f, 0.0f, 1.0f);
	btVector3 axisInB(0.0f, 0.0f, 1.0f);
	bool useReferenceFrameA = false;
	rightHingeConstraint = new btHingeConstraint(
		*rightRigidBody,
		*middleRigidBody,
		pivotInA,
		pivotInB,
		axisInA,
		axisInB,
		useReferenceFrameA);

	const double PI = 3.141592653589793238463;

	// set constraint limit
	const btScalar low = -PI;
	const btScalar high = PI;
	rightHingeConstraint->setLimit(low, high);
	//hingeConstraint->setLimit(0, 0.2f);

	//Create left hinge constraint
	//const btVector3 pivotInA(1.0f, 0.0f, 0.0f);
	//const btVector3 pivotInB(-1.0f, 0.0f, 0.0f);
	//btVector3 axisInA(0.0f, 0.0f, 1.0f);
	//btVector3 axisInB(0.0f, 0.0f, 1.0f);
	//bool useReferenceFrameA = false;
	leftHingeConstraint = new btHingeConstraint(
		*middleRigidBody,
		*leftRigidBody,
		pivotInA,
		pivotInB,
		axisInA,
		axisInB,
		useReferenceFrameA);

	// add constraint to the world
	const bool isDisableCollisionsBetweenLinkedBodies = false;
	pysMan.addNewConstraint(rightHingeConstraint,
		isDisableCollisionsBetweenLinkedBodies);
	pysMan.addNewConstraint(leftHingeConstraint,
		isDisableCollisionsBetweenLinkedBodies);

	//Box
	btCollisionShape* fallShape = new btBoxShape(btVector3(1, 1, 1));

	btDefaultMotionState* fallMotionState = new btDefaultMotionState(btTransform(btQuaternion(0, 0, 0, 1), btVector3(0, 50, 0)));
	btScalar mass = 1;
	btVector3 fallInertia(0, 0, 0);
	fallShape->calculateLocalInertia(mass, fallInertia);
	btRigidBody::btRigidBodyConstructionInfo fallRigidBodyCI(mass, fallMotionState, fallShape, fallInertia);
	btRigidBody* fallRigidBody = new btRigidBody(fallRigidBodyCI);
	pysMan.addBody(fallRigidBody);

	float fps = 0;
	float accumilatedTime = 0;
	//Start render loop
	while (!glfwWindowShouldClose(window)) {
		fps++;
		// Set frame time
		GLfloat currentFrame = glfwGetTime();
		deltaTime = currentFrame - lastFrame;
		lastFrame = currentFrame;
		accumilatedTime += deltaTime;
		if (accumilatedTime > 1.0f) {
			std::cout << "fps: " << fps << std::endl;
			fps = 0;
			accumilatedTime = 0;
		}
		// Check if any events have been activiated (key pressed, mouse moved etc.) and call corresponding response functions
		glfwPollEvents();
		doMovement();
		// Render
		// Clear the colorbuffer
		glClearColor(0.2f, 0.3f, 0.3f, 1.0f);
		glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
		//Render here

		if (applyImpulse) {
			fallRigidBody->activate();
			fallRigidBody->applyImpulse(btVector3(5, 10, 3), btVector3(1, 0, 0));
			applyImpulse = false;
			//float rand = std::rand() % 360;
			//targetAngle = glm::radians(rand);
			//std::cout << targetAngle << std::endl;
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

		pysMan.update(deltaTime, 1);

		btTransform trans;
		fallRigidBody->getMotionState()->getWorldTransform(trans);
		for (int i = 0; i < cubes.size(); i++) {
			if (i == 0) {
				float mat[16];
				trans.getOpenGLMatrix(mat);
				trans.getRotation().getX();
				//Model = glm::rotate(Model, angle_in_degrees, glm::vec3(x, y, z));
				//trans.getRotation().angle();
				//std::cout << trans.getRotation().getAngle() << "   " << trans.getRotation().getX() << "  " << trans.getRotation().getY() << "   " << trans.getRotation().getZ() << "  " << trans.getRotation().getW()<< std::endl;
				cubes[i].setPosition(glm::vec3(trans.getOrigin().getX(), trans.getOrigin().getY(), trans.getOrigin().getZ()));
				float x = trans.getRotation().getX();
				float y = trans.getRotation().getY();
				float z = trans.getRotation().getZ();
				float angle = trans.getRotation().getAngle();
				//std::cout << angle << "   " << x << " " << y << " " << z << std::endl;
				cubes[i].setRotation(angle, glm::vec3(x, y, z));
			}

			cubes[i].render(lightingShader);

		}

		//Rigid body stuff
		rightRigidBody->activate();
		middleRigidBody->activate();

		bool isEnableMotor = true;
		btScalar maxMotorImpulse = 2.0f; // 1.0f / 8.0f is about the minimum

		rightHingeConstraint->enableMotor(isEnableMotor);
		rightHingeConstraint->setMaxMotorImpulse(maxMotorImpulse);

		leftHingeConstraint->enableMotor(isEnableMotor);
		leftHingeConstraint->setMaxMotorImpulse(maxMotorImpulse);
		//targetAngle += 0.1f * deltaTime;

		//if (oldAngle != targetAngle) {
			rightHingeConstraint->setMotorTarget(targetAngle, deltaTime);
			leftHingeConstraint->setMotorTarget(targetAngle, deltaTime);
			oldAngle = targetAngle;
		//}

		std::cout << rightHingeConstraint->getMaxMotorImpulse() << "  target: " << targetAngle << std::endl;

		//Hinge 1
		btTransform hinge1Trans;
		rightRigidBody->getMotionState()->getWorldTransform(hinge1Trans);

		hinge1.setPosition(glm::vec3(hinge1Trans.getOrigin().getX(), hinge1Trans.getOrigin().getY(), hinge1Trans.getOrigin().getZ()));
		//hinge1.setPosition(glm::vec3(0.0f, hinge1Trans.getOrigin().getY(), 0.0f));
		float x = hinge1Trans.getRotation().getX();
		float y = hinge1Trans.getRotation().getY();
		float z = hinge1Trans.getRotation().getZ();
		float angle = hinge1Trans.getRotation().getAngle();
		hinge1.setRotation(angle, glm::vec3(x, y, z));

		//Hinge 2

		btTransform hinge2Trans;
		middleRigidBody->getMotionState()->getWorldTransform(hinge2Trans);

		hinge2.setPosition(glm::vec3(hinge2Trans.getOrigin().getX(), hinge2Trans.getOrigin().getY(), hinge2Trans.getOrigin().getZ()));
		x = hinge2Trans.getRotation().getX();
		y = hinge2Trans.getRotation().getY();
		z = hinge2Trans.getRotation().getZ();
		angle = hinge2Trans.getRotation().getAngle();
		hinge2.setRotation(angle, glm::vec3(x, y, z));

		//Hinge 3
		btTransform hinge3Trans;
		leftRigidBody->getMotionState()->getWorldTransform(hinge3Trans);

		hinge3.setPosition(glm::vec3(hinge3Trans.getOrigin().getX(), hinge3Trans.getOrigin().getY(), hinge3Trans.getOrigin().getZ()));
		x = hinge3Trans.getRotation().getX();
		y = hinge3Trans.getRotation().getY();
		z = hinge3Trans.getRotation().getZ();
		angle = hinge3Trans.getRotation().getAngle();
		hinge3.setRotation(angle, glm::vec3(x, y, z));

		hinge1.render(lightingShader);
		hinge2.render(lightingShader);
		hinge3.render(lightingShader);
		// Swap the screen buffers
		glfwSwapBuffers(window);

	}

	pysMan.removeBody(fallRigidBody);
	delete fallRigidBody->getMotionState();
	delete fallRigidBody;

	pysMan.removeBody(groundRigidBody);
	delete groundRigidBody->getMotionState();
	delete groundRigidBody;

	delete fallShape;
	delete groundShape;
}

btRigidBody* OpenGLTutorial::createRigidBody(btCollisionShape* collisionShape, btScalar mass, const btTransform& transform) const
{
	// calculate inertia
	btVector3 localInertia(0.0f, 0.0f, 0.0f);
	collisionShape->calculateLocalInertia(mass, localInertia);

	// create motion state
	btDefaultMotionState* defaultMotionState
		= new btDefaultMotionState(transform);

	// create rigid body
	btRigidBody::btRigidBodyConstructionInfo rigidBodyConstructionInfo(
		mass, defaultMotionState, collisionShape, localInertia);
	btRigidBody* rigidBody = new btRigidBody(rigidBodyConstructionInfo);

	return rigidBody;
}

//GLint OpenGLTutorial::initShaders() {
//
//	//Build and compile vertex shader
//	GLuint vertexShader;
//	vertexShader = glCreateShader(GL_VERTEX_SHADER);
//	glShaderSource(vertexShader, 1, &vertexShaderSource, NULL);
//	glCompileShader(vertexShader);
//
//	//Check if shader compliation  is successful
//	GLint success;
//	GLchar infoLog[512];
//	glGetShaderiv(vertexShader, GL_COMPILE_STATUS, &success);
//	if (!success) {
//		glGetShaderInfoLog(vertexShader, 512, NULL, infoLog);
//		std::cout << "ERROR::SHADER::VERTEX::COMPLIATION_FAILED\n" << infoLog << std::endl;
//	}
//
//	//Build and compile fragment shader
//	GLint fragmentShader;
//	fragmentShader = glCreateShader(GL_FRAGMENT_SHADER);
//	glShaderSource(fragmentShader, 1, &fragmentShaderSource, NULL);
//	glCompileShader(fragmentShader);
//
//	//Link shaders to shader program
//	GLint shaderProgram;
//	shaderProgram = glCreateProgram();
//	glAttachShader(shaderProgram, vertexShader);
//	glAttachShader(shaderProgram, fragmentShader);
//	glLinkProgram(shaderProgram);
//
//	//Check is shaderProgram compilation is successful
//	glGetProgramiv(shaderProgram, GL_LINK_STATUS, &success);
//	if (!success) {
//		glGetProgramInfoLog(shaderProgram, 512, NULL, infoLog);
//		std::cout << "ERROR::SHADER::PROGRAM::LINKING_FAILED\n" << infoLog << std::endl;
//	}
//
//	//Use shader program
//	glUseProgram(shaderProgram);
//
//	//Delete unused shaders (only need shaderProgram)
//	glDeleteShader(vertexShader);
//	glDeleteShader(fragmentShader);
//
//	return shaderProgram;
//
//}

// Moves/alters the camera positions based on user input
void OpenGLTutorial::doMovement()
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
}

// Is called whenever a key is pressed/released via GLFW
void key_callback(GLFWwindow* window, int key, int scancode, int action, int mode)
{
	//cout << key << endl;
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

	if (key == GLFW_KEY_O && action == GLFW_PRESS) {
		targetAngle += 0.1;
	}
	if (key == GLFW_KEY_P && action == GLFW_PRESS) {
		targetAngle -= 0.1;
	}
}

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


void scroll_callback(GLFWwindow* window, double xoffset, double yoffset)
{
	camera.ProcessMouseScroll(yoffset);
}

OpenGLTutorial::~OpenGLTutorial()
{
}
