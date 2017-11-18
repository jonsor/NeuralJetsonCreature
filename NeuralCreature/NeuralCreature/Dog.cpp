#include "stdafx.h"
#include "Dog.h"
#include <chrono>

/**
Constructor for the creature. Sets up a complete hardcoded
creature (a hip and two legs with thighs, shins and feet).
Also links them together with mobile joints (hinges).
TODO: Create this class customizable, not hardcoded.

@param pm pointer to the PhysicsManager class so that the limbs and joints can be added to the world simulation.
*/
Dog::Dog(PhysicsManager* pm, glm::vec3 startPosition, std::default_random_engine &engine) : m_startPosition(startPosition)
{
	//Limbs:

	//(glm::vec3 position, glm::vec3 color, GLfloat width, GLfloat height, GLfloat depth, btScalar mass)
	// 1.2f, 1.0f, 3.0f
	//glm::vec3 position, glm::vec3 color, GLfloat width, GLfloat height, GLfloat depth, btScalar mass
	body = new Box(glm::vec3(m_startPosition.x, m_startPosition.y, m_startPosition.z), glm::vec3(0.9f, 0.1f, 0.1f), 2.f, 1.0f, 3.f, 25);

	//TIGHS
	//frontRightTigh = new Box(glm::vec3(m_startPosition.x + 1.3, m_startPosition.y - 1.6, m_startPosition.z - 2.5), glm::vec3(0.2f, 0.3f, 0.7f), 0.3f, 1.2f, 0.3f, 15);
	//frontLeftTigh = new Box(glm::vec3(m_startPosition.x - 1.3, m_startPosition.y - 1.6, m_startPosition.z - 2.5), glm::vec3(0.2f, 0.3f, 0.7f), 0.3f, 1.2f, 0.3f, 15);

	//backRightTigh = new Box(glm::vec3(m_startPosition.x + 1.3, m_startPosition.y - 1.6, m_startPosition.z + 2.5), glm::vec3(0.2f, 0.3f, 0.7f), 0.3f, 1.2f, 0.3f, 15);
	//backLeftTigh = new Box(glm::vec3(m_startPosition.x - 1.3, m_startPosition.y - 1.6, m_startPosition.z + 2.5), glm::vec3(0.2f, 0.3f, 0.7f), 0.3f, 1.2f, 0.3f, 15);

	frontRightTigh = new Box(glm::vec3(m_startPosition.x + (body->getWidth() + 0.2), m_startPosition.y - (body->getHeight() + 0.6), m_startPosition.z - (body->getDepth() - 0.5)), glm::vec3(0.2f, 0.3f, 0.7f), 0.3f, 1.2f, 0.3f, 10);
	frontLeftTigh = new Box(glm::vec3(m_startPosition.x - (body->getWidth() + 0.2), m_startPosition.y - (body->getHeight() + 0.6), m_startPosition.z - (body->getDepth() - 0.5)), glm::vec3(0.2f, 0.3f, 0.7f), 0.3f, 1.2f, 0.3f, 10);

	backRightTigh = new Box(glm::vec3(m_startPosition.x + (body->getWidth() + 0.2), m_startPosition.y - (body->getHeight() + 0.6), m_startPosition.z + (body->getDepth() - 0.5)), glm::vec3(0.2f, 0.3f, 0.7f), 0.3f, 1.2f, 0.3f, 10);
	backLeftTigh = new Box(glm::vec3(m_startPosition.x - (body->getWidth() + 0.2), m_startPosition.y - (body->getHeight() + 0.6), m_startPosition.z + (body->getDepth() - 0.5)), glm::vec3(0.2f, 0.3f, 0.7f), 0.3f, 1.2f, 0.3f, 10);


	// SHINS
	frontRightShin = new Box(glm::vec3(m_startPosition.x + (body->getWidth() + 0.2), m_startPosition.y - 4.05, m_startPosition.z - (body->getDepth() - 0.5)), glm::vec3(0.2f, 0.3f, 0.7f), 0.3f, 1.2f, 0.3f, 5);
	frontLeftShin = new Box(glm::vec3(m_startPosition.x - (body->getWidth() + 0.2), m_startPosition.y - 4.05, m_startPosition.z - (body->getDepth() - 0.5)), glm::vec3(0.2f, 0.3f, 0.7f), 0.3f, 1.2f, 0.3f, 5);

	backRightShin = new Box(glm::vec3(m_startPosition.x + (body->getWidth() + 0.2), m_startPosition.y - 4.05, m_startPosition.z + (body->getDepth() - 0.5)), glm::vec3(0.2f, 0.3f, 0.7f), 0.3f, 1.2f, 0.3f, 5);
	backLeftShin = new Box(glm::vec3(m_startPosition.x - (body->getWidth() + 0.2), m_startPosition.y - 4.05, m_startPosition.z + (body->getDepth() - 0.5)), glm::vec3(0.2f, 0.3f, 0.7f), 0.3f, 1.2f, 0.3f, 5);

	//frontRightShin->getRigidBody()->setFriction(2.0f);
	//frontLeftShin->getRigidBody()->setFriction(2.0f);
	//backRightShin->getRigidBody()->setFriction(2.0f);
	//backLeftShin->getRigidBody()->setFriction(2.0f);

	pm->addBody(body->getRigidBody(), 1, 2);

	pm->addBody(frontRightTigh->getRigidBody(), 1, 2);
	pm->addBody(frontLeftTigh->getRigidBody(), 1, 2);

	pm->addBody(backRightTigh->getRigidBody(), 1, 2);
	pm->addBody(backLeftTigh->getRigidBody(), 1, 2);

	pm->addBody(frontRightShin->getRigidBody(), 1, 2);
	pm->addBody(frontLeftShin->getRigidBody(), 1, 2);

	pm->addBody(backRightShin->getRigidBody(), 1, 2);
	pm->addBody(backLeftShin->getRigidBody(), 1, 2);
	//2D
	//body->getRigidBody()->setLinearFactor(btVector3(0, 1, 1));
	//body->getRigidBody()->setAngularFactor(btVector3(1, 0, 0));

	//Freeze
	//body->getRigidBody()->setLinearFactor(btVector3(0, 0, 0));
	//body->getRigidBody()->setAngularFactor(btVector3(0, 0, 0));

	// Joints
	bool noCol = true;
	//body->addDOFConstraintDog(frontRightTigh, noCol, 1.4, -2.5, pm, "frontRightTigh");
	//body->addDOFConstraintDog(frontLeftTigh, noCol, -1.4, -2.5, pm, "frontLeftTigh");

	//body->addDOFConstraintDog(backRightTigh, noCol, 1.4, 2.5, pm, "backRightTigh");
	//body->addDOFConstraintDog(backLeftTigh, noCol, -1.4, 2.5, pm, "backLeftTigh");
	//body->addDOFConstraintDog(frontRightTigh, noCol, body->getWidth() + 0.2, -(body->getDepth() - 0.5), pm, "frontRightTigh");
	//body->addDOFConstraintDog(frontLeftTigh, noCol, -body->getWidth() -0.2, -(body->getDepth() - 0.5), pm, "frontLeftTigh");

	//body->addDOFConstraintDog(backRightTigh, noCol, body->getWidth() + 0.2, (body->getDepth() - 0.5), pm, "backRightTigh");
	//body->addDOFConstraintDog(backLeftTigh, noCol, -body->getWidth() - 0.2, (body->getDepth() - 0.5), pm, "backLeftTigh");

	//glm::vec3 pivotA, glm::vec3 pivotB, glm::vec3 axisA, glm::vec3 axisB, Box* BoxB, bool notCollision, const btScalar minAngle, const btScalar maxAngle, PhysicsManager * pm, std::string name

	body->addHinge(glm::vec3(body->getWidth() + 0.2, 0.f, -(body->getDepth() - 0.5)), glm::vec3(0.0f, 1.2f, 0.0f), glm::vec3(1.0f, 0.0f, 0.0f), glm::vec3(1.0f, 0.0f, 0.0f), frontRightTigh, noCol, -2.0f, PI * 0.5, pm, "frontRightTigh");
	body->addHinge(glm::vec3(-body->getWidth() - 0.2, 0.f, -(body->getDepth() - 0.5)), glm::vec3(0.0f, 1.2f, 0.0f), glm::vec3(1.0f, 0.0f, 0.0f), glm::vec3(1.0f, 0.0f, 0.0f), frontLeftTigh, noCol, -2.0f, PI * 0.5, pm, "frontLeftTigh");

	body->addHinge(glm::vec3(body->getWidth() + 0.2, 0.f, (body->getDepth() - 0.5)), glm::vec3(0.0f, 1.2f, 0.0f), glm::vec3(1.0f, 0.0f, 0.0f), glm::vec3(1.0f, 0.0f, 0.0f), backRightTigh, noCol, -2.0f, PI * 0.5, pm, "backRightTigh");
	body->addHinge(glm::vec3(-body->getWidth() - 0.2, 0.f, (body->getDepth() - 0.5)), glm::vec3(0.0f, 1.2f, 0.0f), glm::vec3(1.0f, 0.0f, 0.0f), glm::vec3(1.0f, 0.0f, 0.0f), backLeftTigh, noCol, -2.0f, PI * 0.5, pm, "backLeftTigh");

	frontRightTigh->addHinge(glm::vec3(0.0f, -1.2f, 0.0f), glm::vec3(0.0f, 1.2f, 0.0f), glm::vec3(1.0f, 0.0f, 0.0f), glm::vec3(1.0f, 0.0f, 0.0f), frontRightShin, noCol, -0.1, PI * 0.8, pm, "frontRightKnee");
	frontLeftTigh->addHinge(glm::vec3(0.0f, -1.2f, 0.0f), glm::vec3(0.0f, 1.2f, 0.0f), glm::vec3(1.0f, 0.0f, 0.0f), glm::vec3(1.0f, 0.0f, 0.0f), frontLeftShin, noCol, -0.1, PI * 0.8, pm, "frontLeftKnee");

	backRightTigh->addHinge(glm::vec3(0.0f, -1.2f, 0.0f), glm::vec3(0.0f, 1.2f, 0.0f), glm::vec3(1.0f, 0.0f, 0.0f), glm::vec3(1.0f, 0.0f, 0.0f), backRightShin, noCol, -0.1, PI * 0.8, pm, "backRightKnee");
	backLeftTigh->addHinge(glm::vec3(0.0f, -1.2f, 0.0f), glm::vec3(0.0f, 1.2f, 0.0f), glm::vec3(1.0f, 0.0f, 0.0f), glm::vec3(1.0f, 0.0f, 0.0f), backLeftShin, noCol, -0.1, PI * 0.8, pm, "backLeftKnee");

	setMaxMotorImpulses(100.0f);

	//Create the neural network
	std::vector<int> topology{ 50, 30, 8 };
	createNeuralNetwork(topology, engine);

	//int numInputs, int numOutputs, double sameSpeciesThreshold, double disjointCoefficient, double excessCoefficient, double averageWeightDifferenceCoefficient
	NEATController controller(48, 8, 0.4f, 0.3f, 0.2f, 0.5f);
	//NEATController controller, int netId, int numInputs, int numOutputs
	m_neatNeuralNetwork = NEATNetwork(controller, 1, 48, 8);

	//Set default fitness
	m_fitness = 0;

	int maxMinAnglesSize = 8;
	for (int i = 0; i < maxMinAnglesSize; i++) {
		maxMinAngles.push_back(0);
	}
	memoryNeurons = { 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 };
	m_maxHeight = 0;

	numberOfSteps = 0;
	lastFootThatStepped = 'n';

	//averageFeetStartPos = getAverageFeetPosition();

	//resultVec = { 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};
	resultVec = { 0, 0, 0, 0, 0, 0, 0, 0 };

	m_shouldUpdate = true;
	timeOnTwoLegs = 0;
	timeOnGround = 0;
	m_startPosition = getPosition();
	m_previousPosition = getPosition();

	m_targetPosition = {2, 1.6, 50};

	maxDistanceToTarget = glm::distance(m_startPosition, m_targetPosition);
}

/**
Calls the render function for each limb of the creature.

@param shader takes the shader for rendering puposes.
*/

void Dog::render(Shader shader)
{
	body->render(shader);

	frontRightTigh->render(shader);
	frontRightShin->render(shader);

	frontLeftTigh->render(shader);
	frontLeftShin->render(shader);

	backRightTigh->render(shader);
	backRightShin->render(shader);

	backLeftTigh->render(shader);
	backLeftShin->render(shader);
}

/**
Updates the physics for each limb in the creature.
Also sets dampening and restitution?
*/
static void updatePhysicsOfLimb(Box * box) {
	box->updatePhysics();
}
void Dog::updatePhysics()
{

	//body->getdofConstraint("frontRightTigh")->getRotationalLimitMotor(2)->m_enableMotor = true;
	//body->getdofConstraint("frontRightTigh")->getRotationalLimitMotor(2)->m_maxMotorForce = 10.f;
	//body->getdofConstraint("frontRightTigh")->getRotationalLimitMotor(2)->m_targetVelocity = -5.f;

	//body->getdofConstraint("frontRightTigh")->getRotationalLimitMotor(1)->m_enableMotor = true;
	//body->getdofConstraint("frontRightTigh")->getRotationalLimitMotor(1)->m_maxMotorForce = 10.f;
	//body->getdofConstraint("frontRightTigh")->getRotationalLimitMotor(1)->m_targetVelocity = -5.f;

	//body->getdofConstraint("frontRightTigh")->getRotationalLimitMotor(3)->m_enableMotor = true;
	//body->getdofConstraint("frontRightTigh")->getRotationalLimitMotor(3)->m_maxMotorForce = 10.f;
	//body->getdofConstraint("frontRightTigh")->getRotationalLimitMotor(3)->m_targetVelocity = -5.f;

	timeAlive++;

	updateNeuralNetwork();

	body->updatePhysics();

	frontRightTigh->updatePhysics();
	frontRightShin->updatePhysics();

	frontLeftTigh->updatePhysics();
	frontLeftShin->updatePhysics();

	backRightTigh->updatePhysics();
	backRightShin->updatePhysics();

	backLeftTigh->updatePhysics();
	backLeftShin->updatePhysics();


}

//glm::vec3 Dog::getPosition()
//{
//	return body->getPosition();
//}


glm::vec3 Dog::getPosition() {
	btVector3 finalPosition(0, 0, 0);
	finalPosition += body->getRigidBody()->getCenterOfMassPosition();
	finalPosition += frontRightTigh->getRigidBody()->getCenterOfMassPosition();
	finalPosition += frontRightShin->getRigidBody()->getCenterOfMassPosition();
	finalPosition += frontLeftTigh->getRigidBody()->getCenterOfMassPosition();
	finalPosition += frontLeftShin->getRigidBody()->getCenterOfMassPosition();
	finalPosition += backRightTigh->getRigidBody()->getCenterOfMassPosition();
	finalPosition += backLeftShin->getRigidBody()->getCenterOfMassPosition();
	//for (int i = 0; i < BODYPART_COUNT; i++)
	//{
	//	finalPosition += m_bodies[i]->getCenterOfMassPosition();
	//}

	finalPosition /= 7;
	return glm::vec3(finalPosition.getX(), finalPosition.getY(), finalPosition.getZ());
}

glm::vec3 Dog::getStartPosition()
{
	return m_startPosition;
}

void Dog::activate() {

	body->getRigidBody()->activate();

	frontRightTigh->getRigidBody()->activate();
	frontRightShin->getRigidBody()->activate();

	frontLeftTigh->getRigidBody()->activate();
	frontLeftShin->getRigidBody()->activate();

	backRightTigh->getRigidBody()->activate();
	backRightShin->getRigidBody()->activate();

	backLeftTigh->getRigidBody()->activate();
	backLeftShin->getRigidBody()->activate();
}

void Dog::createNeuralNetwork(std::vector<int> topology, std::default_random_engine &engine)
{
	m_neuralNetwork = RecurrentNeuralNetwork(topology, engine);
}

void Dog::setNEATNeuralNetwork(NEATNetwork neatNeuralNetwork)
{
	m_neatNeuralNetwork = neatNeuralNetwork;
}

NEATNetwork Dog::getNEATNeuralNetwork()
{
	return m_neatNeuralNetwork;
}

void Dog::setNeuralNetwork(RecurrentNeuralNetwork neuralNetwork)
{
	m_neuralNetwork = neuralNetwork;
}

RecurrentNeuralNetwork Dog::getNeuralNetwork()
{
	return m_neuralNetwork;
}

RecurrentNeuralNetwork* Dog::getNN()
{
	return &m_neuralNetwork;
}

void Dog::setAllTargetVelocities(std::vector<double>& resultVec)
{
	//CHANGE MAX VELOCITY NORMALIZATION IF YOU CHANGE m
	double m = 4;
	double mU = 0.0;

	//HIPS
	// 0 = spin, 1 = fremover/bakover, 2 = side til side
	int rV = 0;
	//for (int i = 0; i < 3; i++) {
	//	if(i == 0){

	//		body->getdofConstraint("frontRightTigh")->getRotationalLimitMotor(i)->m_targetVelocity = resultVec[rV] * m/2;
	//		body->getdofConstraint("frontLeftTigh")->getRotationalLimitMotor(i)->m_targetVelocity = resultVec[rV + 1] * m/2;
	//		body->getdofConstraint("backRightTigh")->getRotationalLimitMotor(i)->m_targetVelocity = resultVec[rV + 2] * m/2;
	//		body->getdofConstraint("backLeftTigh")->getRotationalLimitMotor(i)->m_targetVelocity = resultVec[rV + 3] * m/2;
	//	}
	//	else if (i == 1) {
	//		body->getdofConstraint("frontRightTigh")->getRotationalLimitMotor(i)->m_targetVelocity = resultVec[rV] * m*2;
	//		body->getdofConstraint("frontLeftTigh")->getRotationalLimitMotor(i)->m_targetVelocity = resultVec[rV + 1] * m*2;
	//		body->getdofConstraint("backRightTigh")->getRotationalLimitMotor(i)->m_targetVelocity = resultVec[rV + 2] * m*2;
	//		body->getdofConstraint("backLeftTigh")->getRotationalLimitMotor(i)->m_targetVelocity = resultVec[rV + 3] * m*2;
	//	} else{
	//		body->getdofConstraint("frontRightTigh")->getRotationalLimitMotor(i)->m_targetVelocity = resultVec[rV] * m*2;
	//		body->getdofConstraint("frontLeftTigh")->getRotationalLimitMotor(i)->m_targetVelocity = resultVec[rV + 1] * m*2;
	//		body->getdofConstraint("backRightTigh")->getRotationalLimitMotor(i)->m_targetVelocity = resultVec[rV + 2] * m*2;
	//		body->getdofConstraint("backLeftTigh")->getRotationalLimitMotor(i)->m_targetVelocity = resultVec[rV + 3] * m*2;
	//	}
	//	rV += 4;
	//}
	//for (int i = 0; i < resultVec.size(); i++) {
	//	std::cout << resultVec[i] << "\n";
	//}
	//std::cout << "scale 0: " << Util::scaleToRange(0, body->getHinge("frontRightTigh")->getLowerLimit(), body->getHinge("frontRightTigh")->getUpperLimit()) << "\n";
	
	double frtAngle = Util::scaleToRange(resultVec[0], body->getHinge("frontRightTigh")->getLowerLimit(), body->getHinge("frontRightTigh")->getUpperLimit());
	body->getHinge("frontRightTigh")->setMotorTarget(frtAngle, 1);

	double fltAngle = Util::scaleToRange(resultVec[1], body->getHinge("frontLeftTigh")->getLowerLimit(), body->getHinge("frontLeftTigh")->getUpperLimit());
	body->getHinge("frontLeftTigh")->setMotorTarget(fltAngle, 1);

	double brtAngle = Util::scaleToRange(resultVec[2], body->getHinge("backRightTigh")->getLowerLimit(), body->getHinge("backRightTigh")->getUpperLimit());
	body->getHinge("backRightTigh")->setMotorTarget(brtAngle, 1);

	double bltAngle = Util::scaleToRange(resultVec[3], body->getHinge("backLeftTigh")->getLowerLimit(), body->getHinge("backLeftTigh")->getUpperLimit());
	body->getHinge("backLeftTigh")->setMotorTarget(bltAngle, 1);

	//KNEE
	double frkAngle = Util::scaleToRange(resultVec[4], frontRightTigh->getHinge("frontRightKnee")->getLowerLimit(), frontRightTigh->getHinge("frontRightKnee")->getUpperLimit());
	frontRightTigh->getHinge("frontRightKnee")->setMotorTarget(frkAngle, 1);

	double flkAngle = Util::scaleToRange(resultVec[5], frontLeftTigh->getHinge("frontLeftKnee")->getLowerLimit(), frontLeftTigh->getHinge("frontLeftKnee")->getUpperLimit());
	frontLeftTigh->getHinge("frontLeftKnee")->setMotorTarget(flkAngle, 1);

	double brkAngle = Util::scaleToRange(resultVec[6], backRightTigh->getHinge("backRightKnee")->getLowerLimit(), backRightTigh->getHinge("backRightKnee")->getUpperLimit());
	backRightTigh->getHinge("backRightKnee")->setMotorTarget(brkAngle, 1);

	double blkAngle = Util::scaleToRange(resultVec[7], backLeftTigh->getHinge("backLeftKnee")->getLowerLimit(), backLeftTigh->getHinge("backLeftKnee")->getUpperLimit());
	backLeftTigh->getHinge("backLeftKnee")->setMotorTarget(blkAngle, 1);

	//body->getHinge("frontRightTigh")->setMotorTargetVelocity(resultVec[0] * m);
	//body->getHinge("frontLeftTigh")->setMotorTargetVelocity(resultVec[1] * m);
	//body->getHinge("backRightTigh")->setMotorTargetVelocity(resultVec[2] * m);
	//body->getHinge("backLeftTigh")->setMotorTargetVelocity(resultVec[3] * m);

	//frontRightTigh->getHinge("frontRightKnee")->setMotorTargetVelocity(resultVec[4] * m);
	//frontLeftTigh->getHinge("frontLeftKnee")->setMotorTargetVelocity(resultVec[5] * m);
	//backRightTigh->getHinge("backRightKnee")->setMotorTargetVelocity(resultVec[6] * m);
	//backLeftTigh->getHinge("backLeftKnee")->setMotorTargetVelocity(resultVec[7] * m);
	//double max = 0;
	//double min = 10000;
	//double sum = 0;
	//for (int i = 0; i < resultVec.size(); i++) {
	//	sum += resultVec[i];
	//	min = (min > resultVec[i]) ? resultVec[i] : min;
	//	max = (max < resultVec[i]) ? resultVec[i] : max;
	//}
	//sum /= resultVec.size();
	//std::cout << "average: " << sum << " min: " << min << " max: " << max << "\n";

}


void Dog::updateNeuralNetwork()
{
	std::vector<double> inputs = calculateInputs();
	//m_neatNeuralNetwork.forward(inputs);
	//m_neatNeuralNetwork.getResults(resultVec);
	m_neuralNetwork.forward(inputs);
	m_neuralNetwork.getResults(resultVec);
	//body->getdofConstraint("frontRightTigh")->getRotationalLimitMotor(1)->m_targetVelocity = 2.f;
	//body->getHinge("frontRightTigh")->setMotorTargetVelocity(-2.f);
	//body->getHinge("frontLeftTigh")->setMotorTargetVelocity(-2.f);
	//body->getHinge("backRightTigh")->setMotorTargetVelocity(-2.f);
	//body->getHinge("backLeftTigh")->setMotorTargetVelocity(-2.f);

	//body->getdofConstraint("backRightTigh")->getRotationalLimitMotor(1)->m_targetVelocity = 2.f;
	//backRightTigh->getHinge("backRightKnee")->setMotorTargetVelocity(-2.f);

	//body->getdofConstraint("backLeftTigh")->getRotationalLimitMotor(1)->m_targetVelocity = -2.f;
	//backLeftTigh->getHinge("backLeftKnee")->setMotorTargetVelocity(2.f);


	setAllTargetVelocities(resultVec);

}
std::vector<double> Dog::calculateInputs()
{
	//41 42 13 19
	std::vector<double> inputs;
	//Height
	double height = Util::normalize(getCenterOfMassHeight(), 0, 5);
	inputs.push_back(height);

	//Angular velocities 9*3= 27
	std::vector<double> angularVel = getAllAngularVelocities();
	const double MAX = 7;
	for (int i = 0; i < angularVel.size(); i++) {
		angularVel[i] = Util::normalize(angularVel[i], -MAX, MAX);
	}

	inputs.insert(inputs.end(), angularVel.begin(), angularVel.end());
	//Body orientation
	btMatrix3x3 m = btMatrix3x3(body->getRigidBody()->getOrientation());
	btScalar yaw, pitch, roll;
	m.getEulerZYX(yaw, pitch, roll);
	yaw = Util::normalize(yaw, -PI, PI);
	pitch = Util::normalize(pitch, -PI, PI);
	roll = Util::normalize(roll, -PI, PI);
	//std::cout << "yaw: " << yaw << " pitch: " << pitch << " roll: " << roll << "\n";
	//inputs.push_back(yaw);
	//inputs.push_back(pitch);
	//inputs.push_back(roll);

	inputs.push_back(body->getRigidBody()->getOrientation().getX());
	inputs.push_back(body->getRigidBody()->getOrientation().getY());
	inputs.push_back(body->getRigidBody()->getOrientation().getZ());
	inputs.push_back(body->getRigidBody()->getOrientation().getW());

	//std::cout << "targetPos: " << " x: " << m_targetPosition[0] << " y: " << m_targetPosition[1] << " z: " << m_targetPosition[2] << "\n";
	//Math.toDegrees(Math.atan2(target.x - x, target.y - y));
	double angleToTarget = std::atan2(m_targetPosition[0] - getPosition()[0], m_targetPosition[2] - getPosition()[2]);
	
	//Angle to target
	inputs.push_back(Util::normalize(angleToTarget, -PI, PI));
	//std::cout << "angleToTarget: " << angleToTarget << " angleToTargetNormalized: " << Util::normalize(angleToTarget, -PI, PI) << "\n";
	//Distance to target
	inputs.push_back(Util::normalize(glm::distance(getPosition(), m_targetPosition), 0, maxDistanceToTarget));
	//std::cout << "distanceToTarget: " << Util::normalize(glm::distance(getPosition(), m_targetPosition), 0, maxDistanceToTarget) << "\n";
	const double VEL_MAX = 5;
	//Body Linear Velocity
	inputs.push_back(Util::normalize(body->getRigidBody()->getLinearVelocity().getX(), -VEL_MAX, VEL_MAX));
	inputs.push_back(Util::normalize(body->getRigidBody()->getLinearVelocity().getY(), -VEL_MAX, VEL_MAX));
	inputs.push_back(Util::normalize(body->getRigidBody()->getLinearVelocity().getZ(), -VEL_MAX, VEL_MAX));

	//Hinge angles 16 or 8
	std::vector<double> inputAngles = getAllAngles();

	inputs.insert(inputs.end(), inputAngles.begin(), inputAngles.end());

	double frontRightShinOnGround = (frontRightShin->isCollidingWithGround()) ? 1.0 : -1.0;
	double frontLeftShinOnGround = (frontLeftShin->isCollidingWithGround()) ? 1.0 : -1.0;

	double backRightShinOnGround = (backRightShin->isCollidingWithGround()) ? 1.0 : -1.0;
	double backLeftShinOnGround = (backLeftShin->isCollidingWithGround()) ? 1.0 : -1.0;

	if (frontRightShinOnGround > 0) {
		frontRightShin->setColor(glm::vec3(0.1, 1.0, 0.1));
	}
	else {
		frontRightShin->setColor(glm::vec3(0.2f, 0.3f, 0.7f));
	}

	if (frontLeftShinOnGround > 0) {
		frontLeftShin->setColor(glm::vec3(0.1, 1.0, 0.1));
	}
	else {
		frontLeftShin->setColor(glm::vec3(0.2f, 0.3f, 0.7f));
	}

	if (backRightShinOnGround > 0) {
		backRightShin->setColor(glm::vec3(0.1, 1.0, 0.1));
	}
	else {
		backRightShin->setColor(glm::vec3(0.2f, 0.3f, 0.7f));
	}

	if (backLeftShinOnGround > 0) {
		backLeftShin->setColor(glm::vec3(0.1, 1.0, 0.1));
	}
	else {
		backLeftShin->setColor(glm::vec3(0.2f, 0.3f, 0.7f));
	}
	inputs.push_back(frontRightShinOnGround);
	inputs.push_back(frontLeftShinOnGround);
	inputs.push_back(backRightShinOnGround);
	inputs.push_back(backLeftShinOnGround);

	//Collision Impulses
	//double maxCollisionImpulse = 140.f;
	//inputs.push_back(Util::normalize(frontRightShin->getCollisionImpulse(), 0, maxCollisionImpulse));
	//inputs.push_back(Util::normalize(frontLeftShin->getCollisionImpulse(), 0, maxCollisionImpulse));
	//inputs.push_back(Util::normalize(backRightShin->getCollisionImpulse(), 0, maxCollisionImpulse));
	//inputs.push_back(Util::normalize(backLeftShin->getCollisionImpulse(), 0, maxCollisionImpulse));


	//Joint Speeds
	//Relative Positions only Y values 8
	//std::vector<double> relativePositions = getRelativePositions();
	//inputs.insert(inputs.end(), relativePositions.begin(), relativePositions.end());
	//Joints at Limit 32 inputs or 16
	//getAllMaxMinAngles();
	//inputs.insert(inputs.end(), maxMinAngles.begin(), maxMinAngles.end());

	//Previous result target velocities 16
	//inputs.insert(inputs.end(), resultVec.begin(), resultVec.end());
	//Target Position

	//Target Distance

	//Angle to Target

	//Time pulse

	//inputs.push_back(std::sin(timeAlive/6));


	double max = 0;
	double min = 1000;
	int intMax = 0;
	int intMin = 0;
	for (int i = 0; i < inputs.size(); i++) {
		//max = (max < inputs[i]) ? inputs[i] : max;
		//min = (min > inputs[i]) ? inputs[i] : min;

		if (max < inputs[i]) {
			max = inputs[i];
			intMax = i;
		}

		if (min > inputs[i]) {
			min = inputs[i];
			intMin = i;
		}
	}

	if (max > 1.5f) {
		std::cout << "MAX " << intMax << " : " << max << "\n";
	}
	if (min < -1.5f) {
		std::cout << " MIN: " << intMin << " : " << min << "\n";
	}
	//std::cout << "max; " << max << " min: " << min<<  "\n";

	return inputs;
}

std::vector<double> Dog::getAllAngularVelocities() {

	std::vector<double> angularVelocities;
	//Body
	angularVelocities.push_back(body->getRigidBody()->getAngularVelocity().getX());
	angularVelocities.push_back(body->getRigidBody()->getAngularVelocity().getY());
	angularVelocities.push_back(body->getRigidBody()->getAngularVelocity().getZ());

	//std::cout << "x: " << body->getRigidBody()->getAngularVelocity().getX() << " y: " << body->getRigidBody()->getAngularVelocity().getY() << " z: " << body->getRigidBody()->getAngularVelocity().getZ() << "\n";

	//frontRightTigh
	angularVelocities.push_back(frontRightTigh->getRigidBody()->getAngularVelocity().getX());
	angularVelocities.push_back(frontRightTigh->getRigidBody()->getAngularVelocity().getY());
	angularVelocities.push_back(frontRightTigh->getRigidBody()->getAngularVelocity().getZ());

	//frontRightShin
	angularVelocities.push_back(frontRightShin->getRigidBody()->getAngularVelocity().getX());
	angularVelocities.push_back(frontRightShin->getRigidBody()->getAngularVelocity().getY());
	angularVelocities.push_back(frontRightShin->getRigidBody()->getAngularVelocity().getZ());

	//frontLeftTigh
	angularVelocities.push_back(frontLeftTigh->getRigidBody()->getAngularVelocity().getX());
	angularVelocities.push_back(frontLeftTigh->getRigidBody()->getAngularVelocity().getY());
	angularVelocities.push_back(frontLeftTigh->getRigidBody()->getAngularVelocity().getZ());

	//frontLeftShin
	angularVelocities.push_back(frontLeftShin->getRigidBody()->getAngularVelocity().getX());
	angularVelocities.push_back(frontLeftShin->getRigidBody()->getAngularVelocity().getY());
	angularVelocities.push_back(frontLeftShin->getRigidBody()->getAngularVelocity().getZ());

	//backRightTigh
	angularVelocities.push_back(backRightTigh->getRigidBody()->getAngularVelocity().getX());
	angularVelocities.push_back(backRightTigh->getRigidBody()->getAngularVelocity().getY());
	angularVelocities.push_back(backRightTigh->getRigidBody()->getAngularVelocity().getZ());

	//backRightShin
	angularVelocities.push_back(backRightShin->getRigidBody()->getAngularVelocity().getX());
	angularVelocities.push_back(backRightShin->getRigidBody()->getAngularVelocity().getY());
	angularVelocities.push_back(backRightShin->getRigidBody()->getAngularVelocity().getZ());

	//backLeftTigh
	angularVelocities.push_back(backLeftTigh->getRigidBody()->getAngularVelocity().getX());
	angularVelocities.push_back(backLeftTigh->getRigidBody()->getAngularVelocity().getY());
	angularVelocities.push_back(backLeftTigh->getRigidBody()->getAngularVelocity().getZ());

	//backLeftShin
	angularVelocities.push_back(backLeftShin->getRigidBody()->getAngularVelocity().getX());
	angularVelocities.push_back(backLeftShin->getRigidBody()->getAngularVelocity().getY());
	angularVelocities.push_back(backLeftShin->getRigidBody()->getAngularVelocity().getZ());

	return angularVelocities;
}

void Dog::getAllMaxMinAngles()
{

	//for (int i = 0; i < maxMinAngles.size(); i++) {
	//	maxMinAngles[i] = 0;
	//}


	////frontRightTigh
	//btVector3 frtAngularUpper;
	//body->getdofConstraint("frontRightTigh")->getAngularUpperLimit(frtAngularUpper);

	//btVector3 frtAngularLower;
	//body->getdofConstraint("frontRightTigh")->getAngularLowerLimit(frtAngularLower);

	//if (body->getdofConstraint("frontRightTigh")->getAngle(0) >= frtAngularUpper.getX() - 0.01) {
	//	maxMinAngles[0] = 1.0;
	//}
	//if (body->getdofConstraint("frontRightTigh")->getAngle(1) >= frtAngularUpper.getY() - 0.01) {
	//	maxMinAngles[1] = 1.0;
	//}
	//if (body->getdofConstraint("frontRightTigh")->getAngle(2) >= frtAngularUpper.getZ() - 0.01) {
	//	maxMinAngles[2] = 1.0;
	//}

	//if (body->getdofConstraint("frontRightTigh")->getAngle(0) <= frtAngularLower.getX() + 0.01) {
	//	maxMinAngles[3] = 1.0;
	//}
	//if (body->getdofConstraint("frontRightTigh")->getAngle(1) <= frtAngularLower.getY() + 0.01) {
	//	maxMinAngles[4] = 1.0;
	//}
	//if (body->getdofConstraint("frontRightTigh")->getAngle(2) <= frtAngularLower.getZ() + 0.01) {
	//	maxMinAngles[5] = 1.0;
	//}
	////frontLeftTigh
	//btVector3 fltAngularUpper;
	//body->getdofConstraint("frontLeftTigh")->getAngularUpperLimit(fltAngularUpper);

	//btVector3 fltAngularLower;
	//body->getdofConstraint("frontLeftTigh")->getAngularLowerLimit(fltAngularLower);

	//if (body->getdofConstraint("frontLeftTigh")->getAngle(0) >= fltAngularUpper.getX() - 0.01) {
	//	maxMinAngles[6] = 1.0;
	//}
	//if (body->getdofConstraint("frontLeftTigh")->getAngle(1) >= fltAngularUpper.getY() - 0.01) {
	//	maxMinAngles[7] = 1.0;
	//}
	//if (body->getdofConstraint("frontLeftTigh")->getAngle(2) >= fltAngularUpper.getZ() - 0.01) {
	//	maxMinAngles[8] = 1.0;
	//}

	//if (body->getdofConstraint("frontLeftTigh")->getAngle(0) <= fltAngularLower.getX() + 0.01) {
	//	maxMinAngles[9] = 1.0;
	//}
	//if (body->getdofConstraint("frontLeftTigh")->getAngle(1) <= fltAngularLower.getY() + 0.01) {
	//	maxMinAngles[10] = 1.0;
	//}
	//if (body->getdofConstraint("frontLeftTigh")->getAngle(2) <= fltAngularLower.getZ() + 0.01) {
	//	maxMinAngles[11] = 1.0;
	//}

	////backRightTigh
	//btVector3 brtAngularUpper;
	//body->getdofConstraint("backRightTigh")->getAngularUpperLimit(brtAngularUpper);

	//btVector3 brtAngularLower;
	//body->getdofConstraint("backRightTigh")->getAngularLowerLimit(brtAngularLower);

	//if (body->getdofConstraint("backRightTigh")->getAngle(0) >= brtAngularUpper.getX() - 0.01) {
	//	maxMinAngles[12] = 1.0;
	//}
	//if (body->getdofConstraint("backRightTigh")->getAngle(1) >= brtAngularUpper.getY() - 0.01) {
	//	maxMinAngles[13] = 1.0;
	//}
	//if (body->getdofConstraint("backRightTigh")->getAngle(2) >= brtAngularUpper.getZ() - 0.01) {
	//	maxMinAngles[14] = 1.0;
	//}

	//if (body->getdofConstraint("backRightTigh")->getAngle(0) <= brtAngularLower.getX() + 0.01) {
	//	maxMinAngles[15] = 1.0;
	//}
	//if (body->getdofConstraint("backRightTigh")->getAngle(1) <= brtAngularLower.getY() + 0.01) {
	//	maxMinAngles[16] = 1.0;
	//}
	//if (body->getdofConstraint("backRightTigh")->getAngle(2) <= brtAngularLower.getZ() + 0.01) {
	//	maxMinAngles[17] = 1.0;
	//}

	////backLeftTigh
	//btVector3 bltAngularUpper;
	//body->getdofConstraint("backLeftTigh")->getAngularUpperLimit(bltAngularUpper);

	//btVector3 bltAngularLower;
	//body->getdofConstraint("backLeftTigh")->getAngularLowerLimit(bltAngularLower);

	//if (body->getdofConstraint("backLeftTigh")->getAngle(0) >= bltAngularUpper.getX() - 0.01) {
	//	maxMinAngles[18] = 1.0;
	//}
	//if (body->getdofConstraint("backLeftTigh")->getAngle(1) >= bltAngularUpper.getY() - 0.01) {
	//	maxMinAngles[19] = 1.0;
	//}
	//if (body->getdofConstraint("backLeftTigh")->getAngle(2) >= bltAngularUpper.getZ() - 0.01) {
	//	maxMinAngles[20] = 1.0;
	//}

	//if (body->getdofConstraint("backLeftTigh")->getAngle(0) <= bltAngularLower.getX() + 0.01) {
	//	maxMinAngles[21] = 1.0;
	//}
	//if (body->getdofConstraint("backLeftTigh")->getAngle(1) <= bltAngularLower.getY() + 0.01) {
	//	maxMinAngles[22] = 1.0;
	//}
	//if (body->getdofConstraint("backLeftTigh")->getAngle(2) <= bltAngularLower.getZ() + 0.01) {
	//	maxMinAngles[23] = 1.0;
	//}

	//frontRightTigh
	if (body->getHinge("frontRightTigh")->getHingeAngle() >= body->getHinge("frontRightTigh")->getUpperLimit() - 0.05) {
		maxMinAngles[0] = 1.0;
	}
	if (body->getHinge("frontRightTigh")->getHingeAngle() <= body->getHinge("frontRightTigh")->getLowerLimit() + 0.05) {
		maxMinAngles[0] = -1.0;
	}

	//frontLeftTigh
	if (body->getHinge("frontLeftTigh")->getHingeAngle() >= body->getHinge("frontLeftTigh")->getUpperLimit() - 0.05) {
		maxMinAngles[1] = 1.0;
	}
	if (body->getHinge("frontLeftTigh")->getHingeAngle() <= body->getHinge("frontLeftTigh")->getLowerLimit() + 0.05) {
		maxMinAngles[1] = -1.0;
	}

	//backRightTigh
	if (body->getHinge("backRightTigh")->getHingeAngle() >= body->getHinge("backRightTigh")->getUpperLimit() - 0.05) {
		maxMinAngles[2] = 1.0;
	}
	if (body->getHinge("backRightTigh")->getHingeAngle() <= body->getHinge("backRightTigh")->getLowerLimit() + 0.05) {
		maxMinAngles[2] = -1.0;
	}

	//backLeftTigh
	if (body->getHinge("backLeftTigh")->getHingeAngle() >= body->getHinge("backLeftTigh")->getUpperLimit() - 0.05) {
		maxMinAngles[3] = 1.0;
	}
	if (body->getHinge("backLeftTigh")->getHingeAngle() <= body->getHinge("backLeftTigh")->getLowerLimit() + 0.05) {
		maxMinAngles[3] = -1.0;
	}

	//KNEES

	//frontRightKnee
	if (frontRightTigh->getHinge("frontRightKnee")->getHingeAngle() >= frontRightTigh->getHinge("frontRightKnee")->getUpperLimit() - 0.05) {
		maxMinAngles[4] = 1.0;
	}
	if (frontRightTigh->getHinge("frontRightKnee")->getHingeAngle() <= frontRightTigh->getHinge("frontRightKnee")->getLowerLimit() + 0.05) {
		maxMinAngles[4] = -1.0;
	}

	//frontLeftKnee
	if (frontLeftTigh->getHinge("frontLeftKnee")->getHingeAngle() >= frontLeftTigh->getHinge("frontLeftKnee")->getUpperLimit() - 0.05) {
		maxMinAngles[5] = 1.0;
	}
	if (frontLeftTigh->getHinge("frontLeftKnee")->getHingeAngle() <= frontLeftTigh->getHinge("frontLeftKnee")->getLowerLimit() + 0.05) {
		maxMinAngles[5] = -1.0;
	}

	//backRightKnee
	if (backRightTigh->getHinge("backRightKnee")->getHingeAngle() >= backRightTigh->getHinge("backRightKnee")->getUpperLimit() - 0.05) {
		maxMinAngles[6] = 1.0;
	}
	if (backRightTigh->getHinge("backRightKnee")->getHingeAngle() <= backRightTigh->getHinge("backRightKnee")->getLowerLimit() + 0.05) {
		maxMinAngles[6] = -1.0;
	}

	//backLeftKnee
	if (backLeftTigh->getHinge("backLeftKnee")->getHingeAngle() >= backLeftTigh->getHinge("backLeftKnee")->getUpperLimit() - 0.05) {
		maxMinAngles[7] = 1.0;
	}
	if (backLeftTigh->getHinge("backLeftKnee")->getHingeAngle() <= backLeftTigh->getHinge("backLeftKnee")->getLowerLimit() + 0.05) {
		maxMinAngles[7] = -1.0;
	}
	std::cout << "maxMin: \n";
	for (int i = 0; i < maxMinAngles.size(); i++) {
		std::cout << maxMinAngles[i] << " ";
	}
	std::cout << "\n";

}

std::vector<double> Dog::getAllAngles()
{
	////frontRightTigh
	//btVector3 anglularLowerLimitsFR;
	//body->getdofConstraint("frontRightTigh")->getAngularLowerLimit(anglularLowerLimitsFR);

	//btVector3 anglularUpperLimitsFR;
	//body->getdofConstraint("frontRightTigh")->getAngularUpperLimit(anglularUpperLimitsFR);

	//double frt1 = Util::normalize(body->getdofConstraint("frontRightTigh")->getAngle(0), anglularLowerLimitsFR[0], anglularUpperLimitsFR[0]);
	//double frt2 = Util::normalize(body->getdofConstraint("frontRightTigh")->getAngle(1), anglularLowerLimitsFR[1], anglularUpperLimitsFR[1]);
	//double frt3 = Util::normalize(body->getdofConstraint("frontRightTigh")->getAngle(2), anglularLowerLimitsFR[2], anglularUpperLimitsFR[2]);

	////frontLeftTigh
	//btVector3 anglularLowerLimitsFL;
	//body->getdofConstraint("frontLeftTigh")->getAngularLowerLimit(anglularLowerLimitsFL);

	//btVector3 anglularUpperLimitsFL;
	//body->getdofConstraint("frontLeftTigh")->getAngularUpperLimit(anglularUpperLimitsFL);

	//double flt1 = Util::normalize(body->getdofConstraint("frontLeftTigh")->getAngle(0), anglularLowerLimitsFL[0], anglularUpperLimitsFL[0]);
	//double flt2 = Util::normalize(body->getdofConstraint("frontLeftTigh")->getAngle(1), anglularLowerLimitsFL[1], anglularUpperLimitsFL[1]);
	//double flt3 = Util::normalize(body->getdofConstraint("frontLeftTigh")->getAngle(2), anglularLowerLimitsFL[2], anglularUpperLimitsFL[2]);

	////backRightTigh
	//btVector3 anglularLowerLimitsBR;
	//body->getdofConstraint("backRightTigh")->getAngularLowerLimit(anglularLowerLimitsBR);

	//btVector3 anglularUpperLimitsBR;
	//body->getdofConstraint("backRightTigh")->getAngularUpperLimit(anglularUpperLimitsBR);

	//double brt1 = Util::normalize(body->getdofConstraint("backRightTigh")->getAngle(0), anglularLowerLimitsBR[0], anglularUpperLimitsBR[0]);
	//double brt2 = Util::normalize(body->getdofConstraint("backRightTigh")->getAngle(1), anglularLowerLimitsBR[1], anglularUpperLimitsBR[1]);
	//double brt3 = Util::normalize(body->getdofConstraint("backRightTigh")->getAngle(2), anglularLowerLimitsBR[2], anglularUpperLimitsBR[2]);

	////backLeftTigh
	//btVector3 anglularLowerLimitsBL;
	//body->getdofConstraint("backLeftTigh")->getAngularLowerLimit(anglularLowerLimitsBL);

	//btVector3 anglularUpperLimitsBL;
	//body->getdofConstraint("backLeftTigh")->getAngularUpperLimit(anglularUpperLimitsBL);

	//double blt1 = Util::normalize(body->getdofConstraint("backLeftTigh")->getAngle(0), anglularLowerLimitsBL[0], anglularUpperLimitsBL[0]);
	//double blt2 = Util::normalize(body->getdofConstraint("backLeftTigh")->getAngle(1), anglularLowerLimitsBL[1], anglularUpperLimitsBL[1]);
	//double blt3 = Util::normalize(body->getdofConstraint("backLeftTigh")->getAngle(2), anglularLowerLimitsBL[2], anglularUpperLimitsBL[2]);

	double frt = Util::normalize(body->getHinge("frontRightTigh")->getHingeAngle(), body->getHinge("frontRightTigh")->getLowerLimit(), body->getHinge("frontRightTigh")->getUpperLimit());
	double flt = Util::normalize(body->getHinge("frontLeftTigh")->getHingeAngle(), body->getHinge("frontLeftTigh")->getLowerLimit(), body->getHinge("frontLeftTigh")->getUpperLimit());
	double brt = Util::normalize(body->getHinge("backRightTigh")->getHingeAngle(), body->getHinge("backRightTigh")->getLowerLimit(), body->getHinge("backRightTigh")->getUpperLimit());
	double blt = Util::normalize(body->getHinge("backLeftTigh")->getHingeAngle(), body->getHinge("backLeftTigh")->getLowerLimit(), body->getHinge("backLeftTigh")->getUpperLimit());

	//KNEES
	double frk = Util::normalize(frontRightTigh->getHinge("frontRightKnee")->getHingeAngle(), frontRightTigh->getHinge("frontRightKnee")->getLowerLimit(), frontRightTigh->getHinge("frontRightKnee")->getUpperLimit());
	double flk = Util::normalize(frontLeftTigh->getHinge("frontLeftKnee")->getHingeAngle(), frontLeftTigh->getHinge("frontLeftKnee")->getLowerLimit(), frontLeftTigh->getHinge("frontLeftKnee")->getUpperLimit());

	double brk = Util::normalize(backRightTigh->getHinge("backRightKnee")->getHingeAngle(), backRightTigh->getHinge("backRightKnee")->getLowerLimit(), backRightTigh->getHinge("backRightKnee")->getUpperLimit());
	double blk = Util::normalize(backLeftTigh->getHinge("backLeftKnee")->getHingeAngle(), backLeftTigh->getHinge("backLeftKnee")->getLowerLimit(), backLeftTigh->getHinge("backLeftKnee")->getUpperLimit());

	//return{ frt1, frt2, frt3, flt1, flt2, flt3, brt1, brt2, brt3, blt1, blt2, blt3, frk, flk, brk, blk };
	return{ frt, flt, brt, blt, frk, flk, brk, blk };
}

void Dog::mutate(double mutationRate, double mutationChance, std::default_random_engine engine)
{
	m_neuralNetwork.mutate(mutationRate, mutationChance, engine);
	//m_neatNeuralNetwork.mutate();
}

void Dog::reset()
{
	body->reset();

	frontRightTigh->reset();
	frontRightShin->reset();

	frontLeftTigh->reset();
	frontLeftShin->reset();

	backRightTigh->reset();
	backRightShin->reset();

	backLeftTigh->reset();
	backLeftShin->reset();

	std::vector<double> resetVals = { 0,0,0,0,0,0 };
	//setAllTargetVelocities(resetVals);

}

void Dog::setFitness(double fitness)
{
	m_fitness = fitness;
}

double Dog::getFitness()
{
	return m_fitness;
}


double Dog::getHeight()
{
	return body->getRigidBody()->getCenterOfMassPosition().getY();
}

void Dog::removeBodies(PhysicsManager * pm)
{
	body->remove(pm);

	frontRightTigh->remove(pm);
	frontRightShin->remove(pm);

	frontLeftTigh->remove(pm);
	frontLeftShin->remove(pm);

	backRightTigh->remove(pm);
	backRightShin->remove(pm);

	backLeftTigh->remove(pm);
	backLeftShin->remove(pm);
}


void Dog::removeConstraints(PhysicsManager * pm)
{
	body->removeConstraint(pm);
	frontRightTigh->removeConstraint(pm);
	frontRightShin->removeConstraint(pm);

	frontLeftTigh->removeConstraint(pm);
	frontLeftShin->removeConstraint(pm);

	backRightTigh->removeConstraint(pm);
	backRightShin->removeConstraint(pm);

	backLeftTigh->removeConstraint(pm);
	backLeftShin->removeConstraint(pm);
}

void Dog::setColor(glm::vec3 color)
{
	frontRightTigh->setColor(color);
	frontRightShin->setColor(color);

	frontLeftTigh->setColor(color);
	frontLeftShin->setColor(color);

	backRightTigh->setColor(color);
	backRightShin->setColor(color);

	backLeftTigh->setColor(color);
	backLeftShin->setColor(color);
}

void Dog::incrementToAverage()
{
	double value = getHeight();
	average = (avgSize * average + value) / (avgSize + 1);
	avgSize++;
}

double Dog::getAverageHeight() {
	return average;
}

double Dog::getMaxHeight()
{
	return m_maxHeight;
}

void Dog::updateMaxHeight(double height)
{

	if (height > m_maxHeight) m_maxHeight = height;
}

double Dog::getCenterOfMassHeight()
{
	double centerOfMassHeight = body->getRigidBody()->getCenterOfMassPosition().getY();
	return centerOfMassHeight;
}

double Dog::getTimeOnGround()
{
	return timeOnGround;
}

void Dog::setTimeOnGround(double time)
{
	timeOnGround = time;
}

int Dog::getNumerOfSteps()
{
	return numberOfSteps;
}

double Dog::getDistanceFromHips(Box * box)
{
	glm::vec3 end = box->getPosition();
	glm::vec3 start = body->getPosition();
	return glm::distance(end, start);
}
void Dog::setShouldUpdate(bool update)
{
	m_shouldUpdate = update;
}
bool Dog::shouldUpdate()
{
	return m_shouldUpdate;
}

double Dog::getRotationAmount()
{
	return rotationAmount;
}

double Dog::getTimeAlive()
{
	return timeAlive;
}

void Dog::checkIfMoving()
{
	glm::vec3 p = body->getPosition();
	glm::vec3 p2 = body->getPreviousPosition();

	if (std::abs(p.x - p2.x) > 0.001f && std::abs(p.y - p2.y) > 0.001f && std::abs(p.z - p2.z) > 0.001f) {
		noMovementPenalty++;
	}
}

double Dog::getNoMovementPenalty() {
	return noMovementPenalty;
}

void Dog::calculateSpeed()
{
	glm::vec3 currentPosition = getPosition();
	//double currentSpeed = currentPosition.z - m_previousPosition.z;
	double currentSpeed = body->getRigidBody()->getLinearVelocity().getZ() * 0.016;
	m_totalSpeed += currentSpeed;
	//std::cout << "currentspeed: " << currentSpeed << " linearVelocity: " << hips->getRigidBody()->getLinearVelocity().getZ() * 0.016 << "\n";


	m_previousPosition = currentPosition;
}

double Dog::getTotalSpeed()
{
	return m_totalSpeed;
}

double Dog::getDistanceWalked()
{
	//glm::vec3 end = getPosition();
	//glm::vec3 start = getStartPosition();
	////return start.z - end.z;
	//return end.z - start.z;

	glm::vec3 end = getPosition();
	glm::vec3 target = m_targetPosition;

	return glm::distance(end, target);
}

void Dog::setMaxMotorImpulses(double maxMotorImpulse)
{
	bool isEnableMotor = true;
	//HIPS
	for (int i = 0; i < 3; i++) {
		//body->getdofConstraint("frontRightTigh")->getRotationalLimitMotor(i)->m_enableMotor = isEnableMotor;
		//body->getdofConstraint("frontRightTigh")->getRotationalLimitMotor(i)->m_maxMotorForce = maxMotorImpulse;

		//body->getdofConstraint("frontLeftTigh")->getRotationalLimitMotor(i)->m_enableMotor = isEnableMotor;
		//body->getdofConstraint("frontLeftTigh")->getRotationalLimitMotor(i)->m_maxMotorForce = maxMotorImpulse;

		//body->getdofConstraint("backRightTigh")->getRotationalLimitMotor(i)->m_enableMotor = isEnableMotor;
		//body->getdofConstraint("backRightTigh")->getRotationalLimitMotor(i)->m_maxMotorForce = maxMotorImpulse;

		//body->getdofConstraint("backLeftTigh")->getRotationalLimitMotor(i)->m_enableMotor = isEnableMotor;
		//body->getdofConstraint("backLeftTigh")->getRotationalLimitMotor(i)->m_maxMotorForce = maxMotorImpulse;
	}

	body->getHinge("frontRightTigh")->enableMotor(isEnableMotor);
	body->getHinge("frontRightTigh")->setMaxMotorImpulse(maxMotorImpulse);

	body->getHinge("frontLeftTigh")->enableMotor(isEnableMotor);
	body->getHinge("frontLeftTigh")->setMaxMotorImpulse(maxMotorImpulse);

	body->getHinge("backRightTigh")->enableMotor(isEnableMotor);
	body->getHinge("backRightTigh")->setMaxMotorImpulse(maxMotorImpulse);

	body->getHinge("backLeftTigh")->enableMotor(isEnableMotor);
	body->getHinge("backLeftTigh")->setMaxMotorImpulse(maxMotorImpulse);

	//KNEES
	frontRightTigh->getHinge("frontRightKnee")->enableMotor(isEnableMotor);
	frontRightTigh->getHinge("frontRightKnee")->setMaxMotorImpulse(maxMotorImpulse);

	frontLeftTigh->getHinge("frontLeftKnee")->enableMotor(isEnableMotor);
	frontLeftTigh->getHinge("frontLeftKnee")->setMaxMotorImpulse(maxMotorImpulse);

	backRightTigh->getHinge("backRightKnee")->enableMotor(isEnableMotor);
	backRightTigh->getHinge("backRightKnee")->setMaxMotorImpulse(maxMotorImpulse);

	backLeftTigh->getHinge("backLeftKnee")->enableMotor(isEnableMotor);
	backLeftTigh->getHinge("backLeftKnee")->setMaxMotorImpulse(maxMotorImpulse);
}

bool Dog::noMovement(int currentStep)
{
	bool noMovement = true;
	if (currentStep > 5) {
		glm::vec3 p = body->getPosition();
		glm::vec3 p2 = body->getPreviousPosition();
		//std::cout << "x: " << std::abs(p.x - p2.x) << " y: " << std::abs(p.y - p2.y) << " z: " << std::abs(p.z - p2.z) << "\n";
		if (std::abs(p.x - p2.x) > 0.0001f || std::abs(p.y - p2.y) > 0.0001f || std::abs(p.z - p2.z) > 0.0001f) {
			noMovement = false;
		}
	}
	else {
		noMovement = false;
	}
	return noMovement;
}

void Dog::checkIfJointsAtLimit()
{

	double penalty = 0.01;

	// 0 = spin, 1 = fremover/bakover, 2 = side til side
	//frontRightTigh
	//btVector3 frtAngularUpper;
	//body->getdofConstraint("frontRightTigh")->getAngularUpperLimit(frtAngularUpper);

	//btVector3 frtAngularLower;
	//body->getdofConstraint("frontRightTigh")->getAngularLowerLimit(frtAngularLower);

	//if (body->getdofConstraint("frontRightTigh")->getAngle(1) >= frtAngularUpper.getX() - 0.01 || body->getdofConstraint("frontRightTigh")->getAngle(1) <= frtAngularLower.getX() + 0.01) {
	//	jointsAtLimitPenalty+= penalty;
	//}

	//frontLeftTigh
	//btVector3 fltAngularUpper;
	//body->getdofConstraint("frontLeftTigh")->getAngularUpperLimit(fltAngularUpper);

	//btVector3 fltAngularLower;
	//body->getdofConstraint("frontLeftTigh")->getAngularLowerLimit(fltAngularLower);

	//if (body->getdofConstraint("frontLeftTigh")->getAngle(1) >= fltAngularUpper.getX() - 0.01 || body->getdofConstraint("frontLeftTigh")->getAngle(1) <= fltAngularLower.getX() + 0.01) {
	//	jointsAtLimitPenalty += penalty;

	//}

	////backRightTigh
	//btVector3 brtAngularUpper;
	//body->getdofConstraint("backRightTigh")->getAngularUpperLimit(brtAngularUpper);

	//btVector3 brtAngularLower;
	//body->getdofConstraint("backRightTigh")->getAngularLowerLimit(brtAngularLower);

	//if (body->getdofConstraint("backRightTigh")->getAngle(1) >= brtAngularUpper.getX() - 0.01 || body->getdofConstraint("backRightTigh")->getAngle(1) <= brtAngularLower.getX() + 0.01) {
	//	jointsAtLimitPenalty += penalty;
	//}

	////backLeftTigh
	//btVector3 bltAngularUpper;
	//body->getdofConstraint("backLeftTigh")->getAngularUpperLimit(bltAngularUpper);

	//btVector3 bltAngularLower;
	//body->getdofConstraint("backLeftTigh")->getAngularLowerLimit(bltAngularLower);

	//if (body->getdofConstraint("backLeftTigh")->getAngle(1) >= bltAngularUpper.getX() - 0.01 || body->getdofConstraint("backLeftTigh")->getAngle(1) <= bltAngularLower.getX() + 0.01) {
	//	jointsAtLimitPenalty += penalty;
	//}

	//frontRightKnee
	if (frontRightTigh->getHinge("frontRightKnee")->getHingeAngle() >= frontRightTigh->getHinge("frontRightKnee")->getUpperLimit() - 0.01 || 
		frontRightTigh->getHinge("frontRightKnee")->getHingeAngle() <= frontRightTigh->getHinge("frontRightKnee")->getLowerLimit() + 0.01) {
		jointsAtLimitPenalty += penalty;
	}

	//frontLeftKnee
	if (frontLeftTigh->getHinge("frontLeftKnee")->getHingeAngle() >= frontLeftTigh->getHinge("frontLeftKnee")->getUpperLimit() - 0.01 || 
		frontLeftTigh->getHinge("frontLeftKnee")->getHingeAngle() <= frontLeftTigh->getHinge("frontLeftKnee")->getLowerLimit() + 0.01) {
		jointsAtLimitPenalty += penalty;
	}

	//backRightKnee
	if (backRightTigh->getHinge("backRightKnee")->getHingeAngle() >= backRightTigh->getHinge("backRightKnee")->getUpperLimit() - 0.01 ||
		backRightTigh->getHinge("backRightKnee")->getHingeAngle() <= backRightTigh->getHinge("backRightKnee")->getLowerLimit() + 0.01) {
		jointsAtLimitPenalty += penalty;
	}

	//backLeftKnee
	if (backLeftTigh->getHinge("backLeftKnee")->getHingeAngle() >= backLeftTigh->getHinge("backLeftKnee")->getUpperLimit() - 0.01 || 
		backLeftTigh->getHinge("backLeftKnee")->getHingeAngle() <= backLeftTigh->getHinge("backLeftKnee")->getLowerLimit() + 0.01) {
		jointsAtLimitPenalty += penalty;
	}

}

double Dog::getJointsAtlimitPenalty() {
	return jointsAtLimitPenalty;
}


std::vector<double> Dog::getRelativePositions()
{


	//tighMax: 4.11282 tighMin: 1.90611 ShinMax: 6.4059 ShinMin: 2.1688
	btVector3 bodyPosition = body->getRigidBody()->getCenterOfMassPosition();

	double tighMax = 4.11282;
	double tighMin = 1.90611;

	double shinMax = 6.4059;
	double ShinMin = 2.1688;

	//double frt = Util::normalize(getRelativePosition(frontRightTigh).getY(), tighMin, tighMax);
	//double frs = Util::normalize(getRelativePosition(frontRightShin).getY(), ShinMin, ShinMax);

	//double flt = Util::normalize(getRelativePosition(frontLeftTigh).getY(), tighMin, tighMax);
	//double fls = Util::normalize(getRelativePosition(frontLeftShin).getY(), ShinMin, ShinMax);

	//double brt = Util::normalize(getRelativePosition(backRightTigh).getY(), tighMin, tighMax);
	//double brs = Util::normalize(getRelativePosition(backRightShin).getY(), ShinMin, ShinMax);

	//double blt = Util::normalize(getRelativePosition(backLeftTigh).getY(), tighMin, tighMax);
	//double bls = Util::normalize(getRelativePosition(backLeftShin).getY(), ShinMin, ShinMax);

	double frt = Util::normalize(bodyPosition.distance(frontRightTigh->getRigidBody()->getCenterOfMassPosition()), tighMax, tighMin);
	double frs = Util::normalize(bodyPosition.distance(frontRightShin->getRigidBody()->getCenterOfMassPosition()), shinMax, ShinMin);

	double flt = Util::normalize(bodyPosition.distance(frontLeftTigh->getRigidBody()->getCenterOfMassPosition()), tighMax, tighMin);
	double fls = Util::normalize(bodyPosition.distance(frontLeftShin->getRigidBody()->getCenterOfMassPosition()), shinMax, ShinMin);

	double brt = Util::normalize(bodyPosition.distance(backRightTigh->getRigidBody()->getCenterOfMassPosition()), tighMax, tighMin);
	double brs = Util::normalize(bodyPosition.distance(backRightShin->getRigidBody()->getCenterOfMassPosition()), shinMax, ShinMin);

	double blt = Util::normalize(bodyPosition.distance(backLeftTigh->getRigidBody()->getCenterOfMassPosition()), tighMax, tighMin);
	double bls = Util::normalize(bodyPosition.distance(backLeftShin->getRigidBody()->getCenterOfMassPosition()), shinMax, ShinMin);

	std::vector<double> relativePositions = { frt, frs, flt, fls, brt, brs, blt, bls };

	//tighMax = 0;
	//tighMin = 1000;
	//shinMax = 0;
	//ShinMin = 1000;

	//for (int i = 0; i < relativePositions.size(); i++) {
	//	if (i % 2 == 0) {
	//		//even
	//		tighMax = (relativePositions[i] > tighMax) ? relativePositions[i] : tighMax;
	//		tighMin = (relativePositions[i] < tighMin) ? relativePositions[i] : tighMin;

	//	}
	//	else {
	//		shinMax = (relativePositions[i] > shinMax) ? relativePositions[i] : shinMax;
	//		ShinMin = (relativePositions[i] < ShinMin) ? relativePositions[i] : ShinMin;
	//	}
	//	//std::cout << relativePositions[i] << " "
	//}
	//std::cout << "\n";
	//std::cout << "tighMax: " << tighMax << " tighMin: " << tighMin << " ShinMax: " << shinMax << " ShinMin: " << ShinMin<< "\n";
	return relativePositions;
}

btVector3 Dog::getRelativePosition(Box* limb) {
	btVector3 bodyPosition = body->getRigidBody()->getCenterOfMassPosition();
	btScalar xDist = bodyPosition.getX() - limb->getRigidBody()->getCenterOfMassPosition().getX();
	btScalar yDist = bodyPosition.getY() - limb->getRigidBody()->getCenterOfMassPosition().getY();
	btScalar zDist = bodyPosition.getZ() - limb->getRigidBody()->getCenterOfMassPosition().getZ();

	btVector3 relativePosition(xDist, yDist, zDist);
	return relativePosition;
}

glm::vec3 Dog::getTargetPosition()
{
	return m_targetPosition;
}

void Dog::setTargetPosition(glm::vec3 target)
{
	m_targetPosition = target;
	maxDistanceToTarget = glm::distance(m_startPosition, m_targetPosition);
}

void Dog::checkRotation()
{
	btQuaternion orientation = body->getRigidBody()->getOrientation();
	//x = bakover/forover
	//std::cout << "X: " << orientation.getX() << "\n";
	//y = side til side
	//std::cout << "Y: " << orientation.getY() << "\n";
	//z = klokke rotasjon
	//std::cout << "Z: " << orientation.getZ() << "\n\n";

	double rotationCutoff = 0.4f;
	if (orientation.getY() > rotationCutoff || orientation.getY() < -rotationCutoff) {
		rotationAmount += std::abs(orientation.getY()) - rotationCutoff;
	}

	if (orientation.getZ() > rotationCutoff || orientation.getZ() < -rotationCutoff) {
		rotationAmount += std::abs(orientation.getZ()) - rotationCutoff;
	}

	if (orientation.getX() > rotationCutoff || orientation.getX() < -rotationCutoff) {
		rotationAmount += std::abs(orientation.getX()) - rotationCutoff;
	}
}

Dog::~Dog()
{

	delete body;
	delete frontRightTigh;
	delete frontRightShin;
	delete frontLeftTigh;
	delete frontLeftShin;
	delete backRightTigh;
	delete backRightShin;
	delete backLeftTigh;
	delete backLeftShin;
}

Box * Dog::getBody()
{
	return body;
}

Box * Dog::getFrontRightTigh()
{
	return frontRightTigh;
}

Box * Dog::getFrontRightShin()
{
	return frontRightShin;
}

Box * Dog::getFrontLeftTigh()
{
	return frontLeftTigh;
}

Box * Dog::getFrontLeftShin()
{
	return frontLeftShin;
}

Box * Dog::getBackRightTigh()
{
	return backRightTigh;
}

Box * Dog::getBackRightShin()
{
	return backRightShin;
}

Box * Dog::getBackLeftTigh()
{
	return backLeftTigh;
}

Box * Dog::getBackLeftShin()
{
	return backLeftShin;
}
