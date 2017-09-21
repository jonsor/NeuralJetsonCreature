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

	Box* body;

	Box* frontRightTigh;
	Box* frontRightShin;

	Box* frontLeftTigh;
	Box* frontLeftShin;

	Box* backRightTigh;
	Box* backRightShin;

	Box* backLeftTigh;
	Box* backLeftShin;

	//Limbs:
	//body = new Box(glm::vec3(m_startPosition.x, m_startPosition.y, m_startPosition.z), glm::vec3(0.9f, 0.1f, 0.1f), 1.5f, 2.0f, 0.4f, 25);

	//rightThigh = new Box(glm::vec3(m_startPosition.x - 1.5, m_startPosition.y - 4, m_startPosition.z), glm::vec3(0.2f, 0.3f, 0.7f), 0.5f, 1.8f, 0.3f, 15);
	//rightShin = new Box(glm::vec3(m_startPosition.x - 1.5, m_startPosition.y - 8, m_startPosition.z), glm::vec3(0.2f, 0.3f, 0.7f), 0.5f, 1.8f, 0.3f, 15);
	//rightFoot = new Box(glm::vec3(m_startPosition.x - 1.5, m_startPosition.y - 10.2, m_startPosition.z + 0.3), glm::vec3(0.2f, 0.3f, 0.7f), 0.6f, 0.15f, 1.0f, 5);

	//leftThigh = new Box(glm::vec3(m_startPosition.x + 1.5, m_startPosition.y - 4, m_startPosition.z), glm::vec3(0.2f, 0.3f, 0.7f), 0.5f, 1.8f, 0.3f, 15);
	//leftShin = new Box(glm::vec3(m_startPosition.x + 1.5, m_startPosition.y - 8, m_startPosition.z), glm::vec3(0.2f, 0.3f, 0.7f), 0.5f, 1.8f, 0.3f, 15);
	//leftFoot = new Box(glm::vec3(m_startPosition.x + 1.5, m_startPosition.y - 10.2, m_startPosition.z + 0.3), glm::vec3(0.2f, 0.3f, 0.7f), 0.6f, 0.15f, 1.0f, 5);

	////hips->getRigidBody()->setLinearFactor(btVector3(0, 0, 0));
	////hips->getRigidBody()->setAngularFactor(btVector3(0, 0, 0));
	//rightFoot->getRigidBody()->setFriction(5.0f);
	//leftFoot->getRigidBody()->setFriction(5.0f);
	//pm->addBody(hips->getRigidBody(), 1, 2);

	//pm->addBody(rightThigh->getRigidBody(), 1, 2);
	//pm->addBody(rightShin->getRigidBody(), 1, 2);
	//pm->addBody(rightFoot->getRigidBody(), 1, 2);

	//pm->addBody(leftThigh->getRigidBody(), 1, 2);
	//pm->addBody(leftShin->getRigidBody(), 1, 2);
	//pm->addBody(leftFoot->getRigidBody(), 1, 2);

	//calcCenterPosition();

	////Hinges:
	//bool noCol = true;
	////hips->addHinge(glm::vec3(0.0f, -1.5f, 0.0f), glm::vec3(1.0f, 1.5f, 0.0f), glm::vec3(1.0f, 0.0f, 0.0f), glm::vec3(1.0f, 0.0f, 0.0f), rightThigh, noCol, -1.5, 0.7, pm, "rightHip");
	////hips->addHinge(glm::vec3(0.0f, -1.5f, 0.0f), glm::vec3(-1.0f, 1.5f, 0.0f), glm::vec3(1.0f, 0.0f, 0.0f), glm::vec3(1.0f, 0.0f, 0.0f), leftThigh, noCol, -1.5, 0.7, pm, "leftHip");

	//hips->addDOFConstraint(rightThigh, noCol, -1.7, pm, "rightHip");
	//hips->addDOFConstraint(leftThigh, noCol, 1.7, pm, "leftHip");

	//rightThigh->addHinge(glm::vec3(0.0f, -1.9f, 0.0f), glm::vec3(0.0f, 1.9f, 0.0f), glm::vec3(1.0f, 0.0f, 0.0f), glm::vec3(1.0f, 0.0f, 0.0f), rightShin, noCol, -0.1, PI * 0.8, pm, "rightKnee");
	//leftThigh->addHinge(glm::vec3(0.0f, -1.9f, 0.0f), glm::vec3(0.0f, 1.9f, 0.0f), glm::vec3(1.0f, 0.0f, 0.0f), glm::vec3(1.0f, 0.0f, 0.0f), leftShin, noCol, -0.1, PI * 0.8, pm, "leftKnee");

	//rightShin->addHinge(glm::vec3(0.0f, -2.1f, 0.0f), glm::vec3(0.0f, 0.0f, 0.7f), glm::vec3(1.0f, 0.0f, 0.0f), glm::vec3(1.0f, 0.0f, 0.0f), rightFoot, noCol, -0.5, PI / 4, pm, "rightAnkle");
	//leftShin->addHinge(glm::vec3(0.0f, -2.1f, 0.0f), glm::vec3(0.0f, 0.0f, 0.7f), glm::vec3(1.0f, 0.0f, 0.0f), glm::vec3(1.0f, 0.0f, 0.0f), leftFoot, noCol, -0.5, PI / 4, pm, "leftAnkle");

	//setMaxMotorImpulses(40.0f);

	//Create the neural network
	std::vector<int> topology{ 62, 45, 30, 10 };
	createNeuralNetwork(topology, engine);

	//Set default fitness
	m_fitness = 0;

	int maxMinAnglesSize = 20;
	for (int i = 0; i < maxMinAnglesSize; i++) {
		maxMinAngles.push_back(0);
	}
	memoryNeurons = { 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 };
	m_maxHeight = 0;

	numberOfSteps = 0;
	lastFootThatStepped = 'n';

	//averageFeetStartPos = getAverageFeetPosition();

	resultVec = { 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 };

	m_shouldUpdate = true;
	timeOnTwoLegs = 0;
	timeOnGround = 0;

	m_previousPosition = getPosition();
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

glm::vec3 Dog::getPosition()
{
	return body->getPosition();
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
	m_neuralNetwork = NeuralNetwork(topology, engine);
}

void Dog::setNeuralNetwork(NeuralNetwork neuralNetwork)
{
	m_neuralNetwork = neuralNetwork;
}

NeuralNetwork Dog::getNeuralNetwork()
{
	return m_neuralNetwork;
}

NeuralNetwork* Dog::getNN()
{
	return &m_neuralNetwork;
}

void Dog::updateNeuralNetwork()
{
	//std::vector<double> inputs = calculateInputs();
	//m_neuralNetwork.forward(inputs);
	//m_neuralNetwork.getResults(resultVec);

	//getHips()->getdofConstraint("rightHip")->getRotationalLimitMotor(2)->m_targetVelocity = 1.f;
	//getHips()->getdofConstraint("leftHip")->getRotationalLimitMotor(2)->m_targetVelocity = 1.f;
	//getRightThigh()->getHinge("rightKnee")->setMotorTargetVelocity(-1.f);
	//getLeftThigh()->getHinge("leftKnee")->setMotorTargetVelocity(-1.f);

	//getRightShin()->getHinge("rightAnkle")->setMotorTargetVelocity(-1.f);
	//getLeftShin()->getHinge("leftAnkle")->setMotorTargetVelocity(-1.f);

	//setAllTargetVelocities(resultVec);
}

void Dog::mutate(double mutationRate, double mutationChance, std::default_random_engine engine)
{
	m_neuralNetwork.mutate(mutationRate, mutationChance, engine);
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
	return body->getPosition().y;
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
		rotationAmount += (std::abs(orientation.getY()) - rotationCutoff) / 4;
	}

	//if (orientation.getZ() > rotationCutoff || orientation.getZ() < -rotationCutoff) {
	//	rotationAmount += std::abs(orientation.getZ()) - rotationCutoff;
	//}
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
	glm::vec3 pos = body->getPosition();
	glm::vec3 prevPos = body->getPreviousPosition();

	if (prevPos.x > pos.x + 0.1f || prevPos.x < pos.x - 0.1f &&
		prevPos.z > pos.z + 0.1f || prevPos.z < pos.z - 0.1f) {
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
