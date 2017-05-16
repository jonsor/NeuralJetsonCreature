/**
Spider.cpp
Purpose: Sets up, renders and updates a complete, hardcoded spider creature.

@author Sjur Barndon, Jonas Sørsdal
@version 1.0 29.03.2017
*/

#include "stdafx.h"
#include "Spider.h"

/**
	Constructor for the spider. Sets up a complete hardcoded
	spider (a body and six legs with thighs, shins).
	Also links them together with mobile joints (point to point joints).

	@param pm pointer to the PhysicsManager class so that the limbs and joints can be added to the world simulation.
*/
Spider::Spider(PhysicsManager* pm, glm::vec3 startPosition) : m_startPosition(startPosition)
{
	//Limbs:
	body = new Cube(m_startPosition, glm::vec3(0.9f, 0.1f, 0.9f), 1.0f, 0.2f, 1.5f, 15);
	
	leftFrontUpper = new Cube(glm::vec3(startPosition.x, startPosition.y, startPosition.z), glm::vec3(0.1f, 0.9f, 0.9f), 0.1f, 1.0f, 0.1f, 10);
	leftFrontLower = new Cube(glm::vec3(startPosition.x, startPosition.y, startPosition.z), glm::vec3(0.1f, 0.9f, 0.9f), 0.1f, 1.0f, 0.1f, 10);

	rightFrontUpper = new Cube(glm::vec3(startPosition.x, startPosition.y, startPosition.z), glm::vec3(0.1f, 0.9f, 0.9f), 0.1f, 1.0f, 0.1f, 10);
	rightFrontLower = new Cube(glm::vec3(startPosition.x, startPosition.y, startPosition.z), glm::vec3(0.1f, 0.9f, 0.9f), 0.1f, 1.0f, 0.1f, 10);
	
	leftBackUpper = new Cube(glm::vec3(startPosition.x, startPosition.y, startPosition.z), glm::vec3(0.1f, 0.9f, 0.9f), 0.1f, 1.0f, 0.1f, 10);
	leftBackLower = new Cube(glm::vec3(startPosition.x, startPosition.y, startPosition.z), glm::vec3(0.1f, 0.9f, 0.9f), 0.1f, 1.0f, 0.1f, 10);
	
	rightBackUpper = new Cube(glm::vec3(startPosition.x, startPosition.y, startPosition.z), glm::vec3(0.1f, 0.9f, 0.9f), 0.1f, 1.0f, 0.1f, 10);
	rightBackLower = new Cube(glm::vec3(startPosition.x, startPosition.y, startPosition.z), glm::vec3(0.1f, 0.9f, 0.9f), 0.1f, 1.0f, 0.1f, 10);

	pm->addBody(body->getRigidBody(), 1, 2);

	pm->addBody(leftFrontUpper->getRigidBody(), 1, 2);
	pm->addBody(leftFrontLower->getRigidBody(), 1, 2);

	pm->addBody(rightFrontUpper->getRigidBody(), 1, 2);
	pm->addBody(rightFrontLower->getRigidBody(), 1, 2);


	pm->addBody(leftBackUpper->getRigidBody(), 1, 2);
	pm->addBody(leftBackLower->getRigidBody(), 1, 2);

	pm->addBody(rightBackUpper->getRigidBody(), 1, 2);
	pm->addBody(rightBackLower->getRigidBody(), 1, 2);

	//Hinges:
	bool noCol = true;

	/********/body->addHinge(glm::vec3(1.0f, 0.f, 1.0f), glm::vec3(0.0f, 1.f, 0.0f), glm::vec3(0.0f, 1.5f, 0.0f), glm::vec3(1.0f, -0.5f, 0.0f), leftFrontUpper, noCol, -1.5, 0.7, pm, "leftFrontHip");
	leftFrontUpper->addHinge(glm::vec3(0.0f, -1.0f, 0.0f), glm::vec3(0.0f, 1.0f, 0.0f), glm::vec3(0.0f, 0.0f, 1.0f), glm::vec3(0.0f, 0.0f, 1.0f), leftFrontLower, noCol, -0.7, 1.5, pm, "leftFrontKnee");

	/********/body->addHinge(glm::vec3(-1.0f, 0.f, 1.0f), glm::vec3(0.0f, 1.f, 0.0f), glm::vec3(0.0f, -1.5f, 0.0f), glm::vec3(1.0f, 0.5f, 0.0f), rightFrontUpper, noCol, -1.5, 0.7, pm, "rightFrontHip");
	rightFrontUpper->addHinge(glm::vec3(0.0f, -1.0f, 0.0f), glm::vec3(0.0f, 1.0f, 0.0f), glm::vec3(0.0f, 0.0f, 1.0f), glm::vec3(0.0f, 0.0f, 1.0f), rightFrontLower, noCol, -1.5, 0.7, pm, "rightFrontKnee");
	
	/*******/body->addHinge(glm::vec3(1.0f, 0.f, -1.0f), glm::vec3(0.0f, 1.f, 0.0f), glm::vec3(0.0f, 1.5f, 0.0f), glm::vec3(1.0f, -0.5f, 0.0f), leftBackUpper, noCol, -1.5, 0.7, pm, "leftBackHip");
	leftBackUpper->addHinge(glm::vec3(0.0f, -1.0f, 0.0f), glm::vec3(0.0f, 1.0f, 0.0f), glm::vec3(0.0f, 0.0f, 1.0f), glm::vec3(0.0f, 0.0f, 1.0f), leftBackLower, noCol, -0.7, 1.5, pm, "leftBackKnee");

	/*******/body->addHinge(glm::vec3(-1.0f, 0.f, -1.0f), glm::vec3(0.0f, 1.f, 0.0f), glm::vec3(0.0f, -1.5f, 0.0f), glm::vec3(1.0f, 0.5f, 0.0f), rightBackUpper, noCol, -1.5, 0.7, pm, "rightBackHip");
	rightBackUpper->addHinge(glm::vec3(0.0f, -1.0f, 0.0f), glm::vec3(0.0f, 1.0f, 0.0f), glm::vec3(0.0f, 0.0f, 1.0f), glm::vec3(0.0f, 0.0f, 1.0f), rightBackLower, noCol, -1.5, 0.7, pm, "rightBackKnee");
	
	m_fitness = 0;

	//Create Neural Network
	std::vector<int> topology{42, 20, 20, 8 };
	createNeuralNetwork(topology);

	setMaxMotorImpulses(20.f);
}

/**
Calls the render function for each limb of the creature.

@param shader takes the shader for rendering puposes.
*/
void Spider::render(Shader shader)
{
	body->render(shader);

	leftFrontUpper->render(shader);
	leftFrontLower->render(shader);

	rightFrontUpper->render(shader);
	rightFrontLower->render(shader);

	leftBackUpper->render(shader);
	leftBackLower->render(shader);

	rightBackUpper->render(shader);
	rightBackLower->render(shader);
}

/**
	Updates the physics for each limb in the spider.
*/
void Spider::updatePhysics()
{

	updateNeuralNetwork();

	//body->getRigidBody()->setDamping(2000, 2000);
	//body->getRigidBody()->setRestitution(2000);
	
	body->updatePhysics();

	leftFrontUpper->updatePhysics();
	leftFrontLower->updatePhysics();


	rightFrontUpper->updatePhysics();
	rightFrontLower->updatePhysics();

	leftBackUpper->updatePhysics();
	leftBackLower->updatePhysics();


	rightBackUpper->updatePhysics();
	rightBackLower->updatePhysics();
}

void Spider::setAllTargetVelocities(std::vector<double>& resultVec)
{
	//CHANGE MAX VELOCITY NORMALIZATION IF YOU CHANGE m
	double m = 5;
	double mU = 0.0;
	
	//HIPS
	body->getHinge("leftFrontHip")->setMotorTargetVelocity((resultVec[0] - mU) * m);
	body->getHinge("rightFrontHip")->setMotorTargetVelocity((resultVec[1] - mU) * m);
	body->getHinge("leftBackHip")->setMotorTargetVelocity((resultVec[2] - mU) * m);
	body->getHinge("rightBackHip")->setMotorTargetVelocity((resultVec[3] - mU) * m);

	//KnEES
	leftFrontUpper->getHinge("leftFrontKnee")->setMotorTargetVelocity((resultVec[4] - mU) * m);
	rightFrontUpper->getHinge("rightFrontKnee")->setMotorTargetVelocity((resultVec[5] - mU) * m);
	leftBackUpper->getHinge("leftBackKnee")->setMotorTargetVelocity((resultVec[6] - mU) * m);
	rightBackUpper->getHinge("rightBackKnee")->setMotorTargetVelocity((resultVec[7] - mU) * m);

}

void Spider::setMaxMotorImpulses(double maxMotorImpulse)
{
	bool isEnableMotor = true;
	
	//THEM HIPS
	
	body->getHinge("leftFrontHip")->enableMotor(isEnableMotor);
	body->getHinge("rightFrontHip")->enableMotor(isEnableMotor);
	body->getHinge("leftBackHip")->enableMotor(isEnableMotor);
	body->getHinge("rightBackHip")->enableMotor(isEnableMotor);
	
	body->getHinge("leftFrontHip")->setMaxMotorImpulse(maxMotorImpulse);
	body->getHinge("rightFrontHip")->setMaxMotorImpulse(maxMotorImpulse);
	body->getHinge("leftBackHip")->setMaxMotorImpulse(maxMotorImpulse);
	body->getHinge("rightBackHip")->setMaxMotorImpulse(maxMotorImpulse);


	//KnEES
	leftFrontUpper->getHinge("leftFrontKnee")->enableMotor(isEnableMotor);
	rightFrontUpper->getHinge("rightFrontKnee")->enableMotor(isEnableMotor);
	leftBackUpper->getHinge("leftBackKnee")->enableMotor(isEnableMotor);
	rightBackUpper->getHinge("rightBackKnee")->enableMotor(isEnableMotor);

	leftFrontUpper->getHinge("leftFrontKnee")->setMaxMotorImpulse(maxMotorImpulse);
	rightFrontUpper->getHinge("rightFrontKnee")->setMaxMotorImpulse(maxMotorImpulse);
	leftBackUpper->getHinge("leftBackKnee")->setMaxMotorImpulse(maxMotorImpulse);
	rightBackUpper->getHinge("rightBackKnee")->setMaxMotorImpulse(maxMotorImpulse);
}

void Spider::createNeuralNetwork(std::vector<int> topology)
{
	m_neuralNetwork = NeuralNetwork(topology);
}

void Spider::setNeuralNetwork(NeuralNetwork neuralNetwork)
{
	m_neuralNetwork = neuralNetwork;
}

NeuralNetwork Spider::getNeuralNetwork()
{
	return m_neuralNetwork;
}

void Spider::updateNeuralNetwork()
{
	std::vector<double> inputs = calculateInputs();

	m_neuralNetwork.forward(inputs);
	m_neuralNetwork.getResults(resultVec);
	setAllTargetVelocities(resultVec);
}

void Spider::mutate(double mutationRate)
{
	m_neuralNetwork.mutate(mutationRate);
}

void Spider::setFitness(double fitness)
{
	m_fitness = fitness;
}

double Spider::getFitness()
{
	return m_fitness;
}

std::vector<double> Spider::calculateInputs()
{
	std::vector<double> inputs;
	//Height
	double height = Util::normalize(body->getPosition().y, 0, 6);
	inputs.push_back(height);

	//target velocities
	//TODO NORMALIZE
	std::vector<double> angularVel = getAllAngularVelocities();
	const double MAX = 8;
	for (int i = 0; i < angularVel.size(); i++) {
		angularVel[i] = Util::normalize(angularVel[i], -MAX, MAX);
	}
	inputs.insert(inputs.end(), angularVel.begin(), angularVel.end());

	inputs.push_back(body->getRigidBody()->getOrientation().getX());
	inputs.push_back(body->getRigidBody()->getOrientation().getY());
	inputs.push_back(body->getRigidBody()->getOrientation().getZ());

	const double VEL_MAX = 8;
	inputs.push_back(Util::normalize(body->getRigidBody()->getLinearVelocity().getX(), -VEL_MAX, VEL_MAX));
	inputs.push_back(Util::normalize(body->getRigidBody()->getLinearVelocity().getY(), -VEL_MAX, VEL_MAX));
	inputs.push_back(Util::normalize(body->getRigidBody()->getLinearVelocity().getZ(), -VEL_MAX, VEL_MAX));


	//Hinge angles
	std::vector<double> inputAngles = getAllAngles();
	inputs.insert(inputs.end(), inputAngles.begin(), inputAngles.end());
	//Foot heights
	int numNeg = 0;
	int numPos = 0;
	for (int i = 0; i < inputs.size(); i++) {
		double testCheck = inputs[i];
		if (testCheck < 0) {
			numNeg++;
		}
		else {
			numPos++;
		}
	}
	return inputs;
}

std::vector<double> Spider::getAllAngularVelocities()
{
	std::vector<double> angularVelocities;
	//HIPS
	angularVelocities.push_back(body->getRigidBody()->getAngularVelocity().getX());
	angularVelocities.push_back(body->getRigidBody()->getAngularVelocity().getY());
	angularVelocities.push_back(body->getRigidBody()->getAngularVelocity().getZ());

	//rightFrontUpper
	angularVelocities.push_back(rightFrontUpper->getRigidBody()->getAngularVelocity().getX());
	angularVelocities.push_back(rightFrontUpper->getRigidBody()->getAngularVelocity().getY());
	angularVelocities.push_back(rightFrontUpper->getRigidBody()->getAngularVelocity().getZ());

	//rightFrontLower
	angularVelocities.push_back(rightFrontLower->getRigidBody()->getAngularVelocity().getX());
	angularVelocities.push_back(rightFrontLower->getRigidBody()->getAngularVelocity().getY());
	angularVelocities.push_back(rightFrontLower->getRigidBody()->getAngularVelocity().getZ());

	//leftFrontUpper
	angularVelocities.push_back(leftFrontUpper->getRigidBody()->getAngularVelocity().getX());
	angularVelocities.push_back(leftFrontUpper->getRigidBody()->getAngularVelocity().getY());
	angularVelocities.push_back(leftFrontUpper->getRigidBody()->getAngularVelocity().getZ());

	//leftFrontLower
	angularVelocities.push_back(leftFrontLower->getRigidBody()->getAngularVelocity().getX());
	angularVelocities.push_back(leftFrontLower->getRigidBody()->getAngularVelocity().getY());
	angularVelocities.push_back(leftFrontLower->getRigidBody()->getAngularVelocity().getZ());

	//rightBackUpper
	angularVelocities.push_back(rightBackUpper->getRigidBody()->getAngularVelocity().getX());
	angularVelocities.push_back(rightBackUpper->getRigidBody()->getAngularVelocity().getY());
	angularVelocities.push_back(rightBackUpper->getRigidBody()->getAngularVelocity().getZ());

	//rightBackLower
	angularVelocities.push_back(rightBackLower->getRigidBody()->getAngularVelocity().getX());
	angularVelocities.push_back(rightBackLower->getRigidBody()->getAngularVelocity().getY());
	angularVelocities.push_back(rightBackLower->getRigidBody()->getAngularVelocity().getZ());

	//leftBackUpper
	angularVelocities.push_back(leftBackUpper->getRigidBody()->getAngularVelocity().getX());
	angularVelocities.push_back(leftBackUpper->getRigidBody()->getAngularVelocity().getY());
	angularVelocities.push_back(leftBackUpper->getRigidBody()->getAngularVelocity().getZ());

	//leftBackLower
	angularVelocities.push_back(leftBackLower->getRigidBody()->getAngularVelocity().getX());
	angularVelocities.push_back(leftBackLower->getRigidBody()->getAngularVelocity().getY());
	angularVelocities.push_back(leftBackLower->getRigidBody()->getAngularVelocity().getZ());

	return angularVelocities;
}

std::vector<double> Spider::getAllAngles()
{
	double rfh = Util::normalize(body->getHinge("rightFrontHip")->getHingeAngle(), body->getHinge("rightFrontHip")->getLowerLimit(), body->getHinge("rightFrontHip")->getUpperLimit());
	double lfh = Util::normalize(body->getHinge("leftFrontHip")->getHingeAngle(), body->getHinge("leftFrontHip")->getLowerLimit(), body->getHinge("leftFrontHip")->getUpperLimit());
	double rbh = Util::normalize(body->getHinge("rightBackHip")->getHingeAngle(), body->getHinge("rightBackHip")->getLowerLimit(), body->getHinge("rightBackHip")->getUpperLimit());
	double lbh = Util::normalize(body->getHinge("leftBackHip")->getHingeAngle(), body->getHinge("leftBackHip")->getLowerLimit(), body->getHinge("leftBackHip")->getUpperLimit());

	double rfk = Util::normalize(rightFrontUpper->getHinge("rightFrontKnee")->getHingeAngle(), rightFrontUpper->getHinge("rightFrontKnee")->getLowerLimit(), rightFrontUpper->getHinge("rightFrontKnee")->getUpperLimit());
	double lfk = Util::normalize(leftFrontUpper->getHinge("leftFrontKnee")->getHingeAngle(), leftFrontUpper->getHinge("leftFrontKnee")->getLowerLimit(), leftFrontUpper->getHinge("leftFrontKnee")->getUpperLimit());
	double lbk = Util::normalize(leftBackUpper->getHinge("leftBackKnee")->getHingeAngle(), leftBackUpper->getHinge("leftBackKnee")->getLowerLimit(), leftBackUpper->getHinge("leftBackKnee")->getUpperLimit());
	double rbk = Util::normalize(rightBackUpper->getHinge("rightBackKnee")->getHingeAngle(), rightBackUpper->getHinge("rightBackKnee")->getLowerLimit(), rightBackUpper->getHinge("rightBackKnee")->getUpperLimit());


	return{ rfh, lfh, rbh, lbh, rfk, lfk, lbk, rbk };
}

void Spider::activate() {

	body->getRigidBody()->activate();

	rightFrontUpper->getRigidBody()->activate();
	leftFrontUpper->getRigidBody()->activate();
	leftBackUpper->getRigidBody()->activate();
	rightBackUpper->getRigidBody()->activate();

	rightFrontLower->getRigidBody()->activate();
	leftFrontLower->getRigidBody()->activate();
	leftBackLower->getRigidBody()->activate();
	rightBackLower->getRigidBody()->activate();

}


Cube * Spider::getBody()
{
	return body;
}

Cube * Spider::getLeftFrontUpper()
{
	return leftFrontUpper;
}

Cube * Spider::getLeftFrontLower()
{
	return leftFrontLower;
}

Cube * Spider::getLeftBackUpper()
{
	return leftBackUpper;
}

Cube * Spider::getLeftBackLower()
{
	return leftBackLower;
}

Cube * Spider::getRightFrontUpper()
{
	return rightFrontUpper;
}

Cube * Spider::getRightFrontLower()
{
	return rightFrontLower;
}

Cube * Spider::getRightBackUpper()
{
	return rightBackUpper;
}

Cube * Spider::getRightBackLower()
{
	return rightBackLower;
}

glm::vec3 Spider::getCenterPosition()
{
	glm::vec3 centerPosition = body->getPosition();
	centerPosition.y -= body->getHeight() / 2;
	centerPosition.x += body->getWidth() / 2;
	centerPosition.z += body->getDepth() / 2;
	
	return centerPosition;
}

glm::vec3 Spider::getPosition()
{
	return getCenterPosition();
}

glm::vec3 Spider::getStartPosition()
{
	return m_startPosition;
}


Spider::~Spider()
{
	delete body;
	delete leftFrontUpper;
	delete leftFrontLower;

	delete rightFrontUpper;
	delete rightFrontLower;

	delete leftBackUpper;
	delete leftBackLower;

	delete rightBackUpper;
	delete rightBackLower;
}
