/**
Creature.cpp
Purpose: Sets up, renders and updates a complete, hardcoded creature.

@author Sjur Barndon, Jonas Sørsdal
@version 1.0 23.03.2017
*/


#include "stdafx.h"
#include "Creature.h"
#include <chrono>

/**
	Constructor for the creature. Sets up a complete hardcoded 
	creature (a hip and two legs with thighs, shins and feet).
	Also links them together with mobile joints (hinges).
	TODO: Create this class customizable, not hardcoded.

	@param pm pointer to the PhysicsManager class so that the limbs and joints can be added to the world simulation.
*/
Creature::Creature(PhysicsManager* pm, glm::vec3 startPosition): m_startPosition(startPosition)
{
	//Limbs:
	//chest = new Cube(glm::vec3(6.0f, 15.0f, 1.0f), glm::vec3(0.9f, 0.9f, 0.1f), 1.5f, 2.5f, 0.2f, 20);
	hips = new Cube(glm::vec3(m_startPosition.x, m_startPosition.y, m_startPosition.z), glm::vec3(0.9f, 0.1f, 0.1f), 1.5f, 1.0f, 0.4f, 10);
	
	rightThigh = new Cube(glm::vec3(m_startPosition.x - 1.5, m_startPosition.y - 4, m_startPosition.z), glm::vec3(0.2f, 0.3f, 0.7f), 0.5f, 1.8f, 0.3f, 10);
	rightShin = new Cube(glm::vec3(m_startPosition.x - 1.5, m_startPosition.y - 8, m_startPosition.z), glm::vec3(0.2f, 0.3f, 0.7f), 0.5f, 1.8f, 0.3f, 10);
	rightFoot = new Cube(glm::vec3(m_startPosition.x - 1.5, m_startPosition.y - 10.2, m_startPosition.z + 0.3), glm::vec3(0.2f, 0.3f, 0.7f), 0.6f, 0.15f, 1.0f, 5);
	
	leftThigh = new Cube(glm::vec3(m_startPosition.x + 1.5, m_startPosition.y - 4, m_startPosition.z), glm::vec3(0.2f, 0.3f, 0.7f), 0.5f, 1.8f, 0.3f, 10);
	leftShin = new Cube(glm::vec3(m_startPosition.x + 1.5, m_startPosition.y - 8, m_startPosition.z), glm::vec3(0.2f, 0.3f, 0.7f), 0.5f, 1.8f, 0.3f, 10);
	leftFoot = new Cube(glm::vec3(m_startPosition.x + 1.5, m_startPosition.y - 10.2, m_startPosition.z + 0.3), glm::vec3(0.2f, 0.3f, 0.7f), 0.6f, 0.15f, 1.0f, 5);
	
	m_startPosition.x += hips->getWidth() / 2;
	m_startPosition.y -= hips->getHeight() / 2;
	m_startPosition.z += hips->getDepth() / 2;
	
	//pm->addBody(chest->getRigidBody());
	pm->addBody(hips->getRigidBody());

	pm->addBody(rightThigh->getRigidBody());
	pm->addBody(rightShin->getRigidBody());
	pm->addBody(rightFoot->getRigidBody());

	pm->addBody(leftThigh->getRigidBody());
	pm->addBody(leftShin->getRigidBody());
	pm->addBody(leftFoot->getRigidBody());

	calcCenterPosition();
	//Hinges:
	bool noCol = true;
	//chest->addHinge(glm::vec3(0.0f, -1.8f, 0.0f), glm::vec3(0.0f, 1.8f, 0.0f), glm::vec3(1.0f, 0.0f, 0.0f), glm::vec3(1.0f, 0.0f, 0.0f), hips, noCol, -PI/2, PI/2, pm, "abdomen");
	hips->addHinge(glm::vec3(0.0f, -1.5f, 0.0f), glm::vec3(1.0f, 1.5f, 0.0f), glm::vec3(1.0f, 0.0f, 0.0f), glm::vec3(1.0f, 0.0f, 0.0f), rightThigh, noCol, -1.5, 0.7, pm, "rightHip");
	hips->addHinge(glm::vec3(0.0f, -1.5f, 0.0f), glm::vec3(-1.0f, 1.5f, 0.0f), glm::vec3(1.0f, 0.0f, 0.0f), glm::vec3(1.0f, 0.0f, 0.0f), leftThigh, noCol, -1.5, 0.7, pm, "leftHip");

	rightThigh->addHinge(glm::vec3(0.0f, -1.9f, 0.0f), glm::vec3(0.0f, 1.9f, 0.0f), glm::vec3(1.0f, 0.0f, 0.0f), glm::vec3(1.0f, 0.0f, 0.0f), rightShin, noCol, -0.1, PI * 0.8, pm, "rightKnee");
	leftThigh->addHinge(glm::vec3(0.0f, -1.9f, 0.0f), glm::vec3(0.0f, 1.9f, 0.0f), glm::vec3(1.0f, 0.0f, 0.0f), glm::vec3(1.0f, 0.0f, 0.0f), leftShin, noCol, -0.1, PI/1.2, pm, "leftKnee");

	rightShin->addHinge(glm::vec3(0.0f, -2.1f, 0.0f), glm::vec3(0.0f, 0.0f, 0.7f), glm::vec3(1.0f, 0.0f, 0.0f), glm::vec3(1.0f, 0.0f, 0.0f), rightFoot, noCol, 0, PI/4, pm, "rightAnkle");
	leftShin->addHinge(glm::vec3(0.0f, -2.1f, 0.0f), glm::vec3(0.0f, 0.0f, 0.7f), glm::vec3(1.0f, 0.0f, 0.0f), glm::vec3(1.0f, 0.0f, 0.0f), leftFoot, noCol, 0, PI/4, pm, "leftAnkle");

	setMaxMotorImpulses(20.0f);

	//Create the neural network
	std::vector<int> topology{ 28, 28, 20, 15, 6 };
	createNeuralNetwork(topology);
}

/**
	Calls the render function for each limb of the creature.

	@param shader takes the shader for rendering puposes.
*/
void Creature::render(Shader shader)
{
	//chest->render(shader);
	hips->render(shader);

	rightThigh->render(shader);
	rightShin->render(shader);
	rightFoot->render(shader);

	leftThigh->render(shader);
	leftShin->render(shader);
	leftFoot->render(shader);
}

/**
	Updates the physics for each limb in the creature. 
	Also sets dampening and restitution?
*/

void Creature::updatePhysics()
{
	if (yo >= 2000) {
		std::vector<int> topology{ 28, 28, 20, 15, 6 };
		createNeuralNetwork(topology);
		std::cout << "NEEEEEEEEEEEEEEEEEEEEW NEEEEEEEEEEEEEEEEEEEEEEEEEEEEEET" << std::endl;
		yo = 0;
	}
	yo++;

	updateNeuralNetwork();

	calcCenterPosition();
	//chest->updatePhysics();
	hips->getRigidBody()->setDamping(0, 0);
	hips->getRigidBody()->setRestitution(0);
	hips->updatePhysics();

	rightThigh->updatePhysics();
	rightShin->updatePhysics();
	rightFoot->updatePhysics();

	leftThigh->updatePhysics();
	leftShin->updatePhysics();
	leftFoot->updatePhysics();


}

void Creature::calcCenterPosition()
{
	centerPosition = hips->getPosition();
	centerPosition.y -= hips->getHeight() / 2;
	centerPosition.x += hips->getWidth() / 2;
	centerPosition.z += hips->getDepth() / 2;
	//std::cout << centerPosition.z << std::endl;
}

glm::vec3 Creature::getRelativePosition(Cube* cube) {
	glm::vec3 relativePosition;
	relativePosition.x = cube->getPosition().x - centerPosition.x;
	relativePosition.y = cube->getPosition().y - centerPosition.y;
	relativePosition.z = cube->getPosition().z - centerPosition.z;
	return relativePosition;
}

double Creature::get2DAngle(Cube* cube1, Cube* cube2) {
	return 0.0;
}

void Creature::activate(){
	hips->getRigidBody()->activate();
	rightThigh->getRigidBody()->activate();
	rightShin->getRigidBody()->activate();
	rightFoot->getRigidBody()->activate();
	leftThigh->getRigidBody()->activate();
	leftShin->getRigidBody()->activate();
	leftFoot->getRigidBody()->activate();
}

std::vector<double> Creature::getAllAngles()
{

	double rha = (getHips()->getHinge("rightHip")->getHingeAngle() - getHips()->getHinge("rightHip")->getLowerLimit()) / (getHips()->getHinge("rightHip")->getUpperLimit() - getHips()->getHinge("rightHip")->getLowerLimit());
	double lha = (getHips()->getHinge("leftHip")->getHingeAngle() - getHips()->getHinge("leftHip")->getLowerLimit()) / (getHips()->getHinge("leftHip")->getUpperLimit() - getHips()->getHinge("leftHip")->getLowerLimit());

	double rka = (getRightThigh()->getHinge("rightKnee")->getHingeAngle() - getRightThigh()->getHinge("rightKnee")->getLowerLimit()) / (getRightThigh()->getHinge("rightKnee")->getUpperLimit() - getRightThigh()->getHinge("rightKnee")->getLowerLimit());
	double lka = (getLeftThigh()->getHinge("leftKnee")->getHingeAngle() - getLeftThigh()->getHinge("leftKnee")->getLowerLimit()) / (getLeftThigh()->getHinge("leftKnee")->getUpperLimit() - getLeftThigh()->getHinge("leftKnee")->getLowerLimit());

	double raa = (getRightShin()->getHinge("rightAnkle")->getHingeAngle() - getRightShin()->getHinge("rightAnkle")->getLowerLimit()) / (getRightShin()->getHinge("rightAnkle")->getUpperLimit() - getRightShin()->getHinge("rightAnkle")->getLowerLimit());
	double laa = (getLeftShin()->getHinge("leftAnkle")->getHingeAngle() - getLeftShin()->getHinge("leftAnkle")->getLowerLimit()) / (getLeftShin()->getHinge("leftAnkle")->getUpperLimit() - getLeftShin()->getHinge("leftAnkle")->getLowerLimit());
	rha = (rha < 0) ? 0.0 : rha;
	lha = (lha < 0) ? 0.0 : lha;
	rka = (rka < 0) ? 0.0 : rka;
	lka = (lka < 0) ? 0.0 : lka;
	raa = (raa < 0) ? 0.0 : raa;
	laa = (laa < 0) ? 0.0 : laa;



	return{ rha, lha, rka, lka, raa, laa };
	//std::cout << "NA: " << rka << " " << lka << "\n";
	//double time = sin(0.0002 * std::chrono::system_clock::now().time_since_epoch().count());
	//return{rka, lka};
}

std::vector<double> Creature::calculateInputs()
{
	std::vector<double> inputs;
	//Height
	double height = Util::normalize(hips->getPosition().y, 0, 20);
	inputs.push_back(height);

	//target velocities
	std::vector<double> angularVel = getAllAngularVelocities();
	inputs.reserve(angularVel.size());
	inputs.insert(inputs.end(), angularVel.begin(), angularVel.end());

	//for (int i = 0; i < inputs.size(); i++) {
	//	std::cout << inputs.at(i) << "  ";
	//}
	//std::cout << std::endl;

	//Hinge angles
	std::vector<double> inputAngles = getAllAngles();
	inputs.reserve(inputAngles.size());
	inputs.insert(inputs.end(), inputAngles.begin(), inputAngles.end());
	//Foot heights
	
	return inputs;
}

std::vector<double> Creature::getAllAngularVelocities()
{
	std::vector<double> angularVelocities;
	//HIPS
	angularVelocities.push_back(hips->getRigidBody()->getAngularVelocity().getX());
	angularVelocities.push_back(hips->getRigidBody()->getAngularVelocity().getY());
	angularVelocities.push_back(hips->getRigidBody()->getAngularVelocity().getZ());

	//rightThigh
	angularVelocities.push_back(rightThigh->getRigidBody()->getAngularVelocity().getX());
	angularVelocities.push_back(rightThigh->getRigidBody()->getAngularVelocity().getY());
	angularVelocities.push_back(rightThigh->getRigidBody()->getAngularVelocity().getZ());

	//rightShin
	angularVelocities.push_back(rightShin->getRigidBody()->getAngularVelocity().getX());
	angularVelocities.push_back(rightShin->getRigidBody()->getAngularVelocity().getY());
	angularVelocities.push_back(rightShin->getRigidBody()->getAngularVelocity().getZ());
	
	//rightFoot
	angularVelocities.push_back(rightFoot->getRigidBody()->getAngularVelocity().getX());
	angularVelocities.push_back(rightFoot->getRigidBody()->getAngularVelocity().getY());
	angularVelocities.push_back(rightFoot->getRigidBody()->getAngularVelocity().getZ());

	//leftThigh
	angularVelocities.push_back(leftThigh->getRigidBody()->getAngularVelocity().getX());
	angularVelocities.push_back(leftThigh->getRigidBody()->getAngularVelocity().getY());
	angularVelocities.push_back(leftThigh->getRigidBody()->getAngularVelocity().getZ());

	//leftShin
	angularVelocities.push_back(leftShin->getRigidBody()->getAngularVelocity().getX());
	angularVelocities.push_back(leftShin->getRigidBody()->getAngularVelocity().getY());
	angularVelocities.push_back(leftShin->getRigidBody()->getAngularVelocity().getZ());

	//leftFoot
	angularVelocities.push_back(leftFoot->getRigidBody()->getAngularVelocity().getX());
	angularVelocities.push_back(leftFoot->getRigidBody()->getAngularVelocity().getY());
	angularVelocities.push_back(leftFoot->getRigidBody()->getAngularVelocity().getZ());

	return angularVelocities;
}

void Creature::setAllTargetVelocities(std::vector<double>& resultVec)
{
	double m = 5;
	double mU = 0.0;
	////HIPS
	getHips()->getHinge("leftHip")->setMotorTargetVelocity((resultVec[0] - mU) * m);
	getHips()->getHinge("rightHip")->setMotorTargetVelocity((resultVec[1] - mU) * m);

	//KnEES
	getRightThigh()->getHinge("rightKnee")->setMotorTargetVelocity((resultVec[2] - mU) * m);
	getLeftThigh()->getHinge("leftKnee")->setMotorTargetVelocity((resultVec[3] - mU) * m);


	getRightThigh()->getHinge("rightKnee")->setMotorTargetVelocity((resultVec[0] - mU) * m);
	getLeftThigh()->getHinge("leftKnee")->setMotorTargetVelocity((resultVec[1] - mU) * m);

//	//FEETS
	getRightShin()->getHinge("rightAnkle")->setMotorTargetVelocity((resultVec[4] - mU) * m);
	getLeftShin()->getHinge("leftAnkle")->setMotorTargetVelocity((resultVec[5] - mU) * m);
}

void Creature::setMaxMotorImpulses(double maxMotorImpulse)
{
	bool isEnableMotor = true;
	//THEM HIPS
	getHips()->getHinge("leftHip")->enableMotor(isEnableMotor);
	getHips()->getHinge("rightHip")->enableMotor(isEnableMotor);
	//double mU = 0.0;
	getHips()->getHinge("leftHip")->setMaxMotorImpulse(maxMotorImpulse);
	getHips()->getHinge("rightHip")->setMaxMotorImpulse(maxMotorImpulse);


	//KnEES
	getRightThigh()->getHinge("rightKnee")->enableMotor(isEnableMotor);
	getLeftThigh()->getHinge("leftKnee")->enableMotor(isEnableMotor);
	getRightThigh()->getHinge("rightKnee")->setMaxMotorImpulse(maxMotorImpulse);
	getLeftThigh()->getHinge("leftKnee")->setMaxMotorImpulse(maxMotorImpulse);


	//FEETS
	getRightShin()->getHinge("rightAnkle")->enableMotor(isEnableMotor);
	getLeftShin()->getHinge("leftAnkle")->enableMotor(isEnableMotor);
	getRightShin()->getHinge("rightAnkle")->setMaxMotorImpulse(maxMotorImpulse);
	getLeftShin()->getHinge("leftAnkle")->setMaxMotorImpulse(maxMotorImpulse);
}

void Creature::createNeuralNetwork(std::vector<int> topology)
{
	m_neuralNetwork = new NeuralNetwork(topology);
}

void Creature::setNeuralNetwork(NeuralNetwork neuralNetwork)
{
}

void Creature::updateNeuralNetwork()
{
	std::vector<double> inputs = calculateInputs();
	//std::cout << inputs.size() << std::endl;
	m_neuralNetwork->forward(inputs);

	m_neuralNetwork->getResults(resultVec);
	setAllTargetVelocities(resultVec);
	//std::cout << resultVec[0] << "  " << resultVec[1]<< std::endl;
}

Cube* Creature::getChest()
{
	return chest;
}

Cube* Creature::getHips()
{
	return hips;
}

Cube* Creature::getRightThigh()
{
	return rightThigh;
}

Cube* Creature::getRightShin()
{
	return rightShin;
}

Cube* Creature::getRightFoot()
{
	return rightFoot;
}

Cube* Creature::getLeftThigh()
{
	return leftThigh;
}

Cube* Creature::getLeftShin()
{
	return leftShin;
}

Cube* Creature::getLeftFoot()
{
	return leftFoot;
}

Creature::~Creature()
{
	delete hips;
	delete rightThigh;
	delete rightShin;
	delete rightFoot;
	delete leftThigh;
	delete leftShin;
	delete leftFoot;
}
