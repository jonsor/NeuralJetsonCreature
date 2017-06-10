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
Creature::Creature(PhysicsManager* pm, glm::vec3 startPosition, std::default_random_engine &engine): m_startPosition(startPosition)
{
	//Limbs:
	hips = new Box(glm::vec3(m_startPosition.x, m_startPosition.y, m_startPosition.z), glm::vec3(0.9f, 0.1f, 0.1f), 1.5f, 1.5f, 0.4f, 20);
	//hips->getRigidBody()->setDamping(0, 0);
	//hips->getRigidBody()->setLinearFactor(btVector3(1,1,0));
	//hips->getRigidBody()->setAngularFactor(btVector3(1, 0, 0));
	rightThigh = new Box(glm::vec3(m_startPosition.x - 1.5, m_startPosition.y - 4, m_startPosition.z), glm::vec3(0.2f, 0.3f, 0.7f), 0.5f, 1.8f, 0.3f, 10);
	rightShin = new Box(glm::vec3(m_startPosition.x - 1.5, m_startPosition.y - 8, m_startPosition.z), glm::vec3(0.2f, 0.3f, 0.7f), 0.5f, 1.8f, 0.3f, 10);
	rightFoot = new Box(glm::vec3(m_startPosition.x - 1.5, m_startPosition.y - 10.2, m_startPosition.z + 0.3), glm::vec3(0.2f, 0.3f, 0.7f), 0.6f, 0.15f, 1.0f, 3);

	leftThigh = new Box(glm::vec3(m_startPosition.x + 1.5, m_startPosition.y - 4, m_startPosition.z), glm::vec3(0.2f, 0.3f, 0.7f), 0.5f, 1.8f, 0.3f, 10);
	leftShin = new Box(glm::vec3(m_startPosition.x + 1.5, m_startPosition.y - 8, m_startPosition.z), glm::vec3(0.2f, 0.3f, 0.7f), 0.5f, 1.8f, 0.3f, 10);
	leftFoot = new Box(glm::vec3(m_startPosition.x + 1.5, m_startPosition.y - 10.2, m_startPosition.z + 0.3), glm::vec3(0.2f, 0.3f, 0.7f), 0.6f, 0.15f, 1.0f, 3);

	//leftFoot->getRigidBody()->setFriction(4.f);
	//rightFoot->getRigidBody()->setFriction(4.f);

	//m_startPosition.x += hips->getWidth() / 2;
	//m_startPosition.y -= hips->getHeight() / 2;
	//m_startPosition.z += hips->getDepth() / 2;

	//m_startPosition.x += hips->getRigidBody()->getCenterOfMassPosition().getX();
	//m_startPosition.y -= hips->getRigidBody()->getCenterOfMassPosition().getY();
	//m_startPosition.z += hips->getRigidBody()->getCenterOfMassPosition().getZ();

	pm->addBody(hips->getRigidBody(),1, 2);

	pm->addBody(rightThigh->getRigidBody(),1,2);
	pm->addBody(rightShin->getRigidBody(),1,2);
	pm->addBody(rightFoot->getRigidBody(),1,2);

	pm->addBody(leftThigh->getRigidBody(),1,2);
	pm->addBody(leftShin->getRigidBody(),1,2);
	pm->addBody(leftFoot->getRigidBody(),1,2);

	calcCenterPosition();

	//Hinges:
	bool noCol = true;
	//chest->addHinge(glm::vec3(0.0f, -1.8f, 0.0f), glm::vec3(0.0f, 1.8f, 0.0f), glm::vec3(1.0f, 0.0f, 0.0f), glm::vec3(1.0f, 0.0f, 0.0f), hips, noCol, -PI/2, PI/2, pm, "abdomen");
	hips->addHinge(glm::vec3(0.0f, -1.5f, 0.0f), glm::vec3(1.0f, 1.5f, 0.0f), glm::vec3(1.0f, 0.0f, 0.0f), glm::vec3(1.0f, 0.0f, 0.0f), rightThigh, noCol, -1.5, 0.7, pm, "rightHip");
	hips->addHinge(glm::vec3(0.0f, -1.5f, 0.0f), glm::vec3(-1.0f, 1.5f, 0.0f), glm::vec3(1.0f, 0.0f, 0.0f), glm::vec3(1.0f, 0.0f, 0.0f), leftThigh, noCol, -1.5, 0.7, pm, "leftHip");

	rightThigh->addHinge(glm::vec3(0.0f, -1.9f, 0.0f), glm::vec3(0.0f, 1.9f, 0.0f), glm::vec3(1.0f, 0.0f, 0.0f), glm::vec3(1.0f, 0.0f, 0.0f), rightShin, noCol, -0.1, PI * 0.8, pm, "rightKnee");
	leftThigh->addHinge(glm::vec3(0.0f, -1.9f, 0.0f), glm::vec3(0.0f, 1.9f, 0.0f), glm::vec3(1.0f, 0.0f, 0.0f), glm::vec3(1.0f, 0.0f, 0.0f), leftShin, noCol, -0.1, PI * 0.8, pm, "leftKnee");

	rightShin->addHinge(glm::vec3(0.0f, -2.1f, 0.0f), glm::vec3(0.0f, 0.0f, 0.7f), glm::vec3(1.0f, 0.0f, 0.0f), glm::vec3(1.0f, 0.0f, 0.0f), rightFoot, noCol, -0.5, PI/4, pm, "rightAnkle");
	leftShin->addHinge(glm::vec3(0.0f, -2.1f, 0.0f), glm::vec3(0.0f, 0.0f, 0.7f), glm::vec3(1.0f, 0.0f, 0.0f), glm::vec3(1.0f, 0.0f, 0.0f), leftFoot, noCol, -0.5, PI/4, pm, "leftAnkle");

	setMaxMotorImpulses(20.0f);

	//Create the neural network
	std::vector<int> topology{ 29, 20, 20, 6 };
	createNeuralNetwork(topology, engine);

	//Set default fitness
	m_fitness = 0;

	maxMinAngles = { 1, 1, 1, 1, 1, 1 };

	memoryNeurons = { 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 };
	m_maxHeight = 0;

	previousRightFootHeight = getRightFoot()->getRigidBody()->getCenterOfMassPosition().getY();
	previousLeftFootHeight = getLeftFoot()->getRigidBody()->getCenterOfMassPosition().getY();
	numberOfSteps = 0;
	lastFootThatStepped = 'n';

	averageFeetStartPos = getAverageFeetPosition();

	resultVec = { 0, 0, 0, 0, 0, 0 };

	m_shouldUpdate = true;
	timeOnTwoLegs = 0;
	timeOnGround = 0;
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
static void updatePhysicsOfLimb(Box * box) {
	box->updatePhysics();
}
void Creature::updatePhysics()
{
	//if (yo >= 2000) {
	//	std::vector<int> topology{ 28, 20, 6 };
	//	createNeuralNetwork(topology);
	//	//std::cout << "NEEEEEEEEEEEEEEEEEEEEW NEEEEEEEEEEEEEEEEEEEEEEEEEEEEEET" << std::endl;
	//	yo = 0;
	//}
	//yo++;
	calcCenterPosition();

	updateNeuralNetwork();

	double steppingThreshold = 2.0;
	if (leftFootMovingDownward())
	{
		//TODO CHANGE TO GROUND COLLISION INSTEAD OF THRESHOLD
		if (previousLeftFootHeight > steppingThreshold &&
			getLeftFoot()->getRigidBody()->getCenterOfMassPosition().getY() < steppingThreshold &&
			lastFootThatStepped != 'l')
		{
			//foot just moved down past threshold
			numberOfSteps++;
			lastFootThatStepped = 'l';
		}
	}
	//check for right foot steps
	if (rightFootMovingDownward())
	{
		if (previousRightFootHeight > steppingThreshold &&
			getRightFoot()->getRigidBody()->getCenterOfMassPosition().getY()  < steppingThreshold &&
			lastFootThatStepped != 'r')
		{
			//foot just moved down past threshold
			numberOfSteps++;
			lastFootThatStepped = 'r';
		}
	}

	previousRightFootHeight = getRightFoot()->getRigidBody()->getCenterOfMassPosition().getY();
	previousLeftFootHeight = getLeftFoot()->getRigidBody()->getCenterOfMassPosition().getY();
	//chest->updatePhysics();
	//hips->getRigidBody()->setDamping(0, 0);
	//hips->getRigidBody()->setRestitution(0);
	//std::thread t1(updatePhysicsOfLimb, hips);

	//std::thread t2(updatePhysicsOfLimb, rightThigh);
	//std::thread t3(updatePhysicsOfLimb, rightShin);
	//std::thread t4(updatePhysicsOfLimb, rightFoot);

	//std::thread t5(updatePhysicsOfLimb, leftThigh);
	//std::thread t6(updatePhysicsOfLimb, leftShin);
	//std::thread t7(updatePhysicsOfLimb, leftFoot);

	//t1.join();
	//t2.join();
	//t3.join();
	//t4.join();
	//t5.join();
	//t6.join();
	//t7.join();

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
	//centerPosition.y -= hips->getHeight() / 2;
	//centerPosition.x += hips->getWidth() / 2;
	//centerPosition.z += hips->getDepth() / 2;

	//btVector3 centerMass = hips->getRigidBody()->getCenterOfMassPosition();
	//centerPosition = glm::vec3(centerMass.getX(), centerMass.getY(), centerMass.getZ());
	//std::cout << "centerMass: x: " << centerMass.getX() << " y: " << centerMass.getY() << " z: " << centerMass.getZ() << "\n";
	//std::cout << "centerPosition: x: " << centerPosition.x << " y: " << centerPosition.y << " z: " << centerPosition.z << "\n";
	//std::cout << centerPosition.z << std::endl;
}

glm::vec3 Creature::getPosition()
{
	return centerPosition;
}

glm::vec3 Creature::getStartPosition()
{
	return m_startPosition;
}

glm::vec3 Creature::getRelativePosition(Box* box) {
	glm::vec3 relativePosition;
	relativePosition.x = box->getPosition().x - hips->getPosition().x;
	relativePosition.y = box->getPosition().y - hips->getPosition().y;
	//THIS IS WRONG
	//std::cout << "box: " << box->getPosition().y <<"\n";
	//std::cout << "hips: " << hips->getPosition().y << "\n";
	//std::cout << "relative: " << relativePosition.y << "\n";
	relativePosition.z = box->getPosition().z - hips->getPosition().z;
	return relativePosition;
}

double Creature::get2DAngle(Box* Box1, Box* Box2) {
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
void Creature::getAllMaxMinAngles()
{

	//RIGHT HIP
	if (getHips()->getHinge("rightHip")->getHingeAngle() >= getHips()->getHinge("rightHip")->getUpperLimit() - 0.1) {
		maxMinAngles[0] = 1.0;
		getRightThigh()->setColor(glm::vec3(1.f, 1.f, 0.1f));
	}
	else if (getHips()->getHinge("rightHip")->getHingeAngle() <= getHips()->getHinge("rightHip")->getLowerLimit() + 0.1) {
		maxMinAngles[0] = -1.0;
		getRightThigh()->setColor(glm::vec3(0.f, 0.5f, 1.f));
	}

	//LEFT HIP
	if (getHips()->getHinge("leftHip")->getHingeAngle() >= getHips()->getHinge("leftHip")->getUpperLimit() - 0.1) {
		maxMinAngles[1] = 1.0;
		getLeftThigh()->setColor(glm::vec3(1.f, 1.f, 0.1f));
	}
	else if (getHips()->getHinge("leftHip")->getHingeAngle() <= getHips()->getHinge("leftHip")->getLowerLimit() + 0.1) {
		maxMinAngles[1] = -1.0;
		getLeftThigh()->setColor(glm::vec3(0.f, 0.5f, 1.f));
	}

	//RIGHT KNEE
	if (getRightThigh()->getHinge("rightKnee")->getHingeAngle() >= getRightThigh()->getHinge("rightKnee")->getUpperLimit() - 0.1) {
		maxMinAngles[2] = 1.0;
		getRightShin()->setColor(glm::vec3(1.f, 1.f, 0.1f));
	}
	else if (getRightThigh()->getHinge("rightKnee")->getHingeAngle() <= getRightThigh()->getHinge("rightKnee")->getLowerLimit() + 0.1) {
		maxMinAngles[2] = -1.0;
		getRightShin()->setColor(glm::vec3(0.f, 0.5f, 1.f));
	}

	//LEFT KNEE
	if (getLeftThigh()->getHinge("leftKnee")->getHingeAngle() >= getLeftThigh()->getHinge("leftKnee")->getUpperLimit() - 0.1) {
		maxMinAngles[3] = 1.0;
		getLeftShin()->setColor(glm::vec3(1.f, 1.f, 0.1f));
	}
	else if (getLeftThigh()->getHinge("leftKnee")->getHingeAngle() <= getLeftThigh()->getHinge("leftKnee")->getLowerLimit() + 0.1) {
		maxMinAngles[3] = -1.0;
		getLeftShin()->setColor(glm::vec3(0.f, 0.5f, 1.f));
	}

	//RIGHT ANKLE
	if (getRightShin()->getHinge("rightAnkle")->getHingeAngle() >= getRightShin()->getHinge("rightAnkle")->getUpperLimit() - 0.1) {
		maxMinAngles[4] = 1.0;
	}
	else if (getRightShin()->getHinge("rightAnkle")->getHingeAngle() <= getRightShin()->getHinge("rightAnkle")->getLowerLimit() + 0.1) {
		maxMinAngles[4] = -1.0;
	}

	//LEFT ANKLE
	if (getLeftShin()->getHinge("leftAnkle")->getHingeAngle() >= getLeftShin()->getHinge("leftAnkle")->getUpperLimit() - 0.1) {
		maxMinAngles[5] = 1.0;
	}
	else if (getLeftShin()->getHinge("leftAnkle")->getHingeAngle() <= getLeftShin()->getHinge("leftAnkle")->getLowerLimit() + 0.1) {
		maxMinAngles[5] = -1.0;
	}

}

std::vector<double> Creature::getAllAngles()
{

	//double rha = (getHips()->getHinge("rightHip")->getHingeAngle() - getHips()->getHinge("rightHip")->getLowerLimit()) / (getHips()->getHinge("rightHip")->getUpperLimit() - getHips()->getHinge("rightHip")->getLowerLimit());
	//double lha = (getHips()->getHinge("leftHip")->getHingeAngle() - getHips()->getHinge("leftHip")->getLowerLimit()) / (getHips()->getHinge("leftHip")->getUpperLimit() - getHips()->getHinge("leftHip")->getLowerLimit());

	//double rka = (getRightThigh()->getHinge("rightKnee")->getHingeAngle() - getRightThigh()->getHinge("rightKnee")->getLowerLimit()) / (getRightThigh()->getHinge("rightKnee")->getUpperLimit() - getRightThigh()->getHinge("rightKnee")->getLowerLimit());
	//double lka = (getLeftThigh()->getHinge("leftKnee")->getHingeAngle() - getLeftThigh()->getHinge("leftKnee")->getLowerLimit()) / (getLeftThigh()->getHinge("leftKnee")->getUpperLimit() - getLeftThigh()->getHinge("leftKnee")->getLowerLimit());

	//double raa = (getRightShin()->getHinge("rightAnkle")->getHingeAngle() - getRightShin()->getHinge("rightAnkle")->getLowerLimit()) / (getRightShin()->getHinge("rightAnkle")->getUpperLimit() - getRightShin()->getHinge("rightAnkle")->getLowerLimit());
	//double laa = (getLeftShin()->getHinge("leftAnkle")->getHingeAngle() - getLeftShin()->getHinge("leftAnkle")->getLowerLimit()) / (getLeftShin()->getHinge("leftAnkle")->getUpperLimit() - getLeftShin()->getHinge("leftAnkle")->getLowerLimit());
	//
	double rha = Util::normalize(getHips()->getHinge("rightHip")->getHingeAngle(), getHips()->getHinge("rightHip")->getLowerLimit(), getHips()->getHinge("rightHip")->getUpperLimit());
	double lha = Util::normalize(getHips()->getHinge("leftHip")->getHingeAngle(), getHips()->getHinge("leftHip")->getLowerLimit(), getHips()->getHinge("leftHip")->getUpperLimit());

	double rka = Util::normalize(getRightThigh()->getHinge("rightKnee")->getHingeAngle(), getRightThigh()->getHinge("rightKnee")->getLowerLimit(), getRightThigh()->getHinge("rightKnee")->getUpperLimit());
	double lka = Util::normalize(getLeftThigh()->getHinge("leftKnee")->getHingeAngle(), getLeftThigh()->getHinge("leftKnee")->getLowerLimit(), getLeftThigh()->getHinge("leftKnee")->getUpperLimit());

	double raa = Util::normalize(getRightShin()->getHinge("rightAnkle")->getHingeAngle(), getRightShin()->getHinge("rightAnkle")->getLowerLimit(), getRightShin()->getHinge("rightAnkle")->getUpperLimit());
	double laa = Util::normalize(getLeftShin()->getHinge("leftAnkle")->getHingeAngle(), getLeftShin()->getHinge("leftAnkle")->getLowerLimit(), getLeftShin()->getHinge("leftAnkle")->getUpperLimit());

	//rha = (rha < 0) ? 0.0 : rha;
	//lha = (lha < 0) ? 0.0 : lha;
	//rka = (rka < 0) ? 0.0 : rka;
	//lka = (lka < 0) ? 0.0 : lka;
	//raa = (raa < 0) ? 0.0 : raa;
	//laa = (laa < 0) ? 0.0 : laa;


	//std::cout << rha << lha << rka << lka << raa << laa << std::endl;
	return{ rha, lha, rka, lka, raa, laa };
	//std::cout << "NA: " << rka << " " << lka << "\n";
	//double time = sin(0.0002 * std::chrono::system_clock::now().time_since_epoch().count());
	//return{rka, lka};
}
int test2 = 0;
double tempHigh = 0;

int stepTime = 0;
std::vector<double> Creature::calculateInputs()
{
	std::vector<double> inputs;
	//Height
	//double height = Util::normalize(hips->getRigidBody()->getCenterOfMassPosition().getY(), 0, 10);
	double height = Util::normalize(hips->getPosition().y, 0, 10);
	inputs.push_back(height);

	//target velocities
	//TODO NORMALIZE
	std::vector<double> angularVel = getAllAngularVelocities();
	const double MAX = 10;
	for (int i = 0; i < angularVel.size(); i++) {
		angularVel[i] = Util::normalize(angularVel[i], -MAX, MAX);
	}
	//inputs.reserve(angularVel.size());
	//inputs.insert(inputs.end(), angularVel.begin(), angularVel.end());

	//for (int i = 0; i < inputs.size(); i++) {
	//	std::cout << inputs.at(i) << "  ";
	//}
	//std::cout << std::endl;
	inputs.push_back(getHips()->getRigidBody()->getOrientation().getX());
	inputs.push_back(getHips()->getRigidBody()->getOrientation().getY());
	inputs.push_back(getHips()->getRigidBody()->getOrientation().getZ());

	const double VEL_MAX = 8;
	inputs.push_back(Util::normalize(getHips()->getRigidBody()->getLinearVelocity().getX(), -VEL_MAX, VEL_MAX));
	inputs.push_back(Util::normalize(getHips()->getRigidBody()->getLinearVelocity().getY(), -VEL_MAX, VEL_MAX));
	inputs.push_back(Util::normalize(getHips()->getRigidBody()->getLinearVelocity().getZ(), -VEL_MAX, VEL_MAX));
		
		//Hinge angles
	std::vector<double> inputAngles = getAllAngles();
	//inputs.reserve(inputAngles.size());
	inputs.insert(inputs.end(), inputAngles.begin(), inputAngles.end());

	//MAX AND MIN BOOLS
	getAllMaxMinAngles();
	inputs.insert(inputs.end(), maxMinAngles.begin(), maxMinAngles.end());
	//Foot heights

	double rightFootOnGround = (getRightFoot()->isCollidingWithGround()) ? 1.0 : -1.0;
	double leftFootOnGround = (getLeftFoot()->isCollidingWithGround()) ? 1.0 : -1.0;
	
	//getRightFoot()->setCollidingWithGround((getRightFoot()->getRigidBody()->getCenterOfMassPosition().getY() < 0.5));
	//getLeftFoot()->setCollidingWithGround(getLeftFoot()->getRigidBody()->getCenterOfMassPosition().getY() < 0.5);

	//double rightFootOnGround = (getRightFoot()->getRigidBody()->getCenterOfMassPosition().getY() < 0.5) ? 1.0 : -1.0;
	//double leftFootOnGround = (getLeftFoot()->getRigidBody()->getCenterOfMassPosition().getY() < 0.5) ? 1.0 : -1.0;

	//std::cout << "rightCollides: " << rightFootOnGround << "\n";

	if (rightFootOnGround > 0) {
		getRightFoot()->setColor(glm::vec3(0.1, 1.0, 0.1));
	} else {
		getRightFoot()->setColor(glm::vec3(0.2f, 0.3f, 0.7f));
	}

	if (leftFootOnGround > 0) {
		getLeftFoot()->setColor(glm::vec3(0.1, 1.0, 0.1));
	} else {
		getLeftFoot()->setColor(glm::vec3(0.2f, 0.3f, 0.7f));
	}
	inputs.push_back(rightFootOnGround);
	inputs.push_back(leftFootOnGround);

	//Distance from feet to hips
	//inputs.push_back(Util::normalize(hips->getPosition().y - getRightFoot()->getPosition().y, 0, 8));
	//inputs.push_back(Util::normalize(hips->getPosition().y - getLeftFoot()->getPosition().y, 0, 8));

	inputs.push_back(Util::normalize(getDistanceFromHips(getRightFoot()), 2.5, 11));
	inputs.push_back(Util::normalize(getDistanceFromHips(getLeftFoot()), 2.5, 11));

	//std::cout << "distance: " << getDistanceFromHips(getRightFoot()) << "\n";
	//std::cout << "r: " << hips->getPosition().y - getRightFoot()->getPosition().y << " ";
	//std::cout << "l: " << hips->getPosition().y - getLeftFoot()->getPosition().y << "\n";
	//const double RELATIVEPOSMAXMIN = 10;
	//inputs.push_back(Util::normalize(getRelativePosition(getLeftFoot()).x, -RELATIVEPOSMAXMIN, RELATIVEPOSMAXMIN));
	//inputs.push_back(Util::normalize(getRelativePosition(getLeftFoot()).y, -RELATIVEPOSMAXMIN, RELATIVEPOSMAXMIN));
	//inputs.push_back(Util::normalize(getRelativePosition(getLeftFoot()).z, -RELATIVEPOSMAXMIN, RELATIVEPOSMAXMIN));

	//inputs.push_back(Util::normalize(getRelativePosition(getRightFoot()).x, -RELATIVEPOSMAXMIN, RELATIVEPOSMAXMIN));
	//inputs.push_back(Util::normalize(getRelativePosition(getRightFoot()).y, -RELATIVEPOSMAXMIN, RELATIVEPOSMAXMIN));
	//inputs.push_back(Util::normalize(getRelativePosition(getRightFoot()).z, -RELATIVEPOSMAXMIN, RELATIVEPOSMAXMIN));

	//INSERT MEMORY NEURONS

	//inputs.insert(inputs.end(), memoryNeurons.begin(), memoryNeurons.end());
	// 1 og -1 for max min angles
	//Lage minnenevroner
	//Lage flere competing outputs
	inputs.insert(inputs.end(), resultVec.begin(), resultVec.end());
	//Max: x 8.5 y 7.1 z 8.8
	//Min: x -5.8 y -9.5 z -9.5
	//stepTime++;
	//inputs.push_back(sin((double)(stepTime) / 16));

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
	//CHANGE MAX VELOCITY NORMALIZATION IF YOU CHANGE m
	double m = 8;
	double mU = 0.0;

	//HIPS
	getHips()->getHinge("leftHip")->setMotorTargetVelocity((resultVec[0] - mU) * m);
	getHips()->getHinge("rightHip")->setMotorTargetVelocity((resultVec[1] - mU) * m);

	//KNEES
	getRightThigh()->getHinge("rightKnee")->setMotorTargetVelocity((resultVec[2] - mU) * m);
	getLeftThigh()->getHinge("leftKnee")->setMotorTargetVelocity((resultVec[3] - mU) * m);

	//FEETS
	getRightShin()->getHinge("rightAnkle")->setMotorTargetVelocity((resultVec[4] - mU) * m);
	getLeftShin()->getHinge("leftAnkle")->setMotorTargetVelocity((resultVec[5] - mU) * m);

	//for (int i = 0; i < memoryNeurons.size(); i++) {
	//	memoryNeurons[i] = resultVec[i + 6];
	//}
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
	getRightShin()->getHinge("rightAnkle")->setMaxMotorImpulse(maxMotorImpulse/2);
	getLeftShin()->getHinge("leftAnkle")->setMaxMotorImpulse(maxMotorImpulse/2);
}

void Creature::createNeuralNetwork(std::vector<int> topology, std::default_random_engine &engine)
{
	m_neuralNetwork = NeuralNetwork(topology, engine);
}

void Creature::setNeuralNetwork(NeuralNetwork neuralNetwork)
{
	m_neuralNetwork = neuralNetwork;
}

NeuralNetwork Creature::getNeuralNetwork()
{
	//Creature* tempNN = new Creature(*m_neuralNetwork);
	return m_neuralNetwork;
}

NeuralNetwork* Creature::getNN()
{
	//Creature* tempNN = new Creature(*m_neuralNetwork);
	return &m_neuralNetwork;
}

void Creature::updateNeuralNetwork()
{
	std::vector<double> inputs = calculateInputs();
	m_neuralNetwork.forward(inputs);

	m_neuralNetwork.getResults(resultVec);


	setAllTargetVelocities(resultVec);
}

void Creature::mutate(double mutationRate, double mutationChance,  std::default_random_engine engine)
{
	m_neuralNetwork.mutate(mutationRate, mutationChance, engine);
}

void Creature::reset()
{
	hips->reset();
	rightThigh->reset();
	rightShin->reset();
	rightFoot->reset();

	leftThigh->reset();
	leftShin->reset();
	leftFoot->reset();

	std::vector<double> resetVals = {0,0,0,0,0,0};
	setAllTargetVelocities(resetVals);

}

void Creature::setFitness(double fitness)
{
	m_fitness = fitness;
}

double Creature::getFitness()
{
	return m_fitness;
}

Box* Creature::getHips()
{
	return hips;
}

Box* Creature::getRightThigh()
{
	return rightThigh;
}

Box* Creature::getRightShin()
{
	return rightShin;
}

Box* Creature::getRightFoot()
{
	return rightFoot;
}

Box* Creature::getLeftThigh()
{
	return leftThigh;
}

Box* Creature::getLeftShin()
{
	return leftShin;
}

Box* Creature::getLeftFoot()
{
	return leftFoot;
}

double Creature::getHeight()
{
	return hips->getPosition().y;
}

void Creature::removeBodies(PhysicsManager * pm)
{

	hips->remove(pm);

	rightThigh->remove(pm);
	rightShin->remove(pm);
	rightFoot->remove(pm);

	leftThigh->remove(pm);
	leftShin->remove(pm);
	leftFoot->remove(pm);
	//delete hips;
	//std::cout << std::endl;

}


void Creature::removeConstraints(PhysicsManager * pm)
{

	hips->removeConstraint(pm);

	rightThigh->removeConstraint(pm);

	rightShin->removeConstraint(pm);
	rightFoot->removeConstraint(pm);

	leftThigh->removeConstraint(pm);
	leftShin->removeConstraint(pm);
	leftFoot->removeConstraint(pm);
	//delete hips;
	//std::cout << std::endl;

}

void Creature::setColor(glm::vec3 color)
{
	//hips->setColor(color);

	rightThigh->setColor(color);
	rightShin->setColor(color);
	rightFoot->setColor(color);
	leftThigh->setColor(color);
	leftShin->setColor(color);
	leftFoot->setColor(color);
}

void Creature::incrementToAverage()
{

	double value = getHeight();

	average = (avgSize * average + value) / (avgSize + 1);

	avgSize++;
}

double Creature::getAverageHeight() {
	return average;
}

double Creature::getMaxHeight()
{
	return m_maxHeight;
}

void Creature::updateMaxHeight(double height)
{

	if (height > m_maxHeight) m_maxHeight = height;
}

double Creature::getTimeOnGround()
{
	return timeOnGround;
}

void Creature::setTimeOnGround(double time)
{
	timeOnGround = time;
}

double Creature::getTimeOnTwoLegs()
{
	return timeOnTwoLegs;
}

void Creature::setTimeOnTwoLegs(double time)
{
	timeOnTwoLegs = time;
}

void Creature::checkIfLegsCrossed()
{
	//double rha = (getHips()->getHinge("rightHip")->getHingeAngle() - getHips()->getHinge("rightHip")->getLowerLimit()) / (getHips()->getHinge("rightHip")->getUpperLimit() - getHips()->getHinge("rightHip")->getLowerLimit());
	//double lha = (getHips()->getHinge("leftHip")->getHingeAngle() - getHips()->getHinge("leftHip")->getLowerLimit()) / (getHips()->getHinge("leftHip")->getUpperLimit() - getHips()->getHinge("leftHip")->getLowerLimit());

	//if (notCrossed & rha > lha) {
	//	numTimesLegsCrossed++;
	//	notCrossed = false;
	//	//std::cout << "crossed: " << numTimesLegsCrossed << " " << rha << " " << lha << std::endl;
	//}
	//else if (rha  < lha + 0.2) {
	//	notCrossed = true;
	//	//std::cout << "crossed: " << numTimesLegsCrossed << " " << rha << " " << lha << std::endl;
	//}

	double max = getHips()->getHinge("rightHip")->getUpperLimit();
	double min = getHips()->getHinge("rightHip")->getLowerLimit();
	//std::cout << max << "  " << min << "\n";
	//std::cout << getHips()->getHinge("rightHip")->getHingeAngle() << "\n";
	if (!reachedUpperR && getHips()->getHinge("rightHip")->getHingeAngle() >= max - 0.15) {
		numTimesLegsCrossed++;
		reachedUpperR = true;
	}
	else if (reachedUpperR && getHips()->getHinge("rightHip")->getHingeAngle() <= min + 0.15) {
		reachedUpperR = false;
		numTimesLegsCrossed++;
	}


	if (!reachedUpperL && getHips()->getHinge("leftHip")->getHingeAngle() >= max - 0.15) {
		numTimesLegsCrossed++;
		reachedUpperL = true;
	}
	else if (reachedUpperL && getHips()->getHinge("leftHip")->getHingeAngle() <= min + 0.15) {
		reachedUpperL = false;
		numTimesLegsCrossed++;
	}

}

double Creature::getNumTimesCrossed() {
	return numTimesLegsCrossed;
}


bool Creature::leftFootMovingDownward()
{
	if (getLeftFoot()->getRigidBody()->getLinearVelocity().getY() < 0)
	{
		return true;
	}
	else
	{
		return false;
	}
}

bool Creature::rightFootMovingDownward()
{
	if (getRightFoot()->getRigidBody()->getLinearVelocity().getY() < 0)
	{
		return true;
	}
	else
	{
		return false;
	}
}
int Creature::getNumerOfSteps()
{
	return numberOfSteps;
}
double Creature::getAverageFeetPosition()
{
	return (getLeftFoot()->getPosition().z + getRightFoot()->getPosition().z)/2;
}
double Creature::getAverageFeetStartPos()
{
	return averageFeetStartPos;
}
double Creature::getDistanceFromHips(Box * box)
{
	glm::vec3 end = box->getPosition();
	glm::vec3 start = hips->getPosition();
	return glm::distance(end, start);
}
void Creature::setShouldUpdate(bool update)
{
	m_shouldUpdate = update;
}
bool Creature::shouldUpdate()
{
	return m_shouldUpdate;
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
