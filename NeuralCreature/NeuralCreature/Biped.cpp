/**
Biped.cpp
Purpose: Sets up, renders and updates a complete, hardcoded biped.

@author Sjur Barndon, Jonas Sørsdal
@version 1.0 23.03.2017
*/

#include "stdafx.h"
#include "Biped.h"
#include <chrono>

/**
	Constructor for the biped. Sets up a complete hardcoded 
	biped (a hip and two legs with thighs, shins and feet).
	Also links them together with mobile joints (hinges).
	TODO: Create this class customizable, not hardcoded.

	@param pm pointer to the PhysicsManager class so that the limbs and joints can be added to the world simulation.
*/
Biped::Biped(PhysicsManager* pm, glm::vec3 startPosition, std::default_random_engine &engine): m_startPosition(startPosition)
{
	//Limbs:
	hips = new Box(glm::vec3(m_startPosition.x, m_startPosition.y, m_startPosition.z), glm::vec3(0.9f, 0.1f, 0.1f), 1.5f, 2.0f, 0.4f, 35);

	rightThigh = new Box(glm::vec3(m_startPosition.x - 1.5, m_startPosition.y - 4, m_startPosition.z), glm::vec3(0.2f, 0.3f, 0.7f), 0.5f, 1.8f, 0.3f, 15);
	rightShin = new Box(glm::vec3(m_startPosition.x - 1.5, m_startPosition.y - 8, m_startPosition.z), glm::vec3(0.2f, 0.3f, 0.7f), 0.5f, 1.8f, 0.3f, 15);
	rightFoot = new Box(glm::vec3(m_startPosition.x - 1.5, m_startPosition.y - 10.2, m_startPosition.z + 0.3), glm::vec3(0.2f, 0.3f, 0.7f), 0.6f, 0.15f, 1.0f, 5);

	leftThigh = new Box(glm::vec3(m_startPosition.x + 1.5, m_startPosition.y - 4, m_startPosition.z), glm::vec3(0.2f, 0.3f, 0.7f), 0.5f, 1.8f, 0.3f, 15);
	leftShin = new Box(glm::vec3(m_startPosition.x + 1.5, m_startPosition.y - 8, m_startPosition.z), glm::vec3(0.2f, 0.3f, 0.7f), 0.5f, 1.8f, 0.3f, 15);
	leftFoot = new Box(glm::vec3(m_startPosition.x + 1.5, m_startPosition.y - 10.2, m_startPosition.z + 0.3), glm::vec3(0.2f, 0.3f, 0.7f), 0.6f, 0.15f, 1.0f, 5);

	//hips->getRigidBody()->setLinearFactor(btVector3(0, 0, 0));
	//hips->getRigidBody()->setAngularFactor(btVector3(0, 0, 0));
	//rightFoot->getRigidBody()->setFriction(5.0f);
	//leftFoot->getRigidBody()->setFriction(5.0f);
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
	//hips->addHinge(glm::vec3(0.0f, -1.5f, 0.0f), glm::vec3(1.0f, 1.5f, 0.0f), glm::vec3(1.0f, 0.0f, 0.0f), glm::vec3(1.0f, 0.0f, 0.0f), rightThigh, noCol, -1.5, 0.7, pm, "rightHip");
	//hips->addHinge(glm::vec3(0.0f, -1.5f, 0.0f), glm::vec3(-1.0f, 1.5f, 0.0f), glm::vec3(1.0f, 0.0f, 0.0f), glm::vec3(1.0f, 0.0f, 0.0f), leftThigh, noCol, -1.5, 0.7, pm, "leftHip");

	hips->addDOFConstraint(rightThigh, noCol, -1.7, pm, "rightHip");
	hips->addDOFConstraint(leftThigh, noCol, 1.7, pm, "leftHip");

	rightThigh->addHinge(glm::vec3(0.0f, -1.9f, 0.0f), glm::vec3(0.0f, 1.9f, 0.0f), glm::vec3(1.0f, 0.0f, 0.0f), glm::vec3(1.0f, 0.0f, 0.0f), rightShin, noCol, -0.1, PI * 0.8, pm, "rightKnee");
	leftThigh->addHinge(glm::vec3(0.0f, -1.9f, 0.0f), glm::vec3(0.0f, 1.9f, 0.0f), glm::vec3(1.0f, 0.0f, 0.0f), glm::vec3(1.0f, 0.0f, 0.0f), leftShin, noCol, -0.1, PI * 0.8, pm, "leftKnee");

	rightShin->addHinge(glm::vec3(0.0f, -2.1f, 0.0f), glm::vec3(0.0f, 0.0f, 0.7f), glm::vec3(1.0f, 0.0f, 0.0f), glm::vec3(1.0f, 0.0f, 0.0f), rightFoot, noCol, -0.5, PI/4, pm, "rightAnkle");
	leftShin->addHinge(glm::vec3(0.0f, -2.1f, 0.0f), glm::vec3(0.0f, 0.0f, 0.7f), glm::vec3(1.0f, 0.0f, 0.0f), glm::vec3(1.0f, 0.0f, 0.0f), leftFoot, noCol, -0.5, PI/4, pm, "leftAnkle");

	setMaxMotorImpulses(100.0f);

	//Create the neural network
	std::vector<int> topology{ 42, 16, 10 };
	createNeuralNetwork(topology, engine);

	//Set default fitness
	m_fitness = 0;

	int maxMinAnglesSize = 20;
	for (int i = 0; i < maxMinAnglesSize; i++) {
		maxMinAngles.push_back(0);
	}
	memoryNeurons = { 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 };
	m_maxHeight = 0;

	previousRightFootHeight = getRightFoot()->getRigidBody()->getCenterOfMassPosition().getY();
	previousLeftFootHeight = getLeftFoot()->getRigidBody()->getCenterOfMassPosition().getY();
	numberOfSteps = 0;
	lastFootThatStepped = 'n';

	averageFeetStartPos = getAverageFeetPosition();

	resultVec = { 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 };

	m_shouldUpdate = true;
	timeOnTwoLegs = 0;
	timeOnGround = 0;

	m_previousPosition = getPosition();
	m_startPosition = getPosition();
}

/**
	Calls the render function for each limb of the biped.

	@param shader takes the shader for rendering puposes.
*/
void Biped::render(Shader shader)
{
	hips->render(shader);

	rightThigh->render(shader);
	rightShin->render(shader);
	rightFoot->render(shader);

	leftThigh->render(shader);
	leftShin->render(shader);
	leftFoot->render(shader);
}

/**
	Updates the physics for each limb in the biped. 
	Also sets dampening and restitution?
*/
static void updatePhysicsOfLimb(Box * box) {
	box->updatePhysics();
}
void Biped::updatePhysics()
{
	timeAlive++;
	calcCenterPosition();

	updateNeuralNetwork();

	//hips->getRigidBody()->applyCentralImpulse(btVector3(btScalar(0.), btScalar(10.), btScalar(0.)));
	//rightThigh->getRigidBody()->applyTorque(btVector3(btScalar(0.), btScalar(0.), btScalar(2.)));

	double steppingThreshold = 3.0f;
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

	hips->updatePhysics();
	rightThigh->updatePhysics();
	rightShin->updatePhysics();
	rightFoot->updatePhysics();

	leftThigh->updatePhysics();
	leftShin->updatePhysics();
	leftFoot->updatePhysics();


}
//TODO: move this
//int test2 = 0;
//double tempHigh = 0;
//int stepTime = 0;

void Biped::updateNeuralNetwork()
{
	std::vector<double> inputs = calculateInputs();
	m_neuralNetwork.forward(inputs);
	m_neuralNetwork.getResults(resultVec);
	getHips()->getRigidBody()->applyTorque(btVector3(0, 40, 0));
	//getHips()->getdofConstraint("rightHip")->getRotationalLimitMotor(2)->m_targetVelocity = 1.f;
	//getHips()->getdofConstraint("leftHip")->getRotationalLimitMotor(2)->m_targetVelocity = 1.f;
	//getRightThigh()->getHinge("rightKnee")->setMotorTargetVelocity(-1.f);
	//getLeftThigh()->getHinge("leftKnee")->setMotorTargetVelocity(-1.f);

	//getRightShin()->getHinge("rightAnkle")->setMotorTargetVelocity(-1.f);
	//getLeftShin()->getHinge("leftAnkle")->setMotorTargetVelocity(-1.f);

	setAllTargetVelocities(resultVec);
}

std::vector<double> Biped::calculateInputs()
{
	std::vector<double> inputs;
	//Height
	double height = Util::normalize(hips->getPosition().y, 0, 10);
	inputs.push_back(height);

	//target velocities size: 21
	//TODO NORMALIZE
	std::vector<double> angularVel = getAllAngularVelocities();
	const double MAX = 8;
	for (int i = 0; i < angularVel.size(); i++) {
		angularVel[i] = Util::normalize(angularVel[i], -MAX, MAX);
	}

	inputs.insert(inputs.end(), angularVel.begin(), angularVel.end());


	//std::cout << "X: " << getHips()->getRigidBody()->getOrientation().getX() <<
	//	"Y: " << getHips()->getRigidBody()->getOrientation().getY() << " Z: " << getHips()->getRigidBody()->getOrientation().getZ() <<
	//	" W: " << getHips()->getRigidBody()->getOrientation().getW() << "\n";

	btMatrix3x3 m = btMatrix3x3(getHips()->getRigidBody()->getOrientation());
	btScalar yaw, pitch, roll;
	//m.getEulerZYX(yaw, pitch, roll);
	btQuaternion q = getHips()->getRigidBody()->getOrientation();
	//quaternionToEuler(q, yaw, pitch, roll);
	//yaw = Util::normalize(yaw, -PI, PI);
	//pitch = Util::normalize(pitch, -PI, PI);
	//roll = Util::normalize(roll, -PI, PI);

	getHips()->getRigidBody()->getCenterOfMassTransform().getBasis().getEulerZYX(yaw, pitch, roll, 1);
	//std::cout << "yaw: " << yaw << " pitch: " << pitch << " roll: " << roll << "\n";
	//inputs.push_back(yaw);
	//inputs.push_back(pitch);
	//inputs.push_back(roll);

	inputs.push_back(getHips()->getRigidBody()->getOrientation().getX());
	inputs.push_back(getHips()->getRigidBody()->getOrientation().getY());
	inputs.push_back(getHips()->getRigidBody()->getOrientation().getZ());
	inputs.push_back(getHips()->getRigidBody()->getOrientation().getW());

	const double VEL_MAX = 8;
	inputs.push_back(Util::normalize(getHips()->getRigidBody()->getLinearVelocity().getX(), -VEL_MAX, VEL_MAX));
	inputs.push_back(Util::normalize(getHips()->getRigidBody()->getLinearVelocity().getY(), -VEL_MAX, VEL_MAX));
	inputs.push_back(Util::normalize(getHips()->getRigidBody()->getLinearVelocity().getZ(), -16, 16));
		
	//Hinge angles
	std::vector<double> inputAngles = getAllAngles();
	inputs.insert(inputs.end(), inputAngles.begin(), inputAngles.end());

	//MAX AND MIN BOOLS
	//getAllMaxMinAngles2();
	//inputs.insert(inputs.end(), maxMinAngles.begin(), maxMinAngles.end());
	
	//Foot heights
	double rightFootOnGround = (getRightFoot()->isCollidingWithGround()) ? 1.0 : -1.0;
	double leftFootOnGround = (getLeftFoot()->isCollidingWithGround()) ? 1.0 : -1.0;

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

	inputs.push_back(Util::normalize(getDistanceFromHips(getRightFoot()), 2.5, 11));
	inputs.push_back(Util::normalize(getDistanceFromHips(getLeftFoot()), 2.5, 11));


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
	//inputs.insert(inputs.end(), resultVec.begin(), resultVec.end());
	//Max: x 8.5 y 7.1 z 8.8
	//Min: x -5.8 y -9.5 z -9.5
	//stepTime++;
	//inputs.push_back(sin((double)(stepTime) / 16));


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

	return inputs;
}

void Biped::quaternionToEuler(btQuaternion & quat, double & rotx, double & roty, double & rotz)
{
	double sqw;
	double sqx;
	double sqy;
	double sqz;

	//double rotxrad;
	//double rotyrad;
	//double rotzrad;
	sqw = quat.getW() * quat.getW();
	sqx = quat.getX() * quat.getX();
	sqy = quat.getY()* quat.getY();
	sqz = quat.getZ() * quat.getZ();

	rotx = (double)atan2l(2.0 * (quat.getY() * quat.getZ() + quat.getX() * quat.getW()), (-sqx - sqy + sqz + sqw));
	roty = (double)asinl(-2.0 * (quat.getX() * quat.getZ() - quat.getY() * quat.getW()));
	rotz = (double)atan2l(2.0 * (quat.getX() * quat.getY() + quat.getZ() * quat.getW()), (sqx - sqy - sqz + sqw));

}
void Biped::createNeuralNetwork(std::vector<int> topology, std::default_random_engine &engine)
{
	m_neuralNetwork = RecurrentNeuralNetwork(topology, engine);
}

void Biped::setNeuralNetwork(RecurrentNeuralNetwork neuralNetwork)
{
	m_neuralNetwork = neuralNetwork;
}

RecurrentNeuralNetwork Biped::getNeuralNetwork()
{
	return m_neuralNetwork;
}

RecurrentNeuralNetwork* Biped::getNN()
{
	return &m_neuralNetwork;
}

void Biped::mutate(double mutationRate, double mutationChance,  std::default_random_engine engine)
{
	m_neuralNetwork.mutate(mutationRate, mutationChance, engine);
}

void Biped::reset()
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

void Biped::getAllMaxMinAngles()
{
	//TODO Split up for every maximum and minimum
	////RIGHT HIP
	//if (getHips()->getHinge("rightHip")->getHingeAngle() >= getHips()->getHinge("rightHip")->getUpperLimit() - 0.1) {
	//	maxMinAngles[0] = 1.0;
	//	getRightThigh()->setColor(glm::vec3(1.f, 1.f, 0.1f));
	//}
	//else if (getHips()->getHinge("rightHip")->getHingeAngle() <= getHips()->getHinge("rightHip")->getLowerLimit() + 0.1) {
	//	maxMinAngles[0] = -1.0;
	//	getRightThigh()->setColor(glm::vec3(0.f, 0.5f, 1.f));
	//}
	maxMinAngles[0] = 1.0;
	////LEFT HIP
	//if (getHips()->getHinge("leftHip")->getHingeAngle() >= getHips()->getHinge("leftHip")->getUpperLimit() - 0.1) {
	//	maxMinAngles[1] = 1.0;
	//	getLeftThigh()->setColor(glm::vec3(1.f, 1.f, 0.1f));
	//}
	//else if (getHips()->getHinge("leftHip")->getHingeAngle() <= getHips()->getHinge("leftHip")->getLowerLimit() + 0.1) {
	//	maxMinAngles[1] = -1.0;
	//	getLeftThigh()->setColor(glm::vec3(0.f, 0.5f, 1.f));
	//}
	maxMinAngles[1] = 1.0;
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

void Biped::getAllMaxMinAngles2()
{

	for (int i = 0; i < maxMinAngles.size(); i++) {
		maxMinAngles[i] = 0;
	}

	//RIGHT HIP
	btVector3 rHAngularUpper;
	getHips()->getdofConstraint("rightHip")->getAngularUpperLimit(rHAngularUpper);

	btVector3 rHAngularLower;
	getHips()->getdofConstraint("rightHip")->getAngularLowerLimit(rHAngularLower);
	getRightThigh()->setColor(glm::vec3(0.2f, 0.3f, 0.7f));

	if (getHips()->getdofConstraint("rightHip")->getAngle(0) >= rHAngularUpper.getX() - 0.1) {
		maxMinAngles[0] = 1.0;
		getRightThigh()->setColor(glm::vec3(1.f, 0.1f, 0.1f));
	}
	if (getHips()->getdofConstraint("rightHip")->getAngle(1) >= rHAngularUpper.getY() - 0.1) {
		maxMinAngles[1] = 1.0;
		getRightThigh()->setColor(glm::vec3(0.1f, 1.f, 0.1f));
	}
	if (getHips()->getdofConstraint("rightHip")->getAngle(2) >= rHAngularUpper.getZ() - 0.1) {
		maxMinAngles[2] = 1.0;
		getRightThigh()->setColor(glm::vec3(1.0f, 1.0f, 1.0f));
	}

	if (getHips()->getdofConstraint("rightHip")->getAngle(0) <= rHAngularLower.getX() + 0.1) {
		maxMinAngles[3] = 1.0;
		getRightThigh()->setColor(glm::vec3(0.5f, 0.1f, 0.1f));
	}
	if (getHips()->getdofConstraint("rightHip")->getAngle(1) <= rHAngularLower.getY() + 0.1) {
		maxMinAngles[4] = 1.0;
		getRightThigh()->setColor(glm::vec3(0.1f, 0.5f, 0.1f));
	}
	if (getHips()->getdofConstraint("rightHip")->getAngle(2) <= rHAngularLower.getZ() + 0.1) {
		maxMinAngles[5] = 1.0;
		getRightThigh()->setColor(glm::vec3(0.2f, 0.2f, 0.2f));
	}

	////LEFT HIP
	getLeftThigh()->setColor(glm::vec3(0.2f, 0.3f, 0.7f));
	btVector3 lHAngularUpper;
	getHips()->getdofConstraint("leftHip")->getAngularUpperLimit(lHAngularUpper);

	btVector3 lHAngularLower;
	getHips()->getdofConstraint("leftHip")->getAngularLowerLimit(lHAngularLower);
	getLeftThigh()->setColor(glm::vec3(0.2f, 0.3f, 0.7f));

	if (getHips()->getdofConstraint("leftHip")->getAngle(0) >= lHAngularUpper.getX() - 0.1) {
		maxMinAngles[6] = 1.0;
		getLeftThigh()->setColor(glm::vec3(1.f, 0.1f, 0.1f));
	}
	if (getHips()->getdofConstraint("leftHip")->getAngle(1) >= lHAngularUpper.getY() - 0.1) {
		maxMinAngles[7] = 1.0;
		getLeftThigh()->setColor(glm::vec3(0.1f, 1.f, 0.1f));
	}
	if (getHips()->getdofConstraint("leftHip")->getAngle(2) >= lHAngularUpper.getZ() - 0.1) {
		maxMinAngles[8] = 1.0;
		getLeftThigh()->setColor(glm::vec3(1.0f, 1.0f, 1.0f));
	}

	if (getHips()->getdofConstraint("leftHip")->getAngle(0) <= lHAngularLower.getX() + 0.1) {
		maxMinAngles[9] = 1.0;
		getLeftThigh()->setColor(glm::vec3(0.5f, 0.1f, 0.1f));
	}
	if (getHips()->getdofConstraint("leftHip")->getAngle(1) <= lHAngularLower.getY() + 0.1) {
		maxMinAngles[10] = 1.0;
		getLeftThigh()->setColor(glm::vec3(0.1f, 0.5f, 0.1f));
	}
	if (getHips()->getdofConstraint("leftHip")->getAngle(2) <= lHAngularLower.getZ() + 0.1) {
		maxMinAngles[11] = 1.0;
		getLeftThigh()->setColor(glm::vec3(0.2f, 0.2f, 0.2f));
	}

	//RIGHT KNEE
	if (getRightThigh()->getHinge("rightKnee")->getHingeAngle() >= getRightThigh()->getHinge("rightKnee")->getUpperLimit() - 0.1) {
		maxMinAngles[12] = 1.0;
		getRightShin()->setColor(glm::vec3(1.f, 1.f, 0.1f));
	}
	if (getRightThigh()->getHinge("rightKnee")->getHingeAngle() <= getRightThigh()->getHinge("rightKnee")->getLowerLimit() + 0.1) {
		maxMinAngles[13] = 1.0;
		getRightShin()->setColor(glm::vec3(0.f, 0.5f, 1.f));
	}

	//LEFT KNEE
	if (getLeftThigh()->getHinge("leftKnee")->getHingeAngle() >= getLeftThigh()->getHinge("leftKnee")->getUpperLimit() - 0.1) {
		maxMinAngles[14] = 1.0;
		getLeftShin()->setColor(glm::vec3(1.f, 1.f, 0.1f));
	}

	if (getLeftThigh()->getHinge("leftKnee")->getHingeAngle() <= getLeftThigh()->getHinge("leftKnee")->getLowerLimit() + 0.1) {
		maxMinAngles[15] = 1.0;
		getLeftShin()->setColor(glm::vec3(0.f, 0.5f, 1.f));
	}

	//RIGHT ANKLE
	if (getRightShin()->getHinge("rightAnkle")->getHingeAngle() >= getRightShin()->getHinge("rightAnkle")->getUpperLimit() - 0.1) {
		maxMinAngles[16] = 1.0;
	}
	if (getRightShin()->getHinge("rightAnkle")->getHingeAngle() <= getRightShin()->getHinge("rightAnkle")->getLowerLimit() + 0.1) {
		maxMinAngles[17] = 1.0;
	}

	//LEFT ANKLE
	if (getLeftShin()->getHinge("leftAnkle")->getHingeAngle() >= getLeftShin()->getHinge("leftAnkle")->getUpperLimit() - 0.1) {
		maxMinAngles[18] = 1.0;
	}
	if (getLeftShin()->getHinge("leftAnkle")->getHingeAngle() <= getLeftShin()->getHinge("leftAnkle")->getLowerLimit() + 0.1) {
		maxMinAngles[19] = 1.0;
	}

}

std::vector<double> Biped::getAllAngles()
{
	//double rha = Util::normalize(getHips()->getHinge("rightHip")->getHingeAngle(), getHips()->getHinge("rightHip")->getLowerLimit(), getHips()->getHinge("rightHip")->getUpperLimit());
	//double lha = Util::normalize(getHips()->getHinge("leftHip")->getHingeAngle(), getHips()->getHinge("leftHip")->getLowerLimit(), getHips()->getHinge("leftHip")->getUpperLimit());

	//RIGHT HIP
	btVector3 anglularLowerLimitsR;
	getHips()->getdofConstraint("rightHip")->getAngularLowerLimit(anglularLowerLimitsR);

	btVector3 anglularUpperLimitsR;
	getHips()->getdofConstraint("rightHip")->getAngularUpperLimit(anglularUpperLimitsR);

	double rha1 = Util::normalize(getHips()->getdofConstraint("rightHip")->getAngle(0), anglularLowerLimitsR[0], anglularUpperLimitsR[0]);
	double rha2 = Util::normalize(getHips()->getdofConstraint("rightHip")->getAngle(1), anglularLowerLimitsR[1], anglularUpperLimitsR[1]);
	double rha3 = Util::normalize(getHips()->getdofConstraint("rightHip")->getAngle(2), anglularLowerLimitsR[2], anglularUpperLimitsR[2]);

	//LEFT HIP
	btVector3 anglularLowerLimitsL;
	getHips()->getdofConstraint("rightHip")->getAngularLowerLimit(anglularLowerLimitsL);

	btVector3 anglularUpperLimitsL;
	getHips()->getdofConstraint("rightHip")->getAngularUpperLimit(anglularUpperLimitsL);

	double lha1 = Util::normalize(getHips()->getdofConstraint("rightHip")->getAngle(0), anglularLowerLimitsL[0], anglularUpperLimitsL[0]);
	double lha2 = Util::normalize(getHips()->getdofConstraint("rightHip")->getAngle(1), anglularLowerLimitsL[1], anglularUpperLimitsL[1]);
	double lha3 = Util::normalize(getHips()->getdofConstraint("rightHip")->getAngle(2), anglularLowerLimitsL[2], anglularUpperLimitsL[2]);

	//KNEES
	double rka = Util::normalize(getRightThigh()->getHinge("rightKnee")->getHingeAngle(), getRightThigh()->getHinge("rightKnee")->getLowerLimit(), getRightThigh()->getHinge("rightKnee")->getUpperLimit());
	double lka = Util::normalize(getLeftThigh()->getHinge("leftKnee")->getHingeAngle(), getLeftThigh()->getHinge("leftKnee")->getLowerLimit(), getLeftThigh()->getHinge("leftKnee")->getUpperLimit());

	//FEET
	double raa = Util::normalize(getRightShin()->getHinge("rightAnkle")->getHingeAngle(), getRightShin()->getHinge("rightAnkle")->getLowerLimit(), getRightShin()->getHinge("rightAnkle")->getUpperLimit());
	double laa = Util::normalize(getLeftShin()->getHinge("leftAnkle")->getHingeAngle(), getLeftShin()->getHinge("leftAnkle")->getLowerLimit(), getLeftShin()->getHinge("leftAnkle")->getUpperLimit());

	return{ rha1, rha2, rha3, lha1, lha2, lha3, rka, lka, raa, laa };
}


std::vector<double> Biped::getAllAngularVelocities()
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

void Biped::setAllTargetVelocities(std::vector<double>& resultVec)
{
	//CHANGE MAX VELOCITY NORMALIZATION IF YOU CHANGE m
	double m = 15;
	double mU = 0.0;

	//HIPS
	//getHips()->getHinge("leftHip")->setMotorTargetVelocity((resultVec[0] - mU) * m);
	//getHips()->getHinge("rightHip")->setMotorTargetVelocity((resultVec[1] - mU) * m);

	// 0 = spin, 1 = fremover/bakover, 2 = side til side

	//LEFT HIP
	btVector3 lhLowerLim;
	hips->getdofConstraint("leftHip")->getAngularLowerLimit(lhLowerLim);
	btVector3 lhUpperLim;
	hips->getdofConstraint("leftHip")->getAngularUpperLimit(lhUpperLim);

	double lhAngle1 = Util::scaleToRange(resultVec[0], lhLowerLim[0], lhUpperLim[0]);
	double lhTargetVel1 = computeTargetVelocity(hips->getdofConstraint("leftHip")->getAngle(0), lhAngle1, 1);
	hips->getdofConstraint("leftHip")->getRotationalLimitMotor(0)->m_targetVelocity = lhTargetVel1 * 20;
	//std::cout << "lhLowerLim[1]: " << lhLowerLim[1] << " lhUpperLim[1]: " << lhUpperLim[1] << "\n";
	double lhAngle2 = Util::scaleToRange(resultVec[1], lhLowerLim[1], lhUpperLim[1]);
	double lhTargetVel2 = computeTargetVelocity(hips->getdofConstraint("leftHip")->getAngle(1), lhAngle2, 1);
	hips->getdofConstraint("leftHip")->getRotationalLimitMotor(1)->m_targetVelocity = lhTargetVel2 * 20;
	//std::cout << "resultVec[1]: " << resultVec[1] << " lhTargetVel2: " << lhTargetVel2 << "\n";
	double lhAngle3 = Util::scaleToRange(resultVec[2], lhLowerLim[2], lhUpperLim[2]);
	double lhTargetVel3 = computeTargetVelocity(hips->getdofConstraint("leftHip")->getAngle(2), lhAngle3, 1);
	hips->getdofConstraint("leftHip")->getRotationalLimitMotor(2)->m_targetVelocity = lhTargetVel3;

	//RIGHT HIP
	btVector3 rhLowerLim;
	hips->getdofConstraint("leftHip")->getAngularLowerLimit(rhLowerLim);
	btVector3 rhUpperLim;
	hips->getdofConstraint("leftHip")->getAngularUpperLimit(rhUpperLim);

	double rlhAngle1 = Util::scaleToRange(resultVec[3], rhLowerLim[0], rhUpperLim[0]);
	double rhTargetVel1 = computeTargetVelocity(hips->getdofConstraint("rightHip")->getAngle(0), rlhAngle1, 1);
	hips->getdofConstraint("rightHip")->getRotationalLimitMotor(0)->m_targetVelocity = rhTargetVel1;

	double rlhAngle2 = Util::scaleToRange(resultVec[4], rhLowerLim[1], rhUpperLim[1]);
	double rhTargetVel2 = computeTargetVelocity(hips->getdofConstraint("rightHip")->getAngle(1), rlhAngle2, 1);
	hips->getdofConstraint("rightHip")->getRotationalLimitMotor(1)->m_targetVelocity = rhTargetVel2;

	double rlhAngle3 = Util::scaleToRange(resultVec[5], rhLowerLim[2], rhUpperLim[2]);
	double rhTargetVel3 = computeTargetVelocity(hips->getdofConstraint("rightHip")->getAngle(2), rlhAngle3, 1);
	hips->getdofConstraint("rightHip")->getRotationalLimitMotor(2)->m_targetVelocity = rhTargetVel3;

	//KNEES
	double rkVel = Util::scaleToRange(resultVec[6], getRightThigh()->getHinge("rightKnee")->getLowerLimit(), getRightThigh()->getHinge("rightKnee")->getUpperLimit());
	getRightThigh()->getHinge("rightKnee")->setMotorTarget(rkVel, 1);

	double lkVel = Util::scaleToRange(resultVec[7], getLeftThigh()->getHinge("leftKnee")->getLowerLimit(), getLeftThigh()->getHinge("leftKnee")->getUpperLimit());
	getLeftThigh()->getHinge("leftKnee")->setMotorTarget(lkVel, 1);

	//FEET
	double rfVel = Util::scaleToRange(resultVec[8], getRightShin()->getHinge("rightAnkle")->getLowerLimit(), getRightShin()->getHinge("rightAnkle")->getUpperLimit());
	getRightShin()->getHinge("rightAnkle")->setMotorTarget(rfVel, 1);
	double lfVel = Util::scaleToRange(resultVec[9], getLeftShin()->getHinge("leftAnkle")->getLowerLimit(), getLeftShin()->getHinge("leftAnkle")->getUpperLimit());
	getLeftShin()->getHinge("leftAnkle")->setMotorTarget(lfVel, 1);
	
	////LEFT HIP
	//hips->getdofConstraint("leftHip")->getRotationalLimitMotor(0)->m_targetVelocity = resultVec[0] * m;
	//hips->getdofConstraint("leftHip")->getRotationalLimitMotor(1)->m_targetVelocity = resultVec[1] * m;
	//hips->getdofConstraint("leftHip")->getRotationalLimitMotor(2)->m_targetVelocity = resultVec[2] * m;

	////RIGHT HIP
	//hips->getdofConstraint("rightHip")->getRotationalLimitMotor(0)->m_targetVelocity = resultVec[3] * m;
	//hips->getdofConstraint("rightHip")->getRotationalLimitMotor(1)->m_targetVelocity = resultVec[4] * m;
	//hips->getdofConstraint("rightHip")->getRotationalLimitMotor(2)->m_targetVelocity = -resultVec[5] * m; //******* MINUS HER PGA motsatt side IKKE SIKKER OM DET VIRKER!! ********

	////KNEES
	//getRightThigh()->getHinge("rightKnee")->setMotorTarget(1, 1);
	//getRightThigh()->getHinge("rightKnee")->setMotorTargetVelocity((resultVec[6] - mU) * m);
	//getLeftThigh()->getHinge("leftKnee")->setMotorTargetVelocity((resultVec[7] - mU) * m);

	////FEET
	//getRightShin()->getHinge("rightAnkle")->setMotorTargetVelocity((resultVec[8] - mU) * m);
	//getLeftShin()->getHinge("leftAnkle")->setMotorTargetVelocity((resultVec[9] - mU) * m);
}

btScalar Biped::computeTargetVelocity(btScalar hingeAngle, btScalar targetAngle, double dt) {
	btScalar dAngle = targetAngle - hingeAngle;
	return dAngle / dt;

	if (hingeAngle > targetAngle) {
		return -100.0f;
	}
	else {
		return 100.0f;
	}
}
void Biped::setMaxMotorImpulses(double maxMotorImpulse)
{
	bool isEnableMotor = true;

	//THEM HIPS
	//getHips()->getHinge("leftHip")->enableMotor(isEnableMotor);
	//getHips()->getHinge("rightHip")->enableMotor(isEnableMotor);
	//getHips()->getHinge("leftHip")->setMaxMotorImpulse(maxMotorImpulse);
	//getHips()->getHinge("rightHip")->setMaxMotorImpulse(maxMotorImpulse);

	//LEFT HIP
	getHips()->getdofConstraint("leftHip")->getRotationalLimitMotor(0)->m_enableMotor = isEnableMotor;
	getHips()->getdofConstraint("leftHip")->getRotationalLimitMotor(0)->m_maxMotorForce = maxMotorImpulse;

	getHips()->getdofConstraint("leftHip")->getRotationalLimitMotor(1)->m_enableMotor = isEnableMotor;
	getHips()->getdofConstraint("leftHip")->getRotationalLimitMotor(1)->m_maxMotorForce = maxMotorImpulse;

	getHips()->getdofConstraint("leftHip")->getRotationalLimitMotor(2)->m_enableMotor = isEnableMotor;
	getHips()->getdofConstraint("leftHip")->getRotationalLimitMotor(2)->m_maxMotorForce = maxMotorImpulse;

	//RIGHT HIP
	getHips()->getdofConstraint("rightHip")->getRotationalLimitMotor(0)->m_enableMotor = isEnableMotor;
	getHips()->getdofConstraint("rightHip")->getRotationalLimitMotor(0)->m_maxMotorForce = maxMotorImpulse;

	getHips()->getdofConstraint("rightHip")->getRotationalLimitMotor(1)->m_enableMotor = isEnableMotor;
	getHips()->getdofConstraint("rightHip")->getRotationalLimitMotor(1)->m_maxMotorForce = maxMotorImpulse;

	getHips()->getdofConstraint("rightHip")->getRotationalLimitMotor(2)->m_enableMotor = isEnableMotor;
	getHips()->getdofConstraint("rightHip")->getRotationalLimitMotor(2)->m_maxMotorForce = maxMotorImpulse;

	//KnEES
	getRightThigh()->getHinge("rightKnee")->enableMotor(isEnableMotor);
	getLeftThigh()->getHinge("leftKnee")->enableMotor(isEnableMotor);
	getRightThigh()->getHinge("rightKnee")->setMaxMotorImpulse(maxMotorImpulse);
	getLeftThigh()->getHinge("leftKnee")->setMaxMotorImpulse(maxMotorImpulse);

	//FEETS
	getRightShin()->getHinge("rightAnkle")->enableMotor(isEnableMotor);
	getLeftShin()->getHinge("leftAnkle")->enableMotor(isEnableMotor);
	getRightShin()->getHinge("rightAnkle")->setMaxMotorImpulse(maxMotorImpulse / 2);
	getLeftShin()->getHinge("leftAnkle")->setMaxMotorImpulse(maxMotorImpulse / 2);
}
void Biped::setFitness(double fitness)
{
	m_fitness = fitness;
}

double Biped::getFitness()
{
	return m_fitness;
}

Box* Biped::getHips()
{
	return hips;
}

Box* Biped::getRightThigh()
{
	return rightThigh;
}

Box* Biped::getRightShin()
{
	return rightShin;
}

Box* Biped::getRightFoot()
{
	return rightFoot;
}

Box* Biped::getLeftThigh()
{
	return leftThigh;
}

Box* Biped::getLeftShin()
{
	return leftShin;
}

Box* Biped::getLeftFoot()
{
	return leftFoot;
}

double Biped::getHeight()
{
	return hips->getPosition().y;
}

void Biped::removeBodies(PhysicsManager * pm)
{
	hips->remove(pm);

	rightThigh->remove(pm);
	rightShin->remove(pm);
	rightFoot->remove(pm);

	leftThigh->remove(pm);
	leftShin->remove(pm);
	leftFoot->remove(pm);
}


void Biped::removeConstraints(PhysicsManager * pm)
{
	hips->removeConstraint(pm);

	rightThigh->removeConstraint(pm);

	rightShin->removeConstraint(pm);
	rightFoot->removeConstraint(pm);

	leftThigh->removeConstraint(pm);
	leftShin->removeConstraint(pm);
	leftFoot->removeConstraint(pm);
}

void Biped::setColor(glm::vec3 color)
{
	rightThigh->setColor(color);
	rightShin->setColor(color);
	rightFoot->setColor(color);
	leftThigh->setColor(color);
	leftShin->setColor(color);
	leftFoot->setColor(color);
}

void Biped::incrementToAverage()
{
	double value = getHeight();
	average = (avgSize * average + value) / (avgSize + 1);
	avgSize++;
}

double Biped::getAverageHeight() {
	return average;
}

double Biped::getMaxHeight()
{
	return m_maxHeight;
}

void Biped::updateMaxHeight(double height)
{

	if (height > m_maxHeight) m_maxHeight = height;
}

double Biped::getTimeOnGround()
{
	return timeOnGround;
}

void Biped::setTimeOnGround(double time)
{
	timeOnGround = time;
}

double Biped::getTimeOnTwoLegs()
{
	return timeOnTwoLegs;
}

void Biped::setTimeOnTwoLegs(double time)
{
	timeOnTwoLegs = time;
}

void Biped::checkIfLegsCrossed()
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

double Biped::getNumTimesCrossed() {
	return numTimesLegsCrossed;
}


bool Biped::leftFootMovingDownward()
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

bool Biped::rightFootMovingDownward()
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
int Biped::getNumerOfSteps()
{
	return numberOfSteps;
}
double Biped::getAverageFeetPosition()
{
	return (getLeftFoot()->getPosition().z + getRightFoot()->getPosition().z)/2;
}
double Biped::getAverageFeetStartPos()
{
	return averageFeetStartPos;
}
double Biped::getDistanceFromHips(Box * box)
{
	glm::vec3 end = box->getPosition();
	glm::vec3 start = hips->getPosition();
	return glm::distance(end, start);
}

void Biped::calcCenterPosition()
{
	centerPosition = hips->getPosition();
}

glm::vec3 Biped::getPosition()
{
	return hips->getPosition();
}

glm::vec3 Biped::getStartPosition()
{
	return m_startPosition;
}

glm::vec3 Biped::getRelativePosition(Box* box) {
	glm::vec3 relativePosition;
	relativePosition.x = box->getPosition().x - hips->getPosition().x;
	relativePosition.y = box->getPosition().y - hips->getPosition().y;
	//THIS IS WRONG
	relativePosition.z = box->getPosition().z - hips->getPosition().z;
	return relativePosition;
}

void Biped::activate() {

	hips->getRigidBody()->activate();

	rightThigh->getRigidBody()->activate();
	rightShin->getRigidBody()->activate();
	rightFoot->getRigidBody()->activate();
	leftThigh->getRigidBody()->activate();
	leftShin->getRigidBody()->activate();
	leftFoot->getRigidBody()->activate();
}


void Biped::setShouldUpdate(bool update)
{
	m_shouldUpdate = update;
}
bool Biped::shouldUpdate()
{
	return m_shouldUpdate;
}
void Biped::checkRotation()
{
	btQuaternion orientation = hips->getRigidBody()->getOrientation();
	//x = bakover/forover
	//std::cout << "X: " << orientation.getX() << "\n";
	//y = side til side
	//std::cout << "Y: " << orientation.getY() << "\n";
	//z = klokke rotasjon
	//std::cout << "Z: " << orientation.getZ() << "\n\n";

	double rotationCutoff = 0.4f;
	if (orientation.getY() > rotationCutoff || orientation.getY() < -rotationCutoff) {
		rotationAmount += (std::abs(orientation.getY()) - rotationCutoff)/4;
	}

	if (orientation.getZ() > rotationCutoff || orientation.getZ() < -rotationCutoff) {
		rotationAmount += std::abs(orientation.getZ()) - rotationCutoff;
	}
}

double Biped::getRotationAmount()
{
	return rotationAmount;
}

double Biped::getTimeAlive()
{
	return timeAlive;
}

void Biped::checkIfMoving()
{
	glm::vec3 p = hips->getPosition();
	glm::vec3 p2 = hips->getPreviousPosition();

	if (std::abs(p.x - p2.x) > 0.01f && std::abs(p.y - p2.y) > 0.01f && std::abs(p.z - p2.z) > 0.01f) {
		noMovementPenalty++;
	}
}

double Biped::getNoMovementPenalty() {
	return noMovementPenalty;
}

void Biped::calculateSpeed()
{
	glm::vec3 currentPosition = getPosition();
	//double currentSpeed = currentPosition.z - m_previousPosition.z;
	double currentSpeed = hips->getRigidBody()->getLinearVelocity().getZ() * 0.016;
	m_totalSpeed += currentSpeed;
	//std::cout << "currentspeed: " << currentSpeed << " linearVelocity: " << hips->getRigidBody()->getLinearVelocity().getZ() * 0.016 << "\n";


	m_previousPosition = currentPosition;
}

double Biped::getTotalSpeed()
{
	return m_totalSpeed;
}

double Biped::getDistanceWalked()
{
	glm::vec3 end = getPosition();
	glm::vec3 start = getStartPosition();
	return start.z - end.z;
}

Biped::~Biped()
{
	delete hips;
	delete rightThigh;
	delete rightShin;
	delete rightFoot;
	delete leftThigh;
	delete leftShin;
	delete leftFoot;
}
