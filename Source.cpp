#include "Aria.h"
#include <iostream>
#include <math.h>
#include <fstream>
#include <algorithm>
#include "fuzzyController.h"


using namespace std;

int main(int argc, char **argv){
	Aria::init();
	ArRobot robot;
	ArPose pose;

	ArArgumentParser argParser(&argc, argv);
	argParser.loadDefaultArguments();
	argParser.addDefaultArgument("?connectLaser");


	ArRobotConnector robotConnector(&argParser, &robot);
	if (robotConnector.connectRobot())
		cout << "Robot connected!" << endl;

	robot.runAsync(false);
	robot.lock();
	robot.enableMotors();
	robot.unlock();

	ArLaserConnector laserConnector(&argParser, &robot,
		&robotConnector);
	if (laserConnector.connectLasers())
		std::cout << "Laser connected!" << std::endl;

	ArLaser *laser = robot.findLaser(1);



	ArSensorReading *sonarSensor[8];
	int err_acum = 0;
	int err_prev = 0;
	int err_D = 0;
	int base = 140;
	double kp, ki, kd;
	kp = 0.5;
	ki = 0;
	kd = 1.1;
	//robot.setVel2(base, base);
	int lastLeft = base, lastRight = base;
	ofstream wfile;
	long long count = 0;
	FuzzyController wallFollowing;
	wallFollowing.addSets("inputSetsWall.txt", true);
	wallFollowing.addSets("outputSetsWall.txt", false);
	wallFollowing.addRules("rulesWall.txt");

	FuzzyController avoidance;
	avoidance.addSets("inputSetsAvoidance.txt", true);
	avoidance.addSets("outputSetsAvoidance.txt", false);
	avoidance.addRules("rulesAvoidance.txt");

	FuzzyController controller;
	controller.addSets("inputSetsController.txt", true);
	controller.addRules("rulesController.txt");
	while (true){

		double laserRange[18];
		double laserAngle[18];
		laser->lockDevice();
		int frontL, frontR, frontF;
		frontL = (int)laser->currentReadingPolar(15, 45, &laserAngle[0]);
		frontR = (int)laser->currentReadingPolar(-25, -15, &laserAngle[0]);
		frontF = (int)laser->currentReadingPolar(-15, 15, &laserAngle[0]);
		for (int i = 0; i < 18; i++){
			laserRange[i] = laser->currentReadingPolar(10 * i - 90, 10 * (i + 1) - 90,
				&laserAngle[i]);

		}
		laser->unlockDevice();

		cout << frontL << " " << frontR << " " << frontF << endl;

		
		//follow a left wall
		int desired_distance = 450;
		double range = laserRange[1];
		cout << range << " ";
		int error = desired_distance - range;
		err_acum += error;
		err_D = error - err_prev;
		err_prev = error;

		int output = kp * error + kd*err_D;
		cout << output << " " << err_D;
		cout << endl;
		if (output > 100)
		output = 100;
		int leftVel = base - output;
		int rightVel = base + output;

		if (leftVel < 0)
		leftVel = lastLeft;
		if (rightVel < 0)
		rightVel = lastRight;

		if (laserRange[9] < 600){
		cout << "front\n";
		leftVel = 20;
		rightVel = 170;
		}
		if (range > 700){
		//outside corner
		leftVel = 170;
		rightVel = 50;
		}
		lastLeft = leftVel;
		lastRight = rightVel;

		robot.setVel2(leftVel, rightVel);
		count++;
		

		/*

		vector <int> inputWall;
		vector <double> outputWall;
		vector <int> inputAvoidance;
		vector <double> outputAvoidance;


		if (frontL > 5000)	frontL = 5000;
		if (frontR > 5000)	frontR = 5000;
		if (frontF > 5000) frontF = 5000;

		//cin >> frontL >> frontR;

		int rightF, rightB;
		rightF = (int)laserRange[1];
		rightB = (int)laserRange[0];
		if (rightF > 2000) rightF = 2000;
		if (rightB > 2000) rightB = 2000;

		inputWall.push_back(rightF);
		inputWall.push_back(rightB);

		//inputWall.push_back(frontL);
		//inputWall.push_back(frontR);

		outputWall = wallFollowing.evaluateInput(inputWall);
		inputAvoidance.push_back(frontL);
		inputAvoidance.push_back(frontR);
		inputAvoidance.push_back(frontF);
		outputAvoidance = avoidance.evaluateInput(inputAvoidance);
		//cout << outputWall[0]<<" "<<outputWall[1]<<endl;
		//robot.setVel2(outputWall[0], outputWall[1]);


		//check if avoidance should take place
		double avoidanceCheck = avoidance.getMembership(min(frontL, frontR), 0)[0];
		double wallCheck;
		if (laserRange[2] > laserRange[0]){
			//back sensor closer
			wallCheck = wallFollowing.getMembership(rightB, 1)[0];
		}
		else{
			//front sensor closer
			wallCheck = wallFollowing.getMembership(rightF, 0)[0];
		}
		vector<double> velocities = controller.combineValues(min(frontL, frontR), min(rightF, rightB), outputAvoidance, outputWall, avoidanceCheck, wallCheck);
		//cout << velocities[0] << " " << velocities[1] << endl << endl;
		//robot.setVel2(velocities[0], velocities[1]);

		if (avoidanceCheck > 0){
			//there is a wall in front#
			double leftS, rightS;
			leftS = (outputAvoidance[0] * avoidanceCheck + outputWall[0] * wallCheck) / (avoidanceCheck + wallCheck);
			rightS = (outputAvoidance[1] * avoidanceCheck + outputWall[1] * wallCheck) / (avoidanceCheck + wallCheck);
			cout << "Avoidance: " << leftS << " " << rightS << endl;
			//cout << "otro " << avoidanceCheck << " " << wallCheck << endl;
			robot.setVel2(leftS, rightS);
			if (fabs(velocities[0] - leftS) > 1 || fabs(velocities[1]-rightS)>1)
				cout <<"wrong "<< velocities[0] - leftS << " " << velocities[1] - rightS << endl;
		}
		else{
			//nothing in front
			cout << "nothing: " << outputWall[0] << " " << outputWall[1] << endl;
			robot.setVel2(outputWall[0], outputWall[1]);
			//cout << velocities[0] - outputWall[0] <<" "<< velocities[1] - outputWall[1]<<endl;
			if (fabs(velocities[0] - outputWall[0]) > 1 || fabs(velocities[1] - outputWall[1]) > 1)
				cout <<"wrong2 "<< velocities[0] - outputWall[0] << " " << velocities[1] - outputWall[1] << endl;
		}
		cout << endl;
		*/
		ArUtil::sleep(10);
	}
	wfile.close();
	robot.lock();
	robot.stop();
	robot.unlock();
	Aria::exit();
}