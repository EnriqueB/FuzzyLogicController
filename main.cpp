#include "Aria.h"
#include <Windows.h>
#include <conio.h>
#include <iostream>
#include <math.h>
#include <fstream>
#include <algorithm>
#include "fuzzyController.h"


using namespace std;

//Music player
void beepTones(ArRobot *robot, const vector< pair<double, unsigned char> >& tune)
{
	char buf[40];
	size_t p = 0;
	for (std::vector< pair<double, unsigned char> >::const_iterator i = tune.begin(); i != tune.end() && p < 39; ++i)
	{
		if ((*i).first > 0)
		{
			buf[p++] = (char)((*i).first / 20.0);
			buf[p++] = (char)(*i).second;
		}
	}
	buf[p++] = 0;
	buf[p++] = 0;
	robot->comStrN(ArCommands::SAY, buf, p);
}

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
	robot.setVel2(base, base);
	int lastLeft = base, lastRight = base;
	long long count = 0;

	//Create fuzzy controller for wall following
	FuzzyController wallFollowing;
	wallFollowing.addSets("inputSetsWall.txt", true);
	wallFollowing.addSets("outputSetsWall.txt", false);
	wallFollowing.addRules("rulesWall.txt");

	//Create fuzzy controller for collision avoidance
	FuzzyController avoidance;
	avoidance.addSets("inputSetsAvoidance.txt", true);
	avoidance.addSets("outputSetsAvoidance.txt", false);
	avoidance.addRules("rulesAvoidance.txt");

	//Create fuzzy controller to manage
	//wall following and collision avoidance
	FuzzyController controller;
	controller.addSets("inputSetsController.txt", true);
	controller.addRules("rulesController.txt");

	//Jingle Bells tune
	vector < pair < double, unsigned char > >tune(19);
	tune[0] = make_pair(300, 62);
	tune[1] = make_pair(300, 71);
	tune[2] = make_pair(300, 69);
	tune[3] = make_pair(300, 67);

	tune[4] = make_pair(900, 62);
	tune[5] = make_pair(300, 62);

	tune[6] = make_pair(300, 62);
	tune[7] = make_pair(300, 71);
	tune[8] = make_pair(300, 69);
	tune[9] = make_pair(300, 67);

	tune[10] = make_pair(900, 64);
	tune[11] = make_pair(300, 64);

	tune[12] = make_pair(300, 64);
	tune[13] = make_pair(300, 60);
	tune[14] = make_pair(300, 71);
	tune[15] = make_pair(300, 69);
	tune[16] = make_pair(900, 75);
	tune[17] = make_pair(300, 75);
	tune[18] = make_pair(300, 62);
	beepTones(&robot, tune);


	while (true){
		robot.setVel2(0, 0);

		double laserRange[18];
		double laserAngle[18];
		//PID first
		cout << "Press [ESC] at any time to stop PID from running\n";
		while (true) {
			//break if ESC key is pressed
			if (GetAsyncKeyState(VK_ESCAPE))
				break;

			//obtain laser readings
			laser->lockDevice();
			for (int i = 0; i < 18; i++) {
				laserRange[i] = laser->currentReadingPolar(10 * i - 90, 10 * (i + 1) - 90,
					&laserAngle[i]);

			}
			laser->unlockDevice();

			int desired_distance = 450;
			double range = laserRange[1];

			//obtain errors
			int error = desired_distance - range;
			err_acum += error;
			err_D = error - err_prev;
			err_prev = error;

			//update speed
			int output = kp * error + kd*err_D;
			if (output > 100)
				output = 100;
			int leftVel = base - output;
			int rightVel = base + output;

			if (leftVel < 0)
				leftVel = lastLeft;
			if (rightVel < 0)
				rightVel = lastRight;
			if (leftVel > 300)
				leftVel /= 30;
			if (rightVel > 300)
				rightVel /= 55;

			//collision avoidance
			if (laserRange[9] < 600) {
				leftVel = 20;
				rightVel = 170;
			}

			lastLeft = leftVel;
			lastRight = rightVel;

			robot.setVel2(leftVel, rightVel);
		}
		robot.setVel2(0, 0);

		cout << "Press [SPACE] to start fuzzy logic controller \n";
		while (true){
			if (GetAsyncKeyState(VK_SPACE))
				break;
		}

		cout << "Now running fuzzy logic controller\n";
		while (true) {
			laser->lockDevice();
			int frontL, frontR, frontF;
			frontL = (int)laser->currentReadingPolar(15, 45, &laserAngle[0]);
			frontR = (int)laser->currentReadingPolar(-25, -15, &laserAngle[0]);
			frontF = (int)laser->currentReadingPolar(-15, 15, &laserAngle[0]);
			for (int i = 0; i < 18; i++) {
				laserRange[i] = laser->currentReadingPolar(10 * i - 90, 10 * (i + 1) - 90,
					&laserAngle[i]);

			}
			laser->unlockDevice();

			vector <int> inputWall;
			vector <double> outputWall;
			vector <int> inputAvoidance;
			vector <double> outputAvoidance;

			//Avoid extreme values
			if (frontL > 5000)	frontL = 5000;
			if (frontR > 5000)	frontR = 5000;
			if (frontF > 5000) frontF = 5000;

			int rightF, rightB;
			rightF = (int)laserRange[1];
			rightB = (int)laserRange[0];
			if (rightF > 2000) rightF = 2000;
			if (rightB > 2000) rightB = 2000;

			inputWall.push_back(rightF);
			inputWall.push_back(rightB);

			//inputWall.push_back(frontL);
			//inputWall.push_back(frontR);

			//obtain the output speeds for the
			//wall following controller
			outputWall = wallFollowing.evaluateInput(inputWall);

			inputAvoidance.push_back(frontL);
			inputAvoidance.push_back(frontR);
			inputAvoidance.push_back(frontF);

			//obtain the output speeds for the
			//collision avoidance controller
			outputAvoidance = avoidance.evaluateInput(inputAvoidance);

			//prepare data for the Controller
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

			//obtain the output speeds for the
			//controller that oversees the wall
			//following behaviour and collision
			//avoidance
			vector<double> velocities = controller.combineValues(min(frontL, frontR), min(rightF, rightB), outputAvoidance, outputWall, avoidanceCheck, wallCheck);
			
			robot.setVel2(velocities[0], velocities[1]);
			ArUtil::sleep(10);
		}
		
	}
	robot.lock();
	robot.stop();
	robot.unlock();
	Aria::exit();
}
