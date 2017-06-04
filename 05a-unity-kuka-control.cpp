// This example application runs a controller for the IIWA

#include "model/ModelInterface.h"
#include "redis/RedisClient.h"
#include "timer/LoopTimer.h"

#include <iostream>
#include <string>

#include <signal.h>
bool runloop = true;
void stop(int){runloop = false;}

using namespace std;


const string world_file = "../resources/05-unity-kuka/world.urdf";
const string robot_file = "../resources/05-unity-kuka/kuka_iiwa02.urdf";
const string robot_name = "Kuka-IIWA";

unsigned long long controller_counter = 0;

// redis keys:
// - write:
const std::string JOINT_TORQUES_COMMANDED_KEY = "sai2::KUKA_IIWA::actuators::fgc";
// - read:
const std::string JOINT_ANGLES_KEY  = "sai2::KUKA_IIWA::sensors::q"; 
const std::string JOINT_VELOCITIES_KEY = "sai2::KUKA_IIWA::sensors::dq";
const std::string JOINT_TORQUES_SENSED_KEY = "sai2::KUKA_IIWA::sensors::torques";
const std::string VR_PAN = "vr_pan"; 
const std::string VR_TILT = "vr_tilt"; 

const int PAN_JOINT = 4;
const int TILT_JOINT= 5; 



 void sighandler(int sig)
 { runloop = false; }

Eigen::VectorXd saturatedDesiredVelocity(Eigen::VectorXd Kv, Eigen::VectorXd Kp, Eigen::VectorXd qd, Eigen::VectorXd robot_q);
float sigNum(float i); 
const float DEG_TO_RAD= M_PI/180; 


int main() {
	cout << "Loading URDF world model file: " << world_file << endl;

	float tilt = 0;
	float pan = 0; 
	float tilt_rad= 0;
	float pan_rad= 0;
	bool readyToStart= false; 


	// start redis client
	HiredisServerInfo info;
	info.hostname_ = "127.0.0.1";
	info.port_ = 6379;
	info.timeout_ = { 1, 500000 }; // 1.5 seconds
	auto redis_client = RedisClient();
	redis_client.serverIs(info);

	// set up signal handler
	signal(SIGABRT, &sighandler);
	signal(SIGTERM, &sighandler);
	signal(SIGINT, &sighandler);

	// load robots
	auto robot = new Model::ModelInterface(robot_file, Model::rbdl, Model::urdf, true);

	// read from Redis
	redis_client.getEigenMatrixDerived(JOINT_ANGLES_KEY, robot->_q);
	redis_client.getEigenMatrixDerived(JOINT_VELOCITIES_KEY, robot->_dq);

	////////////////////////////////////////////////
	///    Prepare the different controllers   /////
	////////////////////////////////////////////////
	robot->updateModel();

	int dof = robot->dof();
	Eigen::VectorXd command_torques = Eigen::VectorXd::Zero(dof);

	Eigen::MatrixXd N_prec;

	// Joint control
	Eigen::VectorXd joint_task_desired_position(dof), joint_task_torques(dof);
	Eigen::VectorXd initial_joint_position(dof), initial_desired_joint_position(dof), first(dof);
	initial_joint_position = robot->_q;
	joint_task_desired_position = initial_joint_position; 

	Eigen::MatrixXd joint_kp = Eigen::MatrixXd::Zero(7,7);
	Eigen::MatrixXd joint_kv = Eigen::MatrixXd::Zero(7,7);
	Eigen::VectorXd joint_kp_vector = Eigen::VectorXd::Zero(7);
	Eigen::VectorXd joint_kv_vector = Eigen::VectorXd::Zero(7);
	joint_kp_vector << 50, 50, 30, 10, 10, 8, 5;
	joint_kv_vector << 10, 10, 10, 5, 5, 3, 1;


	joint_kp = joint_kp_vector.asDiagonal();
	joint_kv = joint_kv_vector.asDiagonal();
	
	std::string pan_string;
	std::string tilt_string; 

	// create a loop timer
	double control_freq = 1000;
	LoopTimer timer;
	timer.setLoopFrequency(control_freq);   // 1 KHz
	// timer.setThreadHighPriority();  // make timing more accurate. requires running executable as sudo.
	timer.setCtrlCHandler(stop);    // exit while loop on ctrl-c
	timer.initializeTimer(1000000); // 1 ms pause before starting loop

	//set preferred initial conditions 
	initial_desired_joint_position << 90.0/180.0*M_PI, 
										-50.0/180.0*M_PI, 
										0, 
										-50/180.0*M_PI,
										0,
										-90/180.0*M_PI,
										0; 
	

	while (runloop) {

		// wait for next scheduled loop
		timer.waitForNextLoop();

		// read from Redis
		redis_client.getEigenMatrixDerived(JOINT_ANGLES_KEY, robot->_q);
		redis_client.getEigenMatrixDerived(JOINT_VELOCITIES_KEY, robot->_dq);

		if (readyToStart == false ) {

			joint_task_torques = -joint_kv*( robot->_dq+ saturatedDesiredVelocity(joint_kv_vector, joint_kp_vector, initial_desired_joint_position, robot->_q));

			if (controller_counter > 5000) {
				readyToStart= true; 
				cout<<"ready to start" <<endl;
			}
		} else { 

			if (controller_counter%10 == 0 ) {
				
				joint_task_desired_position= initial_desired_joint_position; 
				pan_string = redis_client.get(VR_PAN); 
				tilt_string = redis_client.get(VR_TILT); 

				pan =  std::stof(pan_string);
				tilt = std::stof(tilt_string);

				if (pan > 180) {
					pan= pan-360; 
				}
				if (tilt >180) {
					tilt= tilt- 360; 
				}

				pan= pan*-1; 
				tilt = tilt*-1;

				if (pan>160) {
					pan = 160; 
				}

				if (pan <-160) {
					pan = -160; 
				}

				if (tilt < -20)  {
					tilt = -20;
					cout<<"don't tilt more than -20deg"<<endl;
				}

				if (tilt >80) {
					tilt = 80; 
				}

				

				cout<<"pan: "<<pan <<endl;
				cout<<"tilt: "<<tilt <<endl;

				pan_rad= pan/180.0*M_PI;
				tilt_rad = tilt/180.0*M_PI;

				joint_task_desired_position(TILT_JOINT)= initial_joint_position(TILT_JOINT)+tilt_rad;
				joint_task_desired_position(PAN_JOINT) = initial_joint_position(PAN_JOINT)+ pan_rad; 

				//cout<<"joint desired pos: " <<joint_task_desired_position<<endl;
				//cout<<"pan angle: " <<pan<<endl;
				//cout<<"tilt angle: " <<tilt<<endl;
			}
			joint_task_torques = -joint_kv*( robot->_dq+ saturatedDesiredVelocity(joint_kv_vector, joint_kp_vector, joint_task_desired_position, robot->_q));

		}

		// update the model 20 times slower
		if(controller_counter%20 == 0)
		{
			robot->updateModel();

		}
		

		////////////////////////////// Compute joint torques
		double time = controller_counter/control_freq;

		//----- Joint control

		//------ Final torques
		command_torques = joint_task_torques;

		//------ senf torques to robot
		redis_client.setEigenMatrixDerived(JOINT_TORQUES_COMMANDED_KEY, command_torques);

		controller_counter++;

	}

    command_torques << 0,0,0,0,0,0,0;
    redis_client.setEigenMatrixDerived(JOINT_TORQUES_COMMANDED_KEY, command_torques);

    double end_time = timer.elapsedTime();
    std::cout << "\n";
    std::cout << "Loop run time  : " << end_time << " seconds\n";
    std::cout << "Loop updates   : " << timer.elapsedCycles() << "\n";
    std::cout << "Loop frequency : " << timer.elapsedCycles()/end_time << "Hz\n";

    return 0;

}


Eigen::VectorXd saturatedDesiredVelocity(Eigen::VectorXd Kv, Eigen::VectorXd Kp, Eigen::VectorXd qd, Eigen::VectorXd robot_q) {
	Eigen::VectorXd q_diff(7);
	Eigen::VectorXd diff(7); 
	Eigen::VectorXd max(7);
	max << 90.0, 90.0, 95.0, 125.0, 135.0, 170.0, 170.0; 
	max*= DEG_TO_RAD; 
	//cout<<"max: " <<max <<endl; 

	for (int i = 0; i<qd.size(); i++) {
	    diff(i)=Kp(i)/Kv(i)*(robot_q(i)-qd(i)); 
	    //cout<<"current " <<i<< "num: " <<diff(i)<<endl;
	    q_diff(i)= sigNum(diff(i)) * std::min(abs(diff(i)), max(i));
	}
	return q_diff; 
}	



float sigNum (float i) {
	if (i<0) return -1;
	if (i>0) return 1;
	return 0; 
}


