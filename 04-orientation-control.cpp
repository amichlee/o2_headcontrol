/*  Operation space orientation control of end effector 
*/

#include <iostream>
#include <string>
#include <thread>
#include <math.h>

#include "model/ModelInterface.h"
#include "graphics/GraphicsInterface.h"
#include "simulation/SimulationInterface.h"

#include "timer/LoopTimer.h"

#include <GLFW/glfw3.h> //must be loaded after loading opengl/glew as part of graphicsinterface

using namespace std;

const string world_fname = "../worlds/03/world.urdf";
const string robot_fname = "../resources/kuka_iiwa_camera/kuka_iiwa.urdf";
const string robot_name = "Kuka-IIWA";

// const string camera_name = "camera_front";
const string camera_name = "camera_top";
const string ee_link_name = "link6";
const string camera_link_name= "link6";
const Eigen::Vector3d pos_in_link(0,0,0);


// simulation loop
bool fSimulationRunning = false; 
void control(Model::ModelInterface* robot, Simulation::SimulationInterface* sim);
void simulation(Model::ModelInterface* robot, Simulation::SimulationInterface* sim);

// initialize window manager
GLFWwindow* glfwInitialize();

// callback to print glfw errors
void glfwError(int error, const char* description);

// callback when a key is pressed
void keySelect(GLFWwindow* window, int key, int scancode, int action, int mods);


//keys for keyboard control

bool rotZn = false; 
bool rotZp = false; 
bool rotXn = false;
bool rotXp = false; 


int main (int argc, char** argv) {
	cout << "Loading URDF world model file: " << world_fname << endl;

	// load graphics scene
	auto graphics = new Graphics::GraphicsInterface(world_fname, Graphics::chai, Graphics::urdf, false);

	// load robots
	auto robot = new Model::ModelInterface(robot_fname, Model::rbdl, Model::urdf, false);

	// load simulation world
	auto sim = new Simulation::SimulationInterface(world_fname, Simulation::sai2simulation, Simulation::urdf, false);

	// set initial condition
	robot->_q << 125.9/180.0*M_PI,
				39.2/180.0*M_PI,
				-49.2/180.0*M_PI,
				70.0/180.0*M_PI,
				-62.4/180.0*M_PI,
				80.2/180.0*M_PI,
				187.2/180.0*M_PI;
	sim->setJointPositions(robot_name, robot->_q);
	robot->updateModel();
	// Eigen::Affine3d ee_trans;
	// robot->transform(ee_trans, ee_link_name);
	// cout << ee_trans.translation().transpose() << endl;
	// cout << ee_trans.rotation() << endl;

	// initialize GLFW window
	GLFWwindow* window = glfwInitialize();

    // set callbacks
	glfwSetKeyCallback(window, keySelect);

	// start the simulation thread first
	fSimulationRunning = true;
	thread sim_thread(simulation, robot, sim);

	// next start the control thread
	thread ctrl_thread(control, robot, sim);
	
    // while window is open:
    while (!glfwWindowShouldClose(window)) {
		// update kinematic models
		// robot->updateModel();

		// update graphics. this automatically waits for the correct amount of time
		int width, height;
		glfwGetFramebufferSize(window, &width, &height);
		graphics->updateGraphics(robot_name, robot);
		graphics->render(camera_name, width, height);
		glfwSwapBuffers(window);
		glFinish();

	    // poll for events
	    glfwPollEvents();
	}

	// stop simulation
	fSimulationRunning = false;
	sim_thread.join();
	ctrl_thread.join();

    // destroy context
    glfwDestroyWindow(window);

    // terminate
    glfwTerminate();

	return 0;
}

//------------------------------------------------------------------------------
void control(Model::ModelInterface* robot, Simulation::SimulationInterface* sim) {
	// create a timer
	LoopTimer timer;
	timer.initializeTimer();
	timer.setLoopFrequency(200); //200Hz timer
	double last_time = timer.elapsedTime(); //secs

	// cache variables
	bool fTimerDidSleep = true;
	Eigen::VectorXd tau = Eigen::VectorXd::Zero(robot->dof());

	double kp_or   = 500 , kv_or   = 40 ; // orientation control
	double kp_pos = 300, kv_pos = 40; 
	double kp_null= 0, kv_null= 10;

	float ang = M_PI/12; 
	
	Eigen::Matrix3d rot_d; 

	Eigen::Matrix3d rot; 
	robot->rotation(rot, ee_link_name); 

	rot_d = rot; 
	cout<< rot <<endl; 
	Eigen::Vector3d rot_error; 

	// initialize matrices 

	Eigen::MatrixXd Jv;
	Eigen::MatrixXd Jw;
	Eigen::MatrixXd J0(6, robot->dof());
	Eigen::MatrixXd Lambda(J0.rows(), J0.rows());
	Eigen::MatrixXd Jbar(J0.cols(), J0.rows());
	Eigen::MatrixXd N(robot->dof(), robot->dof());

	Eigen::MatrixXd controller(6,1); 
	Eigen::Vector3d pos_con; 
	Eigen::Vector3d or_con; 
	Eigen::Vector3d avel;  
	Eigen::Vector3d vel;  
	Eigen::VectorXd F = Eigen::VectorXd::Zero(6);

	Eigen::Matrix3d m_tilt; 
	Eigen::Matrix3d m_pan; 
 

	

	//cout<<"Mass matrix "<< robot->_M  <<endl; 
	//cout<<"Jw is: " <<Jw <<endl; 
	//cout<<"Jv is: " <<Jv <<endl; 



	while (fSimulationRunning) { //automatically set to false when simulation is quit
		fTimerDidSleep = timer.waitForNextLoop();

		// update time
		double curr_time = timer.elapsedTime();
		double loop_dt = curr_time - last_time;

		// read joint positions, velocities
		sim->getJointPositions(robot_name, robot->_q);
		sim->getJointVelocities(robot_name, robot->_dq);
		robot->updateModel();

		//Calculate Jacobians

		robot-> Jw(Jw, ee_link_name);
		robot-> Jv(Jv, ee_link_name, pos_in_link); 
		J0.block(0, 0, 3, robot->dof()) = Jv;  // assign Jv
		J0.block(3, 0, 3, robot->dof()) = Jw; // assign Jw

		
		robot-> operationalSpaceMatrices(Lambda,  Jbar, N, J0);
	
		// calculate current rotation 
		robot->rotation(rot, ee_link_name); 

		//Determine rotation based on keys 
		if (rotZn) {
			m_pan = Eigen::AngleAxisd(ang, -rot.col(2));
			rot_d= m_pan*rot; 
            
        }
        if (rotZp) {
           	m_pan = Eigen::AngleAxisd(ang, rot.col(2));
			rot_d= m_pan*rot; 
        }
        if (rotXp) {
            m_tilt = Eigen::AngleAxisd(ang, rot.col(0));
			rot_d= m_tilt*rot; 
        }
        if (rotXn) {
          	m_tilt = Eigen::AngleAxisd(ang, -rot.col(0));
			rot_d= m_tilt*rot; 
        }


		// Calculate rotation error 
		robot-> orientationError(rot_error, rot_d, rot); 

		// calculate or and pos control; 
		robot-> angularVelocity(avel, ee_link_name); 
		robot-> linearVelocity(vel, ee_link_name, pos_in_link); 

		pos_con = -kv_pos* vel; 
	
		or_con= kp_or*(-rot_error) - kv_or* avel; 
		controller << pos_con, or_con; 

		F= Lambda * controller; 
		tau= J0.transpose() * F + N * (robot-> _M* (-kp_null*robot->_q - kv_null *robot->_dq));
		            

		sim->setJointTorques(robot_name, tau);
		// -------------------------------------------

		// update last time
		last_time = curr_time;
	}
}

//------------------------------------------------------------------------------
void simulation(Model::ModelInterface* robot, Simulation::SimulationInterface* sim) {
	fSimulationRunning = true;

	// create a timer
	LoopTimer timer;
	timer.initializeTimer();
	timer.setLoopFrequency(1000); //1000Hz timer
	double last_time = timer.elapsedTime(); //secs

	bool fTimerDidSleep = true;
	while (fSimulationRunning) {
		fTimerDidSleep = timer.waitForNextLoop();

		// integrate forward
		double curr_time = timer.elapsedTime();
		double loop_dt = curr_time - last_time; 
		sim->integrate(loop_dt);

		// if (!fTimerDidSleep) {
		// 	cout << "Warning: timer underflow! dt: " << loop_dt << "\n";
		// }

		// update last time
		last_time = curr_time;
	}
}

//------------------------------------------------------------------------------
GLFWwindow* glfwInitialize() {
		/*------- Set up visualization -------*/
    // set up error callback
    glfwSetErrorCallback(glfwError);

    // initialize GLFW
    glfwInit();

    // retrieve resolution of computer display and position window accordingly
    GLFWmonitor* primary = glfwGetPrimaryMonitor();
    const GLFWvidmode* mode = glfwGetVideoMode(primary);

    // information about computer screen and GLUT display window
	int screenW = mode->width;
    int screenH = mode->height;
    int windowW = 0.8 * screenH;
    int windowH = 0.5 * screenH;
    int windowPosY = (screenH - windowH) / 2;
    int windowPosX = windowPosY;

    // create window and make it current
    glfwWindowHint(GLFW_VISIBLE, 0);
    GLFWwindow* window = glfwCreateWindow(windowW, windowH, "SAI2.0 - CS327a HW2", NULL, NULL);
	glfwSetWindowPos(window, windowPosX, windowPosY);
	glfwShowWindow(window);
    glfwMakeContextCurrent(window);
	glfwSwapInterval(1);

	return window;
}

//------------------------------------------------------------------------------

void glfwError(int error, const char* description) {
	cerr << "GLFW Error: " << description << endl;
	exit(1);
}

//------------------------------------------------------------------------------

void keySelect(GLFWwindow* window, int key, int scancode, int action, int mods)
{
    bool set = (action != GLFW_RELEASE);
    switch(key) {
        case GLFW_KEY_ESCAPE:
            // exit application
            glfwSetWindowShouldClose(window,GL_TRUE);
            break;
        case GLFW_KEY_RIGHT:
            rotZn = set;
            break;
        case GLFW_KEY_LEFT:
            rotZp = set;
            break;
        case GLFW_KEY_UP:
            rotXp = set;
            break;
        case GLFW_KEY_DOWN:
            rotXn = set;
            break;
        default:
            break;
    }
}