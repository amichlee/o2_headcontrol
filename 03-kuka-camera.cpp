/*  hw1 - main.cpp
This file includes the required code to implement problem 2.2.

Author: Shameek Ganguly shameekg@stanford.edu
Date: 4/9/17
*/



#include <iostream>
#include <string>
#include <thread>
#include <math.h>
#include <graphics/ChaiGraphics.h>

#include "graphics/chai_extension/CRobotLink.h"
#include "graphics/chai_extension/CRobotBase.h"


#include "model/ModelInterface.h"
#include "graphics/GraphicsInterface.h"

#include "timer/LoopTimer.h"

#include <GLFW/glfw3.h> //must be loaded after loading opengl/glew as part of graphicsinterface

using namespace std;

const string world_fname = "../worlds/03/world.urdf";
const string robot_fname = "../resources/kuka_iiwa_camera/kuka_iiwa.urdf";
const string robot_name = "Kuka-IIWA";
const string camera_name = "camera_link";
// const string camera_name = "camera_top";
const string ee_link_name = "link6";
const string camera_link= "link6"; 

// simulation loop
bool fSimulationRunning = false;
void simulation(Model::ModelInterface* robot);

// initialize window manager
GLFWwindow* glfwInitialize();

// callback to print glfw errors
void glfwError(int error, const char* description);

// callback when a key is pressed
void keySelect(GLFWwindow* window, int key, int scancode, int action, int mods);

//get link cobject 
//chai3d::cRobotLink* getLink(const std::string& link_name, const std::string& robot_name); 
//chai3d::cRobotLink* searchLinkInParent(const std::string& link_name, chai3d::cGenericObject* parent); 
Graphics::ChaiGraphics* graphics; 






int main (int argc, char** argv) {

	// load graphics scene
	
	graphics = new Graphics::ChaiGraphics(world_fname, Graphics::urdf, false);

	// load robots
	auto robot = new Model::ModelInterface(robot_fname, Model::rbdl, Model::urdf, false);

	//cout<<" Done loading robos" <<endl;
	// load camera onto kuka
	chai3d::cCamera* robo_camera = graphics->getCamera(camera_name); 

	//cout<<"got camera" <<endl; 

	graphics-> _world-> removeChild(robo_camera); 
	//cout<<"removed camera" <<endl; 
	
	chai3d::cRobotLink* cam_link = graphics->getLink(camera_link, robot_name); 
	cam_link -> addChild(robo_camera);
	

	//cout<<"Done loading new camera" <<endl; 
	// set initial condition
	robot->_q << 125.9/180.0*M_PI, //0,
		     39.2/180.0*M_PI,
		     -49.2/180.0*M_PI,
		     70.0/180.0*M_PI,
		     -62.4/180.0*M_PI,
		     80.2/180.0*M_PI,
		     187.2/180.0*M_PI;

	robot->updateModel();
	
	// Eigen::Affine3d ee_trans;
	// robot->transform(ee_trans, ee_link_name);
	// cout << ee_trans.translation().transpose() << endl;
	// cout << ee_trans.rotation() << endl;

	// initialize GLFW window
	GLFWwindow* window = glfwInitialize();

    	// set callbacks
	glfwSetKeyCallback(window, keySelect);

	// start the simulation
	thread sim_thread(simulation, robot);
	
    	// while window is open:
    	while (!glfwWindowShouldClose(window)) {
		// update kinematic models
		// robot->updateModel();
		// update graphics. this automatically waits for the correct amount of time
		int width, height;
		glfwGetFramebufferSize(window, &width, &height);
		graphics->updateGraphics(robot_name, robot);
		

		//graphics->render(camera_name, width, height);
		
		// should implement in chaigraphics 
		robo_camera -> renderView(width, height);
		glfwSwapBuffers(window);
		glFinish();

	    // poll for events
	    glfwPollEvents();
	}

	// stop simulation
	fSimulationRunning = false;
	sim_thread.join();

    // destroy context
    glfwDestroyWindow(window);

    // terminate
    glfwTerminate();

	return 0;
}

//------------------------------------------------------------------------------
void simulation(Model::ModelInterface* robot) {
	fSimulationRunning = true;

	// create a timer
	LoopTimer timer;
	timer.initializeTimer();
	timer.setLoopFrequency(500); //500Hz timer
	double last_time = timer.elapsedTime(); //secs
	bool fTimerDidSleep = true;

	// variables
	Eigen::MatrixXd Jbar;
	Eigen::MatrixXd Jv;
	Eigen::MatrixXd Jw;
	Eigen::MatrixXd J0;
	const Eigen::Vector3d pos_in_link(0,0,0);
	Eigen::Matrix<double,3,4> Er_inv;	
	Eigen::Matrix<double,7,1> Xd;
	Eigen::Matrix<double,3,1> d_Xd1;
	Eigen::Matrix<double,4,1> d_Xd2;
	Eigen::Matrix<double,3,1> w;
	Eigen::Matrix<double,6,1> V;

	// simulation
	while (fSimulationRunning) {
		
		fTimerDidSleep = timer.waitForNextLoop();

		// integrate joint velocity to joint positions
		double curr_time = timer.elapsedTime();
		double loop_dt = curr_time - last_time; 
		robot->_q += robot->_dq * loop_dt;

		// update kinematic models
		robot->updateModel();

		//----------------------------------------------------
		//-----------------	HW 1 CODE ------------------------

		// (1) Curent desired position and orientation: Xd(ti)	
		//----------------------------------------------------
		double xd = 0;
		double yd = 0.5 + 0.1 * cos((2*M_PI*curr_time)/5);
		double zd = 0.65 - 0.05 * cos((4*M_PI*curr_time)/5);
		double lambda0 = (1/sqrt(2)) * sin((M_PI/4)*cos((2*M_PI*curr_time)/5));
		double lambda1 = (1/sqrt(2)) * cos((M_PI/4)*cos((2*M_PI*curr_time)/5));
		double lambda2 = (1/sqrt(2)) * sin((M_PI/4)*cos((2*M_PI*curr_time)/5));
		double lambda3 = (1/sqrt(2)) * cos((M_PI/4)*cos((2*M_PI*curr_time)/5));

		Xd<< xd,
			yd,
			zd,
			lambda0,
			lambda1,
			lambda2,
			lambda3;


		// (2) Desired velocities: V = (E+) * d_Xd
		//----------------------------------------
		
		// desired operational space velocity vector components @ current time
		double d_xd = 0;
		double d_yd = (-0.2*M_PI/5) * sin((2*M_PI*curr_time)/5);
		double d_zd = ( 0.2*M_PI/5) * sin((4*M_PI*curr_time)/5);
		double d_lambda0 = (1/sqrt(2))  * cos((M_PI/4)*cos((2*M_PI*curr_time)/5)) * ((-pow(M_PI,2))/10)  * sin((2*M_PI*curr_time)/5);
		double d_lambda1 = (-1/sqrt(2)) * sin((M_PI/4)*cos((2*M_PI*curr_time)/5)) * ((-pow(M_PI,2))/10)  * sin((2*M_PI*curr_time)/5);
		double d_lambda2 = (1/sqrt(2))  * cos((M_PI/4)*cos((2*M_PI*curr_time)/5)) * ((-pow(M_PI,2))/10)  * sin((2*M_PI*curr_time)/5);
		double d_lambda3 = (-1/sqrt(2)) * sin((M_PI/4)*cos((2*M_PI*curr_time)/5)) * ((-pow(M_PI,2))/10)  * sin((2*M_PI*curr_time)/5);

		// separate desired velocity vector into two chunks 
		d_Xd1<< d_xd,
				d_yd,
				d_zd;
		d_Xd2<< d_lambda0,
				d_lambda1, 
				d_lambda2,
				d_lambda3;
		
		// use Er inverse to get the angular velocities
		Er_inv<< -lambda1, lambda0, -lambda3, lambda2,
			  	 -lambda2, lambda3, lambda0, -lambda1,
			  	 -lambda3, -lambda2, lambda1, lambda0;
		Er_inv = 2*Er_inv;
		w = Er_inv * d_Xd2;

		// Ep is the identity matrix so the linear velocities will simply be the desired operational space 
		//velocities stored in d_Xd1 (1st chunk). To obtain V, I concatenate both vectors.
		V<< w,
			d_Xd1; //Reversed order because the Jacobian has the linear and angular velocities switched too

		
		// (3) New joint velocities: dq = (Jbar) * V
		//-------------------------------------------
		robot-> J(J0, ee_link_name, pos_in_link); 
		Jbar = (robot->_M_inv)*(J0.transpose()) * ((J0*(robot->_M_inv)*(J0.transpose())).inverse());
		robot->_dq = Jbar * V;

		// ---------------------------------------------------------
		//---------------end of HW 1 CODE---------------------------

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
    GLFWwindow* window = glfwCreateWindow(windowW, windowH, "SAI2.0 - CS327a HW1", NULL, NULL);
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
    // option ESC: exit
    if (key == GLFW_KEY_ESCAPE && action == GLFW_PRESS)
    {
        // exit application
         glfwSetWindowShouldClose(window, 1);
    }
}


//-----------------------------------------------------------------------------
// find camera link, now in ChaiGraphics 

/*

chai3d::cRobotLink* searchLinkInParent(const std::string& link_name, chai3d::cGenericObject* parent) {
	cout<<"searching link at "<< parent->m_name <<endl;
	chai3d::cRobotLink* link2; 
	if (link_name == parent->m_name){
		cout<<"Found the link!" <<endl;
		return dynamic_cast<chai3d::cRobotLink*>(parent);
	} 
	for (unsigned int i=0; i< parent-> getNumChildren(); i++) {
		cout<< parent->m_name <<" has " <<parent->getNumChildren()<< " children, and this is number: "<<i<<endl; 
		if (parent->getChild(i)!=NULL) {
		 	link2 = searchLinkInParent(link_name, parent->getChild(i)); 
		 	if (link2 != NULL) {
		 		return link2; 
		 	}
		} else {
			//cout<<"unfortunately this child is null: "<<parent->getChild(i)<<endl; 
			return NULL; 
		}
	}
	return NULL;
}

chai3d::cRobotLink* getLink(const std::string& link_name, const std::string& robot_name) {
	chai3d::cRobotLink* link = NULL; 
	chai3d::cRobotBase* base = NULL;	
	for (unsigned int i = 0; i < graphics->_world->getNumChildren(); ++i) {
		cout<<i <<endl;
		if (robot_name == graphics->_world->getChild(i)->m_name) {
			//cout<<"found robot base" <<endl;
			base = dynamic_cast<chai3d::cRobotBase*>(graphics->_world->getChild(i));
			if (base!= NULL) {
				break; 
			}
		}
	}
	if (base == NULL) {
		cerr << "Could not find robot named " << link_name << endl;
		abort();
	}
	for (unsigned int i=0; i< base-> getNumChildren(); i++) {
			chai3d::cGenericObject* child = base->getChild(i);
			if (child !=NULL) {
				link= searchLinkInParent(link_name, child); 
			}	
	}		
	if (link == NULL) {
		cerr << "Could not find link named " << link_name << endl;
		abort();
		//TODO: throw exception instead
	} 
	return link; 		
}*/ 
