// This example application loads a URDF world file and simulates two robots
// with physics and contact in a Dynamics3D virtual world. A graphics model of it is also shown using 
// Chai3D.

#include "model/ModelInterface.h"
#include "simulation/SimulationInterface.h"
#include "graphics/GraphicsInterface.h"
#include "chai3d.h"




#include <GLFW/glfw3.h> //must be loaded after loading opengl/glew


#include <sstream>
#include <iostream>
#include <string>
#include <Eigen/Dense>


using namespace std;
using namespace Eigen;


const string world_file = "resources/world.urdf";
const string robot_file = "resources/pbot.urdf";
const float ROTATE= 0.523; 
const string camera[]= {"camera_1", "camera_2"}; 

string camera_name = camera[0]; 


// callback to print glfw errors
void glfwError(int error, const char* description);

// callback when a key is pressed
void keySelect(GLFWwindow* window, int key, int scancode, int action, int mods);
auto graphics = new Graphics::GraphicsInterface(world_file, Graphics::chai, Graphics::urdf, true);


int main() {
	cout << "Loading URDF world model file: " << world_file << endl;


	// load simulation world
	auto sim = new Simulation::SimulationInterface(world_file, Simulation::sai2simulation, Simulation::urdf, false);

	// load graphics scene
	//auto graphics = new Graphics::GraphicsInterface(world_file, Graphics::chai, Graphics::urdf, true);



	
	cout <<"You can toggle between camera 1 and camera 2 by typing 1 or 2 in terminal" <<endl;

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
    GLFWwindow* window = glfwCreateWindow(windowW, windowH, "03-sim_graphics_urdfmodel", NULL, NULL);
	glfwSetWindowPos(window, windowPosX, windowPosY);
	glfwShowWindow(window);
    glfwMakeContextCurrent(window);
	glfwSwapInterval(1);

    // set callbacks
	glfwSetKeyCallback(window, keySelect);


	



    // while window is open:
    while (!glfwWindowShouldClose(window))
	{
			
		// update simulation by 1ms
		sim->integrate(0.01);

		// update kinematic models
		// update graphics. this automatically waits for the correct amount of time
		int width, height;
		glfwGetFramebufferSize(window, &width, &height);
		//graphics->updateGraphics(robot_name1, robot1);
		//graphics->updateGraphics(robot_name2, robot2);
		graphics->render(camera_name, width, height);

		// swap buffers
		glfwSwapBuffers(window);

		// wait until all GL commands are completed
		glFinish();

		// check for any OpenGL errors
		GLenum err;
		err = glGetError();
		assert(err == GL_NO_ERROR);

	    // poll for events
	    glfwPollEvents(); 
	}

    // destroy context
    glfwDestroyWindow(window);

    // terminate
    glfwTerminate();

	return 0;
}

//------------------------------------------------------------------------------

void glfwError(int error, const char* description) {
	cerr << "GLFW Error: " << description << endl;
	exit(1);
}

//------------------------------------------------------------------------------

void rotateCamera(string& direction) {
    Eigen::Vector3d p;
    Eigen::Vector3d v; 
    Eigen::Vector3d l;
    graphics -> getCameraPose(camera_name, p,v,l); 
    
    Eigen::Matrix3d m; 

    if (direction =="left") {
        m << cos(-ROTATE), -sin(-ROTATE), 0,
            sin(-ROTATE), cos(-ROTATE), 0,
            0, 0, 1; 
    } else if (direction =="right") {
         m << cos(ROTATE), -sin(ROTATE), 0,
            sin(ROTATE), cos(ROTATE), 0,
            0, 0, 1; 
    } else if (direction =="up") {
         m << 1, 0, 0, 
            0, cos(ROTATE), -sin(ROTATE), 
            0, sin(ROTATE), cos(ROTATE); 
    } else if (direction=="down"){
        m << 1, 0, 0, 
            0, cos(-ROTATE), -sin(-ROTATE),
            0, sin(-ROTATE), cos(-ROTATE); 
    }
    
    p=m*p; 
    
    graphics -> setCameraPose(camera_name, p, v, l);

}


void keySelect(GLFWwindow* window, int key, int scancode, int action, int mods)
{
    if (action ==GLFW_PRESS) {
                // option ESC: exit
        if (key == GLFW_KEY_ESCAPE )
        {
            // exit application
             glfwSetWindowShouldClose(window, 1);
        } else if (key==GLFW_KEY_1)
        {
            camera_name = camera[0]; 
        } else if (key==GLFW_KEY_2 )
        {
            camera_name = camera[1]; 

        } else if(key==GLFW_KEY_LEFT)
        {   
            string str="left";
            if (camera_name == camera[1]) {
                rotateCamera(str);
            }
        } else if(key==GLFW_KEY_RIGHT) {
            string str="right";
            if (camera_name == camera[1]) {
                rotateCamera(str);
            }
        } else if(key==GLFW_KEY_UP){
            string str="up";
            if (camera_name == camera[1]) {
                rotateCamera(str);
            }
        } else if(key==GLFW_KEY_DOWN){
            string str="down";
            if (camera_name == camera[1]) {
                rotateCamera(str);
            }
        }
        
    }
    /*


    */

}
