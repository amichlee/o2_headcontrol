// This example application loads a URDF world file and simulates two robots
// with physics and contact in a Dynamics3D virtual world. A graphics model of it is also shown using 
// Chai3D.

#include "model/ModelInterface.h"
#include "simulation/SimulationInterface.h"
#include "graphics/GraphicsInterface.h"
#include <graphics/GraphicsInterface.h>
#include <graphics/ChaiGraphics.h>
#include "chai3d.h"
#include <GLFW/glfw3.h> //must be loaded after loading opengl/glew
#include <sstream>
#include <iostream>
#include <string>
#include <Eigen/Dense>
#include <cmath>


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

// callback when a mouse button is pressed

void mouseClick(GLFWwindow* window, int button, int action, int mods);

// callback when user scrolls
void mouseScroll(GLFWwindow* window, double xoffset, double yoffset);

// flags for scene camera movement
bool fTransXp = false;
bool fTransXn = false;
bool fTransYp = false;
bool fTransYn = false;
bool fRotPanTilt = false;
bool fZoom = false;
double zoomSpeed = 0.0;
bool fRobotLinkSelect = false;

int main() {
    cout << "Loading URDF world model file: " << world_file << endl;


    // load simulation world
    auto sim = new Simulation::SimulationInterface(world_file, Simulation::sai2simulation, Simulation::urdf, false);

    // load graphics scene
    auto graphics_int = new Graphics::GraphicsInterface(world_file, Graphics::chai, Graphics::urdf, true);
    Graphics::ChaiGraphics* graphics;
    graphics = dynamic_cast<Graphics::ChaiGraphics*>(graphics_int->_graphics_internal);
    Eigen::Vector3d camera_pos, camera_lookat, camera_vertical;
    graphics->getCameraPose(camera_name, camera_pos, camera_vertical, camera_lookat);


    

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
    glfwSetMouseButtonCallback(window, mouseClick);
    glfwSetScrollCallback(window, mouseScroll);

    double last_cursorx, last_cursory;

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

        Eigen::Vector3d cam_up_axis;

        cam_up_axis << 0.0, 0.0, 1.0; //TODO: there might be a better way to do this
        Eigen::Vector3d cam_roll_axis = (camera_lookat - camera_pos).cross(cam_up_axis);
        cam_roll_axis.normalize();
        Eigen::Vector3d cam_lookat_axis = camera_lookat;
        cam_lookat_axis.normalize();
        if (fTransXp) {
            camera_pos = camera_pos + 0.05*cam_roll_axis;
            camera_lookat = camera_lookat + 0.05*cam_roll_axis;
        }
        if (fTransXn) {
            camera_pos = camera_pos - 0.05*cam_roll_axis;
            camera_lookat = camera_lookat - 0.05*cam_roll_axis;
        }
        if (fTransYp) {
            // camera_pos = camera_pos + 0.05*cam_lookat_axis;
            camera_pos = camera_pos + 0.05*cam_up_axis;
            camera_lookat = camera_lookat + 0.05*cam_up_axis;
        }
        if (fTransYn) {
            // camera_pos = camera_pos - 0.05*cam_lookat_axis;
            camera_pos = camera_pos - 0.05*cam_up_axis;
            camera_lookat = camera_lookat - 0.05*cam_up_axis;
        }
        if (fRotPanTilt) {
            // get current cursor position
            double cursorx, cursory;
            glfwGetCursorPos(window, &cursorx, &cursory);
            //TODO: might need to re-scale from screen units to physical units
            double compass = 0.006*(cursorx - last_cursorx);
            double azimuth = 0.006*(cursory - last_cursory);
            double radius = (camera_pos - camera_lookat).norm();
            Eigen::Matrix3d m_tilt; m_tilt = Eigen::AngleAxisd(azimuth, -cam_roll_axis);
            camera_pos = camera_lookat + m_tilt*(camera_pos - camera_lookat);
            Eigen::Matrix3d m_pan; m_pan = Eigen::AngleAxisd(compass, -cam_up_axis);
            // camera_pos = camera_lookat + m_pan*(camera_pos - camera_lookat);
            // TODO: the above doesn't work as intended because Chai treats the lookat
            // vector as a direction vector in the local frame, rather than as a lookat point
            camera_pos = m_pan*(camera_pos);
            camera_lookat = m_pan*(camera_lookat);
            // TODO: the above fix is a HUGE hack. Think about improving this.
        }
        if (fZoom) {
            camera_pos = camera_pos + 0.04*camera_lookat*zoomSpeed;
            fZoom = false;
        }
        graphics->setCameraPose(camera_name, camera_pos, cam_up_axis, camera_lookat);
        glfwGetCursorPos(window, &last_cursorx, &last_cursory);
        if (fRobotLinkSelect) {
            //activate widget
            //force_widget.setEnable(true);
            // get current cursor position
            double cursorx, cursory;
            glfwGetCursorPos(window, &cursorx, &cursory);
            int wwidth_scr, wheight_scr;
            int wwidth_pix, wheight_pix;
            glfwGetWindowSize(window, &wwidth_scr, &wheight_scr);
            glfwGetFramebufferSize(window, &wwidth_pix, &wheight_pix);
            int viewx, viewy;
            viewx = floor(cursorx/wwidth_scr * wwidth_pix);
            viewy = floor(cursory/wheight_scr * wheight_pix);
            std::string ret_link_name;
            Eigen::Vector3d ret_pos;
            //if (cursorx > 0 && cursory > 0) {
                //force_widget.setInteractionParams(camera_name, viewx, wheight_pix-viewy, wwidth_pix, wheight_pix);
                //TODO: this behavior might be wrong. this will allow the user to click elsewhere in the screen
                // then drag the mouse over a link to start applying a force to it.
            //}
        } //else {
            //force_widget.setEnable(false);
        //}

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




void keySelect(GLFWwindow* window, int key, int scancode, int action, int mods)
{
    bool set = (action != GLFW_RELEASE);
    switch(key) {
        case GLFW_KEY_ESCAPE:
            // exit application
            glfwSetWindowShouldClose(window,GL_TRUE);
            break;
        case GLFW_KEY_RIGHT:
            fTransXp = set;
            break;
        case GLFW_KEY_LEFT:
            fTransXn = set;
            break;
        case GLFW_KEY_UP:
            fTransYp = set;
            break;
        case GLFW_KEY_DOWN:
            fTransYn = set;
            break;
        default:
            break;
    }
}

//------------------------------------------------------------------------------

void mouseClick(GLFWwindow* window, int button, int action, int mods) {
    bool set = (action != GLFW_RELEASE);
    //TODO: mouse interaction with robot
    switch (button) {
        // left click pans and tilts
        case GLFW_MOUSE_BUTTON_LEFT:
            fRotPanTilt = set;
            // NOTE: the code below is recommended but doesn't work well
            // if (fRotPanTilt) {
            //  // lock cursor
            //  glfwSetInputMode(window, GLFW_CURSOR, GLFW_CURSOR_DISABLED);
            // } else {
            //  glfwSetInputMode(window, GLFW_CURSOR, GLFW_CURSOR_NORMAL);
            // }
            break;
        // if right click: don't handle. this is for menu selection
        case GLFW_MOUSE_BUTTON_RIGHT:
            fRobotLinkSelect = set;
            // TODO: move link select to shift + left click
            // TODO: set menu
            break;
        // if middle click: don't handle. doesn't work well on laptops
        case GLFW_MOUSE_BUTTON_MIDDLE:
            break;
        default:
            break;
    }
}

//------------------------------------------------------------------------------
void mouseScroll(GLFWwindow* window, double xoffset, double yoffset) {
    fZoom = true;
    zoomSpeed = yoffset;
}
