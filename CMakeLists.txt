CMAKE_MINIMUM_REQUIRED(VERSION 2.8.12)

PROJECT (UNITY_KUKA)
# SG: dont need version
SET (PROJECT_VERSION 0.1.0) #major version is zero for now since api will certainly change in future

SET(CMAKE_CXX_FLAGS "-std=c++11")

set (O2_COMMON_SOURCE
	${PROJECT_SOURCE_DIR}/src/redis/RedisClient.h
	${PROJECT_SOURCE_DIR}/src/timer/LoopTimer.h
	${PROJECT_SOURCE_DIR}/src/force_sensor/ForceSensorSim.h
	${PROJECT_SOURCE_DIR}/src/force_sensor/ForceSensorDisplay.h)
include_directories(${PROJECT_SOURCE_DIR}/src)

# include Chai3d (required by Dynamics3D)
# -- this is required by Chai3d to pick the correct path for OpenGL headers
FIND_PACKAGE(CHAI3D REQUIRED)
ADD_DEFINITIONS(${CHAI3D_DEFINITIONS})
if (CMAKE_SYSTEM_NAME MATCHES Darwin) 
  ADD_DEFINITIONS(-D_MACOSX -DMACOSX)
elseif (CMAKE_SYSTEM_NAME MATCHES Linux)
  ADD_DEFINITIONS(-D_LINUX -DLINUX)
endif () #NOTE: Windows is currently unsupported
include_directories(${CHAI3D_INCLUDE_DIRS})
LINK_DIRECTORIES(${CHAI3D_LIBRARY_DIRS})

# include Sai2-Simulation
FIND_PACKAGE(SAI2-SIMULATION REQUIRED)
#SG: add include directories and symbols from sai2-simulation
include_directories(${SAI2-SIMULATION_INCLUDE_DIRS})
add_definitions(${SAI2-SIMULATION_DEFINITIONS})

# - SAI2-COMMON
find_package(SAI2-COMMON REQUIRED)
include_directories(${SAI2-COMMON_INCLUDE_DIRS})
add_definitions(${SAI2-COMMON_DEFINITIONS})

# include Eigen
FIND_PACKAGE(Eigen3 REQUIRED)
#SG: include eigen symbols
include_directories(${EIGEN3_INCLUDE_DIR})

# include GLFW3
FIND_PACKAGE(glfw3 QUIET)
FIND_LIBRARY(GLFW_LIB glfw)

# - hiredis
find_library(HIREDIS_LIBRARY hiredis)

# JSON 
if (CMAKE_SYSTEM_NAME MATCHES Darwin) 
	find_library(JSONCPP_LIBRARY jsoncpp)
elseif (CMAKE_SYSTEM_NAME MATCHES Linux)
	find_package(PkgConfig REQUIRED)
	pkg_check_modules(JSONCPP jsoncpp)
	include_directories(${JSONCPP_INCLUDE_DIRS})
	link_directories(${JSONCPP_LIBRARY_DIRS})
	set(JSONCPP_LIBRARY ${JSONCPP_LIBRARIES})
endif ()

#SG: Not required. 
# include RBDL
FIND_PACKAGE(RBDL QUIET)

# create an executable
ADD_EXECUTABLE(05a-control-unity-kuka 05a-unity-kuka-control.cpp src/redis/RedisClient.cpp)
ADD_EXECUTABLE(05b-simulation-unity-kuka 05b-unity-kuka-simulation.cpp)
ADD_EXECUTABLE(05c-visualizer-unity-kuka 05c-unity-kuka-visualizer.cpp)
#ADD_EXECUTABLE(04-orientation-control 04-orientation-control.cpp)

#SG: add chai libraries

set(O2_COMMON_LIBRARIES
	${CHAI3D_LIBRARIES}
	${SAI2-COMMON_LIBRARIES}
	${SAI2-SIMULATION_LIBRARIES}
	${GLFW_LIB}
	${HIREDIS_LIBRARY}
	${JSONCPP_LIBRARY})

TARGET_LINK_LIBRARIES (05a-control-unity-kuka
	${O2_COMMON_LIBRARIES})

TARGET_LINK_LIBRARIES (05b-simulation-unity-kuka
	${O2_COMMON_LIBRARIES})

TARGET_LINK_LIBRARIES (05c-visualizer-unity-kuka 
	${O2_COMMON_LIBRARIES})


#TARGET_LINK_LIBRARIES (04-orientation-control
#	${O2_COMMON_LIBRARIES})

SET(RESOURCE_DIR ${CMAKE_CURRENT_BINARY_DIR}/resources)
FILE(MAKE_DIRECTORY ${RESOURCE_DIR})
	