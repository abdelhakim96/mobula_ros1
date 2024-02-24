cmake_minimum_required(VERSION 3.0.2)
project(gp_wind_regression)
set(PROJECT_NAME gp_wind_regression)
set(ROS_BUILD_TYPE Debug)

include(CheckCXXCompilerFlag)
CHECK_CXX_COMPILER_FLAG("-std=c++14" COMPILER_SUPPORTS_CXX14)
CHECK_CXX_COMPILER_FLAG("-std=c++11" COMPILER_SUPPORTS_CXX11)
CHECK_CXX_COMPILER_FLAG("-std=c++0x" COMPILER_SUPPORTS_CXX0X)
if(COMPILER_SUPPORTS_CXX14)
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++14")
elseif(COMPILER_SUPPORTS_CXX11)
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11 ${CMAKE_CXX_FLAGS_DEBUG} -O1")
elseif(COMPILER_SUPPORTS_CXX0X)
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++0x")
else()
    message(STATUS "The compiler ${CMAKE_CXX_COMPILER} has no C++14 or C++11 support. Please use a different C++ compiler.")
endif()

## Custom options
set(MAKE_SHARED true CACHE BOOL "Build and use shared libraries")

## Define settings for static build
if(NOT MAKE_SHARED)
    ## This is needed by find_package
    set(CMAKE_FIND_LIBRARY_SUFFIXES ".a")
    ## Link against static libs
    set(CMAKE_EXE_LINKER_FLAGS "-static-libgcc -static-libstdc++ -static")
    ## Produce static libs
    set(BUILD_SHARED_LIBS OFF)
endif()

## Find catkin macros and libraries
set(${PROJECT_NAME}_CATKIN_DEPS
    roscpp 
    roslib
    rospy 
    std_msgs 
    mavros_msgs
    geometry_msgs
    sensor_msgs 
    tf 
    #ros_libgp_bridge
)

find_package(catkin REQUIRED COMPONENTS ${${PROJECT_NAME}_CATKIN_DEPS})
find_package(Eigen3 3.0.1 REQUIRED)
include_directories(${EIGEN3_INCLUDE_DIRS})

catkin_package(
    INCLUDE_DIRS include
    CATKIN_DEPENDS ${${PROJECT_NAME}_CATKIN_DEPS}
)

# Set the location of libgp library
# set(libgp_BINARY_LOCATION "${CMAKE_BINARY_DIR}/../../libgp/build")

#set(libgp_BINARY_LOCATION "/home/hakim/catkin_ws/src/mobula_ros1/wind-regression/libgp/build")
#message(STATUS "libgp_BINARY_LOCATION = ${libgp_BINARY_LOCATION}")


set(libgp_BINARY_LOCATION "${CMAKE_CURRENT_SOURCE_DIR}/../libgp_sparse/build")
message(STATUS "libgp_BINARY_LOCATION = ${libgp_BINARY_LOCATION}")


include_directories(
    ${catkin_INCLUDE_DIRS}
    ${PROJECT_SOURCE_DIR}/include
    ${CMAKE_CURRENT_SOURCE_DIR}/../libgp_sparse  # Adjust this path
    ${CMAKE_CURRENT_SOURCE_DIR}/../libgp_sparse/include
    ${CMAKE_CURRENT_SOURCE_DIR}/../libgp_sparse/src
)

# List of source files
set(PROJECT_SOURCE
    src/gp_wind_regression.cpp
    src/gp_model.cpp
    src/gp_model_base.cpp
    src/gp_model_type_concatenated.cpp
    src/feature_data_model.cpp
    src/regression_data_model.cpp
    src/utils.cpp
)

# Node executable
add_executable(gp_wind_regression src/gp_wind_regression_main.cpp ${PROJECT_SOURCE})

# Node dependencies
add_dependencies(gp_wind_regression ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

# Node linking
target_link_libraries(gp_wind_regression ${catkin_LIBRARIES} ${libgp_BINARY_LOCATION}/libgp.a)


# Node executable
add_executable(feature_generator src/feature_generator.cpp ${PROJECT_SOURCE})

# Node dependencies
add_dependencies(feature_generator ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})


target_link_libraries(feature_generator ${catkin_LIBRARIES} ${libgp_BINARY_LOCATION}/libgp.a)


# Install headers
install(DIRECTORY include
    DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)

# Install executables
install(TARGETS gp_wind_regression
    RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
























