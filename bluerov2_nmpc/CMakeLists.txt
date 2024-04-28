cmake_minimum_required(VERSION 3.8)
project(bluerov2_nmpc)


set(ROS_BUILD_TYPE Debug)


if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()


# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(Eigen3 3.0.1 REQUIRED)
include_directories(${EIGEN3_INCLUDE_DIRS})


# Message generation
find_package(rosidl_default_generators REQUIRED)
rosidl_generate_interfaces(${PROJECT_NAME}
  msg/PwmCmd.msg
 )
ament_export_dependencies(rosidl_default_runtime)


# Add library
add_library(lib${PROJECT_NAME} 
  solver/qpoases/SRC/Bounds.cpp
  solver/qpoases/SRC/Constraints.cpp
  solver/qpoases/SRC/CyclingManager.cpp
  solver/qpoases/SRC/Indexlist.cpp
  solver/qpoases/SRC/MessageHandling.cpp
  solver/qpoases/SRC/QProblem.cpp
  solver/qpoases/SRC/QProblemB.cpp
  solver/qpoases/SRC/SubjectTo.cpp
  solver/qpoases/SRC/Utils.cpp
  solver/qpoases/SRC/EXTRAS/SolutionAnalysis.cpp
  solver/nmpc_qpoases_interface.cpp
  solver/nmpc_integrator.c
  solver/nmpc_solver.c
  solver/nmpc_auxiliary_functions.c
)

target_compile_features(lib${PROJECT_NAME} PRIVATE c_std_99 cxx_std_17) 
target_include_directories(lib${PROJECT_NAME} PRIVATE
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/solver
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/solver/qpoases
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/solver/qpoases/INCLUDE
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/solver/qpoases/INCLUDE/EXTRAS
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/solver/qpoases/SRC
)

# Causes the visibility macros to use dllexport rather than dllimport,
# which is appropriate when building the dll but not consuming it.
target_compile_definitions(lib${PROJECT_NAME} PRIVATE "BLUEROV2_NMPC_BUILDING_LIBRARY")

install(
  DIRECTORY include/
  DESTINATION include
)
install(
  TARGETS lib${PROJECT_NAME}
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)


# Add node executables
add_executable(${PROJECT_NAME}_node 
  src/nmpc_bluerov2_2D_main.cpp 
  src/nmpc_bluerov2_2D.cpp 
)
target_link_libraries(${PROJECT_NAME}_node PUBLIC
  lib${PROJECT_NAME}
  ${catkin_LIBRARIES}
)

install(TARGETS ${PROJECT_NAME}_node
  DESTINATION lib/${PROJECT_NAME})

install(PROGRAMS
  scripts/thrust_allocator.py
  DESTINATION lib/${PROJECT_NAME}
)


# Testing
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
