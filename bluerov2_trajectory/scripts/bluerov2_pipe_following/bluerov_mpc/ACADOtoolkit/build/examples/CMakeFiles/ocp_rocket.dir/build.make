# CMAKE generated file: DO NOT EDIT!
# Generated by "Unix Makefiles" Generator, CMake Version 3.10

# Delete rule output on recipe failure.
.DELETE_ON_ERROR:


#=============================================================================
# Special targets provided by cmake.

# Disable implicit rules so canonical targets will work.
.SUFFIXES:


# Remove some rules from gmake that .SUFFIXES does not remove.
SUFFIXES =

.SUFFIXES: .hpux_make_needs_suffix_list


# Suppress display of executed commands.
$(VERBOSE).SILENT:


# A target that is always out of date.
cmake_force:

.PHONY : cmake_force

#=============================================================================
# Set environment variables for the build.

# The shell in which to execute make rules.
SHELL = /bin/sh

# The CMake executable.
CMAKE_COMMAND = /usr/bin/cmake

# The command to remove a file.
RM = /usr/bin/cmake -E remove -f

# Escaping for special characters.
EQUALS = =

# The top-level source directory on which CMake was run.
CMAKE_SOURCE_DIR = /home/hakim/Desktop/ACADOtoolkit

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /home/hakim/Desktop/ACADOtoolkit/build

# Include any dependencies generated for this target.
include examples/CMakeFiles/ocp_rocket.dir/depend.make

# Include the progress variables for this target.
include examples/CMakeFiles/ocp_rocket.dir/progress.make

# Include the compile flags for this target's objects.
include examples/CMakeFiles/ocp_rocket.dir/flags.make

examples/CMakeFiles/ocp_rocket.dir/ocp/rocket.cpp.o: examples/CMakeFiles/ocp_rocket.dir/flags.make
examples/CMakeFiles/ocp_rocket.dir/ocp/rocket.cpp.o: ../examples/ocp/rocket.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/hakim/Desktop/ACADOtoolkit/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building CXX object examples/CMakeFiles/ocp_rocket.dir/ocp/rocket.cpp.o"
	cd /home/hakim/Desktop/ACADOtoolkit/build/examples && /usr/lib/ccache/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/ocp_rocket.dir/ocp/rocket.cpp.o -c /home/hakim/Desktop/ACADOtoolkit/examples/ocp/rocket.cpp

examples/CMakeFiles/ocp_rocket.dir/ocp/rocket.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/ocp_rocket.dir/ocp/rocket.cpp.i"
	cd /home/hakim/Desktop/ACADOtoolkit/build/examples && /usr/lib/ccache/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/hakim/Desktop/ACADOtoolkit/examples/ocp/rocket.cpp > CMakeFiles/ocp_rocket.dir/ocp/rocket.cpp.i

examples/CMakeFiles/ocp_rocket.dir/ocp/rocket.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/ocp_rocket.dir/ocp/rocket.cpp.s"
	cd /home/hakim/Desktop/ACADOtoolkit/build/examples && /usr/lib/ccache/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/hakim/Desktop/ACADOtoolkit/examples/ocp/rocket.cpp -o CMakeFiles/ocp_rocket.dir/ocp/rocket.cpp.s

examples/CMakeFiles/ocp_rocket.dir/ocp/rocket.cpp.o.requires:

.PHONY : examples/CMakeFiles/ocp_rocket.dir/ocp/rocket.cpp.o.requires

examples/CMakeFiles/ocp_rocket.dir/ocp/rocket.cpp.o.provides: examples/CMakeFiles/ocp_rocket.dir/ocp/rocket.cpp.o.requires
	$(MAKE) -f examples/CMakeFiles/ocp_rocket.dir/build.make examples/CMakeFiles/ocp_rocket.dir/ocp/rocket.cpp.o.provides.build
.PHONY : examples/CMakeFiles/ocp_rocket.dir/ocp/rocket.cpp.o.provides

examples/CMakeFiles/ocp_rocket.dir/ocp/rocket.cpp.o.provides.build: examples/CMakeFiles/ocp_rocket.dir/ocp/rocket.cpp.o


# Object files for target ocp_rocket
ocp_rocket_OBJECTS = \
"CMakeFiles/ocp_rocket.dir/ocp/rocket.cpp.o"

# External object files for target ocp_rocket
ocp_rocket_EXTERNAL_OBJECTS =

../examples/ocp/rocket: examples/CMakeFiles/ocp_rocket.dir/ocp/rocket.cpp.o
../examples/ocp/rocket: examples/CMakeFiles/ocp_rocket.dir/build.make
../examples/ocp/rocket: lib/libacado_toolkit_s.so.1.2.2beta
../examples/ocp/rocket: lib/libacado_casadi.a
../examples/ocp/rocket: lib/libacado_qpoases.a
../examples/ocp/rocket: lib/libacado_csparse.a
../examples/ocp/rocket: examples/CMakeFiles/ocp_rocket.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir=/home/hakim/Desktop/ACADOtoolkit/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Linking CXX executable ../../examples/ocp/rocket"
	cd /home/hakim/Desktop/ACADOtoolkit/build/examples && $(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/ocp_rocket.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
examples/CMakeFiles/ocp_rocket.dir/build: ../examples/ocp/rocket

.PHONY : examples/CMakeFiles/ocp_rocket.dir/build

examples/CMakeFiles/ocp_rocket.dir/requires: examples/CMakeFiles/ocp_rocket.dir/ocp/rocket.cpp.o.requires

.PHONY : examples/CMakeFiles/ocp_rocket.dir/requires

examples/CMakeFiles/ocp_rocket.dir/clean:
	cd /home/hakim/Desktop/ACADOtoolkit/build/examples && $(CMAKE_COMMAND) -P CMakeFiles/ocp_rocket.dir/cmake_clean.cmake
.PHONY : examples/CMakeFiles/ocp_rocket.dir/clean

examples/CMakeFiles/ocp_rocket.dir/depend:
	cd /home/hakim/Desktop/ACADOtoolkit/build && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/hakim/Desktop/ACADOtoolkit /home/hakim/Desktop/ACADOtoolkit/examples /home/hakim/Desktop/ACADOtoolkit/build /home/hakim/Desktop/ACADOtoolkit/build/examples /home/hakim/Desktop/ACADOtoolkit/build/examples/CMakeFiles/ocp_rocket.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : examples/CMakeFiles/ocp_rocket.dir/depend

