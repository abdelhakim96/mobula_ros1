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
include examples/CMakeFiles/controller_getting_started_classical.dir/depend.make

# Include the progress variables for this target.
include examples/CMakeFiles/controller_getting_started_classical.dir/progress.make

# Include the compile flags for this target's objects.
include examples/CMakeFiles/controller_getting_started_classical.dir/flags.make

examples/CMakeFiles/controller_getting_started_classical.dir/controller/getting_started_classical.cpp.o: examples/CMakeFiles/controller_getting_started_classical.dir/flags.make
examples/CMakeFiles/controller_getting_started_classical.dir/controller/getting_started_classical.cpp.o: ../examples/controller/getting_started_classical.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/hakim/Desktop/ACADOtoolkit/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building CXX object examples/CMakeFiles/controller_getting_started_classical.dir/controller/getting_started_classical.cpp.o"
	cd /home/hakim/Desktop/ACADOtoolkit/build/examples && /usr/lib/ccache/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/controller_getting_started_classical.dir/controller/getting_started_classical.cpp.o -c /home/hakim/Desktop/ACADOtoolkit/examples/controller/getting_started_classical.cpp

examples/CMakeFiles/controller_getting_started_classical.dir/controller/getting_started_classical.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/controller_getting_started_classical.dir/controller/getting_started_classical.cpp.i"
	cd /home/hakim/Desktop/ACADOtoolkit/build/examples && /usr/lib/ccache/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/hakim/Desktop/ACADOtoolkit/examples/controller/getting_started_classical.cpp > CMakeFiles/controller_getting_started_classical.dir/controller/getting_started_classical.cpp.i

examples/CMakeFiles/controller_getting_started_classical.dir/controller/getting_started_classical.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/controller_getting_started_classical.dir/controller/getting_started_classical.cpp.s"
	cd /home/hakim/Desktop/ACADOtoolkit/build/examples && /usr/lib/ccache/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/hakim/Desktop/ACADOtoolkit/examples/controller/getting_started_classical.cpp -o CMakeFiles/controller_getting_started_classical.dir/controller/getting_started_classical.cpp.s

examples/CMakeFiles/controller_getting_started_classical.dir/controller/getting_started_classical.cpp.o.requires:

.PHONY : examples/CMakeFiles/controller_getting_started_classical.dir/controller/getting_started_classical.cpp.o.requires

examples/CMakeFiles/controller_getting_started_classical.dir/controller/getting_started_classical.cpp.o.provides: examples/CMakeFiles/controller_getting_started_classical.dir/controller/getting_started_classical.cpp.o.requires
	$(MAKE) -f examples/CMakeFiles/controller_getting_started_classical.dir/build.make examples/CMakeFiles/controller_getting_started_classical.dir/controller/getting_started_classical.cpp.o.provides.build
.PHONY : examples/CMakeFiles/controller_getting_started_classical.dir/controller/getting_started_classical.cpp.o.provides

examples/CMakeFiles/controller_getting_started_classical.dir/controller/getting_started_classical.cpp.o.provides.build: examples/CMakeFiles/controller_getting_started_classical.dir/controller/getting_started_classical.cpp.o


# Object files for target controller_getting_started_classical
controller_getting_started_classical_OBJECTS = \
"CMakeFiles/controller_getting_started_classical.dir/controller/getting_started_classical.cpp.o"

# External object files for target controller_getting_started_classical
controller_getting_started_classical_EXTERNAL_OBJECTS =

../examples/controller/getting_started_classical: examples/CMakeFiles/controller_getting_started_classical.dir/controller/getting_started_classical.cpp.o
../examples/controller/getting_started_classical: examples/CMakeFiles/controller_getting_started_classical.dir/build.make
../examples/controller/getting_started_classical: lib/libacado_toolkit_s.so.1.2.2beta
../examples/controller/getting_started_classical: lib/libacado_casadi.a
../examples/controller/getting_started_classical: lib/libacado_qpoases.a
../examples/controller/getting_started_classical: lib/libacado_csparse.a
../examples/controller/getting_started_classical: examples/CMakeFiles/controller_getting_started_classical.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir=/home/hakim/Desktop/ACADOtoolkit/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Linking CXX executable ../../examples/controller/getting_started_classical"
	cd /home/hakim/Desktop/ACADOtoolkit/build/examples && $(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/controller_getting_started_classical.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
examples/CMakeFiles/controller_getting_started_classical.dir/build: ../examples/controller/getting_started_classical

.PHONY : examples/CMakeFiles/controller_getting_started_classical.dir/build

examples/CMakeFiles/controller_getting_started_classical.dir/requires: examples/CMakeFiles/controller_getting_started_classical.dir/controller/getting_started_classical.cpp.o.requires

.PHONY : examples/CMakeFiles/controller_getting_started_classical.dir/requires

examples/CMakeFiles/controller_getting_started_classical.dir/clean:
	cd /home/hakim/Desktop/ACADOtoolkit/build/examples && $(CMAKE_COMMAND) -P CMakeFiles/controller_getting_started_classical.dir/cmake_clean.cmake
.PHONY : examples/CMakeFiles/controller_getting_started_classical.dir/clean

examples/CMakeFiles/controller_getting_started_classical.dir/depend:
	cd /home/hakim/Desktop/ACADOtoolkit/build && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/hakim/Desktop/ACADOtoolkit /home/hakim/Desktop/ACADOtoolkit/examples /home/hakim/Desktop/ACADOtoolkit/build /home/hakim/Desktop/ACADOtoolkit/build/examples /home/hakim/Desktop/ACADOtoolkit/build/examples/CMakeFiles/controller_getting_started_classical.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : examples/CMakeFiles/controller_getting_started_classical.dir/depend

