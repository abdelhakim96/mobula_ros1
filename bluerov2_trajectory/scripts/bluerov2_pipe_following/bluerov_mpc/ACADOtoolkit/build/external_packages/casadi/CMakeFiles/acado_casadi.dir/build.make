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
include external_packages/casadi/CMakeFiles/acado_casadi.dir/depend.make

# Include the progress variables for this target.
include external_packages/casadi/CMakeFiles/acado_casadi.dir/progress.make

# Include the compile flags for this target's objects.
include external_packages/casadi/CMakeFiles/acado_casadi.dir/flags.make

external_packages/casadi/CMakeFiles/acado_casadi.dir/symbolic/printable_object.cpp.o: external_packages/casadi/CMakeFiles/acado_casadi.dir/flags.make
external_packages/casadi/CMakeFiles/acado_casadi.dir/symbolic/printable_object.cpp.o: ../external_packages/casadi/symbolic/printable_object.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/hakim/Desktop/ACADOtoolkit/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building CXX object external_packages/casadi/CMakeFiles/acado_casadi.dir/symbolic/printable_object.cpp.o"
	cd /home/hakim/Desktop/ACADOtoolkit/build/external_packages/casadi && /usr/lib/ccache/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/acado_casadi.dir/symbolic/printable_object.cpp.o -c /home/hakim/Desktop/ACADOtoolkit/external_packages/casadi/symbolic/printable_object.cpp

external_packages/casadi/CMakeFiles/acado_casadi.dir/symbolic/printable_object.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/acado_casadi.dir/symbolic/printable_object.cpp.i"
	cd /home/hakim/Desktop/ACADOtoolkit/build/external_packages/casadi && /usr/lib/ccache/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/hakim/Desktop/ACADOtoolkit/external_packages/casadi/symbolic/printable_object.cpp > CMakeFiles/acado_casadi.dir/symbolic/printable_object.cpp.i

external_packages/casadi/CMakeFiles/acado_casadi.dir/symbolic/printable_object.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/acado_casadi.dir/symbolic/printable_object.cpp.s"
	cd /home/hakim/Desktop/ACADOtoolkit/build/external_packages/casadi && /usr/lib/ccache/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/hakim/Desktop/ACADOtoolkit/external_packages/casadi/symbolic/printable_object.cpp -o CMakeFiles/acado_casadi.dir/symbolic/printable_object.cpp.s

external_packages/casadi/CMakeFiles/acado_casadi.dir/symbolic/printable_object.cpp.o.requires:

.PHONY : external_packages/casadi/CMakeFiles/acado_casadi.dir/symbolic/printable_object.cpp.o.requires

external_packages/casadi/CMakeFiles/acado_casadi.dir/symbolic/printable_object.cpp.o.provides: external_packages/casadi/CMakeFiles/acado_casadi.dir/symbolic/printable_object.cpp.o.requires
	$(MAKE) -f external_packages/casadi/CMakeFiles/acado_casadi.dir/build.make external_packages/casadi/CMakeFiles/acado_casadi.dir/symbolic/printable_object.cpp.o.provides.build
.PHONY : external_packages/casadi/CMakeFiles/acado_casadi.dir/symbolic/printable_object.cpp.o.provides

external_packages/casadi/CMakeFiles/acado_casadi.dir/symbolic/printable_object.cpp.o.provides.build: external_packages/casadi/CMakeFiles/acado_casadi.dir/symbolic/printable_object.cpp.o


external_packages/casadi/CMakeFiles/acado_casadi.dir/symbolic/shared_object.cpp.o: external_packages/casadi/CMakeFiles/acado_casadi.dir/flags.make
external_packages/casadi/CMakeFiles/acado_casadi.dir/symbolic/shared_object.cpp.o: ../external_packages/casadi/symbolic/shared_object.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/hakim/Desktop/ACADOtoolkit/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Building CXX object external_packages/casadi/CMakeFiles/acado_casadi.dir/symbolic/shared_object.cpp.o"
	cd /home/hakim/Desktop/ACADOtoolkit/build/external_packages/casadi && /usr/lib/ccache/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/acado_casadi.dir/symbolic/shared_object.cpp.o -c /home/hakim/Desktop/ACADOtoolkit/external_packages/casadi/symbolic/shared_object.cpp

external_packages/casadi/CMakeFiles/acado_casadi.dir/symbolic/shared_object.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/acado_casadi.dir/symbolic/shared_object.cpp.i"
	cd /home/hakim/Desktop/ACADOtoolkit/build/external_packages/casadi && /usr/lib/ccache/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/hakim/Desktop/ACADOtoolkit/external_packages/casadi/symbolic/shared_object.cpp > CMakeFiles/acado_casadi.dir/symbolic/shared_object.cpp.i

external_packages/casadi/CMakeFiles/acado_casadi.dir/symbolic/shared_object.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/acado_casadi.dir/symbolic/shared_object.cpp.s"
	cd /home/hakim/Desktop/ACADOtoolkit/build/external_packages/casadi && /usr/lib/ccache/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/hakim/Desktop/ACADOtoolkit/external_packages/casadi/symbolic/shared_object.cpp -o CMakeFiles/acado_casadi.dir/symbolic/shared_object.cpp.s

external_packages/casadi/CMakeFiles/acado_casadi.dir/symbolic/shared_object.cpp.o.requires:

.PHONY : external_packages/casadi/CMakeFiles/acado_casadi.dir/symbolic/shared_object.cpp.o.requires

external_packages/casadi/CMakeFiles/acado_casadi.dir/symbolic/shared_object.cpp.o.provides: external_packages/casadi/CMakeFiles/acado_casadi.dir/symbolic/shared_object.cpp.o.requires
	$(MAKE) -f external_packages/casadi/CMakeFiles/acado_casadi.dir/build.make external_packages/casadi/CMakeFiles/acado_casadi.dir/symbolic/shared_object.cpp.o.provides.build
.PHONY : external_packages/casadi/CMakeFiles/acado_casadi.dir/symbolic/shared_object.cpp.o.provides

external_packages/casadi/CMakeFiles/acado_casadi.dir/symbolic/shared_object.cpp.o.provides.build: external_packages/casadi/CMakeFiles/acado_casadi.dir/symbolic/shared_object.cpp.o


# Object files for target acado_casadi
acado_casadi_OBJECTS = \
"CMakeFiles/acado_casadi.dir/symbolic/printable_object.cpp.o" \
"CMakeFiles/acado_casadi.dir/symbolic/shared_object.cpp.o"

# External object files for target acado_casadi
acado_casadi_EXTERNAL_OBJECTS =

lib/libacado_casadi.a: external_packages/casadi/CMakeFiles/acado_casadi.dir/symbolic/printable_object.cpp.o
lib/libacado_casadi.a: external_packages/casadi/CMakeFiles/acado_casadi.dir/symbolic/shared_object.cpp.o
lib/libacado_casadi.a: external_packages/casadi/CMakeFiles/acado_casadi.dir/build.make
lib/libacado_casadi.a: external_packages/casadi/CMakeFiles/acado_casadi.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir=/home/hakim/Desktop/ACADOtoolkit/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_3) "Linking CXX static library ../../lib/libacado_casadi.a"
	cd /home/hakim/Desktop/ACADOtoolkit/build/external_packages/casadi && $(CMAKE_COMMAND) -P CMakeFiles/acado_casadi.dir/cmake_clean_target.cmake
	cd /home/hakim/Desktop/ACADOtoolkit/build/external_packages/casadi && $(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/acado_casadi.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
external_packages/casadi/CMakeFiles/acado_casadi.dir/build: lib/libacado_casadi.a

.PHONY : external_packages/casadi/CMakeFiles/acado_casadi.dir/build

external_packages/casadi/CMakeFiles/acado_casadi.dir/requires: external_packages/casadi/CMakeFiles/acado_casadi.dir/symbolic/printable_object.cpp.o.requires
external_packages/casadi/CMakeFiles/acado_casadi.dir/requires: external_packages/casadi/CMakeFiles/acado_casadi.dir/symbolic/shared_object.cpp.o.requires

.PHONY : external_packages/casadi/CMakeFiles/acado_casadi.dir/requires

external_packages/casadi/CMakeFiles/acado_casadi.dir/clean:
	cd /home/hakim/Desktop/ACADOtoolkit/build/external_packages/casadi && $(CMAKE_COMMAND) -P CMakeFiles/acado_casadi.dir/cmake_clean.cmake
.PHONY : external_packages/casadi/CMakeFiles/acado_casadi.dir/clean

external_packages/casadi/CMakeFiles/acado_casadi.dir/depend:
	cd /home/hakim/Desktop/ACADOtoolkit/build && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/hakim/Desktop/ACADOtoolkit /home/hakim/Desktop/ACADOtoolkit/external_packages/casadi /home/hakim/Desktop/ACADOtoolkit/build /home/hakim/Desktop/ACADOtoolkit/build/external_packages/casadi /home/hakim/Desktop/ACADOtoolkit/build/external_packages/casadi/CMakeFiles/acado_casadi.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : external_packages/casadi/CMakeFiles/acado_casadi.dir/depend

