# CMAKE generated file: DO NOT EDIT!
# Generated by "Unix Makefiles" Generator, CMake Version 3.22

# Delete rule output on recipe failure.
.DELETE_ON_ERROR:

#=============================================================================
# Special targets provided by cmake.

# Disable implicit rules so canonical targets will work.
.SUFFIXES:

# Disable VCS-based implicit rules.
% : %,v

# Disable VCS-based implicit rules.
% : RCS/%

# Disable VCS-based implicit rules.
% : RCS/%,v

# Disable VCS-based implicit rules.
% : SCCS/s.%

# Disable VCS-based implicit rules.
% : s.%

.SUFFIXES: .hpux_make_needs_suffix_list

# Command-line flag to silence nested $(MAKE).
$(VERBOSE)MAKESILENT = -s

#Suppress display of executed commands.
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
RM = /usr/bin/cmake -E rm -f

# Escaping for special characters.
EQUALS = =

# The top-level source directory on which CMake was run.
CMAKE_SOURCE_DIR = /home/udayton/Desktop/GPS-IMU/GPS-IMU/eigen-3.4.0

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /home/udayton/Desktop/GPS-IMU/GPS-IMU/eigen-3.4.0/build

# Include any dependencies generated for this target.
include doc/examples/CMakeFiles/Cwise_erf.dir/depend.make
# Include any dependencies generated by the compiler for this target.
include doc/examples/CMakeFiles/Cwise_erf.dir/compiler_depend.make

# Include the progress variables for this target.
include doc/examples/CMakeFiles/Cwise_erf.dir/progress.make

# Include the compile flags for this target's objects.
include doc/examples/CMakeFiles/Cwise_erf.dir/flags.make

doc/examples/CMakeFiles/Cwise_erf.dir/Cwise_erf.cpp.o: doc/examples/CMakeFiles/Cwise_erf.dir/flags.make
doc/examples/CMakeFiles/Cwise_erf.dir/Cwise_erf.cpp.o: ../doc/examples/Cwise_erf.cpp
doc/examples/CMakeFiles/Cwise_erf.dir/Cwise_erf.cpp.o: doc/examples/CMakeFiles/Cwise_erf.dir/compiler_depend.ts
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/udayton/Desktop/GPS-IMU/GPS-IMU/eigen-3.4.0/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building CXX object doc/examples/CMakeFiles/Cwise_erf.dir/Cwise_erf.cpp.o"
	cd /home/udayton/Desktop/GPS-IMU/GPS-IMU/eigen-3.4.0/build/doc/examples && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -MD -MT doc/examples/CMakeFiles/Cwise_erf.dir/Cwise_erf.cpp.o -MF CMakeFiles/Cwise_erf.dir/Cwise_erf.cpp.o.d -o CMakeFiles/Cwise_erf.dir/Cwise_erf.cpp.o -c /home/udayton/Desktop/GPS-IMU/GPS-IMU/eigen-3.4.0/doc/examples/Cwise_erf.cpp

doc/examples/CMakeFiles/Cwise_erf.dir/Cwise_erf.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/Cwise_erf.dir/Cwise_erf.cpp.i"
	cd /home/udayton/Desktop/GPS-IMU/GPS-IMU/eigen-3.4.0/build/doc/examples && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/udayton/Desktop/GPS-IMU/GPS-IMU/eigen-3.4.0/doc/examples/Cwise_erf.cpp > CMakeFiles/Cwise_erf.dir/Cwise_erf.cpp.i

doc/examples/CMakeFiles/Cwise_erf.dir/Cwise_erf.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/Cwise_erf.dir/Cwise_erf.cpp.s"
	cd /home/udayton/Desktop/GPS-IMU/GPS-IMU/eigen-3.4.0/build/doc/examples && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/udayton/Desktop/GPS-IMU/GPS-IMU/eigen-3.4.0/doc/examples/Cwise_erf.cpp -o CMakeFiles/Cwise_erf.dir/Cwise_erf.cpp.s

# Object files for target Cwise_erf
Cwise_erf_OBJECTS = \
"CMakeFiles/Cwise_erf.dir/Cwise_erf.cpp.o"

# External object files for target Cwise_erf
Cwise_erf_EXTERNAL_OBJECTS =

doc/examples/Cwise_erf: doc/examples/CMakeFiles/Cwise_erf.dir/Cwise_erf.cpp.o
doc/examples/Cwise_erf: doc/examples/CMakeFiles/Cwise_erf.dir/build.make
doc/examples/Cwise_erf: doc/examples/CMakeFiles/Cwise_erf.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir=/home/udayton/Desktop/GPS-IMU/GPS-IMU/eigen-3.4.0/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Linking CXX executable Cwise_erf"
	cd /home/udayton/Desktop/GPS-IMU/GPS-IMU/eigen-3.4.0/build/doc/examples && $(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/Cwise_erf.dir/link.txt --verbose=$(VERBOSE)
	cd /home/udayton/Desktop/GPS-IMU/GPS-IMU/eigen-3.4.0/build/doc/examples && ./Cwise_erf >/home/udayton/Desktop/GPS-IMU/GPS-IMU/eigen-3.4.0/build/doc/examples/Cwise_erf.out

# Rule to build all files generated by this target.
doc/examples/CMakeFiles/Cwise_erf.dir/build: doc/examples/Cwise_erf
.PHONY : doc/examples/CMakeFiles/Cwise_erf.dir/build

doc/examples/CMakeFiles/Cwise_erf.dir/clean:
	cd /home/udayton/Desktop/GPS-IMU/GPS-IMU/eigen-3.4.0/build/doc/examples && $(CMAKE_COMMAND) -P CMakeFiles/Cwise_erf.dir/cmake_clean.cmake
.PHONY : doc/examples/CMakeFiles/Cwise_erf.dir/clean

doc/examples/CMakeFiles/Cwise_erf.dir/depend:
	cd /home/udayton/Desktop/GPS-IMU/GPS-IMU/eigen-3.4.0/build && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/udayton/Desktop/GPS-IMU/GPS-IMU/eigen-3.4.0 /home/udayton/Desktop/GPS-IMU/GPS-IMU/eigen-3.4.0/doc/examples /home/udayton/Desktop/GPS-IMU/GPS-IMU/eigen-3.4.0/build /home/udayton/Desktop/GPS-IMU/GPS-IMU/eigen-3.4.0/build/doc/examples /home/udayton/Desktop/GPS-IMU/GPS-IMU/eigen-3.4.0/build/doc/examples/CMakeFiles/Cwise_erf.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : doc/examples/CMakeFiles/Cwise_erf.dir/depend

