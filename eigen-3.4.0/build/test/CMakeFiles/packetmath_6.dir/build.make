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
include test/CMakeFiles/packetmath_6.dir/depend.make
# Include any dependencies generated by the compiler for this target.
include test/CMakeFiles/packetmath_6.dir/compiler_depend.make

# Include the progress variables for this target.
include test/CMakeFiles/packetmath_6.dir/progress.make

# Include the compile flags for this target's objects.
include test/CMakeFiles/packetmath_6.dir/flags.make

test/CMakeFiles/packetmath_6.dir/packetmath.cpp.o: test/CMakeFiles/packetmath_6.dir/flags.make
test/CMakeFiles/packetmath_6.dir/packetmath.cpp.o: ../test/packetmath.cpp
test/CMakeFiles/packetmath_6.dir/packetmath.cpp.o: test/CMakeFiles/packetmath_6.dir/compiler_depend.ts
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/udayton/Desktop/GPS-IMU/GPS-IMU/eigen-3.4.0/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building CXX object test/CMakeFiles/packetmath_6.dir/packetmath.cpp.o"
	cd /home/udayton/Desktop/GPS-IMU/GPS-IMU/eigen-3.4.0/build/test && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -MD -MT test/CMakeFiles/packetmath_6.dir/packetmath.cpp.o -MF CMakeFiles/packetmath_6.dir/packetmath.cpp.o.d -o CMakeFiles/packetmath_6.dir/packetmath.cpp.o -c /home/udayton/Desktop/GPS-IMU/GPS-IMU/eigen-3.4.0/test/packetmath.cpp

test/CMakeFiles/packetmath_6.dir/packetmath.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/packetmath_6.dir/packetmath.cpp.i"
	cd /home/udayton/Desktop/GPS-IMU/GPS-IMU/eigen-3.4.0/build/test && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/udayton/Desktop/GPS-IMU/GPS-IMU/eigen-3.4.0/test/packetmath.cpp > CMakeFiles/packetmath_6.dir/packetmath.cpp.i

test/CMakeFiles/packetmath_6.dir/packetmath.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/packetmath_6.dir/packetmath.cpp.s"
	cd /home/udayton/Desktop/GPS-IMU/GPS-IMU/eigen-3.4.0/build/test && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/udayton/Desktop/GPS-IMU/GPS-IMU/eigen-3.4.0/test/packetmath.cpp -o CMakeFiles/packetmath_6.dir/packetmath.cpp.s

# Object files for target packetmath_6
packetmath_6_OBJECTS = \
"CMakeFiles/packetmath_6.dir/packetmath.cpp.o"

# External object files for target packetmath_6
packetmath_6_EXTERNAL_OBJECTS =

test/packetmath_6: test/CMakeFiles/packetmath_6.dir/packetmath.cpp.o
test/packetmath_6: test/CMakeFiles/packetmath_6.dir/build.make
test/packetmath_6: test/CMakeFiles/packetmath_6.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir=/home/udayton/Desktop/GPS-IMU/GPS-IMU/eigen-3.4.0/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Linking CXX executable packetmath_6"
	cd /home/udayton/Desktop/GPS-IMU/GPS-IMU/eigen-3.4.0/build/test && $(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/packetmath_6.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
test/CMakeFiles/packetmath_6.dir/build: test/packetmath_6
.PHONY : test/CMakeFiles/packetmath_6.dir/build

test/CMakeFiles/packetmath_6.dir/clean:
	cd /home/udayton/Desktop/GPS-IMU/GPS-IMU/eigen-3.4.0/build/test && $(CMAKE_COMMAND) -P CMakeFiles/packetmath_6.dir/cmake_clean.cmake
.PHONY : test/CMakeFiles/packetmath_6.dir/clean

test/CMakeFiles/packetmath_6.dir/depend:
	cd /home/udayton/Desktop/GPS-IMU/GPS-IMU/eigen-3.4.0/build && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/udayton/Desktop/GPS-IMU/GPS-IMU/eigen-3.4.0 /home/udayton/Desktop/GPS-IMU/GPS-IMU/eigen-3.4.0/test /home/udayton/Desktop/GPS-IMU/GPS-IMU/eigen-3.4.0/build /home/udayton/Desktop/GPS-IMU/GPS-IMU/eigen-3.4.0/build/test /home/udayton/Desktop/GPS-IMU/GPS-IMU/eigen-3.4.0/build/test/CMakeFiles/packetmath_6.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : test/CMakeFiles/packetmath_6.dir/depend

