# CMAKE generated file: DO NOT EDIT!
# Generated by "Unix Makefiles" Generator, CMake Version 3.13

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
CMAKE_COMMAND = /opt/clion-2019.1/bin/cmake/linux/bin/cmake

# The command to remove a file.
RM = /opt/clion-2019.1/bin/cmake/linux/bin/cmake -E remove -f

# Escaping for special characters.
EQUALS = =

# The top-level source directory on which CMake was run.
CMAKE_SOURCE_DIR = /home/gokul/Robot_Ik_trajectory_tracking

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /home/gokul/Robot_Ik_trajectory_tracking/cmake-build-debug

# Include any dependencies generated for this target.
include CMakeFiles/Robot.dir/depend.make

# Include the progress variables for this target.
include CMakeFiles/Robot.dir/progress.make

# Include the compile flags for this target's objects.
include CMakeFiles/Robot.dir/flags.make

CMakeFiles/Robot.dir/src/Robot.cpp.o: CMakeFiles/Robot.dir/flags.make
CMakeFiles/Robot.dir/src/Robot.cpp.o: ../src/Robot.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/gokul/Robot_Ik_trajectory_tracking/cmake-build-debug/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building CXX object CMakeFiles/Robot.dir/src/Robot.cpp.o"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/Robot.dir/src/Robot.cpp.o -c /home/gokul/Robot_Ik_trajectory_tracking/src/Robot.cpp

CMakeFiles/Robot.dir/src/Robot.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/Robot.dir/src/Robot.cpp.i"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/gokul/Robot_Ik_trajectory_tracking/src/Robot.cpp > CMakeFiles/Robot.dir/src/Robot.cpp.i

CMakeFiles/Robot.dir/src/Robot.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/Robot.dir/src/Robot.cpp.s"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/gokul/Robot_Ik_trajectory_tracking/src/Robot.cpp -o CMakeFiles/Robot.dir/src/Robot.cpp.s

CMakeFiles/Robot.dir/src/main.cpp.o: CMakeFiles/Robot.dir/flags.make
CMakeFiles/Robot.dir/src/main.cpp.o: ../src/main.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/gokul/Robot_Ik_trajectory_tracking/cmake-build-debug/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Building CXX object CMakeFiles/Robot.dir/src/main.cpp.o"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/Robot.dir/src/main.cpp.o -c /home/gokul/Robot_Ik_trajectory_tracking/src/main.cpp

CMakeFiles/Robot.dir/src/main.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/Robot.dir/src/main.cpp.i"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/gokul/Robot_Ik_trajectory_tracking/src/main.cpp > CMakeFiles/Robot.dir/src/main.cpp.i

CMakeFiles/Robot.dir/src/main.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/Robot.dir/src/main.cpp.s"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/gokul/Robot_Ik_trajectory_tracking/src/main.cpp -o CMakeFiles/Robot.dir/src/main.cpp.s

CMakeFiles/Robot.dir/src/connection.cpp.o: CMakeFiles/Robot.dir/flags.make
CMakeFiles/Robot.dir/src/connection.cpp.o: ../src/connection.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/gokul/Robot_Ik_trajectory_tracking/cmake-build-debug/CMakeFiles --progress-num=$(CMAKE_PROGRESS_3) "Building CXX object CMakeFiles/Robot.dir/src/connection.cpp.o"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/Robot.dir/src/connection.cpp.o -c /home/gokul/Robot_Ik_trajectory_tracking/src/connection.cpp

CMakeFiles/Robot.dir/src/connection.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/Robot.dir/src/connection.cpp.i"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/gokul/Robot_Ik_trajectory_tracking/src/connection.cpp > CMakeFiles/Robot.dir/src/connection.cpp.i

CMakeFiles/Robot.dir/src/connection.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/Robot.dir/src/connection.cpp.s"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/gokul/Robot_Ik_trajectory_tracking/src/connection.cpp -o CMakeFiles/Robot.dir/src/connection.cpp.s

# Object files for target Robot
Robot_OBJECTS = \
"CMakeFiles/Robot.dir/src/Robot.cpp.o" \
"CMakeFiles/Robot.dir/src/main.cpp.o" \
"CMakeFiles/Robot.dir/src/connection.cpp.o"

# External object files for target Robot
Robot_EXTERNAL_OBJECTS =

Robot: CMakeFiles/Robot.dir/src/Robot.cpp.o
Robot: CMakeFiles/Robot.dir/src/main.cpp.o
Robot: CMakeFiles/Robot.dir/src/connection.cpp.o
Robot: CMakeFiles/Robot.dir/build.make
Robot: CMakeFiles/Robot.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir=/home/gokul/Robot_Ik_trajectory_tracking/cmake-build-debug/CMakeFiles --progress-num=$(CMAKE_PROGRESS_4) "Linking CXX executable Robot"
	$(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/Robot.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
CMakeFiles/Robot.dir/build: Robot

.PHONY : CMakeFiles/Robot.dir/build

CMakeFiles/Robot.dir/clean:
	$(CMAKE_COMMAND) -P CMakeFiles/Robot.dir/cmake_clean.cmake
.PHONY : CMakeFiles/Robot.dir/clean

CMakeFiles/Robot.dir/depend:
	cd /home/gokul/Robot_Ik_trajectory_tracking/cmake-build-debug && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/gokul/Robot_Ik_trajectory_tracking /home/gokul/Robot_Ik_trajectory_tracking /home/gokul/Robot_Ik_trajectory_tracking/cmake-build-debug /home/gokul/Robot_Ik_trajectory_tracking/cmake-build-debug /home/gokul/Robot_Ik_trajectory_tracking/cmake-build-debug/CMakeFiles/Robot.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : CMakeFiles/Robot.dir/depend

