# CMAKE generated file: DO NOT EDIT!
# Generated by "Unix Makefiles" Generator, CMake Version 3.16

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
CMAKE_SOURCE_DIR = /home/jackoozy/catkin_ws/src/selfie_drawing_robot

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /home/jackoozy/catkin_ws/src/selfie_drawing_robot/build

# Include any dependencies generated for this target.
include CMakeFiles/operation.dir/depend.make

# Include the progress variables for this target.
include CMakeFiles/operation.dir/progress.make

# Include the compile flags for this target's objects.
include CMakeFiles/operation.dir/flags.make

CMakeFiles/operation.dir/src/operations.cpp.o: CMakeFiles/operation.dir/flags.make
CMakeFiles/operation.dir/src/operations.cpp.o: ../src/operations.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/jackoozy/catkin_ws/src/selfie_drawing_robot/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building CXX object CMakeFiles/operation.dir/src/operations.cpp.o"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/operation.dir/src/operations.cpp.o -c /home/jackoozy/catkin_ws/src/selfie_drawing_robot/src/operations.cpp

CMakeFiles/operation.dir/src/operations.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/operation.dir/src/operations.cpp.i"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/jackoozy/catkin_ws/src/selfie_drawing_robot/src/operations.cpp > CMakeFiles/operation.dir/src/operations.cpp.i

CMakeFiles/operation.dir/src/operations.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/operation.dir/src/operations.cpp.s"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/jackoozy/catkin_ws/src/selfie_drawing_robot/src/operations.cpp -o CMakeFiles/operation.dir/src/operations.cpp.s

# Object files for target operation
operation_OBJECTS = \
"CMakeFiles/operation.dir/src/operations.cpp.o"

# External object files for target operation
operation_EXTERNAL_OBJECTS =

devel/lib/selfie_drawing_robot/operation: CMakeFiles/operation.dir/src/operations.cpp.o
devel/lib/selfie_drawing_robot/operation: CMakeFiles/operation.dir/build.make
devel/lib/selfie_drawing_robot/operation: CMakeFiles/operation.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir=/home/jackoozy/catkin_ws/src/selfie_drawing_robot/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Linking CXX executable devel/lib/selfie_drawing_robot/operation"
	$(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/operation.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
CMakeFiles/operation.dir/build: devel/lib/selfie_drawing_robot/operation

.PHONY : CMakeFiles/operation.dir/build

CMakeFiles/operation.dir/clean:
	$(CMAKE_COMMAND) -P CMakeFiles/operation.dir/cmake_clean.cmake
.PHONY : CMakeFiles/operation.dir/clean

CMakeFiles/operation.dir/depend:
	cd /home/jackoozy/catkin_ws/src/selfie_drawing_robot/build && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/jackoozy/catkin_ws/src/selfie_drawing_robot /home/jackoozy/catkin_ws/src/selfie_drawing_robot /home/jackoozy/catkin_ws/src/selfie_drawing_robot/build /home/jackoozy/catkin_ws/src/selfie_drawing_robot/build /home/jackoozy/catkin_ws/src/selfie_drawing_robot/build/CMakeFiles/operation.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : CMakeFiles/operation.dir/depend

