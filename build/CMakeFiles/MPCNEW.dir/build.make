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
CMAKE_SOURCE_DIR = /home/ls/Project/CPlus/mpc

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /home/ls/Project/CPlus/mpc/build

# Include any dependencies generated for this target.
include CMakeFiles/MPCNEW.dir/depend.make

# Include the progress variables for this target.
include CMakeFiles/MPCNEW.dir/progress.make

# Include the compile flags for this target's objects.
include CMakeFiles/MPCNEW.dir/flags.make

CMakeFiles/MPCNEW.dir/main.cpp.o: CMakeFiles/MPCNEW.dir/flags.make
CMakeFiles/MPCNEW.dir/main.cpp.o: ../main.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/ls/Project/CPlus/mpc/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building CXX object CMakeFiles/MPCNEW.dir/main.cpp.o"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/MPCNEW.dir/main.cpp.o -c /home/ls/Project/CPlus/mpc/main.cpp

CMakeFiles/MPCNEW.dir/main.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/MPCNEW.dir/main.cpp.i"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/ls/Project/CPlus/mpc/main.cpp > CMakeFiles/MPCNEW.dir/main.cpp.i

CMakeFiles/MPCNEW.dir/main.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/MPCNEW.dir/main.cpp.s"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/ls/Project/CPlus/mpc/main.cpp -o CMakeFiles/MPCNEW.dir/main.cpp.s

CMakeFiles/MPCNEW.dir/src/CarModule.cpp.o: CMakeFiles/MPCNEW.dir/flags.make
CMakeFiles/MPCNEW.dir/src/CarModule.cpp.o: ../src/CarModule.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/ls/Project/CPlus/mpc/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Building CXX object CMakeFiles/MPCNEW.dir/src/CarModule.cpp.o"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/MPCNEW.dir/src/CarModule.cpp.o -c /home/ls/Project/CPlus/mpc/src/CarModule.cpp

CMakeFiles/MPCNEW.dir/src/CarModule.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/MPCNEW.dir/src/CarModule.cpp.i"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/ls/Project/CPlus/mpc/src/CarModule.cpp > CMakeFiles/MPCNEW.dir/src/CarModule.cpp.i

CMakeFiles/MPCNEW.dir/src/CarModule.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/MPCNEW.dir/src/CarModule.cpp.s"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/ls/Project/CPlus/mpc/src/CarModule.cpp -o CMakeFiles/MPCNEW.dir/src/CarModule.cpp.s

CMakeFiles/MPCNEW.dir/src/MPCPlan.cpp.o: CMakeFiles/MPCNEW.dir/flags.make
CMakeFiles/MPCNEW.dir/src/MPCPlan.cpp.o: ../src/MPCPlan.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/ls/Project/CPlus/mpc/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_3) "Building CXX object CMakeFiles/MPCNEW.dir/src/MPCPlan.cpp.o"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/MPCNEW.dir/src/MPCPlan.cpp.o -c /home/ls/Project/CPlus/mpc/src/MPCPlan.cpp

CMakeFiles/MPCNEW.dir/src/MPCPlan.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/MPCNEW.dir/src/MPCPlan.cpp.i"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/ls/Project/CPlus/mpc/src/MPCPlan.cpp > CMakeFiles/MPCNEW.dir/src/MPCPlan.cpp.i

CMakeFiles/MPCNEW.dir/src/MPCPlan.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/MPCNEW.dir/src/MPCPlan.cpp.s"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/ls/Project/CPlus/mpc/src/MPCPlan.cpp -o CMakeFiles/MPCNEW.dir/src/MPCPlan.cpp.s

CMakeFiles/MPCNEW.dir/src/RefWay.cpp.o: CMakeFiles/MPCNEW.dir/flags.make
CMakeFiles/MPCNEW.dir/src/RefWay.cpp.o: ../src/RefWay.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/ls/Project/CPlus/mpc/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_4) "Building CXX object CMakeFiles/MPCNEW.dir/src/RefWay.cpp.o"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/MPCNEW.dir/src/RefWay.cpp.o -c /home/ls/Project/CPlus/mpc/src/RefWay.cpp

CMakeFiles/MPCNEW.dir/src/RefWay.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/MPCNEW.dir/src/RefWay.cpp.i"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/ls/Project/CPlus/mpc/src/RefWay.cpp > CMakeFiles/MPCNEW.dir/src/RefWay.cpp.i

CMakeFiles/MPCNEW.dir/src/RefWay.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/MPCNEW.dir/src/RefWay.cpp.s"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/ls/Project/CPlus/mpc/src/RefWay.cpp -o CMakeFiles/MPCNEW.dir/src/RefWay.cpp.s

# Object files for target MPCNEW
MPCNEW_OBJECTS = \
"CMakeFiles/MPCNEW.dir/main.cpp.o" \
"CMakeFiles/MPCNEW.dir/src/CarModule.cpp.o" \
"CMakeFiles/MPCNEW.dir/src/MPCPlan.cpp.o" \
"CMakeFiles/MPCNEW.dir/src/RefWay.cpp.o"

# External object files for target MPCNEW
MPCNEW_EXTERNAL_OBJECTS =

MPCNEW: CMakeFiles/MPCNEW.dir/main.cpp.o
MPCNEW: CMakeFiles/MPCNEW.dir/src/CarModule.cpp.o
MPCNEW: CMakeFiles/MPCNEW.dir/src/MPCPlan.cpp.o
MPCNEW: CMakeFiles/MPCNEW.dir/src/RefWay.cpp.o
MPCNEW: CMakeFiles/MPCNEW.dir/build.make
MPCNEW: /usr/local/lib/libosqp.a
MPCNEW: /usr/local/lib/libOsqpEigen.so.0.7.0
MPCNEW: /usr/local/lib/libosqp.so
MPCNEW: CMakeFiles/MPCNEW.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir=/home/ls/Project/CPlus/mpc/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_5) "Linking CXX executable MPCNEW"
	$(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/MPCNEW.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
CMakeFiles/MPCNEW.dir/build: MPCNEW

.PHONY : CMakeFiles/MPCNEW.dir/build

CMakeFiles/MPCNEW.dir/clean:
	$(CMAKE_COMMAND) -P CMakeFiles/MPCNEW.dir/cmake_clean.cmake
.PHONY : CMakeFiles/MPCNEW.dir/clean

CMakeFiles/MPCNEW.dir/depend:
	cd /home/ls/Project/CPlus/mpc/build && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/ls/Project/CPlus/mpc /home/ls/Project/CPlus/mpc /home/ls/Project/CPlus/mpc/build /home/ls/Project/CPlus/mpc/build /home/ls/Project/CPlus/mpc/build/CMakeFiles/MPCNEW.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : CMakeFiles/MPCNEW.dir/depend

