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
CMAKE_SOURCE_DIR = /home/cscarbone/SwarmSimulators/01_UAVswarmInspectionSimulator/release

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /home/cscarbone/SwarmSimulators/01_UAVswarmInspectionSimulator/release/build

# Include any dependencies generated for this target.
include CMakeFiles/MACPP.dir/depend.make

# Include the progress variables for this target.
include CMakeFiles/MACPP.dir/progress.make

# Include the compile flags for this target's objects.
include CMakeFiles/MACPP.dir/flags.make

CMakeFiles/MACPP.dir/src/macpp.cpp.o: CMakeFiles/MACPP.dir/flags.make
CMakeFiles/MACPP.dir/src/macpp.cpp.o: ../src/macpp.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/cscarbone/SwarmSimulators/01_UAVswarmInspectionSimulator/release/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building CXX object CMakeFiles/MACPP.dir/src/macpp.cpp.o"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/MACPP.dir/src/macpp.cpp.o -c /home/cscarbone/SwarmSimulators/01_UAVswarmInspectionSimulator/release/src/macpp.cpp

CMakeFiles/MACPP.dir/src/macpp.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/MACPP.dir/src/macpp.cpp.i"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/cscarbone/SwarmSimulators/01_UAVswarmInspectionSimulator/release/src/macpp.cpp > CMakeFiles/MACPP.dir/src/macpp.cpp.i

CMakeFiles/MACPP.dir/src/macpp.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/MACPP.dir/src/macpp.cpp.s"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/cscarbone/SwarmSimulators/01_UAVswarmInspectionSimulator/release/src/macpp.cpp -o CMakeFiles/MACPP.dir/src/macpp.cpp.s

CMakeFiles/MACPP.dir/src/macpp.cpp.o.requires:

.PHONY : CMakeFiles/MACPP.dir/src/macpp.cpp.o.requires

CMakeFiles/MACPP.dir/src/macpp.cpp.o.provides: CMakeFiles/MACPP.dir/src/macpp.cpp.o.requires
	$(MAKE) -f CMakeFiles/MACPP.dir/build.make CMakeFiles/MACPP.dir/src/macpp.cpp.o.provides.build
.PHONY : CMakeFiles/MACPP.dir/src/macpp.cpp.o.provides

CMakeFiles/MACPP.dir/src/macpp.cpp.o.provides.build: CMakeFiles/MACPP.dir/src/macpp.cpp.o


CMakeFiles/MACPP.dir/src/world.cpp.o: CMakeFiles/MACPP.dir/flags.make
CMakeFiles/MACPP.dir/src/world.cpp.o: ../src/world.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/cscarbone/SwarmSimulators/01_UAVswarmInspectionSimulator/release/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Building CXX object CMakeFiles/MACPP.dir/src/world.cpp.o"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/MACPP.dir/src/world.cpp.o -c /home/cscarbone/SwarmSimulators/01_UAVswarmInspectionSimulator/release/src/world.cpp

CMakeFiles/MACPP.dir/src/world.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/MACPP.dir/src/world.cpp.i"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/cscarbone/SwarmSimulators/01_UAVswarmInspectionSimulator/release/src/world.cpp > CMakeFiles/MACPP.dir/src/world.cpp.i

CMakeFiles/MACPP.dir/src/world.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/MACPP.dir/src/world.cpp.s"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/cscarbone/SwarmSimulators/01_UAVswarmInspectionSimulator/release/src/world.cpp -o CMakeFiles/MACPP.dir/src/world.cpp.s

CMakeFiles/MACPP.dir/src/world.cpp.o.requires:

.PHONY : CMakeFiles/MACPP.dir/src/world.cpp.o.requires

CMakeFiles/MACPP.dir/src/world.cpp.o.provides: CMakeFiles/MACPP.dir/src/world.cpp.o.requires
	$(MAKE) -f CMakeFiles/MACPP.dir/build.make CMakeFiles/MACPP.dir/src/world.cpp.o.provides.build
.PHONY : CMakeFiles/MACPP.dir/src/world.cpp.o.provides

CMakeFiles/MACPP.dir/src/world.cpp.o.provides.build: CMakeFiles/MACPP.dir/src/world.cpp.o


CMakeFiles/MACPP.dir/src/graphics/VertexShader.cpp.o: CMakeFiles/MACPP.dir/flags.make
CMakeFiles/MACPP.dir/src/graphics/VertexShader.cpp.o: ../src/graphics/VertexShader.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/cscarbone/SwarmSimulators/01_UAVswarmInspectionSimulator/release/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_3) "Building CXX object CMakeFiles/MACPP.dir/src/graphics/VertexShader.cpp.o"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/MACPP.dir/src/graphics/VertexShader.cpp.o -c /home/cscarbone/SwarmSimulators/01_UAVswarmInspectionSimulator/release/src/graphics/VertexShader.cpp

CMakeFiles/MACPP.dir/src/graphics/VertexShader.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/MACPP.dir/src/graphics/VertexShader.cpp.i"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/cscarbone/SwarmSimulators/01_UAVswarmInspectionSimulator/release/src/graphics/VertexShader.cpp > CMakeFiles/MACPP.dir/src/graphics/VertexShader.cpp.i

CMakeFiles/MACPP.dir/src/graphics/VertexShader.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/MACPP.dir/src/graphics/VertexShader.cpp.s"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/cscarbone/SwarmSimulators/01_UAVswarmInspectionSimulator/release/src/graphics/VertexShader.cpp -o CMakeFiles/MACPP.dir/src/graphics/VertexShader.cpp.s

CMakeFiles/MACPP.dir/src/graphics/VertexShader.cpp.o.requires:

.PHONY : CMakeFiles/MACPP.dir/src/graphics/VertexShader.cpp.o.requires

CMakeFiles/MACPP.dir/src/graphics/VertexShader.cpp.o.provides: CMakeFiles/MACPP.dir/src/graphics/VertexShader.cpp.o.requires
	$(MAKE) -f CMakeFiles/MACPP.dir/build.make CMakeFiles/MACPP.dir/src/graphics/VertexShader.cpp.o.provides.build
.PHONY : CMakeFiles/MACPP.dir/src/graphics/VertexShader.cpp.o.provides

CMakeFiles/MACPP.dir/src/graphics/VertexShader.cpp.o.provides.build: CMakeFiles/MACPP.dir/src/graphics/VertexShader.cpp.o


CMakeFiles/MACPP.dir/src/agent.cpp.o: CMakeFiles/MACPP.dir/flags.make
CMakeFiles/MACPP.dir/src/agent.cpp.o: ../src/agent.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/cscarbone/SwarmSimulators/01_UAVswarmInspectionSimulator/release/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_4) "Building CXX object CMakeFiles/MACPP.dir/src/agent.cpp.o"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/MACPP.dir/src/agent.cpp.o -c /home/cscarbone/SwarmSimulators/01_UAVswarmInspectionSimulator/release/src/agent.cpp

CMakeFiles/MACPP.dir/src/agent.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/MACPP.dir/src/agent.cpp.i"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/cscarbone/SwarmSimulators/01_UAVswarmInspectionSimulator/release/src/agent.cpp > CMakeFiles/MACPP.dir/src/agent.cpp.i

CMakeFiles/MACPP.dir/src/agent.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/MACPP.dir/src/agent.cpp.s"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/cscarbone/SwarmSimulators/01_UAVswarmInspectionSimulator/release/src/agent.cpp -o CMakeFiles/MACPP.dir/src/agent.cpp.s

CMakeFiles/MACPP.dir/src/agent.cpp.o.requires:

.PHONY : CMakeFiles/MACPP.dir/src/agent.cpp.o.requires

CMakeFiles/MACPP.dir/src/agent.cpp.o.provides: CMakeFiles/MACPP.dir/src/agent.cpp.o.requires
	$(MAKE) -f CMakeFiles/MACPP.dir/build.make CMakeFiles/MACPP.dir/src/agent.cpp.o.provides.build
.PHONY : CMakeFiles/MACPP.dir/src/agent.cpp.o.provides

CMakeFiles/MACPP.dir/src/agent.cpp.o.provides.build: CMakeFiles/MACPP.dir/src/agent.cpp.o


CMakeFiles/MACPP.dir/src/randomwalk.cpp.o: CMakeFiles/MACPP.dir/flags.make
CMakeFiles/MACPP.dir/src/randomwalk.cpp.o: ../src/randomwalk.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/cscarbone/SwarmSimulators/01_UAVswarmInspectionSimulator/release/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_5) "Building CXX object CMakeFiles/MACPP.dir/src/randomwalk.cpp.o"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/MACPP.dir/src/randomwalk.cpp.o -c /home/cscarbone/SwarmSimulators/01_UAVswarmInspectionSimulator/release/src/randomwalk.cpp

CMakeFiles/MACPP.dir/src/randomwalk.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/MACPP.dir/src/randomwalk.cpp.i"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/cscarbone/SwarmSimulators/01_UAVswarmInspectionSimulator/release/src/randomwalk.cpp > CMakeFiles/MACPP.dir/src/randomwalk.cpp.i

CMakeFiles/MACPP.dir/src/randomwalk.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/MACPP.dir/src/randomwalk.cpp.s"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/cscarbone/SwarmSimulators/01_UAVswarmInspectionSimulator/release/src/randomwalk.cpp -o CMakeFiles/MACPP.dir/src/randomwalk.cpp.s

CMakeFiles/MACPP.dir/src/randomwalk.cpp.o.requires:

.PHONY : CMakeFiles/MACPP.dir/src/randomwalk.cpp.o.requires

CMakeFiles/MACPP.dir/src/randomwalk.cpp.o.provides: CMakeFiles/MACPP.dir/src/randomwalk.cpp.o.requires
	$(MAKE) -f CMakeFiles/MACPP.dir/build.make CMakeFiles/MACPP.dir/src/randomwalk.cpp.o.provides.build
.PHONY : CMakeFiles/MACPP.dir/src/randomwalk.cpp.o.provides

CMakeFiles/MACPP.dir/src/randomwalk.cpp.o.provides.build: CMakeFiles/MACPP.dir/src/randomwalk.cpp.o


CMakeFiles/MACPP.dir/src/informationGain.cpp.o: CMakeFiles/MACPP.dir/flags.make
CMakeFiles/MACPP.dir/src/informationGain.cpp.o: ../src/informationGain.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/cscarbone/SwarmSimulators/01_UAVswarmInspectionSimulator/release/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_6) "Building CXX object CMakeFiles/MACPP.dir/src/informationGain.cpp.o"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/MACPP.dir/src/informationGain.cpp.o -c /home/cscarbone/SwarmSimulators/01_UAVswarmInspectionSimulator/release/src/informationGain.cpp

CMakeFiles/MACPP.dir/src/informationGain.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/MACPP.dir/src/informationGain.cpp.i"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/cscarbone/SwarmSimulators/01_UAVswarmInspectionSimulator/release/src/informationGain.cpp > CMakeFiles/MACPP.dir/src/informationGain.cpp.i

CMakeFiles/MACPP.dir/src/informationGain.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/MACPP.dir/src/informationGain.cpp.s"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/cscarbone/SwarmSimulators/01_UAVswarmInspectionSimulator/release/src/informationGain.cpp -o CMakeFiles/MACPP.dir/src/informationGain.cpp.s

CMakeFiles/MACPP.dir/src/informationGain.cpp.o.requires:

.PHONY : CMakeFiles/MACPP.dir/src/informationGain.cpp.o.requires

CMakeFiles/MACPP.dir/src/informationGain.cpp.o.provides: CMakeFiles/MACPP.dir/src/informationGain.cpp.o.requires
	$(MAKE) -f CMakeFiles/MACPP.dir/build.make CMakeFiles/MACPP.dir/src/informationGain.cpp.o.provides.build
.PHONY : CMakeFiles/MACPP.dir/src/informationGain.cpp.o.provides

CMakeFiles/MACPP.dir/src/informationGain.cpp.o.provides.build: CMakeFiles/MACPP.dir/src/informationGain.cpp.o


CMakeFiles/MACPP.dir/src/orca.cpp.o: CMakeFiles/MACPP.dir/flags.make
CMakeFiles/MACPP.dir/src/orca.cpp.o: ../src/orca.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/cscarbone/SwarmSimulators/01_UAVswarmInspectionSimulator/release/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_7) "Building CXX object CMakeFiles/MACPP.dir/src/orca.cpp.o"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/MACPP.dir/src/orca.cpp.o -c /home/cscarbone/SwarmSimulators/01_UAVswarmInspectionSimulator/release/src/orca.cpp

CMakeFiles/MACPP.dir/src/orca.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/MACPP.dir/src/orca.cpp.i"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/cscarbone/SwarmSimulators/01_UAVswarmInspectionSimulator/release/src/orca.cpp > CMakeFiles/MACPP.dir/src/orca.cpp.i

CMakeFiles/MACPP.dir/src/orca.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/MACPP.dir/src/orca.cpp.s"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/cscarbone/SwarmSimulators/01_UAVswarmInspectionSimulator/release/src/orca.cpp -o CMakeFiles/MACPP.dir/src/orca.cpp.s

CMakeFiles/MACPP.dir/src/orca.cpp.o.requires:

.PHONY : CMakeFiles/MACPP.dir/src/orca.cpp.o.requires

CMakeFiles/MACPP.dir/src/orca.cpp.o.provides: CMakeFiles/MACPP.dir/src/orca.cpp.o.requires
	$(MAKE) -f CMakeFiles/MACPP.dir/build.make CMakeFiles/MACPP.dir/src/orca.cpp.o.provides.build
.PHONY : CMakeFiles/MACPP.dir/src/orca.cpp.o.provides

CMakeFiles/MACPP.dir/src/orca.cpp.o.provides.build: CMakeFiles/MACPP.dir/src/orca.cpp.o


# Object files for target MACPP
MACPP_OBJECTS = \
"CMakeFiles/MACPP.dir/src/macpp.cpp.o" \
"CMakeFiles/MACPP.dir/src/world.cpp.o" \
"CMakeFiles/MACPP.dir/src/graphics/VertexShader.cpp.o" \
"CMakeFiles/MACPP.dir/src/agent.cpp.o" \
"CMakeFiles/MACPP.dir/src/randomwalk.cpp.o" \
"CMakeFiles/MACPP.dir/src/informationGain.cpp.o" \
"CMakeFiles/MACPP.dir/src/orca.cpp.o"

# External object files for target MACPP
MACPP_EXTERNAL_OBJECTS =

MACPP: CMakeFiles/MACPP.dir/src/macpp.cpp.o
MACPP: CMakeFiles/MACPP.dir/src/world.cpp.o
MACPP: CMakeFiles/MACPP.dir/src/graphics/VertexShader.cpp.o
MACPP: CMakeFiles/MACPP.dir/src/agent.cpp.o
MACPP: CMakeFiles/MACPP.dir/src/randomwalk.cpp.o
MACPP: CMakeFiles/MACPP.dir/src/informationGain.cpp.o
MACPP: CMakeFiles/MACPP.dir/src/orca.cpp.o
MACPP: CMakeFiles/MACPP.dir/build.make
MACPP: libraries/glfw-3.3.2/src/libglfw3.a
MACPP: lib/libGLEW.so.2.2.0
MACPP: /usr/lib/x86_64-linux-gnu/libboost_program_options.so
MACPP: /usr/lib/x86_64-linux-gnu/librt.so
MACPP: /usr/lib/x86_64-linux-gnu/libm.so
MACPP: /usr/lib/x86_64-linux-gnu/libGL.so
MACPP: /usr/lib/x86_64-linux-gnu/libGLU.so
MACPP: /usr/lib/x86_64-linux-gnu/libSM.so
MACPP: /usr/lib/x86_64-linux-gnu/libICE.so
MACPP: /usr/lib/x86_64-linux-gnu/libX11.so
MACPP: /usr/lib/x86_64-linux-gnu/libXext.so
MACPP: CMakeFiles/MACPP.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir=/home/cscarbone/SwarmSimulators/01_UAVswarmInspectionSimulator/release/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_8) "Linking CXX executable MACPP"
	$(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/MACPP.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
CMakeFiles/MACPP.dir/build: MACPP

.PHONY : CMakeFiles/MACPP.dir/build

CMakeFiles/MACPP.dir/requires: CMakeFiles/MACPP.dir/src/macpp.cpp.o.requires
CMakeFiles/MACPP.dir/requires: CMakeFiles/MACPP.dir/src/world.cpp.o.requires
CMakeFiles/MACPP.dir/requires: CMakeFiles/MACPP.dir/src/graphics/VertexShader.cpp.o.requires
CMakeFiles/MACPP.dir/requires: CMakeFiles/MACPP.dir/src/agent.cpp.o.requires
CMakeFiles/MACPP.dir/requires: CMakeFiles/MACPP.dir/src/randomwalk.cpp.o.requires
CMakeFiles/MACPP.dir/requires: CMakeFiles/MACPP.dir/src/informationGain.cpp.o.requires
CMakeFiles/MACPP.dir/requires: CMakeFiles/MACPP.dir/src/orca.cpp.o.requires

.PHONY : CMakeFiles/MACPP.dir/requires

CMakeFiles/MACPP.dir/clean:
	$(CMAKE_COMMAND) -P CMakeFiles/MACPP.dir/cmake_clean.cmake
.PHONY : CMakeFiles/MACPP.dir/clean

CMakeFiles/MACPP.dir/depend:
	cd /home/cscarbone/SwarmSimulators/01_UAVswarmInspectionSimulator/release/build && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/cscarbone/SwarmSimulators/01_UAVswarmInspectionSimulator/release /home/cscarbone/SwarmSimulators/01_UAVswarmInspectionSimulator/release /home/cscarbone/SwarmSimulators/01_UAVswarmInspectionSimulator/release/build /home/cscarbone/SwarmSimulators/01_UAVswarmInspectionSimulator/release/build /home/cscarbone/SwarmSimulators/01_UAVswarmInspectionSimulator/release/build/CMakeFiles/MACPP.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : CMakeFiles/MACPP.dir/depend

