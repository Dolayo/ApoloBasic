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
CMAKE_SOURCE_DIR = /home/daniel/Desktop/ApoloBasic

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /home/daniel/Desktop/ApoloBasic/build

# Include any dependencies generated for this target.
include CMakeFiles/ApoloBasic.dir/depend.make

# Include the progress variables for this target.
include CMakeFiles/ApoloBasic.dir/progress.make

# Include the compile flags for this target's objects.
include CMakeFiles/ApoloBasic.dir/flags.make

CMakeFiles/ApoloBasic.dir/src/main.cpp.o: CMakeFiles/ApoloBasic.dir/flags.make
CMakeFiles/ApoloBasic.dir/src/main.cpp.o: ../src/main.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/daniel/Desktop/ApoloBasic/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building CXX object CMakeFiles/ApoloBasic.dir/src/main.cpp.o"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/ApoloBasic.dir/src/main.cpp.o -c /home/daniel/Desktop/ApoloBasic/src/main.cpp

CMakeFiles/ApoloBasic.dir/src/main.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/ApoloBasic.dir/src/main.cpp.i"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/daniel/Desktop/ApoloBasic/src/main.cpp > CMakeFiles/ApoloBasic.dir/src/main.cpp.i

CMakeFiles/ApoloBasic.dir/src/main.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/ApoloBasic.dir/src/main.cpp.s"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/daniel/Desktop/ApoloBasic/src/main.cpp -o CMakeFiles/ApoloBasic.dir/src/main.cpp.s

CMakeFiles/ApoloBasic.dir/src/main.cpp.o.requires:

.PHONY : CMakeFiles/ApoloBasic.dir/src/main.cpp.o.requires

CMakeFiles/ApoloBasic.dir/src/main.cpp.o.provides: CMakeFiles/ApoloBasic.dir/src/main.cpp.o.requires
	$(MAKE) -f CMakeFiles/ApoloBasic.dir/build.make CMakeFiles/ApoloBasic.dir/src/main.cpp.o.provides.build
.PHONY : CMakeFiles/ApoloBasic.dir/src/main.cpp.o.provides

CMakeFiles/ApoloBasic.dir/src/main.cpp.o.provides.build: CMakeFiles/ApoloBasic.dir/src/main.cpp.o


CMakeFiles/ApoloBasic.dir/src/mainWindow.cpp.o: CMakeFiles/ApoloBasic.dir/flags.make
CMakeFiles/ApoloBasic.dir/src/mainWindow.cpp.o: ../src/mainWindow.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/daniel/Desktop/ApoloBasic/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Building CXX object CMakeFiles/ApoloBasic.dir/src/mainWindow.cpp.o"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/ApoloBasic.dir/src/mainWindow.cpp.o -c /home/daniel/Desktop/ApoloBasic/src/mainWindow.cpp

CMakeFiles/ApoloBasic.dir/src/mainWindow.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/ApoloBasic.dir/src/mainWindow.cpp.i"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/daniel/Desktop/ApoloBasic/src/mainWindow.cpp > CMakeFiles/ApoloBasic.dir/src/mainWindow.cpp.i

CMakeFiles/ApoloBasic.dir/src/mainWindow.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/ApoloBasic.dir/src/mainWindow.cpp.s"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/daniel/Desktop/ApoloBasic/src/mainWindow.cpp -o CMakeFiles/ApoloBasic.dir/src/mainWindow.cpp.s

CMakeFiles/ApoloBasic.dir/src/mainWindow.cpp.o.requires:

.PHONY : CMakeFiles/ApoloBasic.dir/src/mainWindow.cpp.o.requires

CMakeFiles/ApoloBasic.dir/src/mainWindow.cpp.o.provides: CMakeFiles/ApoloBasic.dir/src/mainWindow.cpp.o.requires
	$(MAKE) -f CMakeFiles/ApoloBasic.dir/build.make CMakeFiles/ApoloBasic.dir/src/mainWindow.cpp.o.provides.build
.PHONY : CMakeFiles/ApoloBasic.dir/src/mainWindow.cpp.o.provides

CMakeFiles/ApoloBasic.dir/src/mainWindow.cpp.o.provides.build: CMakeFiles/ApoloBasic.dir/src/mainWindow.cpp.o


CMakeFiles/ApoloBasic.dir/src/canvas.cpp.o: CMakeFiles/ApoloBasic.dir/flags.make
CMakeFiles/ApoloBasic.dir/src/canvas.cpp.o: ../src/canvas.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/daniel/Desktop/ApoloBasic/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_3) "Building CXX object CMakeFiles/ApoloBasic.dir/src/canvas.cpp.o"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/ApoloBasic.dir/src/canvas.cpp.o -c /home/daniel/Desktop/ApoloBasic/src/canvas.cpp

CMakeFiles/ApoloBasic.dir/src/canvas.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/ApoloBasic.dir/src/canvas.cpp.i"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/daniel/Desktop/ApoloBasic/src/canvas.cpp > CMakeFiles/ApoloBasic.dir/src/canvas.cpp.i

CMakeFiles/ApoloBasic.dir/src/canvas.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/ApoloBasic.dir/src/canvas.cpp.s"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/daniel/Desktop/ApoloBasic/src/canvas.cpp -o CMakeFiles/ApoloBasic.dir/src/canvas.cpp.s

CMakeFiles/ApoloBasic.dir/src/canvas.cpp.o.requires:

.PHONY : CMakeFiles/ApoloBasic.dir/src/canvas.cpp.o.requires

CMakeFiles/ApoloBasic.dir/src/canvas.cpp.o.provides: CMakeFiles/ApoloBasic.dir/src/canvas.cpp.o.requires
	$(MAKE) -f CMakeFiles/ApoloBasic.dir/build.make CMakeFiles/ApoloBasic.dir/src/canvas.cpp.o.provides.build
.PHONY : CMakeFiles/ApoloBasic.dir/src/canvas.cpp.o.provides

CMakeFiles/ApoloBasic.dir/src/canvas.cpp.o.provides.build: CMakeFiles/ApoloBasic.dir/src/canvas.cpp.o


# Object files for target ApoloBasic
ApoloBasic_OBJECTS = \
"CMakeFiles/ApoloBasic.dir/src/main.cpp.o" \
"CMakeFiles/ApoloBasic.dir/src/mainWindow.cpp.o" \
"CMakeFiles/ApoloBasic.dir/src/canvas.cpp.o"

# External object files for target ApoloBasic
ApoloBasic_EXTERNAL_OBJECTS =

ApoloBasic: CMakeFiles/ApoloBasic.dir/src/main.cpp.o
ApoloBasic: CMakeFiles/ApoloBasic.dir/src/mainWindow.cpp.o
ApoloBasic: CMakeFiles/ApoloBasic.dir/src/canvas.cpp.o
ApoloBasic: CMakeFiles/ApoloBasic.dir/build.make
ApoloBasic: /usr/lib/x86_64-linux-gnu/libOpenGL.so
ApoloBasic: /usr/lib/x86_64-linux-gnu/libGLX.so
ApoloBasic: /usr/lib/x86_64-linux-gnu/libGLU.so
ApoloBasic: CMakeFiles/ApoloBasic.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir=/home/daniel/Desktop/ApoloBasic/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_4) "Linking CXX executable ApoloBasic"
	$(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/ApoloBasic.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
CMakeFiles/ApoloBasic.dir/build: ApoloBasic

.PHONY : CMakeFiles/ApoloBasic.dir/build

CMakeFiles/ApoloBasic.dir/requires: CMakeFiles/ApoloBasic.dir/src/main.cpp.o.requires
CMakeFiles/ApoloBasic.dir/requires: CMakeFiles/ApoloBasic.dir/src/mainWindow.cpp.o.requires
CMakeFiles/ApoloBasic.dir/requires: CMakeFiles/ApoloBasic.dir/src/canvas.cpp.o.requires

.PHONY : CMakeFiles/ApoloBasic.dir/requires

CMakeFiles/ApoloBasic.dir/clean:
	$(CMAKE_COMMAND) -P CMakeFiles/ApoloBasic.dir/cmake_clean.cmake
.PHONY : CMakeFiles/ApoloBasic.dir/clean

CMakeFiles/ApoloBasic.dir/depend:
	cd /home/daniel/Desktop/ApoloBasic/build && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/daniel/Desktop/ApoloBasic /home/daniel/Desktop/ApoloBasic /home/daniel/Desktop/ApoloBasic/build /home/daniel/Desktop/ApoloBasic/build /home/daniel/Desktop/ApoloBasic/build/CMakeFiles/ApoloBasic.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : CMakeFiles/ApoloBasic.dir/depend

