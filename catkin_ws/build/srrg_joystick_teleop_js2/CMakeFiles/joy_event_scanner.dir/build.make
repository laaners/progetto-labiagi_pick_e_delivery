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
CMAKE_SOURCE_DIR = /home/alessio/Desktop/progetto-labiagi/catkin_ws/src/srrg_joystick_teleop_js2

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /home/alessio/Desktop/progetto-labiagi/catkin_ws/build/srrg_joystick_teleop_js2

# Include any dependencies generated for this target.
include CMakeFiles/joy_event_scanner.dir/depend.make

# Include the progress variables for this target.
include CMakeFiles/joy_event_scanner.dir/progress.make

# Include the compile flags for this target's objects.
include CMakeFiles/joy_event_scanner.dir/flags.make

CMakeFiles/joy_event_scanner.dir/src/joy_event_scanner.cpp.o: CMakeFiles/joy_event_scanner.dir/flags.make
CMakeFiles/joy_event_scanner.dir/src/joy_event_scanner.cpp.o: /home/alessio/Desktop/progetto-labiagi/catkin_ws/src/srrg_joystick_teleop_js2/src/joy_event_scanner.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/alessio/Desktop/progetto-labiagi/catkin_ws/build/srrg_joystick_teleop_js2/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building CXX object CMakeFiles/joy_event_scanner.dir/src/joy_event_scanner.cpp.o"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/joy_event_scanner.dir/src/joy_event_scanner.cpp.o -c /home/alessio/Desktop/progetto-labiagi/catkin_ws/src/srrg_joystick_teleop_js2/src/joy_event_scanner.cpp

CMakeFiles/joy_event_scanner.dir/src/joy_event_scanner.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/joy_event_scanner.dir/src/joy_event_scanner.cpp.i"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/alessio/Desktop/progetto-labiagi/catkin_ws/src/srrg_joystick_teleop_js2/src/joy_event_scanner.cpp > CMakeFiles/joy_event_scanner.dir/src/joy_event_scanner.cpp.i

CMakeFiles/joy_event_scanner.dir/src/joy_event_scanner.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/joy_event_scanner.dir/src/joy_event_scanner.cpp.s"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/alessio/Desktop/progetto-labiagi/catkin_ws/src/srrg_joystick_teleop_js2/src/joy_event_scanner.cpp -o CMakeFiles/joy_event_scanner.dir/src/joy_event_scanner.cpp.s

CMakeFiles/joy_event_scanner.dir/src/joy_event_scanner.cpp.o.requires:

.PHONY : CMakeFiles/joy_event_scanner.dir/src/joy_event_scanner.cpp.o.requires

CMakeFiles/joy_event_scanner.dir/src/joy_event_scanner.cpp.o.provides: CMakeFiles/joy_event_scanner.dir/src/joy_event_scanner.cpp.o.requires
	$(MAKE) -f CMakeFiles/joy_event_scanner.dir/build.make CMakeFiles/joy_event_scanner.dir/src/joy_event_scanner.cpp.o.provides.build
.PHONY : CMakeFiles/joy_event_scanner.dir/src/joy_event_scanner.cpp.o.provides

CMakeFiles/joy_event_scanner.dir/src/joy_event_scanner.cpp.o.provides.build: CMakeFiles/joy_event_scanner.dir/src/joy_event_scanner.cpp.o


# Object files for target joy_event_scanner
joy_event_scanner_OBJECTS = \
"CMakeFiles/joy_event_scanner.dir/src/joy_event_scanner.cpp.o"

# External object files for target joy_event_scanner
joy_event_scanner_EXTERNAL_OBJECTS =

/home/alessio/Desktop/progetto-labiagi/catkin_ws/devel/.private/srrg_joystick_teleop_js2/lib/srrg_joystick_teleop_js2/joy_event_scanner: CMakeFiles/joy_event_scanner.dir/src/joy_event_scanner.cpp.o
/home/alessio/Desktop/progetto-labiagi/catkin_ws/devel/.private/srrg_joystick_teleop_js2/lib/srrg_joystick_teleop_js2/joy_event_scanner: CMakeFiles/joy_event_scanner.dir/build.make
/home/alessio/Desktop/progetto-labiagi/catkin_ws/devel/.private/srrg_joystick_teleop_js2/lib/srrg_joystick_teleop_js2/joy_event_scanner: CMakeFiles/joy_event_scanner.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir=/home/alessio/Desktop/progetto-labiagi/catkin_ws/build/srrg_joystick_teleop_js2/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Linking CXX executable /home/alessio/Desktop/progetto-labiagi/catkin_ws/devel/.private/srrg_joystick_teleop_js2/lib/srrg_joystick_teleop_js2/joy_event_scanner"
	$(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/joy_event_scanner.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
CMakeFiles/joy_event_scanner.dir/build: /home/alessio/Desktop/progetto-labiagi/catkin_ws/devel/.private/srrg_joystick_teleop_js2/lib/srrg_joystick_teleop_js2/joy_event_scanner

.PHONY : CMakeFiles/joy_event_scanner.dir/build

CMakeFiles/joy_event_scanner.dir/requires: CMakeFiles/joy_event_scanner.dir/src/joy_event_scanner.cpp.o.requires

.PHONY : CMakeFiles/joy_event_scanner.dir/requires

CMakeFiles/joy_event_scanner.dir/clean:
	$(CMAKE_COMMAND) -P CMakeFiles/joy_event_scanner.dir/cmake_clean.cmake
.PHONY : CMakeFiles/joy_event_scanner.dir/clean

CMakeFiles/joy_event_scanner.dir/depend:
	cd /home/alessio/Desktop/progetto-labiagi/catkin_ws/build/srrg_joystick_teleop_js2 && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/alessio/Desktop/progetto-labiagi/catkin_ws/src/srrg_joystick_teleop_js2 /home/alessio/Desktop/progetto-labiagi/catkin_ws/src/srrg_joystick_teleop_js2 /home/alessio/Desktop/progetto-labiagi/catkin_ws/build/srrg_joystick_teleop_js2 /home/alessio/Desktop/progetto-labiagi/catkin_ws/build/srrg_joystick_teleop_js2 /home/alessio/Desktop/progetto-labiagi/catkin_ws/build/srrg_joystick_teleop_js2/CMakeFiles/joy_event_scanner.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : CMakeFiles/joy_event_scanner.dir/depend

