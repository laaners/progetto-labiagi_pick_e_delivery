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
CMAKE_SOURCE_DIR = /home/alessiohu/Desktop/progetto-labiagi/catkin_ws/src/srrg2_core/srrg2_core

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /home/alessiohu/Desktop/progetto-labiagi/catkin_ws/build/srrg2_core

# Include any dependencies generated for this target.
include src/examples/CMakeFiles/boss_example.dir/depend.make

# Include the progress variables for this target.
include src/examples/CMakeFiles/boss_example.dir/progress.make

# Include the compile flags for this target's objects.
include src/examples/CMakeFiles/boss_example.dir/flags.make

src/examples/CMakeFiles/boss_example.dir/boss_example.cpp.o: src/examples/CMakeFiles/boss_example.dir/flags.make
src/examples/CMakeFiles/boss_example.dir/boss_example.cpp.o: /home/alessiohu/Desktop/progetto-labiagi/catkin_ws/src/srrg2_core/srrg2_core/src/examples/boss_example.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/alessiohu/Desktop/progetto-labiagi/catkin_ws/build/srrg2_core/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building CXX object src/examples/CMakeFiles/boss_example.dir/boss_example.cpp.o"
	cd /home/alessiohu/Desktop/progetto-labiagi/catkin_ws/build/srrg2_core/src/examples && /usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/boss_example.dir/boss_example.cpp.o -c /home/alessiohu/Desktop/progetto-labiagi/catkin_ws/src/srrg2_core/srrg2_core/src/examples/boss_example.cpp

src/examples/CMakeFiles/boss_example.dir/boss_example.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/boss_example.dir/boss_example.cpp.i"
	cd /home/alessiohu/Desktop/progetto-labiagi/catkin_ws/build/srrg2_core/src/examples && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/alessiohu/Desktop/progetto-labiagi/catkin_ws/src/srrg2_core/srrg2_core/src/examples/boss_example.cpp > CMakeFiles/boss_example.dir/boss_example.cpp.i

src/examples/CMakeFiles/boss_example.dir/boss_example.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/boss_example.dir/boss_example.cpp.s"
	cd /home/alessiohu/Desktop/progetto-labiagi/catkin_ws/build/srrg2_core/src/examples && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/alessiohu/Desktop/progetto-labiagi/catkin_ws/src/srrg2_core/srrg2_core/src/examples/boss_example.cpp -o CMakeFiles/boss_example.dir/boss_example.cpp.s

src/examples/CMakeFiles/boss_example.dir/boss_example.cpp.o.requires:

.PHONY : src/examples/CMakeFiles/boss_example.dir/boss_example.cpp.o.requires

src/examples/CMakeFiles/boss_example.dir/boss_example.cpp.o.provides: src/examples/CMakeFiles/boss_example.dir/boss_example.cpp.o.requires
	$(MAKE) -f src/examples/CMakeFiles/boss_example.dir/build.make src/examples/CMakeFiles/boss_example.dir/boss_example.cpp.o.provides.build
.PHONY : src/examples/CMakeFiles/boss_example.dir/boss_example.cpp.o.provides

src/examples/CMakeFiles/boss_example.dir/boss_example.cpp.o.provides.build: src/examples/CMakeFiles/boss_example.dir/boss_example.cpp.o


# Object files for target boss_example
boss_example_OBJECTS = \
"CMakeFiles/boss_example.dir/boss_example.cpp.o"

# External object files for target boss_example
boss_example_EXTERNAL_OBJECTS =

/home/alessiohu/Desktop/progetto-labiagi/catkin_ws/devel/.private/srrg2_core/lib/srrg2_core/boss_example: src/examples/CMakeFiles/boss_example.dir/boss_example.cpp.o
/home/alessiohu/Desktop/progetto-labiagi/catkin_ws/devel/.private/srrg2_core/lib/srrg2_core/boss_example: src/examples/CMakeFiles/boss_example.dir/build.make
/home/alessiohu/Desktop/progetto-labiagi/catkin_ws/devel/.private/srrg2_core/lib/srrg2_core/boss_example: /home/alessiohu/Desktop/progetto-labiagi/catkin_ws/devel/.private/srrg2_core/lib/libsrrg2_system_utils_library.so
/home/alessiohu/Desktop/progetto-labiagi/catkin_ws/devel/.private/srrg2_core/lib/srrg2_core/boss_example: /home/alessiohu/Desktop/progetto-labiagi/catkin_ws/devel/.private/srrg2_core/lib/libsrrg2_boss_library.so
/home/alessiohu/Desktop/progetto-labiagi/catkin_ws/devel/.private/srrg2_core/lib/srrg2_core/boss_example: src/examples/CMakeFiles/boss_example.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir=/home/alessiohu/Desktop/progetto-labiagi/catkin_ws/build/srrg2_core/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Linking CXX executable /home/alessiohu/Desktop/progetto-labiagi/catkin_ws/devel/.private/srrg2_core/lib/srrg2_core/boss_example"
	cd /home/alessiohu/Desktop/progetto-labiagi/catkin_ws/build/srrg2_core/src/examples && $(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/boss_example.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
src/examples/CMakeFiles/boss_example.dir/build: /home/alessiohu/Desktop/progetto-labiagi/catkin_ws/devel/.private/srrg2_core/lib/srrg2_core/boss_example

.PHONY : src/examples/CMakeFiles/boss_example.dir/build

src/examples/CMakeFiles/boss_example.dir/requires: src/examples/CMakeFiles/boss_example.dir/boss_example.cpp.o.requires

.PHONY : src/examples/CMakeFiles/boss_example.dir/requires

src/examples/CMakeFiles/boss_example.dir/clean:
	cd /home/alessiohu/Desktop/progetto-labiagi/catkin_ws/build/srrg2_core/src/examples && $(CMAKE_COMMAND) -P CMakeFiles/boss_example.dir/cmake_clean.cmake
.PHONY : src/examples/CMakeFiles/boss_example.dir/clean

src/examples/CMakeFiles/boss_example.dir/depend:
	cd /home/alessiohu/Desktop/progetto-labiagi/catkin_ws/build/srrg2_core && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/alessiohu/Desktop/progetto-labiagi/catkin_ws/src/srrg2_core/srrg2_core /home/alessiohu/Desktop/progetto-labiagi/catkin_ws/src/srrg2_core/srrg2_core/src/examples /home/alessiohu/Desktop/progetto-labiagi/catkin_ws/build/srrg2_core /home/alessiohu/Desktop/progetto-labiagi/catkin_ws/build/srrg2_core/src/examples /home/alessiohu/Desktop/progetto-labiagi/catkin_ws/build/srrg2_core/src/examples/CMakeFiles/boss_example.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : src/examples/CMakeFiles/boss_example.dir/depend

