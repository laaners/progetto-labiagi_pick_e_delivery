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
include src/examples/CMakeFiles/point_cloud_example.dir/depend.make

# Include the progress variables for this target.
include src/examples/CMakeFiles/point_cloud_example.dir/progress.make

# Include the compile flags for this target's objects.
include src/examples/CMakeFiles/point_cloud_example.dir/flags.make

src/examples/CMakeFiles/point_cloud_example.dir/point_cloud_example.cpp.o: src/examples/CMakeFiles/point_cloud_example.dir/flags.make
src/examples/CMakeFiles/point_cloud_example.dir/point_cloud_example.cpp.o: /home/alessiohu/Desktop/progetto-labiagi/catkin_ws/src/srrg2_core/srrg2_core/src/examples/point_cloud_example.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/alessiohu/Desktop/progetto-labiagi/catkin_ws/build/srrg2_core/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building CXX object src/examples/CMakeFiles/point_cloud_example.dir/point_cloud_example.cpp.o"
	cd /home/alessiohu/Desktop/progetto-labiagi/catkin_ws/build/srrg2_core/src/examples && /usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/point_cloud_example.dir/point_cloud_example.cpp.o -c /home/alessiohu/Desktop/progetto-labiagi/catkin_ws/src/srrg2_core/srrg2_core/src/examples/point_cloud_example.cpp

src/examples/CMakeFiles/point_cloud_example.dir/point_cloud_example.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/point_cloud_example.dir/point_cloud_example.cpp.i"
	cd /home/alessiohu/Desktop/progetto-labiagi/catkin_ws/build/srrg2_core/src/examples && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/alessiohu/Desktop/progetto-labiagi/catkin_ws/src/srrg2_core/srrg2_core/src/examples/point_cloud_example.cpp > CMakeFiles/point_cloud_example.dir/point_cloud_example.cpp.i

src/examples/CMakeFiles/point_cloud_example.dir/point_cloud_example.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/point_cloud_example.dir/point_cloud_example.cpp.s"
	cd /home/alessiohu/Desktop/progetto-labiagi/catkin_ws/build/srrg2_core/src/examples && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/alessiohu/Desktop/progetto-labiagi/catkin_ws/src/srrg2_core/srrg2_core/src/examples/point_cloud_example.cpp -o CMakeFiles/point_cloud_example.dir/point_cloud_example.cpp.s

src/examples/CMakeFiles/point_cloud_example.dir/point_cloud_example.cpp.o.requires:

.PHONY : src/examples/CMakeFiles/point_cloud_example.dir/point_cloud_example.cpp.o.requires

src/examples/CMakeFiles/point_cloud_example.dir/point_cloud_example.cpp.o.provides: src/examples/CMakeFiles/point_cloud_example.dir/point_cloud_example.cpp.o.requires
	$(MAKE) -f src/examples/CMakeFiles/point_cloud_example.dir/build.make src/examples/CMakeFiles/point_cloud_example.dir/point_cloud_example.cpp.o.provides.build
.PHONY : src/examples/CMakeFiles/point_cloud_example.dir/point_cloud_example.cpp.o.provides

src/examples/CMakeFiles/point_cloud_example.dir/point_cloud_example.cpp.o.provides.build: src/examples/CMakeFiles/point_cloud_example.dir/point_cloud_example.cpp.o


# Object files for target point_cloud_example
point_cloud_example_OBJECTS = \
"CMakeFiles/point_cloud_example.dir/point_cloud_example.cpp.o"

# External object files for target point_cloud_example
point_cloud_example_EXTERNAL_OBJECTS =

/home/alessiohu/Desktop/progetto-labiagi/catkin_ws/devel/.private/srrg2_core/lib/srrg2_core/point_cloud_example: src/examples/CMakeFiles/point_cloud_example.dir/point_cloud_example.cpp.o
/home/alessiohu/Desktop/progetto-labiagi/catkin_ws/devel/.private/srrg2_core/lib/srrg2_core/point_cloud_example: src/examples/CMakeFiles/point_cloud_example.dir/build.make
/home/alessiohu/Desktop/progetto-labiagi/catkin_ws/devel/.private/srrg2_core/lib/srrg2_core/point_cloud_example: /home/alessiohu/Desktop/progetto-labiagi/catkin_ws/devel/.private/srrg2_core/lib/libsrrg2_point_cloud_library.so
/home/alessiohu/Desktop/progetto-labiagi/catkin_ws/devel/.private/srrg2_core/lib/srrg2_core/point_cloud_example: /home/alessiohu/Desktop/progetto-labiagi/catkin_ws/devel/.private/srrg2_core/lib/libsrrg2_config_library.so
/home/alessiohu/Desktop/progetto-labiagi/catkin_ws/devel/.private/srrg2_core/lib/srrg2_core/point_cloud_example: /home/alessiohu/Desktop/progetto-labiagi/catkin_ws/devel/.private/srrg2_core/lib/libsrrg2_system_utils_library.so
/home/alessiohu/Desktop/progetto-labiagi/catkin_ws/devel/.private/srrg2_core/lib/srrg2_core/point_cloud_example: /home/alessiohu/Desktop/progetto-labiagi/catkin_ws/devel/.private/srrg2_core/lib/libsrrg2_property_library.so
/home/alessiohu/Desktop/progetto-labiagi/catkin_ws/devel/.private/srrg2_core/lib/srrg2_core/point_cloud_example: /home/alessiohu/Desktop/progetto-labiagi/catkin_ws/devel/.private/srrg2_core/lib/libsrrg2_boss_library.so
/home/alessiohu/Desktop/progetto-labiagi/catkin_ws/devel/.private/srrg2_core/lib/srrg2_core/point_cloud_example: /usr/lib/x86_64-linux-gnu/libopencv_shape.so.3.2.0
/home/alessiohu/Desktop/progetto-labiagi/catkin_ws/devel/.private/srrg2_core/lib/srrg2_core/point_cloud_example: /usr/lib/x86_64-linux-gnu/libopencv_stitching.so.3.2.0
/home/alessiohu/Desktop/progetto-labiagi/catkin_ws/devel/.private/srrg2_core/lib/srrg2_core/point_cloud_example: /usr/lib/x86_64-linux-gnu/libopencv_superres.so.3.2.0
/home/alessiohu/Desktop/progetto-labiagi/catkin_ws/devel/.private/srrg2_core/lib/srrg2_core/point_cloud_example: /usr/lib/x86_64-linux-gnu/libopencv_videostab.so.3.2.0
/home/alessiohu/Desktop/progetto-labiagi/catkin_ws/devel/.private/srrg2_core/lib/srrg2_core/point_cloud_example: /usr/lib/x86_64-linux-gnu/libopencv_aruco.so.3.2.0
/home/alessiohu/Desktop/progetto-labiagi/catkin_ws/devel/.private/srrg2_core/lib/srrg2_core/point_cloud_example: /usr/lib/x86_64-linux-gnu/libopencv_bgsegm.so.3.2.0
/home/alessiohu/Desktop/progetto-labiagi/catkin_ws/devel/.private/srrg2_core/lib/srrg2_core/point_cloud_example: /usr/lib/x86_64-linux-gnu/libopencv_bioinspired.so.3.2.0
/home/alessiohu/Desktop/progetto-labiagi/catkin_ws/devel/.private/srrg2_core/lib/srrg2_core/point_cloud_example: /usr/lib/x86_64-linux-gnu/libopencv_ccalib.so.3.2.0
/home/alessiohu/Desktop/progetto-labiagi/catkin_ws/devel/.private/srrg2_core/lib/srrg2_core/point_cloud_example: /usr/lib/x86_64-linux-gnu/libopencv_datasets.so.3.2.0
/home/alessiohu/Desktop/progetto-labiagi/catkin_ws/devel/.private/srrg2_core/lib/srrg2_core/point_cloud_example: /usr/lib/x86_64-linux-gnu/libopencv_dpm.so.3.2.0
/home/alessiohu/Desktop/progetto-labiagi/catkin_ws/devel/.private/srrg2_core/lib/srrg2_core/point_cloud_example: /usr/lib/x86_64-linux-gnu/libopencv_face.so.3.2.0
/home/alessiohu/Desktop/progetto-labiagi/catkin_ws/devel/.private/srrg2_core/lib/srrg2_core/point_cloud_example: /usr/lib/x86_64-linux-gnu/libopencv_freetype.so.3.2.0
/home/alessiohu/Desktop/progetto-labiagi/catkin_ws/devel/.private/srrg2_core/lib/srrg2_core/point_cloud_example: /usr/lib/x86_64-linux-gnu/libopencv_fuzzy.so.3.2.0
/home/alessiohu/Desktop/progetto-labiagi/catkin_ws/devel/.private/srrg2_core/lib/srrg2_core/point_cloud_example: /usr/lib/x86_64-linux-gnu/libopencv_hdf.so.3.2.0
/home/alessiohu/Desktop/progetto-labiagi/catkin_ws/devel/.private/srrg2_core/lib/srrg2_core/point_cloud_example: /usr/lib/x86_64-linux-gnu/libopencv_line_descriptor.so.3.2.0
/home/alessiohu/Desktop/progetto-labiagi/catkin_ws/devel/.private/srrg2_core/lib/srrg2_core/point_cloud_example: /usr/lib/x86_64-linux-gnu/libopencv_optflow.so.3.2.0
/home/alessiohu/Desktop/progetto-labiagi/catkin_ws/devel/.private/srrg2_core/lib/srrg2_core/point_cloud_example: /usr/lib/x86_64-linux-gnu/libopencv_video.so.3.2.0
/home/alessiohu/Desktop/progetto-labiagi/catkin_ws/devel/.private/srrg2_core/lib/srrg2_core/point_cloud_example: /usr/lib/x86_64-linux-gnu/libopencv_plot.so.3.2.0
/home/alessiohu/Desktop/progetto-labiagi/catkin_ws/devel/.private/srrg2_core/lib/srrg2_core/point_cloud_example: /usr/lib/x86_64-linux-gnu/libopencv_reg.so.3.2.0
/home/alessiohu/Desktop/progetto-labiagi/catkin_ws/devel/.private/srrg2_core/lib/srrg2_core/point_cloud_example: /usr/lib/x86_64-linux-gnu/libopencv_saliency.so.3.2.0
/home/alessiohu/Desktop/progetto-labiagi/catkin_ws/devel/.private/srrg2_core/lib/srrg2_core/point_cloud_example: /usr/lib/x86_64-linux-gnu/libopencv_stereo.so.3.2.0
/home/alessiohu/Desktop/progetto-labiagi/catkin_ws/devel/.private/srrg2_core/lib/srrg2_core/point_cloud_example: /usr/lib/x86_64-linux-gnu/libopencv_structured_light.so.3.2.0
/home/alessiohu/Desktop/progetto-labiagi/catkin_ws/devel/.private/srrg2_core/lib/srrg2_core/point_cloud_example: /usr/lib/x86_64-linux-gnu/libopencv_viz.so.3.2.0
/home/alessiohu/Desktop/progetto-labiagi/catkin_ws/devel/.private/srrg2_core/lib/srrg2_core/point_cloud_example: /usr/lib/x86_64-linux-gnu/libopencv_phase_unwrapping.so.3.2.0
/home/alessiohu/Desktop/progetto-labiagi/catkin_ws/devel/.private/srrg2_core/lib/srrg2_core/point_cloud_example: /usr/lib/x86_64-linux-gnu/libopencv_rgbd.so.3.2.0
/home/alessiohu/Desktop/progetto-labiagi/catkin_ws/devel/.private/srrg2_core/lib/srrg2_core/point_cloud_example: /usr/lib/x86_64-linux-gnu/libopencv_surface_matching.so.3.2.0
/home/alessiohu/Desktop/progetto-labiagi/catkin_ws/devel/.private/srrg2_core/lib/srrg2_core/point_cloud_example: /usr/lib/x86_64-linux-gnu/libopencv_text.so.3.2.0
/home/alessiohu/Desktop/progetto-labiagi/catkin_ws/devel/.private/srrg2_core/lib/srrg2_core/point_cloud_example: /usr/lib/x86_64-linux-gnu/libopencv_ximgproc.so.3.2.0
/home/alessiohu/Desktop/progetto-labiagi/catkin_ws/devel/.private/srrg2_core/lib/srrg2_core/point_cloud_example: /usr/lib/x86_64-linux-gnu/libopencv_calib3d.so.3.2.0
/home/alessiohu/Desktop/progetto-labiagi/catkin_ws/devel/.private/srrg2_core/lib/srrg2_core/point_cloud_example: /usr/lib/x86_64-linux-gnu/libopencv_features2d.so.3.2.0
/home/alessiohu/Desktop/progetto-labiagi/catkin_ws/devel/.private/srrg2_core/lib/srrg2_core/point_cloud_example: /usr/lib/x86_64-linux-gnu/libopencv_flann.so.3.2.0
/home/alessiohu/Desktop/progetto-labiagi/catkin_ws/devel/.private/srrg2_core/lib/srrg2_core/point_cloud_example: /usr/lib/x86_64-linux-gnu/libopencv_xobjdetect.so.3.2.0
/home/alessiohu/Desktop/progetto-labiagi/catkin_ws/devel/.private/srrg2_core/lib/srrg2_core/point_cloud_example: /usr/lib/x86_64-linux-gnu/libopencv_objdetect.so.3.2.0
/home/alessiohu/Desktop/progetto-labiagi/catkin_ws/devel/.private/srrg2_core/lib/srrg2_core/point_cloud_example: /usr/lib/x86_64-linux-gnu/libopencv_ml.so.3.2.0
/home/alessiohu/Desktop/progetto-labiagi/catkin_ws/devel/.private/srrg2_core/lib/srrg2_core/point_cloud_example: /usr/lib/x86_64-linux-gnu/libopencv_xphoto.so.3.2.0
/home/alessiohu/Desktop/progetto-labiagi/catkin_ws/devel/.private/srrg2_core/lib/srrg2_core/point_cloud_example: /usr/lib/x86_64-linux-gnu/libopencv_highgui.so.3.2.0
/home/alessiohu/Desktop/progetto-labiagi/catkin_ws/devel/.private/srrg2_core/lib/srrg2_core/point_cloud_example: /usr/lib/x86_64-linux-gnu/libopencv_photo.so.3.2.0
/home/alessiohu/Desktop/progetto-labiagi/catkin_ws/devel/.private/srrg2_core/lib/srrg2_core/point_cloud_example: /usr/lib/x86_64-linux-gnu/libopencv_videoio.so.3.2.0
/home/alessiohu/Desktop/progetto-labiagi/catkin_ws/devel/.private/srrg2_core/lib/srrg2_core/point_cloud_example: /usr/lib/x86_64-linux-gnu/libopencv_imgcodecs.so.3.2.0
/home/alessiohu/Desktop/progetto-labiagi/catkin_ws/devel/.private/srrg2_core/lib/srrg2_core/point_cloud_example: /usr/lib/x86_64-linux-gnu/libopencv_imgproc.so.3.2.0
/home/alessiohu/Desktop/progetto-labiagi/catkin_ws/devel/.private/srrg2_core/lib/srrg2_core/point_cloud_example: /usr/lib/x86_64-linux-gnu/libopencv_core.so.3.2.0
/home/alessiohu/Desktop/progetto-labiagi/catkin_ws/devel/.private/srrg2_core/lib/srrg2_core/point_cloud_example: src/examples/CMakeFiles/point_cloud_example.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir=/home/alessiohu/Desktop/progetto-labiagi/catkin_ws/build/srrg2_core/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Linking CXX executable /home/alessiohu/Desktop/progetto-labiagi/catkin_ws/devel/.private/srrg2_core/lib/srrg2_core/point_cloud_example"
	cd /home/alessiohu/Desktop/progetto-labiagi/catkin_ws/build/srrg2_core/src/examples && $(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/point_cloud_example.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
src/examples/CMakeFiles/point_cloud_example.dir/build: /home/alessiohu/Desktop/progetto-labiagi/catkin_ws/devel/.private/srrg2_core/lib/srrg2_core/point_cloud_example

.PHONY : src/examples/CMakeFiles/point_cloud_example.dir/build

src/examples/CMakeFiles/point_cloud_example.dir/requires: src/examples/CMakeFiles/point_cloud_example.dir/point_cloud_example.cpp.o.requires

.PHONY : src/examples/CMakeFiles/point_cloud_example.dir/requires

src/examples/CMakeFiles/point_cloud_example.dir/clean:
	cd /home/alessiohu/Desktop/progetto-labiagi/catkin_ws/build/srrg2_core/src/examples && $(CMAKE_COMMAND) -P CMakeFiles/point_cloud_example.dir/cmake_clean.cmake
.PHONY : src/examples/CMakeFiles/point_cloud_example.dir/clean

src/examples/CMakeFiles/point_cloud_example.dir/depend:
	cd /home/alessiohu/Desktop/progetto-labiagi/catkin_ws/build/srrg2_core && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/alessiohu/Desktop/progetto-labiagi/catkin_ws/src/srrg2_core/srrg2_core /home/alessiohu/Desktop/progetto-labiagi/catkin_ws/src/srrg2_core/srrg2_core/src/examples /home/alessiohu/Desktop/progetto-labiagi/catkin_ws/build/srrg2_core /home/alessiohu/Desktop/progetto-labiagi/catkin_ws/build/srrg2_core/src/examples /home/alessiohu/Desktop/progetto-labiagi/catkin_ws/build/srrg2_core/src/examples/CMakeFiles/point_cloud_example.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : src/examples/CMakeFiles/point_cloud_example.dir/depend

