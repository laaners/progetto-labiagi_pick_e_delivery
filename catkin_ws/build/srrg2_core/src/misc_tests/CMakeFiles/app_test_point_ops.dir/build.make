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
include src/misc_tests/CMakeFiles/app_test_point_ops.dir/depend.make

# Include the progress variables for this target.
include src/misc_tests/CMakeFiles/app_test_point_ops.dir/progress.make

# Include the compile flags for this target's objects.
include src/misc_tests/CMakeFiles/app_test_point_ops.dir/flags.make

src/misc_tests/CMakeFiles/app_test_point_ops.dir/test_point_ops.cpp.o: src/misc_tests/CMakeFiles/app_test_point_ops.dir/flags.make
src/misc_tests/CMakeFiles/app_test_point_ops.dir/test_point_ops.cpp.o: /home/alessiohu/Desktop/progetto-labiagi/catkin_ws/src/srrg2_core/srrg2_core/src/misc_tests/test_point_ops.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/alessiohu/Desktop/progetto-labiagi/catkin_ws/build/srrg2_core/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building CXX object src/misc_tests/CMakeFiles/app_test_point_ops.dir/test_point_ops.cpp.o"
	cd /home/alessiohu/Desktop/progetto-labiagi/catkin_ws/build/srrg2_core/src/misc_tests && /usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/app_test_point_ops.dir/test_point_ops.cpp.o -c /home/alessiohu/Desktop/progetto-labiagi/catkin_ws/src/srrg2_core/srrg2_core/src/misc_tests/test_point_ops.cpp

src/misc_tests/CMakeFiles/app_test_point_ops.dir/test_point_ops.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/app_test_point_ops.dir/test_point_ops.cpp.i"
	cd /home/alessiohu/Desktop/progetto-labiagi/catkin_ws/build/srrg2_core/src/misc_tests && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/alessiohu/Desktop/progetto-labiagi/catkin_ws/src/srrg2_core/srrg2_core/src/misc_tests/test_point_ops.cpp > CMakeFiles/app_test_point_ops.dir/test_point_ops.cpp.i

src/misc_tests/CMakeFiles/app_test_point_ops.dir/test_point_ops.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/app_test_point_ops.dir/test_point_ops.cpp.s"
	cd /home/alessiohu/Desktop/progetto-labiagi/catkin_ws/build/srrg2_core/src/misc_tests && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/alessiohu/Desktop/progetto-labiagi/catkin_ws/src/srrg2_core/srrg2_core/src/misc_tests/test_point_ops.cpp -o CMakeFiles/app_test_point_ops.dir/test_point_ops.cpp.s

src/misc_tests/CMakeFiles/app_test_point_ops.dir/test_point_ops.cpp.o.requires:

.PHONY : src/misc_tests/CMakeFiles/app_test_point_ops.dir/test_point_ops.cpp.o.requires

src/misc_tests/CMakeFiles/app_test_point_ops.dir/test_point_ops.cpp.o.provides: src/misc_tests/CMakeFiles/app_test_point_ops.dir/test_point_ops.cpp.o.requires
	$(MAKE) -f src/misc_tests/CMakeFiles/app_test_point_ops.dir/build.make src/misc_tests/CMakeFiles/app_test_point_ops.dir/test_point_ops.cpp.o.provides.build
.PHONY : src/misc_tests/CMakeFiles/app_test_point_ops.dir/test_point_ops.cpp.o.provides

src/misc_tests/CMakeFiles/app_test_point_ops.dir/test_point_ops.cpp.o.provides.build: src/misc_tests/CMakeFiles/app_test_point_ops.dir/test_point_ops.cpp.o


# Object files for target app_test_point_ops
app_test_point_ops_OBJECTS = \
"CMakeFiles/app_test_point_ops.dir/test_point_ops.cpp.o"

# External object files for target app_test_point_ops
app_test_point_ops_EXTERNAL_OBJECTS =

/home/alessiohu/Desktop/progetto-labiagi/catkin_ws/devel/.private/srrg2_core/lib/srrg2_core/app_test_point_ops: src/misc_tests/CMakeFiles/app_test_point_ops.dir/test_point_ops.cpp.o
/home/alessiohu/Desktop/progetto-labiagi/catkin_ws/devel/.private/srrg2_core/lib/srrg2_core/app_test_point_ops: src/misc_tests/CMakeFiles/app_test_point_ops.dir/build.make
/home/alessiohu/Desktop/progetto-labiagi/catkin_ws/devel/.private/srrg2_core/lib/srrg2_core/app_test_point_ops: /home/alessiohu/Desktop/progetto-labiagi/catkin_ws/devel/.private/srrg2_core/lib/libsrrg2_point_cloud_library.so
/home/alessiohu/Desktop/progetto-labiagi/catkin_ws/devel/.private/srrg2_core/lib/srrg2_core/app_test_point_ops: /home/alessiohu/Desktop/progetto-labiagi/catkin_ws/devel/.private/srrg2_core/lib/libsrrg2_config_library.so
/home/alessiohu/Desktop/progetto-labiagi/catkin_ws/devel/.private/srrg2_core/lib/srrg2_core/app_test_point_ops: /home/alessiohu/Desktop/progetto-labiagi/catkin_ws/devel/.private/srrg2_core/lib/libsrrg2_property_library.so
/home/alessiohu/Desktop/progetto-labiagi/catkin_ws/devel/.private/srrg2_core/lib/srrg2_core/app_test_point_ops: /usr/lib/x86_64-linux-gnu/libopencv_shape.so.3.2.0
/home/alessiohu/Desktop/progetto-labiagi/catkin_ws/devel/.private/srrg2_core/lib/srrg2_core/app_test_point_ops: /usr/lib/x86_64-linux-gnu/libopencv_stitching.so.3.2.0
/home/alessiohu/Desktop/progetto-labiagi/catkin_ws/devel/.private/srrg2_core/lib/srrg2_core/app_test_point_ops: /usr/lib/x86_64-linux-gnu/libopencv_superres.so.3.2.0
/home/alessiohu/Desktop/progetto-labiagi/catkin_ws/devel/.private/srrg2_core/lib/srrg2_core/app_test_point_ops: /usr/lib/x86_64-linux-gnu/libopencv_videostab.so.3.2.0
/home/alessiohu/Desktop/progetto-labiagi/catkin_ws/devel/.private/srrg2_core/lib/srrg2_core/app_test_point_ops: /usr/lib/x86_64-linux-gnu/libopencv_aruco.so.3.2.0
/home/alessiohu/Desktop/progetto-labiagi/catkin_ws/devel/.private/srrg2_core/lib/srrg2_core/app_test_point_ops: /usr/lib/x86_64-linux-gnu/libopencv_bgsegm.so.3.2.0
/home/alessiohu/Desktop/progetto-labiagi/catkin_ws/devel/.private/srrg2_core/lib/srrg2_core/app_test_point_ops: /usr/lib/x86_64-linux-gnu/libopencv_bioinspired.so.3.2.0
/home/alessiohu/Desktop/progetto-labiagi/catkin_ws/devel/.private/srrg2_core/lib/srrg2_core/app_test_point_ops: /usr/lib/x86_64-linux-gnu/libopencv_ccalib.so.3.2.0
/home/alessiohu/Desktop/progetto-labiagi/catkin_ws/devel/.private/srrg2_core/lib/srrg2_core/app_test_point_ops: /usr/lib/x86_64-linux-gnu/libopencv_datasets.so.3.2.0
/home/alessiohu/Desktop/progetto-labiagi/catkin_ws/devel/.private/srrg2_core/lib/srrg2_core/app_test_point_ops: /usr/lib/x86_64-linux-gnu/libopencv_dpm.so.3.2.0
/home/alessiohu/Desktop/progetto-labiagi/catkin_ws/devel/.private/srrg2_core/lib/srrg2_core/app_test_point_ops: /usr/lib/x86_64-linux-gnu/libopencv_face.so.3.2.0
/home/alessiohu/Desktop/progetto-labiagi/catkin_ws/devel/.private/srrg2_core/lib/srrg2_core/app_test_point_ops: /usr/lib/x86_64-linux-gnu/libopencv_freetype.so.3.2.0
/home/alessiohu/Desktop/progetto-labiagi/catkin_ws/devel/.private/srrg2_core/lib/srrg2_core/app_test_point_ops: /usr/lib/x86_64-linux-gnu/libopencv_fuzzy.so.3.2.0
/home/alessiohu/Desktop/progetto-labiagi/catkin_ws/devel/.private/srrg2_core/lib/srrg2_core/app_test_point_ops: /usr/lib/x86_64-linux-gnu/libopencv_hdf.so.3.2.0
/home/alessiohu/Desktop/progetto-labiagi/catkin_ws/devel/.private/srrg2_core/lib/srrg2_core/app_test_point_ops: /usr/lib/x86_64-linux-gnu/libopencv_line_descriptor.so.3.2.0
/home/alessiohu/Desktop/progetto-labiagi/catkin_ws/devel/.private/srrg2_core/lib/srrg2_core/app_test_point_ops: /usr/lib/x86_64-linux-gnu/libopencv_optflow.so.3.2.0
/home/alessiohu/Desktop/progetto-labiagi/catkin_ws/devel/.private/srrg2_core/lib/srrg2_core/app_test_point_ops: /usr/lib/x86_64-linux-gnu/libopencv_video.so.3.2.0
/home/alessiohu/Desktop/progetto-labiagi/catkin_ws/devel/.private/srrg2_core/lib/srrg2_core/app_test_point_ops: /usr/lib/x86_64-linux-gnu/libopencv_plot.so.3.2.0
/home/alessiohu/Desktop/progetto-labiagi/catkin_ws/devel/.private/srrg2_core/lib/srrg2_core/app_test_point_ops: /usr/lib/x86_64-linux-gnu/libopencv_reg.so.3.2.0
/home/alessiohu/Desktop/progetto-labiagi/catkin_ws/devel/.private/srrg2_core/lib/srrg2_core/app_test_point_ops: /usr/lib/x86_64-linux-gnu/libopencv_saliency.so.3.2.0
/home/alessiohu/Desktop/progetto-labiagi/catkin_ws/devel/.private/srrg2_core/lib/srrg2_core/app_test_point_ops: /usr/lib/x86_64-linux-gnu/libopencv_stereo.so.3.2.0
/home/alessiohu/Desktop/progetto-labiagi/catkin_ws/devel/.private/srrg2_core/lib/srrg2_core/app_test_point_ops: /usr/lib/x86_64-linux-gnu/libopencv_structured_light.so.3.2.0
/home/alessiohu/Desktop/progetto-labiagi/catkin_ws/devel/.private/srrg2_core/lib/srrg2_core/app_test_point_ops: /usr/lib/x86_64-linux-gnu/libopencv_viz.so.3.2.0
/home/alessiohu/Desktop/progetto-labiagi/catkin_ws/devel/.private/srrg2_core/lib/srrg2_core/app_test_point_ops: /usr/lib/x86_64-linux-gnu/libopencv_phase_unwrapping.so.3.2.0
/home/alessiohu/Desktop/progetto-labiagi/catkin_ws/devel/.private/srrg2_core/lib/srrg2_core/app_test_point_ops: /usr/lib/x86_64-linux-gnu/libopencv_rgbd.so.3.2.0
/home/alessiohu/Desktop/progetto-labiagi/catkin_ws/devel/.private/srrg2_core/lib/srrg2_core/app_test_point_ops: /usr/lib/x86_64-linux-gnu/libopencv_surface_matching.so.3.2.0
/home/alessiohu/Desktop/progetto-labiagi/catkin_ws/devel/.private/srrg2_core/lib/srrg2_core/app_test_point_ops: /usr/lib/x86_64-linux-gnu/libopencv_text.so.3.2.0
/home/alessiohu/Desktop/progetto-labiagi/catkin_ws/devel/.private/srrg2_core/lib/srrg2_core/app_test_point_ops: /usr/lib/x86_64-linux-gnu/libopencv_ximgproc.so.3.2.0
/home/alessiohu/Desktop/progetto-labiagi/catkin_ws/devel/.private/srrg2_core/lib/srrg2_core/app_test_point_ops: /usr/lib/x86_64-linux-gnu/libopencv_calib3d.so.3.2.0
/home/alessiohu/Desktop/progetto-labiagi/catkin_ws/devel/.private/srrg2_core/lib/srrg2_core/app_test_point_ops: /usr/lib/x86_64-linux-gnu/libopencv_features2d.so.3.2.0
/home/alessiohu/Desktop/progetto-labiagi/catkin_ws/devel/.private/srrg2_core/lib/srrg2_core/app_test_point_ops: /usr/lib/x86_64-linux-gnu/libopencv_flann.so.3.2.0
/home/alessiohu/Desktop/progetto-labiagi/catkin_ws/devel/.private/srrg2_core/lib/srrg2_core/app_test_point_ops: /usr/lib/x86_64-linux-gnu/libopencv_xobjdetect.so.3.2.0
/home/alessiohu/Desktop/progetto-labiagi/catkin_ws/devel/.private/srrg2_core/lib/srrg2_core/app_test_point_ops: /usr/lib/x86_64-linux-gnu/libopencv_objdetect.so.3.2.0
/home/alessiohu/Desktop/progetto-labiagi/catkin_ws/devel/.private/srrg2_core/lib/srrg2_core/app_test_point_ops: /usr/lib/x86_64-linux-gnu/libopencv_ml.so.3.2.0
/home/alessiohu/Desktop/progetto-labiagi/catkin_ws/devel/.private/srrg2_core/lib/srrg2_core/app_test_point_ops: /usr/lib/x86_64-linux-gnu/libopencv_xphoto.so.3.2.0
/home/alessiohu/Desktop/progetto-labiagi/catkin_ws/devel/.private/srrg2_core/lib/srrg2_core/app_test_point_ops: /usr/lib/x86_64-linux-gnu/libopencv_highgui.so.3.2.0
/home/alessiohu/Desktop/progetto-labiagi/catkin_ws/devel/.private/srrg2_core/lib/srrg2_core/app_test_point_ops: /usr/lib/x86_64-linux-gnu/libopencv_photo.so.3.2.0
/home/alessiohu/Desktop/progetto-labiagi/catkin_ws/devel/.private/srrg2_core/lib/srrg2_core/app_test_point_ops: /usr/lib/x86_64-linux-gnu/libopencv_videoio.so.3.2.0
/home/alessiohu/Desktop/progetto-labiagi/catkin_ws/devel/.private/srrg2_core/lib/srrg2_core/app_test_point_ops: /usr/lib/x86_64-linux-gnu/libopencv_imgcodecs.so.3.2.0
/home/alessiohu/Desktop/progetto-labiagi/catkin_ws/devel/.private/srrg2_core/lib/srrg2_core/app_test_point_ops: /usr/lib/x86_64-linux-gnu/libopencv_imgproc.so.3.2.0
/home/alessiohu/Desktop/progetto-labiagi/catkin_ws/devel/.private/srrg2_core/lib/srrg2_core/app_test_point_ops: /home/alessiohu/Desktop/progetto-labiagi/catkin_ws/devel/.private/srrg2_core/lib/libsrrg2_system_utils_library.so
/home/alessiohu/Desktop/progetto-labiagi/catkin_ws/devel/.private/srrg2_core/lib/srrg2_core/app_test_point_ops: /home/alessiohu/Desktop/progetto-labiagi/catkin_ws/devel/.private/srrg2_core/lib/libsrrg2_boss_library.so
/home/alessiohu/Desktop/progetto-labiagi/catkin_ws/devel/.private/srrg2_core/lib/srrg2_core/app_test_point_ops: /usr/lib/x86_64-linux-gnu/libopencv_core.so.3.2.0
/home/alessiohu/Desktop/progetto-labiagi/catkin_ws/devel/.private/srrg2_core/lib/srrg2_core/app_test_point_ops: src/misc_tests/CMakeFiles/app_test_point_ops.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir=/home/alessiohu/Desktop/progetto-labiagi/catkin_ws/build/srrg2_core/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Linking CXX executable /home/alessiohu/Desktop/progetto-labiagi/catkin_ws/devel/.private/srrg2_core/lib/srrg2_core/app_test_point_ops"
	cd /home/alessiohu/Desktop/progetto-labiagi/catkin_ws/build/srrg2_core/src/misc_tests && $(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/app_test_point_ops.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
src/misc_tests/CMakeFiles/app_test_point_ops.dir/build: /home/alessiohu/Desktop/progetto-labiagi/catkin_ws/devel/.private/srrg2_core/lib/srrg2_core/app_test_point_ops

.PHONY : src/misc_tests/CMakeFiles/app_test_point_ops.dir/build

src/misc_tests/CMakeFiles/app_test_point_ops.dir/requires: src/misc_tests/CMakeFiles/app_test_point_ops.dir/test_point_ops.cpp.o.requires

.PHONY : src/misc_tests/CMakeFiles/app_test_point_ops.dir/requires

src/misc_tests/CMakeFiles/app_test_point_ops.dir/clean:
	cd /home/alessiohu/Desktop/progetto-labiagi/catkin_ws/build/srrg2_core/src/misc_tests && $(CMAKE_COMMAND) -P CMakeFiles/app_test_point_ops.dir/cmake_clean.cmake
.PHONY : src/misc_tests/CMakeFiles/app_test_point_ops.dir/clean

src/misc_tests/CMakeFiles/app_test_point_ops.dir/depend:
	cd /home/alessiohu/Desktop/progetto-labiagi/catkin_ws/build/srrg2_core && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/alessiohu/Desktop/progetto-labiagi/catkin_ws/src/srrg2_core/srrg2_core /home/alessiohu/Desktop/progetto-labiagi/catkin_ws/src/srrg2_core/srrg2_core/src/misc_tests /home/alessiohu/Desktop/progetto-labiagi/catkin_ws/build/srrg2_core /home/alessiohu/Desktop/progetto-labiagi/catkin_ws/build/srrg2_core/src/misc_tests /home/alessiohu/Desktop/progetto-labiagi/catkin_ws/build/srrg2_core/src/misc_tests/CMakeFiles/app_test_point_ops.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : src/misc_tests/CMakeFiles/app_test_point_ops.dir/depend

