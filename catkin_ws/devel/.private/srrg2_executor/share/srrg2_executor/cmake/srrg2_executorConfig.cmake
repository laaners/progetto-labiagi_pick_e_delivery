# generated from catkin/cmake/template/pkgConfig.cmake.in

# append elements to a list and remove existing duplicates from the list
# copied from catkin/cmake/list_append_deduplicate.cmake to keep pkgConfig
# self contained
macro(_list_append_deduplicate listname)
  if(NOT "${ARGN}" STREQUAL "")
    if(${listname})
      list(REMOVE_ITEM ${listname} ${ARGN})
    endif()
    list(APPEND ${listname} ${ARGN})
  endif()
endmacro()

# append elements to a list if they are not already in the list
# copied from catkin/cmake/list_append_unique.cmake to keep pkgConfig
# self contained
macro(_list_append_unique listname)
  foreach(_item ${ARGN})
    list(FIND ${listname} ${_item} _index)
    if(_index EQUAL -1)
      list(APPEND ${listname} ${_item})
    endif()
  endforeach()
endmacro()

# pack a list of libraries with optional build configuration keywords
# copied from catkin/cmake/catkin_libraries.cmake to keep pkgConfig
# self contained
macro(_pack_libraries_with_build_configuration VAR)
  set(${VAR} "")
  set(_argn ${ARGN})
  list(LENGTH _argn _count)
  set(_index 0)
  while(${_index} LESS ${_count})
    list(GET _argn ${_index} lib)
    if("${lib}" MATCHES "^(debug|optimized|general)$")
      math(EXPR _index "${_index} + 1")
      if(${_index} EQUAL ${_count})
        message(FATAL_ERROR "_pack_libraries_with_build_configuration() the list of libraries '${ARGN}' ends with '${lib}' which is a build configuration keyword and must be followed by a library")
      endif()
      list(GET _argn ${_index} library)
      list(APPEND ${VAR} "${lib}${CATKIN_BUILD_CONFIGURATION_KEYWORD_SEPARATOR}${library}")
    else()
      list(APPEND ${VAR} "${lib}")
    endif()
    math(EXPR _index "${_index} + 1")
  endwhile()
endmacro()

# unpack a list of libraries with optional build configuration keyword prefixes
# copied from catkin/cmake/catkin_libraries.cmake to keep pkgConfig
# self contained
macro(_unpack_libraries_with_build_configuration VAR)
  set(${VAR} "")
  foreach(lib ${ARGN})
    string(REGEX REPLACE "^(debug|optimized|general)${CATKIN_BUILD_CONFIGURATION_KEYWORD_SEPARATOR}(.+)$" "\\1;\\2" lib "${lib}")
    list(APPEND ${VAR} "${lib}")
  endforeach()
endmacro()


if(srrg2_executor_CONFIG_INCLUDED)
  return()
endif()
set(srrg2_executor_CONFIG_INCLUDED TRUE)

# set variables for source/devel/install prefixes
if("TRUE" STREQUAL "TRUE")
  set(srrg2_executor_SOURCE_PREFIX /home/alessio/Desktop/progetto-labiagi/catkin_ws/src/srrg2_executor/srrg2_executor)
  set(srrg2_executor_DEVEL_PREFIX /home/alessio/Desktop/progetto-labiagi/catkin_ws/devel/.private/srrg2_executor)
  set(srrg2_executor_INSTALL_PREFIX "")
  set(srrg2_executor_PREFIX ${srrg2_executor_DEVEL_PREFIX})
else()
  set(srrg2_executor_SOURCE_PREFIX "")
  set(srrg2_executor_DEVEL_PREFIX "")
  set(srrg2_executor_INSTALL_PREFIX /home/alessio/Desktop/progetto-labiagi/catkin_ws/install)
  set(srrg2_executor_PREFIX ${srrg2_executor_INSTALL_PREFIX})
endif()

# warn when using a deprecated package
if(NOT "" STREQUAL "")
  set(_msg "WARNING: package 'srrg2_executor' is deprecated")
  # append custom deprecation text if available
  if(NOT "" STREQUAL "TRUE")
    set(_msg "${_msg} ()")
  endif()
  message("${_msg}")
endif()

# flag project as catkin-based to distinguish if a find_package()-ed project is a catkin project
set(srrg2_executor_FOUND_CATKIN_PROJECT TRUE)

if(NOT "/usr/include;/usr/include/opencv;/usr/include/x86_64-linux-gnu/qt5/;/usr/include/x86_64-linux-gnu/qt5/QtXml;/usr/include/x86_64-linux-gnu/qt5/QtCore;/usr/lib/x86_64-linux-gnu/qt5//mkspecs/linux-g++;/usr/include/x86_64-linux-gnu/qt5/QtOpenGL;/usr/include/x86_64-linux-gnu/qt5/QtWidgets;/usr/include/x86_64-linux-gnu/qt5/QtGui " STREQUAL " ")
  set(srrg2_executor_INCLUDE_DIRS "")
  set(_include_dirs "/usr/include;/usr/include/opencv;/usr/include/x86_64-linux-gnu/qt5/;/usr/include/x86_64-linux-gnu/qt5/QtXml;/usr/include/x86_64-linux-gnu/qt5/QtCore;/usr/lib/x86_64-linux-gnu/qt5//mkspecs/linux-g++;/usr/include/x86_64-linux-gnu/qt5/QtOpenGL;/usr/include/x86_64-linux-gnu/qt5/QtWidgets;/usr/include/x86_64-linux-gnu/qt5/QtGui")
  if(NOT " " STREQUAL " ")
    set(_report "Check the issue tracker '' and consider creating a ticket if the problem has not been reported yet.")
  elseif(NOT " " STREQUAL " ")
    set(_report "Check the website '' for information and consider reporting the problem.")
  else()
    set(_report "Report the problem to the maintainer 'Giorgio Grisetti <grisetti@diag.uniroma1.it>' and request to fix the problem.")
  endif()
  foreach(idir ${_include_dirs})
    if(IS_ABSOLUTE ${idir} AND IS_DIRECTORY ${idir})
      set(include ${idir})
    elseif("${idir} " STREQUAL "include ")
      get_filename_component(include "${srrg2_executor_DIR}/../../../include" ABSOLUTE)
      if(NOT IS_DIRECTORY ${include})
        message(FATAL_ERROR "Project 'srrg2_executor' specifies '${idir}' as an include dir, which is not found.  It does not exist in '${include}'.  ${_report}")
      endif()
    else()
      message(FATAL_ERROR "Project 'srrg2_executor' specifies '${idir}' as an include dir, which is not found.  It does neither exist as an absolute directory nor in '/home/alessio/Desktop/progetto-labiagi/catkin_ws/src/srrg2_executor/srrg2_executor/${idir}'.  ${_report}")
    endif()
    _list_append_unique(srrg2_executor_INCLUDE_DIRS ${include})
  endforeach()
endif()

set(libraries "/usr/lib/x86_64-linux-gnu/libopencv_calib3d.so.3.2.0;/usr/lib/x86_64-linux-gnu/libopencv_core.so.3.2.0;/usr/lib/x86_64-linux-gnu/libopencv_features2d.so.3.2.0;/usr/lib/x86_64-linux-gnu/libopencv_flann.so.3.2.0;/usr/lib/x86_64-linux-gnu/libopencv_highgui.so.3.2.0;/usr/lib/x86_64-linux-gnu/libopencv_imgcodecs.so.3.2.0;/usr/lib/x86_64-linux-gnu/libopencv_imgproc.so.3.2.0;/usr/lib/x86_64-linux-gnu/libopencv_ml.so.3.2.0;/usr/lib/x86_64-linux-gnu/libopencv_objdetect.so.3.2.0;/usr/lib/x86_64-linux-gnu/libopencv_photo.so.3.2.0;/usr/lib/x86_64-linux-gnu/libopencv_shape.so.3.2.0;/usr/lib/x86_64-linux-gnu/libopencv_stitching.so.3.2.0;/usr/lib/x86_64-linux-gnu/libopencv_superres.so.3.2.0;/usr/lib/x86_64-linux-gnu/libopencv_video.so.3.2.0;/usr/lib/x86_64-linux-gnu/libopencv_videoio.so.3.2.0;/usr/lib/x86_64-linux-gnu/libopencv_videostab.so.3.2.0;/usr/lib/x86_64-linux-gnu/libopencv_viz.so.3.2.0;/usr/lib/x86_64-linux-gnu/libopencv_aruco.so.3.2.0;/usr/lib/x86_64-linux-gnu/libopencv_bgsegm.so.3.2.0;/usr/lib/x86_64-linux-gnu/libopencv_bioinspired.so.3.2.0;/usr/lib/x86_64-linux-gnu/libopencv_ccalib.so.3.2.0;/usr/lib/x86_64-linux-gnu/libopencv_datasets.so.3.2.0;/usr/lib/x86_64-linux-gnu/libopencv_dpm.so.3.2.0;/usr/lib/x86_64-linux-gnu/libopencv_face.so.3.2.0;/usr/lib/x86_64-linux-gnu/libopencv_freetype.so.3.2.0;/usr/lib/x86_64-linux-gnu/libopencv_fuzzy.so.3.2.0;/usr/lib/x86_64-linux-gnu/libopencv_hdf.so.3.2.0;/usr/lib/x86_64-linux-gnu/libopencv_line_descriptor.so.3.2.0;/usr/lib/x86_64-linux-gnu/libopencv_optflow.so.3.2.0;/usr/lib/x86_64-linux-gnu/libopencv_phase_unwrapping.so.3.2.0;/usr/lib/x86_64-linux-gnu/libopencv_plot.so.3.2.0;/usr/lib/x86_64-linux-gnu/libopencv_reg.so.3.2.0;/usr/lib/x86_64-linux-gnu/libopencv_rgbd.so.3.2.0;/usr/lib/x86_64-linux-gnu/libopencv_saliency.so.3.2.0;/usr/lib/x86_64-linux-gnu/libopencv_stereo.so.3.2.0;/usr/lib/x86_64-linux-gnu/libopencv_structured_light.so.3.2.0;/usr/lib/x86_64-linux-gnu/libopencv_surface_matching.so.3.2.0;/usr/lib/x86_64-linux-gnu/libopencv_text.so.3.2.0;/usr/lib/x86_64-linux-gnu/libopencv_ximgproc.so.3.2.0;/usr/lib/x86_64-linux-gnu/libopencv_xobjdetect.so.3.2.0;/usr/lib/x86_64-linux-gnu/libopencv_xphoto.so.3.2.0;/usr/lib/x86_64-linux-gnu/libQt5OpenGL.so.5.9.5;/usr/lib/x86_64-linux-gnu/libQt5Xml.so.5.9.5;/usr/lib/x86_64-linux-gnu/libQt5Gui.so.5.9.5")
foreach(library ${libraries})
  # keep build configuration keywords, target names and absolute libraries as-is
  if("${library}" MATCHES "^(debug|optimized|general)$")
    list(APPEND srrg2_executor_LIBRARIES ${library})
  elseif(${library} MATCHES "^-l")
    list(APPEND srrg2_executor_LIBRARIES ${library})
  elseif(${library} MATCHES "^-")
    # This is a linker flag/option (like -pthread)
    # There's no standard variable for these, so create an interface library to hold it
    if(NOT srrg2_executor_NUM_DUMMY_TARGETS)
      set(srrg2_executor_NUM_DUMMY_TARGETS 0)
    endif()
    # Make sure the target name is unique
    set(interface_target_name "catkin::srrg2_executor::wrapped-linker-option${srrg2_executor_NUM_DUMMY_TARGETS}")
    while(TARGET "${interface_target_name}")
      math(EXPR srrg2_executor_NUM_DUMMY_TARGETS "${srrg2_executor_NUM_DUMMY_TARGETS}+1")
      set(interface_target_name "catkin::srrg2_executor::wrapped-linker-option${srrg2_executor_NUM_DUMMY_TARGETS}")
    endwhile()
    add_library("${interface_target_name}" INTERFACE IMPORTED)
    if("${CMAKE_VERSION}" VERSION_LESS "3.13.0")
      set_property(
        TARGET
        "${interface_target_name}"
        APPEND PROPERTY
        INTERFACE_LINK_LIBRARIES "${library}")
    else()
      target_link_options("${interface_target_name}" INTERFACE "${library}")
    endif()
    list(APPEND srrg2_executor_LIBRARIES "${interface_target_name}")
  elseif(TARGET ${library})
    list(APPEND srrg2_executor_LIBRARIES ${library})
  elseif(IS_ABSOLUTE ${library})
    list(APPEND srrg2_executor_LIBRARIES ${library})
  else()
    set(lib_path "")
    set(lib "${library}-NOTFOUND")
    # since the path where the library is found is returned we have to iterate over the paths manually
    foreach(path /home/alessio/Desktop/progetto-labiagi/catkin_ws/devel/.private/srrg2_executor/lib;/home/alessio/Desktop/progetto-labiagi/catkin_ws/devel/lib;/home/alessio/Desktop/progetto-labiagi/catkin_ws/devel/.private/catkin_tools_prebuild/lib;/home/alessio/Desktop/labiagi/workspaces/srrg2_labiagi/devel/lib;/opt/ros/melodic/lib;/home/alessio/catkin_ws/devel/lib)
      find_library(lib ${library}
        PATHS ${path}
        NO_DEFAULT_PATH NO_CMAKE_FIND_ROOT_PATH)
      if(lib)
        set(lib_path ${path})
        break()
      endif()
    endforeach()
    if(lib)
      _list_append_unique(srrg2_executor_LIBRARY_DIRS ${lib_path})
      list(APPEND srrg2_executor_LIBRARIES ${lib})
    else()
      # as a fall back for non-catkin libraries try to search globally
      find_library(lib ${library})
      if(NOT lib)
        message(FATAL_ERROR "Project '${PROJECT_NAME}' tried to find library '${library}'.  The library is neither a target nor built/installed properly.  Did you compile project 'srrg2_executor'?  Did you find_package() it before the subdirectory containing its code is included?")
      endif()
      list(APPEND srrg2_executor_LIBRARIES ${lib})
    endif()
  endif()
endforeach()

set(srrg2_executor_EXPORTED_TARGETS "")
# create dummy targets for exported code generation targets to make life of users easier
foreach(t ${srrg2_executor_EXPORTED_TARGETS})
  if(NOT TARGET ${t})
    add_custom_target(${t})
  endif()
endforeach()

set(depends "srrg2_core;srrg2_core_ros;srrg2_qgl_viewport;srrg2_qgl_viewport_ros")
foreach(depend ${depends})
  string(REPLACE " " ";" depend_list ${depend})
  # the package name of the dependency must be kept in a unique variable so that it is not overwritten in recursive calls
  list(GET depend_list 0 srrg2_executor_dep)
  list(LENGTH depend_list count)
  if(${count} EQUAL 1)
    # simple dependencies must only be find_package()-ed once
    if(NOT ${srrg2_executor_dep}_FOUND)
      find_package(${srrg2_executor_dep} REQUIRED NO_MODULE)
    endif()
  else()
    # dependencies with components must be find_package()-ed again
    list(REMOVE_AT depend_list 0)
    find_package(${srrg2_executor_dep} REQUIRED NO_MODULE ${depend_list})
  endif()
  _list_append_unique(srrg2_executor_INCLUDE_DIRS ${${srrg2_executor_dep}_INCLUDE_DIRS})

  # merge build configuration keywords with library names to correctly deduplicate
  _pack_libraries_with_build_configuration(srrg2_executor_LIBRARIES ${srrg2_executor_LIBRARIES})
  _pack_libraries_with_build_configuration(_libraries ${${srrg2_executor_dep}_LIBRARIES})
  _list_append_deduplicate(srrg2_executor_LIBRARIES ${_libraries})
  # undo build configuration keyword merging after deduplication
  _unpack_libraries_with_build_configuration(srrg2_executor_LIBRARIES ${srrg2_executor_LIBRARIES})

  _list_append_unique(srrg2_executor_LIBRARY_DIRS ${${srrg2_executor_dep}_LIBRARY_DIRS})
  list(APPEND srrg2_executor_EXPORTED_TARGETS ${${srrg2_executor_dep}_EXPORTED_TARGETS})
endforeach()

set(pkg_cfg_extras "")
foreach(extra ${pkg_cfg_extras})
  if(NOT IS_ABSOLUTE ${extra})
    set(extra ${srrg2_executor_DIR}/${extra})
  endif()
  include(${extra})
endforeach()
