# generated from catkin/cmake/template/pkg.context.pc.in
CATKIN_PACKAGE_PREFIX = ""
PROJECT_PKG_CONFIG_INCLUDE_DIRS = "${prefix}/include;/usr/include/QGLViewer".split(';') if "${prefix}/include;/usr/include/QGLViewer" != "" else []
PROJECT_CATKIN_DEPENDS = "srrg_cmake_modules;srrg2_qgl_viewport;srrg2_laser_slam_2d;srrg2_core_ros;tf;image_transport".replace(';', ' ')
PKG_CONFIG_LIBRARIES_WITH_PREFIX = "-lsrrg2_navigation_2d_ros_library;-ltf_helpers_library;/usr/lib/x86_64-linux-gnu/libQGLViewer-qt5.so;/usr/lib/x86_64-linux-gnu/libglut.so;/usr/lib/x86_64-linux-gnu/libXmu.so;/usr/lib/x86_64-linux-gnu/libXi.so".split(';') if "-lsrrg2_navigation_2d_ros_library;-ltf_helpers_library;/usr/lib/x86_64-linux-gnu/libQGLViewer-qt5.so;/usr/lib/x86_64-linux-gnu/libglut.so;/usr/lib/x86_64-linux-gnu/libXmu.so;/usr/lib/x86_64-linux-gnu/libXi.so" != "" else []
PROJECT_NAME = "srrg2_navigation_2d_ros"
PROJECT_SPACE_DIR = "/home/alessiohu/Desktop/progetto-labiagi/catkin_ws/install"
PROJECT_VERSION = "0.9.0"
