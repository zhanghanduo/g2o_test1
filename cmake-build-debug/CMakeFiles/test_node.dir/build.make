# CMAKE generated file: DO NOT EDIT!
# Generated by "Unix Makefiles" Generator, CMake Version 3.6

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
CMAKE_COMMAND = /home/zh/softwares/clion-2016.3/bin/cmake/bin/cmake

# The command to remove a file.
RM = /home/zh/softwares/clion-2016.3/bin/cmake/bin/cmake -E remove -f

# Escaping for special characters.
EQUALS = =

# The top-level source directory on which CMake was run.
CMAKE_SOURCE_DIR = /home/zh/softwares/g2o_test

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /home/zh/softwares/g2o_test/cmake-build-debug

# Include any dependencies generated for this target.
include CMakeFiles/test_node.dir/depend.make

# Include the progress variables for this target.
include CMakeFiles/test_node.dir/progress.make

# Include the compile flags for this target's objects.
include CMakeFiles/test_node.dir/flags.make

CMakeFiles/test_node.dir/test.cpp.o: CMakeFiles/test_node.dir/flags.make
CMakeFiles/test_node.dir/test.cpp.o: ../test.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/zh/softwares/g2o_test/cmake-build-debug/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building CXX object CMakeFiles/test_node.dir/test.cpp.o"
	/usr/bin/c++   $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/test_node.dir/test.cpp.o -c /home/zh/softwares/g2o_test/test.cpp

CMakeFiles/test_node.dir/test.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/test_node.dir/test.cpp.i"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/zh/softwares/g2o_test/test.cpp > CMakeFiles/test_node.dir/test.cpp.i

CMakeFiles/test_node.dir/test.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/test_node.dir/test.cpp.s"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/zh/softwares/g2o_test/test.cpp -o CMakeFiles/test_node.dir/test.cpp.s

CMakeFiles/test_node.dir/test.cpp.o.requires:

.PHONY : CMakeFiles/test_node.dir/test.cpp.o.requires

CMakeFiles/test_node.dir/test.cpp.o.provides: CMakeFiles/test_node.dir/test.cpp.o.requires
	$(MAKE) -f CMakeFiles/test_node.dir/build.make CMakeFiles/test_node.dir/test.cpp.o.provides.build
.PHONY : CMakeFiles/test_node.dir/test.cpp.o.provides

CMakeFiles/test_node.dir/test.cpp.o.provides.build: CMakeFiles/test_node.dir/test.cpp.o


# Object files for target test_node
test_node_OBJECTS = \
"CMakeFiles/test_node.dir/test.cpp.o"

# External object files for target test_node
test_node_EXTERNAL_OBJECTS =

test_node: CMakeFiles/test_node.dir/test.cpp.o
test_node: CMakeFiles/test_node.dir/build.make
test_node: /opt/ros/kinetic/lib/libopencv_xphoto3.so.3.1.0
test_node: /opt/ros/kinetic/lib/libopencv_xobjdetect3.so.3.1.0
test_node: /opt/ros/kinetic/lib/libopencv_tracking3.so.3.1.0
test_node: /opt/ros/kinetic/lib/libopencv_surface_matching3.so.3.1.0
test_node: /opt/ros/kinetic/lib/libopencv_structured_light3.so.3.1.0
test_node: /opt/ros/kinetic/lib/libopencv_stereo3.so.3.1.0
test_node: /opt/ros/kinetic/lib/libopencv_saliency3.so.3.1.0
test_node: /opt/ros/kinetic/lib/libopencv_rgbd3.so.3.1.0
test_node: /opt/ros/kinetic/lib/libopencv_reg3.so.3.1.0
test_node: /opt/ros/kinetic/lib/libopencv_plot3.so.3.1.0
test_node: /opt/ros/kinetic/lib/libopencv_optflow3.so.3.1.0
test_node: /opt/ros/kinetic/lib/libopencv_line_descriptor3.so.3.1.0
test_node: /opt/ros/kinetic/lib/libopencv_hdf3.so.3.1.0
test_node: /opt/ros/kinetic/lib/libopencv_fuzzy3.so.3.1.0
test_node: /opt/ros/kinetic/lib/libopencv_dpm3.so.3.1.0
test_node: /opt/ros/kinetic/lib/libopencv_dnn3.so.3.1.0
test_node: /opt/ros/kinetic/lib/libopencv_datasets3.so.3.1.0
test_node: /opt/ros/kinetic/lib/libopencv_cvv3.so.3.1.0
test_node: /opt/ros/kinetic/lib/libopencv_ccalib3.so.3.1.0
test_node: /opt/ros/kinetic/lib/libopencv_bioinspired3.so.3.1.0
test_node: /opt/ros/kinetic/lib/libopencv_bgsegm3.so.3.1.0
test_node: /opt/ros/kinetic/lib/libopencv_aruco3.so.3.1.0
test_node: /opt/ros/kinetic/lib/libopencv_viz3.so.3.1.0
test_node: /opt/ros/kinetic/lib/libopencv_videostab3.so.3.1.0
test_node: /opt/ros/kinetic/lib/libopencv_superres3.so.3.1.0
test_node: /opt/ros/kinetic/lib/libopencv_stitching3.so.3.1.0
test_node: /opt/ros/kinetic/lib/libopencv_photo3.so.3.1.0
test_node: /usr/lib/x86_64-linux-gnu/libcholmod.so
test_node: /usr/lib/x86_64-linux-gnu/libamd.so
test_node: /usr/lib/x86_64-linux-gnu/libcolamd.so
test_node: /usr/lib/x86_64-linux-gnu/libcamd.so
test_node: /usr/lib/x86_64-linux-gnu/libccolamd.so
test_node: /usr/lib/x86_64-linux-gnu/libsuitesparseconfig.so
test_node: /opt/ros/kinetic/lib/libopencv_text3.so.3.1.0
test_node: /opt/ros/kinetic/lib/libopencv_face3.so.3.1.0
test_node: /opt/ros/kinetic/lib/libopencv_ximgproc3.so.3.1.0
test_node: /opt/ros/kinetic/lib/libopencv_xfeatures2d3.so.3.1.0
test_node: /opt/ros/kinetic/lib/libopencv_shape3.so.3.1.0
test_node: /opt/ros/kinetic/lib/libopencv_video3.so.3.1.0
test_node: /opt/ros/kinetic/lib/libopencv_objdetect3.so.3.1.0
test_node: /opt/ros/kinetic/lib/libopencv_calib3d3.so.3.1.0
test_node: /opt/ros/kinetic/lib/libopencv_features2d3.so.3.1.0
test_node: /opt/ros/kinetic/lib/libopencv_ml3.so.3.1.0
test_node: /opt/ros/kinetic/lib/libopencv_highgui3.so.3.1.0
test_node: /opt/ros/kinetic/lib/libopencv_videoio3.so.3.1.0
test_node: /opt/ros/kinetic/lib/libopencv_imgcodecs3.so.3.1.0
test_node: /opt/ros/kinetic/lib/libopencv_imgproc3.so.3.1.0
test_node: /opt/ros/kinetic/lib/libopencv_flann3.so.3.1.0
test_node: /opt/ros/kinetic/lib/libopencv_core3.so.3.1.0
test_node: /usr/lib/x86_64-linux-gnu/libcholmod.so
test_node: /usr/lib/x86_64-linux-gnu/libamd.so
test_node: /usr/lib/x86_64-linux-gnu/libcolamd.so
test_node: /usr/lib/x86_64-linux-gnu/libcamd.so
test_node: /usr/lib/x86_64-linux-gnu/libccolamd.so
test_node: /usr/lib/x86_64-linux-gnu/libsuitesparseconfig.so
test_node: CMakeFiles/test_node.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir=/home/zh/softwares/g2o_test/cmake-build-debug/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Linking CXX executable test_node"
	$(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/test_node.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
CMakeFiles/test_node.dir/build: test_node

.PHONY : CMakeFiles/test_node.dir/build

CMakeFiles/test_node.dir/requires: CMakeFiles/test_node.dir/test.cpp.o.requires

.PHONY : CMakeFiles/test_node.dir/requires

CMakeFiles/test_node.dir/clean:
	$(CMAKE_COMMAND) -P CMakeFiles/test_node.dir/cmake_clean.cmake
.PHONY : CMakeFiles/test_node.dir/clean

CMakeFiles/test_node.dir/depend:
	cd /home/zh/softwares/g2o_test/cmake-build-debug && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/zh/softwares/g2o_test /home/zh/softwares/g2o_test /home/zh/softwares/g2o_test/cmake-build-debug /home/zh/softwares/g2o_test/cmake-build-debug /home/zh/softwares/g2o_test/cmake-build-debug/CMakeFiles/test_node.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : CMakeFiles/test_node.dir/depend

