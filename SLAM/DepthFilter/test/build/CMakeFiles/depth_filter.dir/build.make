# CMAKE generated file: DO NOT EDIT!
# Generated by "Unix Makefiles" Generator, CMake Version 3.5

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
CMAKE_SOURCE_DIR = /home/ubuntu/Projects/PaperSummary/SLAM/DepthFilter/test

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /home/ubuntu/Projects/PaperSummary/SLAM/DepthFilter/test/build

# Include any dependencies generated for this target.
include CMakeFiles/depth_filter.dir/depend.make

# Include the progress variables for this target.
include CMakeFiles/depth_filter.dir/progress.make

# Include the compile flags for this target's objects.
include CMakeFiles/depth_filter.dir/flags.make

CMakeFiles/depth_filter.dir/main.cc.o: CMakeFiles/depth_filter.dir/flags.make
CMakeFiles/depth_filter.dir/main.cc.o: ../main.cc
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/ubuntu/Projects/PaperSummary/SLAM/DepthFilter/test/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building CXX object CMakeFiles/depth_filter.dir/main.cc.o"
	/usr/bin/c++   $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/depth_filter.dir/main.cc.o -c /home/ubuntu/Projects/PaperSummary/SLAM/DepthFilter/test/main.cc

CMakeFiles/depth_filter.dir/main.cc.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/depth_filter.dir/main.cc.i"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/ubuntu/Projects/PaperSummary/SLAM/DepthFilter/test/main.cc > CMakeFiles/depth_filter.dir/main.cc.i

CMakeFiles/depth_filter.dir/main.cc.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/depth_filter.dir/main.cc.s"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/ubuntu/Projects/PaperSummary/SLAM/DepthFilter/test/main.cc -o CMakeFiles/depth_filter.dir/main.cc.s

CMakeFiles/depth_filter.dir/main.cc.o.requires:

.PHONY : CMakeFiles/depth_filter.dir/main.cc.o.requires

CMakeFiles/depth_filter.dir/main.cc.o.provides: CMakeFiles/depth_filter.dir/main.cc.o.requires
	$(MAKE) -f CMakeFiles/depth_filter.dir/build.make CMakeFiles/depth_filter.dir/main.cc.o.provides.build
.PHONY : CMakeFiles/depth_filter.dir/main.cc.o.provides

CMakeFiles/depth_filter.dir/main.cc.o.provides.build: CMakeFiles/depth_filter.dir/main.cc.o


CMakeFiles/depth_filter.dir/utils.cc.o: CMakeFiles/depth_filter.dir/flags.make
CMakeFiles/depth_filter.dir/utils.cc.o: ../utils.cc
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/ubuntu/Projects/PaperSummary/SLAM/DepthFilter/test/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Building CXX object CMakeFiles/depth_filter.dir/utils.cc.o"
	/usr/bin/c++   $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/depth_filter.dir/utils.cc.o -c /home/ubuntu/Projects/PaperSummary/SLAM/DepthFilter/test/utils.cc

CMakeFiles/depth_filter.dir/utils.cc.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/depth_filter.dir/utils.cc.i"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/ubuntu/Projects/PaperSummary/SLAM/DepthFilter/test/utils.cc > CMakeFiles/depth_filter.dir/utils.cc.i

CMakeFiles/depth_filter.dir/utils.cc.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/depth_filter.dir/utils.cc.s"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/ubuntu/Projects/PaperSummary/SLAM/DepthFilter/test/utils.cc -o CMakeFiles/depth_filter.dir/utils.cc.s

CMakeFiles/depth_filter.dir/utils.cc.o.requires:

.PHONY : CMakeFiles/depth_filter.dir/utils.cc.o.requires

CMakeFiles/depth_filter.dir/utils.cc.o.provides: CMakeFiles/depth_filter.dir/utils.cc.o.requires
	$(MAKE) -f CMakeFiles/depth_filter.dir/build.make CMakeFiles/depth_filter.dir/utils.cc.o.provides.build
.PHONY : CMakeFiles/depth_filter.dir/utils.cc.o.provides

CMakeFiles/depth_filter.dir/utils.cc.o.provides.build: CMakeFiles/depth_filter.dir/utils.cc.o


CMakeFiles/depth_filter.dir/depth_filter.cc.o: CMakeFiles/depth_filter.dir/flags.make
CMakeFiles/depth_filter.dir/depth_filter.cc.o: ../depth_filter.cc
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/ubuntu/Projects/PaperSummary/SLAM/DepthFilter/test/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_3) "Building CXX object CMakeFiles/depth_filter.dir/depth_filter.cc.o"
	/usr/bin/c++   $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/depth_filter.dir/depth_filter.cc.o -c /home/ubuntu/Projects/PaperSummary/SLAM/DepthFilter/test/depth_filter.cc

CMakeFiles/depth_filter.dir/depth_filter.cc.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/depth_filter.dir/depth_filter.cc.i"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/ubuntu/Projects/PaperSummary/SLAM/DepthFilter/test/depth_filter.cc > CMakeFiles/depth_filter.dir/depth_filter.cc.i

CMakeFiles/depth_filter.dir/depth_filter.cc.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/depth_filter.dir/depth_filter.cc.s"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/ubuntu/Projects/PaperSummary/SLAM/DepthFilter/test/depth_filter.cc -o CMakeFiles/depth_filter.dir/depth_filter.cc.s

CMakeFiles/depth_filter.dir/depth_filter.cc.o.requires:

.PHONY : CMakeFiles/depth_filter.dir/depth_filter.cc.o.requires

CMakeFiles/depth_filter.dir/depth_filter.cc.o.provides: CMakeFiles/depth_filter.dir/depth_filter.cc.o.requires
	$(MAKE) -f CMakeFiles/depth_filter.dir/build.make CMakeFiles/depth_filter.dir/depth_filter.cc.o.provides.build
.PHONY : CMakeFiles/depth_filter.dir/depth_filter.cc.o.provides

CMakeFiles/depth_filter.dir/depth_filter.cc.o.provides.build: CMakeFiles/depth_filter.dir/depth_filter.cc.o


# Object files for target depth_filter
depth_filter_OBJECTS = \
"CMakeFiles/depth_filter.dir/main.cc.o" \
"CMakeFiles/depth_filter.dir/utils.cc.o" \
"CMakeFiles/depth_filter.dir/depth_filter.cc.o"

# External object files for target depth_filter
depth_filter_EXTERNAL_OBJECTS =

depth_filter: CMakeFiles/depth_filter.dir/main.cc.o
depth_filter: CMakeFiles/depth_filter.dir/utils.cc.o
depth_filter: CMakeFiles/depth_filter.dir/depth_filter.cc.o
depth_filter: CMakeFiles/depth_filter.dir/build.make
depth_filter: /usr/local/lib/libopencv_videostab.so.3.4.3
depth_filter: /usr/local/lib/libopencv_superres.so.3.4.3
depth_filter: /usr/local/lib/libopencv_stitching.so.3.4.3
depth_filter: /usr/local/lib/libopencv_fuzzy.so.3.4.3
depth_filter: /usr/local/lib/libopencv_face.so.3.4.3
depth_filter: /usr/local/lib/libopencv_bgsegm.so.3.4.3
depth_filter: /usr/local/lib/libopencv_stereo.so.3.4.3
depth_filter: /usr/local/lib/libopencv_structured_light.so.3.4.3
depth_filter: /usr/local/lib/libopencv_bioinspired.so.3.4.3
depth_filter: /usr/local/lib/libopencv_xphoto.so.3.4.3
depth_filter: /usr/local/lib/libopencv_tracking.so.3.4.3
depth_filter: /usr/local/lib/libopencv_optflow.so.3.4.3
depth_filter: /usr/local/lib/libopencv_line_descriptor.so.3.4.3
depth_filter: /usr/local/lib/libopencv_phase_unwrapping.so.3.4.3
depth_filter: /usr/local/lib/libopencv_surface_matching.so.3.4.3
depth_filter: /usr/local/lib/libopencv_reg.so.3.4.3
depth_filter: /usr/local/lib/libopencv_img_hash.so.3.4.3
depth_filter: /usr/local/lib/libopencv_hdf.so.3.4.3
depth_filter: /usr/local/lib/libopencv_ccalib.so.3.4.3
depth_filter: /usr/local/lib/libopencv_dnn_objdetect.so.3.4.3
depth_filter: /usr/local/lib/libopencv_sfm.so.3.4.3
depth_filter: /usr/local/lib/libopencv_freetype.so.3.4.3
depth_filter: /usr/local/lib/libopencv_ximgproc.so.3.4.3
depth_filter: /usr/local/lib/libopencv_hfs.so.3.4.3
depth_filter: /usr/local/lib/libopencv_xobjdetect.so.3.4.3
depth_filter: /usr/local/lib/libopencv_dpm.so.3.4.3
depth_filter: /usr/local/lib/libopencv_plot.so.3.4.3
depth_filter: /usr/local/lib/libopencv_rgbd.so.3.4.3
depth_filter: /usr/local/lib/libopencv_aruco.so.3.4.3
depth_filter: /usr/local/lib/libopencv_saliency.so.3.4.3
depth_filter: /usr/local/lib/libopencv_xfeatures2d.so.3.4.3
depth_filter: /home/ubuntu/Downloads/Sophus/build/libSophus.so
depth_filter: /usr/lib/x86_64-linux-gnu/libboost_thread.so
depth_filter: /usr/lib/x86_64-linux-gnu/libboost_system.so
depth_filter: /usr/lib/x86_64-linux-gnu/libboost_chrono.so
depth_filter: /usr/lib/x86_64-linux-gnu/libboost_date_time.so
depth_filter: /usr/lib/x86_64-linux-gnu/libboost_atomic.so
depth_filter: /usr/lib/x86_64-linux-gnu/libpthread.so
depth_filter: /usr/local/lib/libopencv_photo.so.3.4.3
depth_filter: /usr/local/lib/libopencv_datasets.so.3.4.3
depth_filter: /usr/local/lib/libopencv_text.so.3.4.3
depth_filter: /usr/local/lib/libopencv_dnn.so.3.4.3
depth_filter: /usr/local/lib/libopencv_shape.so.3.4.3
depth_filter: /usr/local/lib/libopencv_video.so.3.4.3
depth_filter: /usr/local/lib/libopencv_ml.so.3.4.3
depth_filter: /usr/local/lib/libopencv_objdetect.so.3.4.3
depth_filter: /usr/local/lib/libopencv_calib3d.so.3.4.3
depth_filter: /usr/local/lib/libopencv_features2d.so.3.4.3
depth_filter: /usr/local/lib/libopencv_highgui.so.3.4.3
depth_filter: /usr/local/lib/libopencv_videoio.so.3.4.3
depth_filter: /usr/local/lib/libopencv_flann.so.3.4.3
depth_filter: /usr/local/lib/libopencv_imgcodecs.so.3.4.3
depth_filter: /usr/local/lib/libopencv_imgproc.so.3.4.3
depth_filter: /usr/local/lib/libopencv_core.so.3.4.3
depth_filter: CMakeFiles/depth_filter.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir=/home/ubuntu/Projects/PaperSummary/SLAM/DepthFilter/test/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_4) "Linking CXX executable depth_filter"
	$(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/depth_filter.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
CMakeFiles/depth_filter.dir/build: depth_filter

.PHONY : CMakeFiles/depth_filter.dir/build

CMakeFiles/depth_filter.dir/requires: CMakeFiles/depth_filter.dir/main.cc.o.requires
CMakeFiles/depth_filter.dir/requires: CMakeFiles/depth_filter.dir/utils.cc.o.requires
CMakeFiles/depth_filter.dir/requires: CMakeFiles/depth_filter.dir/depth_filter.cc.o.requires

.PHONY : CMakeFiles/depth_filter.dir/requires

CMakeFiles/depth_filter.dir/clean:
	$(CMAKE_COMMAND) -P CMakeFiles/depth_filter.dir/cmake_clean.cmake
.PHONY : CMakeFiles/depth_filter.dir/clean

CMakeFiles/depth_filter.dir/depend:
	cd /home/ubuntu/Projects/PaperSummary/SLAM/DepthFilter/test/build && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/ubuntu/Projects/PaperSummary/SLAM/DepthFilter/test /home/ubuntu/Projects/PaperSummary/SLAM/DepthFilter/test /home/ubuntu/Projects/PaperSummary/SLAM/DepthFilter/test/build /home/ubuntu/Projects/PaperSummary/SLAM/DepthFilter/test/build /home/ubuntu/Projects/PaperSummary/SLAM/DepthFilter/test/build/CMakeFiles/depth_filter.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : CMakeFiles/depth_filter.dir/depend
