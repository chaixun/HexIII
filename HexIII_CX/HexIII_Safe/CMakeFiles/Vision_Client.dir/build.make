# CMAKE generated file: DO NOT EDIT!
# Generated by "Unix Makefiles" Generator, CMake Version 2.8

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
CMAKE_SOURCE_DIR = /home/hex/HexIII_CX/HexIII_Safe

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /home/hex/HexIII_CX/HexIII_Safe

# Include any dependencies generated for this target.
include CMakeFiles/Vision_Client.dir/depend.make

# Include the progress variables for this target.
include CMakeFiles/Vision_Client.dir/progress.make

# Include the compile flags for this target's objects.
include CMakeFiles/Vision_Client.dir/flags.make

CMakeFiles/Vision_Client.dir/Vision_Client.cpp.o: CMakeFiles/Vision_Client.dir/flags.make
CMakeFiles/Vision_Client.dir/Vision_Client.cpp.o: Vision_Client.cpp
	$(CMAKE_COMMAND) -E cmake_progress_report /home/hex/HexIII_CX/HexIII_Safe/CMakeFiles $(CMAKE_PROGRESS_1)
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Building CXX object CMakeFiles/Vision_Client.dir/Vision_Client.cpp.o"
	/usr/bin/c++   $(CXX_DEFINES) $(CXX_FLAGS) -o CMakeFiles/Vision_Client.dir/Vision_Client.cpp.o -c /home/hex/HexIII_CX/HexIII_Safe/Vision_Client.cpp

CMakeFiles/Vision_Client.dir/Vision_Client.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/Vision_Client.dir/Vision_Client.cpp.i"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_FLAGS) -E /home/hex/HexIII_CX/HexIII_Safe/Vision_Client.cpp > CMakeFiles/Vision_Client.dir/Vision_Client.cpp.i

CMakeFiles/Vision_Client.dir/Vision_Client.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/Vision_Client.dir/Vision_Client.cpp.s"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_FLAGS) -S /home/hex/HexIII_CX/HexIII_Safe/Vision_Client.cpp -o CMakeFiles/Vision_Client.dir/Vision_Client.cpp.s

CMakeFiles/Vision_Client.dir/Vision_Client.cpp.o.requires:
.PHONY : CMakeFiles/Vision_Client.dir/Vision_Client.cpp.o.requires

CMakeFiles/Vision_Client.dir/Vision_Client.cpp.o.provides: CMakeFiles/Vision_Client.dir/Vision_Client.cpp.o.requires
	$(MAKE) -f CMakeFiles/Vision_Client.dir/build.make CMakeFiles/Vision_Client.dir/Vision_Client.cpp.o.provides.build
.PHONY : CMakeFiles/Vision_Client.dir/Vision_Client.cpp.o.provides

CMakeFiles/Vision_Client.dir/Vision_Client.cpp.o.provides.build: CMakeFiles/Vision_Client.dir/Vision_Client.cpp.o

CMakeFiles/Vision_Client.dir/Vision_main.cpp.o: CMakeFiles/Vision_Client.dir/flags.make
CMakeFiles/Vision_Client.dir/Vision_main.cpp.o: Vision_main.cpp
	$(CMAKE_COMMAND) -E cmake_progress_report /home/hex/HexIII_CX/HexIII_Safe/CMakeFiles $(CMAKE_PROGRESS_2)
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Building CXX object CMakeFiles/Vision_Client.dir/Vision_main.cpp.o"
	/usr/bin/c++   $(CXX_DEFINES) $(CXX_FLAGS) -o CMakeFiles/Vision_Client.dir/Vision_main.cpp.o -c /home/hex/HexIII_CX/HexIII_Safe/Vision_main.cpp

CMakeFiles/Vision_Client.dir/Vision_main.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/Vision_Client.dir/Vision_main.cpp.i"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_FLAGS) -E /home/hex/HexIII_CX/HexIII_Safe/Vision_main.cpp > CMakeFiles/Vision_Client.dir/Vision_main.cpp.i

CMakeFiles/Vision_Client.dir/Vision_main.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/Vision_Client.dir/Vision_main.cpp.s"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_FLAGS) -S /home/hex/HexIII_CX/HexIII_Safe/Vision_main.cpp -o CMakeFiles/Vision_Client.dir/Vision_main.cpp.s

CMakeFiles/Vision_Client.dir/Vision_main.cpp.o.requires:
.PHONY : CMakeFiles/Vision_Client.dir/Vision_main.cpp.o.requires

CMakeFiles/Vision_Client.dir/Vision_main.cpp.o.provides: CMakeFiles/Vision_Client.dir/Vision_main.cpp.o.requires
	$(MAKE) -f CMakeFiles/Vision_Client.dir/build.make CMakeFiles/Vision_Client.dir/Vision_main.cpp.o.provides.build
.PHONY : CMakeFiles/Vision_Client.dir/Vision_main.cpp.o.provides

CMakeFiles/Vision_Client.dir/Vision_main.cpp.o.provides.build: CMakeFiles/Vision_Client.dir/Vision_main.cpp.o

CMakeFiles/Vision_Client.dir/Kinect_Test.cpp.o: CMakeFiles/Vision_Client.dir/flags.make
CMakeFiles/Vision_Client.dir/Kinect_Test.cpp.o: Kinect_Test.cpp
	$(CMAKE_COMMAND) -E cmake_progress_report /home/hex/HexIII_CX/HexIII_Safe/CMakeFiles $(CMAKE_PROGRESS_3)
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Building CXX object CMakeFiles/Vision_Client.dir/Kinect_Test.cpp.o"
	/usr/bin/c++   $(CXX_DEFINES) $(CXX_FLAGS) -o CMakeFiles/Vision_Client.dir/Kinect_Test.cpp.o -c /home/hex/HexIII_CX/HexIII_Safe/Kinect_Test.cpp

CMakeFiles/Vision_Client.dir/Kinect_Test.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/Vision_Client.dir/Kinect_Test.cpp.i"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_FLAGS) -E /home/hex/HexIII_CX/HexIII_Safe/Kinect_Test.cpp > CMakeFiles/Vision_Client.dir/Kinect_Test.cpp.i

CMakeFiles/Vision_Client.dir/Kinect_Test.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/Vision_Client.dir/Kinect_Test.cpp.s"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_FLAGS) -S /home/hex/HexIII_CX/HexIII_Safe/Kinect_Test.cpp -o CMakeFiles/Vision_Client.dir/Kinect_Test.cpp.s

CMakeFiles/Vision_Client.dir/Kinect_Test.cpp.o.requires:
.PHONY : CMakeFiles/Vision_Client.dir/Kinect_Test.cpp.o.requires

CMakeFiles/Vision_Client.dir/Kinect_Test.cpp.o.provides: CMakeFiles/Vision_Client.dir/Kinect_Test.cpp.o.requires
	$(MAKE) -f CMakeFiles/Vision_Client.dir/build.make CMakeFiles/Vision_Client.dir/Kinect_Test.cpp.o.provides.build
.PHONY : CMakeFiles/Vision_Client.dir/Kinect_Test.cpp.o.provides

CMakeFiles/Vision_Client.dir/Kinect_Test.cpp.o.provides.build: CMakeFiles/Vision_Client.dir/Kinect_Test.cpp.o

# Object files for target Vision_Client
Vision_Client_OBJECTS = \
"CMakeFiles/Vision_Client.dir/Vision_Client.cpp.o" \
"CMakeFiles/Vision_Client.dir/Vision_main.cpp.o" \
"CMakeFiles/Vision_Client.dir/Kinect_Test.cpp.o"

# External object files for target Vision_Client
Vision_Client_EXTERNAL_OBJECTS =

bin/Vision_Client: CMakeFiles/Vision_Client.dir/Vision_Client.cpp.o
bin/Vision_Client: CMakeFiles/Vision_Client.dir/Vision_main.cpp.o
bin/Vision_Client: CMakeFiles/Vision_Client.dir/Kinect_Test.cpp.o
bin/Vision_Client: CMakeFiles/Vision_Client.dir/build.make
bin/Vision_Client: /usr/lib/x86_64-linux-gnu/libboost_system.so
bin/Vision_Client: /usr/lib/x86_64-linux-gnu/libboost_filesystem.so
bin/Vision_Client: /usr/lib/x86_64-linux-gnu/libboost_thread.so
bin/Vision_Client: /usr/lib/x86_64-linux-gnu/libboost_date_time.so
bin/Vision_Client: /usr/lib/x86_64-linux-gnu/libboost_iostreams.so
bin/Vision_Client: /usr/lib/x86_64-linux-gnu/libboost_serialization.so
bin/Vision_Client: /usr/lib/x86_64-linux-gnu/libboost_chrono.so
bin/Vision_Client: /usr/lib/x86_64-linux-gnu/libpthread.so
bin/Vision_Client: /usr/lib/libpcl_common.so
bin/Vision_Client: /usr/lib/x86_64-linux-gnu/libflann_cpp_s.a
bin/Vision_Client: /usr/lib/libpcl_kdtree.so
bin/Vision_Client: /usr/lib/libpcl_octree.so
bin/Vision_Client: /usr/lib/libpcl_search.so
bin/Vision_Client: /usr/lib/x86_64-linux-gnu/libqhull.so
bin/Vision_Client: /usr/lib/libpcl_surface.so
bin/Vision_Client: /usr/lib/libpcl_sample_consensus.so
bin/Vision_Client: /usr/lib/libOpenNI.so
bin/Vision_Client: /usr/lib/libOpenNI2.so
bin/Vision_Client: /usr/lib/libvtkCommon.so.5.8.0
bin/Vision_Client: /usr/lib/libvtkFiltering.so.5.8.0
bin/Vision_Client: /usr/lib/libvtkImaging.so.5.8.0
bin/Vision_Client: /usr/lib/libvtkGraphics.so.5.8.0
bin/Vision_Client: /usr/lib/libvtkGenericFiltering.so.5.8.0
bin/Vision_Client: /usr/lib/libvtkIO.so.5.8.0
bin/Vision_Client: /usr/lib/libvtkRendering.so.5.8.0
bin/Vision_Client: /usr/lib/libvtkVolumeRendering.so.5.8.0
bin/Vision_Client: /usr/lib/libvtkHybrid.so.5.8.0
bin/Vision_Client: /usr/lib/libvtkWidgets.so.5.8.0
bin/Vision_Client: /usr/lib/libvtkParallel.so.5.8.0
bin/Vision_Client: /usr/lib/libvtkInfovis.so.5.8.0
bin/Vision_Client: /usr/lib/libvtkGeovis.so.5.8.0
bin/Vision_Client: /usr/lib/libvtkViews.so.5.8.0
bin/Vision_Client: /usr/lib/libvtkCharts.so.5.8.0
bin/Vision_Client: /usr/lib/libpcl_io.so
bin/Vision_Client: /usr/lib/libpcl_filters.so
bin/Vision_Client: /usr/lib/libpcl_features.so
bin/Vision_Client: /usr/lib/libpcl_keypoints.so
bin/Vision_Client: /usr/lib/libpcl_registration.so
bin/Vision_Client: /usr/lib/libpcl_segmentation.so
bin/Vision_Client: /usr/lib/libpcl_recognition.so
bin/Vision_Client: /usr/lib/libpcl_visualization.so
bin/Vision_Client: /usr/lib/libpcl_people.so
bin/Vision_Client: /usr/lib/libpcl_outofcore.so
bin/Vision_Client: /usr/lib/libpcl_tracking.so
bin/Vision_Client: /usr/lib/libpcl_apps.so
bin/Vision_Client: /usr/lib/x86_64-linux-gnu/libboost_system.so
bin/Vision_Client: /usr/lib/x86_64-linux-gnu/libboost_filesystem.so
bin/Vision_Client: /usr/lib/x86_64-linux-gnu/libboost_thread.so
bin/Vision_Client: /usr/lib/x86_64-linux-gnu/libboost_date_time.so
bin/Vision_Client: /usr/lib/x86_64-linux-gnu/libboost_iostreams.so
bin/Vision_Client: /usr/lib/x86_64-linux-gnu/libboost_serialization.so
bin/Vision_Client: /usr/lib/x86_64-linux-gnu/libboost_chrono.so
bin/Vision_Client: /usr/lib/x86_64-linux-gnu/libpthread.so
bin/Vision_Client: /usr/lib/x86_64-linux-gnu/libqhull.so
bin/Vision_Client: /usr/lib/libOpenNI.so
bin/Vision_Client: /usr/lib/libOpenNI2.so
bin/Vision_Client: /usr/lib/x86_64-linux-gnu/libflann_cpp_s.a
bin/Vision_Client: /usr/lib/libvtkCommon.so.5.8.0
bin/Vision_Client: /usr/lib/libvtkFiltering.so.5.8.0
bin/Vision_Client: /usr/lib/libvtkImaging.so.5.8.0
bin/Vision_Client: /usr/lib/libvtkGraphics.so.5.8.0
bin/Vision_Client: /usr/lib/libvtkGenericFiltering.so.5.8.0
bin/Vision_Client: /usr/lib/libvtkIO.so.5.8.0
bin/Vision_Client: /usr/lib/libvtkRendering.so.5.8.0
bin/Vision_Client: /usr/lib/libvtkVolumeRendering.so.5.8.0
bin/Vision_Client: /usr/lib/libvtkHybrid.so.5.8.0
bin/Vision_Client: /usr/lib/libvtkWidgets.so.5.8.0
bin/Vision_Client: /usr/lib/libvtkParallel.so.5.8.0
bin/Vision_Client: /usr/lib/libvtkInfovis.so.5.8.0
bin/Vision_Client: /usr/lib/libvtkGeovis.so.5.8.0
bin/Vision_Client: /usr/lib/libvtkViews.so.5.8.0
bin/Vision_Client: /usr/lib/libvtkCharts.so.5.8.0
bin/Vision_Client: /usr/lib/libpcl_common.so
bin/Vision_Client: /usr/lib/libpcl_kdtree.so
bin/Vision_Client: /usr/lib/libpcl_octree.so
bin/Vision_Client: /usr/lib/libpcl_search.so
bin/Vision_Client: /usr/lib/libpcl_surface.so
bin/Vision_Client: /usr/lib/libpcl_sample_consensus.so
bin/Vision_Client: /usr/lib/libpcl_io.so
bin/Vision_Client: /usr/lib/libpcl_filters.so
bin/Vision_Client: /usr/lib/libpcl_features.so
bin/Vision_Client: /usr/lib/libpcl_keypoints.so
bin/Vision_Client: /usr/lib/libpcl_registration.so
bin/Vision_Client: /usr/lib/libpcl_segmentation.so
bin/Vision_Client: /usr/lib/libpcl_recognition.so
bin/Vision_Client: /usr/lib/libpcl_visualization.so
bin/Vision_Client: /usr/lib/libpcl_people.so
bin/Vision_Client: /usr/lib/libpcl_outofcore.so
bin/Vision_Client: /usr/lib/libpcl_tracking.so
bin/Vision_Client: /usr/lib/libpcl_apps.so
bin/Vision_Client: /usr/lib/libvtkViews.so.5.8.0
bin/Vision_Client: /usr/lib/libvtkInfovis.so.5.8.0
bin/Vision_Client: /usr/lib/libvtkWidgets.so.5.8.0
bin/Vision_Client: /usr/lib/libvtkVolumeRendering.so.5.8.0
bin/Vision_Client: /usr/lib/libvtkHybrid.so.5.8.0
bin/Vision_Client: /usr/lib/libvtkParallel.so.5.8.0
bin/Vision_Client: /usr/lib/libvtkRendering.so.5.8.0
bin/Vision_Client: /usr/lib/libvtkImaging.so.5.8.0
bin/Vision_Client: /usr/lib/libvtkGraphics.so.5.8.0
bin/Vision_Client: /usr/lib/libvtkIO.so.5.8.0
bin/Vision_Client: /usr/lib/libvtkFiltering.so.5.8.0
bin/Vision_Client: /usr/lib/libvtkCommon.so.5.8.0
bin/Vision_Client: /usr/lib/libvtksys.so.5.8.0
bin/Vision_Client: CMakeFiles/Vision_Client.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --red --bold "Linking CXX executable bin/Vision_Client"
	$(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/Vision_Client.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
CMakeFiles/Vision_Client.dir/build: bin/Vision_Client
.PHONY : CMakeFiles/Vision_Client.dir/build

CMakeFiles/Vision_Client.dir/requires: CMakeFiles/Vision_Client.dir/Vision_Client.cpp.o.requires
CMakeFiles/Vision_Client.dir/requires: CMakeFiles/Vision_Client.dir/Vision_main.cpp.o.requires
CMakeFiles/Vision_Client.dir/requires: CMakeFiles/Vision_Client.dir/Kinect_Test.cpp.o.requires
.PHONY : CMakeFiles/Vision_Client.dir/requires

CMakeFiles/Vision_Client.dir/clean:
	$(CMAKE_COMMAND) -P CMakeFiles/Vision_Client.dir/cmake_clean.cmake
.PHONY : CMakeFiles/Vision_Client.dir/clean

CMakeFiles/Vision_Client.dir/depend:
	cd /home/hex/HexIII_CX/HexIII_Safe && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/hex/HexIII_CX/HexIII_Safe /home/hex/HexIII_CX/HexIII_Safe /home/hex/HexIII_CX/HexIII_Safe /home/hex/HexIII_CX/HexIII_Safe /home/hex/HexIII_CX/HexIII_Safe/CMakeFiles/Vision_Client.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : CMakeFiles/Vision_Client.dir/depend

