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
CMAKE_SOURCE_DIR = /home/victor0891/test/esp-who/examples/single_chip/camera_web_server

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /home/victor0891/test/esp-who/examples/single_chip/camera_web_server/build

# Include any dependencies generated for this target.
include cxx/CMakeFiles/cxx.dir/depend.make

# Include the progress variables for this target.
include cxx/CMakeFiles/cxx.dir/progress.make

# Include the compile flags for this target's objects.
include cxx/CMakeFiles/cxx.dir/flags.make

cxx/CMakeFiles/cxx.dir/cxx_exception_stubs.cpp.obj: cxx/CMakeFiles/cxx.dir/flags.make
cxx/CMakeFiles/cxx.dir/cxx_exception_stubs.cpp.obj: /home/victor0891/test/esp-who/esp-idf/components/cxx/cxx_exception_stubs.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/victor0891/test/esp-who/examples/single_chip/camera_web_server/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building CXX object cxx/CMakeFiles/cxx.dir/cxx_exception_stubs.cpp.obj"
	cd /home/victor0891/test/esp-who/examples/single_chip/camera_web_server/build/cxx && /home/victor0891/esp/xtensa-esp32-elf/bin/xtensa-esp32-elf-g++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/cxx.dir/cxx_exception_stubs.cpp.obj -c /home/victor0891/test/esp-who/esp-idf/components/cxx/cxx_exception_stubs.cpp

cxx/CMakeFiles/cxx.dir/cxx_exception_stubs.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/cxx.dir/cxx_exception_stubs.cpp.i"
	cd /home/victor0891/test/esp-who/examples/single_chip/camera_web_server/build/cxx && /home/victor0891/esp/xtensa-esp32-elf/bin/xtensa-esp32-elf-g++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/victor0891/test/esp-who/esp-idf/components/cxx/cxx_exception_stubs.cpp > CMakeFiles/cxx.dir/cxx_exception_stubs.cpp.i

cxx/CMakeFiles/cxx.dir/cxx_exception_stubs.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/cxx.dir/cxx_exception_stubs.cpp.s"
	cd /home/victor0891/test/esp-who/examples/single_chip/camera_web_server/build/cxx && /home/victor0891/esp/xtensa-esp32-elf/bin/xtensa-esp32-elf-g++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/victor0891/test/esp-who/esp-idf/components/cxx/cxx_exception_stubs.cpp -o CMakeFiles/cxx.dir/cxx_exception_stubs.cpp.s

cxx/CMakeFiles/cxx.dir/cxx_exception_stubs.cpp.obj.requires:

.PHONY : cxx/CMakeFiles/cxx.dir/cxx_exception_stubs.cpp.obj.requires

cxx/CMakeFiles/cxx.dir/cxx_exception_stubs.cpp.obj.provides: cxx/CMakeFiles/cxx.dir/cxx_exception_stubs.cpp.obj.requires
	$(MAKE) -f cxx/CMakeFiles/cxx.dir/build.make cxx/CMakeFiles/cxx.dir/cxx_exception_stubs.cpp.obj.provides.build
.PHONY : cxx/CMakeFiles/cxx.dir/cxx_exception_stubs.cpp.obj.provides

cxx/CMakeFiles/cxx.dir/cxx_exception_stubs.cpp.obj.provides.build: cxx/CMakeFiles/cxx.dir/cxx_exception_stubs.cpp.obj


cxx/CMakeFiles/cxx.dir/cxx_guards.cpp.obj: cxx/CMakeFiles/cxx.dir/flags.make
cxx/CMakeFiles/cxx.dir/cxx_guards.cpp.obj: /home/victor0891/test/esp-who/esp-idf/components/cxx/cxx_guards.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/victor0891/test/esp-who/examples/single_chip/camera_web_server/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Building CXX object cxx/CMakeFiles/cxx.dir/cxx_guards.cpp.obj"
	cd /home/victor0891/test/esp-who/examples/single_chip/camera_web_server/build/cxx && /home/victor0891/esp/xtensa-esp32-elf/bin/xtensa-esp32-elf-g++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/cxx.dir/cxx_guards.cpp.obj -c /home/victor0891/test/esp-who/esp-idf/components/cxx/cxx_guards.cpp

cxx/CMakeFiles/cxx.dir/cxx_guards.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/cxx.dir/cxx_guards.cpp.i"
	cd /home/victor0891/test/esp-who/examples/single_chip/camera_web_server/build/cxx && /home/victor0891/esp/xtensa-esp32-elf/bin/xtensa-esp32-elf-g++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/victor0891/test/esp-who/esp-idf/components/cxx/cxx_guards.cpp > CMakeFiles/cxx.dir/cxx_guards.cpp.i

cxx/CMakeFiles/cxx.dir/cxx_guards.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/cxx.dir/cxx_guards.cpp.s"
	cd /home/victor0891/test/esp-who/examples/single_chip/camera_web_server/build/cxx && /home/victor0891/esp/xtensa-esp32-elf/bin/xtensa-esp32-elf-g++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/victor0891/test/esp-who/esp-idf/components/cxx/cxx_guards.cpp -o CMakeFiles/cxx.dir/cxx_guards.cpp.s

cxx/CMakeFiles/cxx.dir/cxx_guards.cpp.obj.requires:

.PHONY : cxx/CMakeFiles/cxx.dir/cxx_guards.cpp.obj.requires

cxx/CMakeFiles/cxx.dir/cxx_guards.cpp.obj.provides: cxx/CMakeFiles/cxx.dir/cxx_guards.cpp.obj.requires
	$(MAKE) -f cxx/CMakeFiles/cxx.dir/build.make cxx/CMakeFiles/cxx.dir/cxx_guards.cpp.obj.provides.build
.PHONY : cxx/CMakeFiles/cxx.dir/cxx_guards.cpp.obj.provides

cxx/CMakeFiles/cxx.dir/cxx_guards.cpp.obj.provides.build: cxx/CMakeFiles/cxx.dir/cxx_guards.cpp.obj


# Object files for target cxx
cxx_OBJECTS = \
"CMakeFiles/cxx.dir/cxx_exception_stubs.cpp.obj" \
"CMakeFiles/cxx.dir/cxx_guards.cpp.obj"

# External object files for target cxx
cxx_EXTERNAL_OBJECTS =

cxx/libcxx.a: cxx/CMakeFiles/cxx.dir/cxx_exception_stubs.cpp.obj
cxx/libcxx.a: cxx/CMakeFiles/cxx.dir/cxx_guards.cpp.obj
cxx/libcxx.a: cxx/CMakeFiles/cxx.dir/build.make
cxx/libcxx.a: cxx/CMakeFiles/cxx.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir=/home/victor0891/test/esp-who/examples/single_chip/camera_web_server/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_3) "Linking CXX static library libcxx.a"
	cd /home/victor0891/test/esp-who/examples/single_chip/camera_web_server/build/cxx && $(CMAKE_COMMAND) -P CMakeFiles/cxx.dir/cmake_clean_target.cmake
	cd /home/victor0891/test/esp-who/examples/single_chip/camera_web_server/build/cxx && $(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/cxx.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
cxx/CMakeFiles/cxx.dir/build: cxx/libcxx.a

.PHONY : cxx/CMakeFiles/cxx.dir/build

cxx/CMakeFiles/cxx.dir/requires: cxx/CMakeFiles/cxx.dir/cxx_exception_stubs.cpp.obj.requires
cxx/CMakeFiles/cxx.dir/requires: cxx/CMakeFiles/cxx.dir/cxx_guards.cpp.obj.requires

.PHONY : cxx/CMakeFiles/cxx.dir/requires

cxx/CMakeFiles/cxx.dir/clean:
	cd /home/victor0891/test/esp-who/examples/single_chip/camera_web_server/build/cxx && $(CMAKE_COMMAND) -P CMakeFiles/cxx.dir/cmake_clean.cmake
.PHONY : cxx/CMakeFiles/cxx.dir/clean

cxx/CMakeFiles/cxx.dir/depend:
	cd /home/victor0891/test/esp-who/examples/single_chip/camera_web_server/build && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/victor0891/test/esp-who/examples/single_chip/camera_web_server /home/victor0891/test/esp-who/esp-idf/components/cxx /home/victor0891/test/esp-who/examples/single_chip/camera_web_server/build /home/victor0891/test/esp-who/examples/single_chip/camera_web_server/build/cxx /home/victor0891/test/esp-who/examples/single_chip/camera_web_server/build/cxx/CMakeFiles/cxx.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : cxx/CMakeFiles/cxx.dir/depend

