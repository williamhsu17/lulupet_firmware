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
include ulp/CMakeFiles/ulp.dir/depend.make

# Include the progress variables for this target.
include ulp/CMakeFiles/ulp.dir/progress.make

# Include the compile flags for this target's objects.
include ulp/CMakeFiles/ulp.dir/flags.make

ulp/CMakeFiles/ulp.dir/ulp.c.obj: ulp/CMakeFiles/ulp.dir/flags.make
ulp/CMakeFiles/ulp.dir/ulp.c.obj: /home/victor0891/test/esp-who/esp-idf/components/ulp/ulp.c
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/victor0891/test/esp-who/examples/single_chip/camera_web_server/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building C object ulp/CMakeFiles/ulp.dir/ulp.c.obj"
	cd /home/victor0891/test/esp-who/examples/single_chip/camera_web_server/build/ulp && /home/victor0891/esp/xtensa-esp32-elf/bin/xtensa-esp32-elf-gcc $(C_DEFINES) $(C_INCLUDES) $(C_FLAGS) -o CMakeFiles/ulp.dir/ulp.c.obj   -c /home/victor0891/test/esp-who/esp-idf/components/ulp/ulp.c

ulp/CMakeFiles/ulp.dir/ulp.c.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing C source to CMakeFiles/ulp.dir/ulp.c.i"
	cd /home/victor0891/test/esp-who/examples/single_chip/camera_web_server/build/ulp && /home/victor0891/esp/xtensa-esp32-elf/bin/xtensa-esp32-elf-gcc $(C_DEFINES) $(C_INCLUDES) $(C_FLAGS) -E /home/victor0891/test/esp-who/esp-idf/components/ulp/ulp.c > CMakeFiles/ulp.dir/ulp.c.i

ulp/CMakeFiles/ulp.dir/ulp.c.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling C source to assembly CMakeFiles/ulp.dir/ulp.c.s"
	cd /home/victor0891/test/esp-who/examples/single_chip/camera_web_server/build/ulp && /home/victor0891/esp/xtensa-esp32-elf/bin/xtensa-esp32-elf-gcc $(C_DEFINES) $(C_INCLUDES) $(C_FLAGS) -S /home/victor0891/test/esp-who/esp-idf/components/ulp/ulp.c -o CMakeFiles/ulp.dir/ulp.c.s

ulp/CMakeFiles/ulp.dir/ulp.c.obj.requires:

.PHONY : ulp/CMakeFiles/ulp.dir/ulp.c.obj.requires

ulp/CMakeFiles/ulp.dir/ulp.c.obj.provides: ulp/CMakeFiles/ulp.dir/ulp.c.obj.requires
	$(MAKE) -f ulp/CMakeFiles/ulp.dir/build.make ulp/CMakeFiles/ulp.dir/ulp.c.obj.provides.build
.PHONY : ulp/CMakeFiles/ulp.dir/ulp.c.obj.provides

ulp/CMakeFiles/ulp.dir/ulp.c.obj.provides.build: ulp/CMakeFiles/ulp.dir/ulp.c.obj


ulp/CMakeFiles/ulp.dir/ulp_macro.c.obj: ulp/CMakeFiles/ulp.dir/flags.make
ulp/CMakeFiles/ulp.dir/ulp_macro.c.obj: /home/victor0891/test/esp-who/esp-idf/components/ulp/ulp_macro.c
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/victor0891/test/esp-who/examples/single_chip/camera_web_server/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Building C object ulp/CMakeFiles/ulp.dir/ulp_macro.c.obj"
	cd /home/victor0891/test/esp-who/examples/single_chip/camera_web_server/build/ulp && /home/victor0891/esp/xtensa-esp32-elf/bin/xtensa-esp32-elf-gcc $(C_DEFINES) $(C_INCLUDES) $(C_FLAGS) -o CMakeFiles/ulp.dir/ulp_macro.c.obj   -c /home/victor0891/test/esp-who/esp-idf/components/ulp/ulp_macro.c

ulp/CMakeFiles/ulp.dir/ulp_macro.c.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing C source to CMakeFiles/ulp.dir/ulp_macro.c.i"
	cd /home/victor0891/test/esp-who/examples/single_chip/camera_web_server/build/ulp && /home/victor0891/esp/xtensa-esp32-elf/bin/xtensa-esp32-elf-gcc $(C_DEFINES) $(C_INCLUDES) $(C_FLAGS) -E /home/victor0891/test/esp-who/esp-idf/components/ulp/ulp_macro.c > CMakeFiles/ulp.dir/ulp_macro.c.i

ulp/CMakeFiles/ulp.dir/ulp_macro.c.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling C source to assembly CMakeFiles/ulp.dir/ulp_macro.c.s"
	cd /home/victor0891/test/esp-who/examples/single_chip/camera_web_server/build/ulp && /home/victor0891/esp/xtensa-esp32-elf/bin/xtensa-esp32-elf-gcc $(C_DEFINES) $(C_INCLUDES) $(C_FLAGS) -S /home/victor0891/test/esp-who/esp-idf/components/ulp/ulp_macro.c -o CMakeFiles/ulp.dir/ulp_macro.c.s

ulp/CMakeFiles/ulp.dir/ulp_macro.c.obj.requires:

.PHONY : ulp/CMakeFiles/ulp.dir/ulp_macro.c.obj.requires

ulp/CMakeFiles/ulp.dir/ulp_macro.c.obj.provides: ulp/CMakeFiles/ulp.dir/ulp_macro.c.obj.requires
	$(MAKE) -f ulp/CMakeFiles/ulp.dir/build.make ulp/CMakeFiles/ulp.dir/ulp_macro.c.obj.provides.build
.PHONY : ulp/CMakeFiles/ulp.dir/ulp_macro.c.obj.provides

ulp/CMakeFiles/ulp.dir/ulp_macro.c.obj.provides.build: ulp/CMakeFiles/ulp.dir/ulp_macro.c.obj


# Object files for target ulp
ulp_OBJECTS = \
"CMakeFiles/ulp.dir/ulp.c.obj" \
"CMakeFiles/ulp.dir/ulp_macro.c.obj"

# External object files for target ulp
ulp_EXTERNAL_OBJECTS =

ulp/libulp.a: ulp/CMakeFiles/ulp.dir/ulp.c.obj
ulp/libulp.a: ulp/CMakeFiles/ulp.dir/ulp_macro.c.obj
ulp/libulp.a: ulp/CMakeFiles/ulp.dir/build.make
ulp/libulp.a: ulp/CMakeFiles/ulp.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir=/home/victor0891/test/esp-who/examples/single_chip/camera_web_server/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_3) "Linking C static library libulp.a"
	cd /home/victor0891/test/esp-who/examples/single_chip/camera_web_server/build/ulp && $(CMAKE_COMMAND) -P CMakeFiles/ulp.dir/cmake_clean_target.cmake
	cd /home/victor0891/test/esp-who/examples/single_chip/camera_web_server/build/ulp && $(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/ulp.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
ulp/CMakeFiles/ulp.dir/build: ulp/libulp.a

.PHONY : ulp/CMakeFiles/ulp.dir/build

ulp/CMakeFiles/ulp.dir/requires: ulp/CMakeFiles/ulp.dir/ulp.c.obj.requires
ulp/CMakeFiles/ulp.dir/requires: ulp/CMakeFiles/ulp.dir/ulp_macro.c.obj.requires

.PHONY : ulp/CMakeFiles/ulp.dir/requires

ulp/CMakeFiles/ulp.dir/clean:
	cd /home/victor0891/test/esp-who/examples/single_chip/camera_web_server/build/ulp && $(CMAKE_COMMAND) -P CMakeFiles/ulp.dir/cmake_clean.cmake
.PHONY : ulp/CMakeFiles/ulp.dir/clean

ulp/CMakeFiles/ulp.dir/depend:
	cd /home/victor0891/test/esp-who/examples/single_chip/camera_web_server/build && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/victor0891/test/esp-who/examples/single_chip/camera_web_server /home/victor0891/test/esp-who/esp-idf/components/ulp /home/victor0891/test/esp-who/examples/single_chip/camera_web_server/build /home/victor0891/test/esp-who/examples/single_chip/camera_web_server/build/ulp /home/victor0891/test/esp-who/examples/single_chip/camera_web_server/build/ulp/CMakeFiles/ulp.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : ulp/CMakeFiles/ulp.dir/depend

