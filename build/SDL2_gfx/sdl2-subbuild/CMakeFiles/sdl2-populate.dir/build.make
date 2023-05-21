# CMAKE generated file: DO NOT EDIT!
# Generated by "MinGW Makefiles" Generator, CMake Version 3.26

# Delete rule output on recipe failure.
.DELETE_ON_ERROR:

#=============================================================================
# Special targets provided by cmake.

# Disable implicit rules so canonical targets will work.
.SUFFIXES:

# Disable VCS-based implicit rules.
% : %,v

# Disable VCS-based implicit rules.
% : RCS/%

# Disable VCS-based implicit rules.
% : RCS/%,v

# Disable VCS-based implicit rules.
% : SCCS/s.%

# Disable VCS-based implicit rules.
% : s.%

.SUFFIXES: .hpux_make_needs_suffix_list

# Command-line flag to silence nested $(MAKE).
$(VERBOSE)MAKESILENT = -s

#Suppress display of executed commands.
$(VERBOSE).SILENT:

# A target that is always out of date.
cmake_force:
.PHONY : cmake_force

#=============================================================================
# Set environment variables for the build.

SHELL = cmd.exe

# The CMake executable.
CMAKE_COMMAND = "C:\Program Files\CMake\bin\cmake.exe"

# The command to remove a file.
RM = "C:\Program Files\CMake\bin\cmake.exe" -E rm -f

# Escaping for special characters.
EQUALS = =

# The top-level source directory on which CMake was run.
CMAKE_SOURCE_DIR = C:\tp_projekt_3\build\SDL2_gfx\sdl2-subbuild

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = C:\tp_projekt_3\build\SDL2_gfx\sdl2-subbuild

# Utility rule file for sdl2-populate.

# Include any custom commands dependencies for this target.
include CMakeFiles/sdl2-populate.dir/compiler_depend.make

# Include the progress variables for this target.
include CMakeFiles/sdl2-populate.dir/progress.make

CMakeFiles/sdl2-populate: CMakeFiles/sdl2-populate-complete

CMakeFiles/sdl2-populate-complete: sdl2-populate-prefix/src/sdl2-populate-stamp/sdl2-populate-install
CMakeFiles/sdl2-populate-complete: sdl2-populate-prefix/src/sdl2-populate-stamp/sdl2-populate-mkdir
CMakeFiles/sdl2-populate-complete: sdl2-populate-prefix/src/sdl2-populate-stamp/sdl2-populate-download
CMakeFiles/sdl2-populate-complete: sdl2-populate-prefix/src/sdl2-populate-stamp/sdl2-populate-update
CMakeFiles/sdl2-populate-complete: sdl2-populate-prefix/src/sdl2-populate-stamp/sdl2-populate-patch
CMakeFiles/sdl2-populate-complete: sdl2-populate-prefix/src/sdl2-populate-stamp/sdl2-populate-configure
CMakeFiles/sdl2-populate-complete: sdl2-populate-prefix/src/sdl2-populate-stamp/sdl2-populate-build
CMakeFiles/sdl2-populate-complete: sdl2-populate-prefix/src/sdl2-populate-stamp/sdl2-populate-install
CMakeFiles/sdl2-populate-complete: sdl2-populate-prefix/src/sdl2-populate-stamp/sdl2-populate-test
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --blue --bold --progress-dir=C:\tp_projekt_3\build\SDL2_gfx\sdl2-subbuild\CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Completed 'sdl2-populate'"
	"C:\Program Files\CMake\bin\cmake.exe" -E make_directory C:/tp_projekt_3/build/SDL2_gfx/sdl2-subbuild/CMakeFiles
	"C:\Program Files\CMake\bin\cmake.exe" -E touch C:/tp_projekt_3/build/SDL2_gfx/sdl2-subbuild/CMakeFiles/sdl2-populate-complete
	"C:\Program Files\CMake\bin\cmake.exe" -E touch C:/tp_projekt_3/build/SDL2_gfx/sdl2-subbuild/sdl2-populate-prefix/src/sdl2-populate-stamp/sdl2-populate-done

sdl2-populate-prefix/src/sdl2-populate-stamp/sdl2-populate-build: sdl2-populate-prefix/src/sdl2-populate-stamp/sdl2-populate-configure
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --blue --bold --progress-dir=C:\tp_projekt_3\build\SDL2_gfx\sdl2-subbuild\CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "No build step for 'sdl2-populate'"
	cd /d C:\tp_projekt_3\build\SDL2_gfx\sdl2-build && "C:\Program Files\CMake\bin\cmake.exe" -E echo_append
	cd /d C:\tp_projekt_3\build\SDL2_gfx\sdl2-build && "C:\Program Files\CMake\bin\cmake.exe" -E touch C:/tp_projekt_3/build/SDL2_gfx/sdl2-subbuild/sdl2-populate-prefix/src/sdl2-populate-stamp/sdl2-populate-build

sdl2-populate-prefix/src/sdl2-populate-stamp/sdl2-populate-configure: sdl2-populate-prefix/tmp/sdl2-populate-cfgcmd.txt
sdl2-populate-prefix/src/sdl2-populate-stamp/sdl2-populate-configure: sdl2-populate-prefix/src/sdl2-populate-stamp/sdl2-populate-patch
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --blue --bold --progress-dir=C:\tp_projekt_3\build\SDL2_gfx\sdl2-subbuild\CMakeFiles --progress-num=$(CMAKE_PROGRESS_3) "No configure step for 'sdl2-populate'"
	cd /d C:\tp_projekt_3\build\SDL2_gfx\sdl2-build && "C:\Program Files\CMake\bin\cmake.exe" -E echo_append
	cd /d C:\tp_projekt_3\build\SDL2_gfx\sdl2-build && "C:\Program Files\CMake\bin\cmake.exe" -E touch C:/tp_projekt_3/build/SDL2_gfx/sdl2-subbuild/sdl2-populate-prefix/src/sdl2-populate-stamp/sdl2-populate-configure

sdl2-populate-prefix/src/sdl2-populate-stamp/sdl2-populate-download: sdl2-populate-prefix/src/sdl2-populate-stamp/download-sdl2-populate.cmake
sdl2-populate-prefix/src/sdl2-populate-stamp/sdl2-populate-download: sdl2-populate-prefix/src/sdl2-populate-stamp/sdl2-populate-urlinfo.txt
sdl2-populate-prefix/src/sdl2-populate-stamp/sdl2-populate-download: sdl2-populate-prefix/src/sdl2-populate-stamp/sdl2-populate-mkdir
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --blue --bold --progress-dir=C:\tp_projekt_3\build\SDL2_gfx\sdl2-subbuild\CMakeFiles --progress-num=$(CMAKE_PROGRESS_4) "Performing download step (download, verify and extract) for 'sdl2-populate'"
	cd /d C:\tp_projekt_3\build\SDL2_gfx && "C:\Program Files\CMake\bin\cmake.exe" -P C:/tp_projekt_3/build/SDL2_gfx/sdl2-subbuild/sdl2-populate-prefix/src/sdl2-populate-stamp/download-sdl2-populate.cmake
	cd /d C:\tp_projekt_3\build\SDL2_gfx && "C:\Program Files\CMake\bin\cmake.exe" -P C:/tp_projekt_3/build/SDL2_gfx/sdl2-subbuild/sdl2-populate-prefix/src/sdl2-populate-stamp/verify-sdl2-populate.cmake
	cd /d C:\tp_projekt_3\build\SDL2_gfx && "C:\Program Files\CMake\bin\cmake.exe" -P C:/tp_projekt_3/build/SDL2_gfx/sdl2-subbuild/sdl2-populate-prefix/src/sdl2-populate-stamp/extract-sdl2-populate.cmake
	cd /d C:\tp_projekt_3\build\SDL2_gfx && "C:\Program Files\CMake\bin\cmake.exe" -E touch C:/tp_projekt_3/build/SDL2_gfx/sdl2-subbuild/sdl2-populate-prefix/src/sdl2-populate-stamp/sdl2-populate-download

sdl2-populate-prefix/src/sdl2-populate-stamp/sdl2-populate-install: sdl2-populate-prefix/src/sdl2-populate-stamp/sdl2-populate-build
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --blue --bold --progress-dir=C:\tp_projekt_3\build\SDL2_gfx\sdl2-subbuild\CMakeFiles --progress-num=$(CMAKE_PROGRESS_5) "No install step for 'sdl2-populate'"
	cd /d C:\tp_projekt_3\build\SDL2_gfx\sdl2-build && "C:\Program Files\CMake\bin\cmake.exe" -E echo_append
	cd /d C:\tp_projekt_3\build\SDL2_gfx\sdl2-build && "C:\Program Files\CMake\bin\cmake.exe" -E touch C:/tp_projekt_3/build/SDL2_gfx/sdl2-subbuild/sdl2-populate-prefix/src/sdl2-populate-stamp/sdl2-populate-install

sdl2-populate-prefix/src/sdl2-populate-stamp/sdl2-populate-mkdir:
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --blue --bold --progress-dir=C:\tp_projekt_3\build\SDL2_gfx\sdl2-subbuild\CMakeFiles --progress-num=$(CMAKE_PROGRESS_6) "Creating directories for 'sdl2-populate'"
	"C:\Program Files\CMake\bin\cmake.exe" -Dcfgdir= -P C:/tp_projekt_3/build/SDL2_gfx/sdl2-subbuild/sdl2-populate-prefix/tmp/sdl2-populate-mkdirs.cmake
	"C:\Program Files\CMake\bin\cmake.exe" -E touch C:/tp_projekt_3/build/SDL2_gfx/sdl2-subbuild/sdl2-populate-prefix/src/sdl2-populate-stamp/sdl2-populate-mkdir

sdl2-populate-prefix/src/sdl2-populate-stamp/sdl2-populate-patch: sdl2-populate-prefix/src/sdl2-populate-stamp/sdl2-populate-update
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --blue --bold --progress-dir=C:\tp_projekt_3\build\SDL2_gfx\sdl2-subbuild\CMakeFiles --progress-num=$(CMAKE_PROGRESS_7) "No patch step for 'sdl2-populate'"
	"C:\Program Files\CMake\bin\cmake.exe" -E echo_append
	"C:\Program Files\CMake\bin\cmake.exe" -E touch C:/tp_projekt_3/build/SDL2_gfx/sdl2-subbuild/sdl2-populate-prefix/src/sdl2-populate-stamp/sdl2-populate-patch

sdl2-populate-prefix/src/sdl2-populate-stamp/sdl2-populate-test: sdl2-populate-prefix/src/sdl2-populate-stamp/sdl2-populate-install
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --blue --bold --progress-dir=C:\tp_projekt_3\build\SDL2_gfx\sdl2-subbuild\CMakeFiles --progress-num=$(CMAKE_PROGRESS_8) "No test step for 'sdl2-populate'"
	cd /d C:\tp_projekt_3\build\SDL2_gfx\sdl2-build && "C:\Program Files\CMake\bin\cmake.exe" -E echo_append
	cd /d C:\tp_projekt_3\build\SDL2_gfx\sdl2-build && "C:\Program Files\CMake\bin\cmake.exe" -E touch C:/tp_projekt_3/build/SDL2_gfx/sdl2-subbuild/sdl2-populate-prefix/src/sdl2-populate-stamp/sdl2-populate-test

sdl2-populate-prefix/src/sdl2-populate-stamp/sdl2-populate-update: sdl2-populate-prefix/src/sdl2-populate-stamp/sdl2-populate-download
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --blue --bold --progress-dir=C:\tp_projekt_3\build\SDL2_gfx\sdl2-subbuild\CMakeFiles --progress-num=$(CMAKE_PROGRESS_9) "No update step for 'sdl2-populate'"
	"C:\Program Files\CMake\bin\cmake.exe" -E echo_append
	"C:\Program Files\CMake\bin\cmake.exe" -E touch C:/tp_projekt_3/build/SDL2_gfx/sdl2-subbuild/sdl2-populate-prefix/src/sdl2-populate-stamp/sdl2-populate-update

sdl2-populate: CMakeFiles/sdl2-populate
sdl2-populate: CMakeFiles/sdl2-populate-complete
sdl2-populate: sdl2-populate-prefix/src/sdl2-populate-stamp/sdl2-populate-build
sdl2-populate: sdl2-populate-prefix/src/sdl2-populate-stamp/sdl2-populate-configure
sdl2-populate: sdl2-populate-prefix/src/sdl2-populate-stamp/sdl2-populate-download
sdl2-populate: sdl2-populate-prefix/src/sdl2-populate-stamp/sdl2-populate-install
sdl2-populate: sdl2-populate-prefix/src/sdl2-populate-stamp/sdl2-populate-mkdir
sdl2-populate: sdl2-populate-prefix/src/sdl2-populate-stamp/sdl2-populate-patch
sdl2-populate: sdl2-populate-prefix/src/sdl2-populate-stamp/sdl2-populate-test
sdl2-populate: sdl2-populate-prefix/src/sdl2-populate-stamp/sdl2-populate-update
sdl2-populate: CMakeFiles/sdl2-populate.dir/build.make
.PHONY : sdl2-populate

# Rule to build all files generated by this target.
CMakeFiles/sdl2-populate.dir/build: sdl2-populate
.PHONY : CMakeFiles/sdl2-populate.dir/build

CMakeFiles/sdl2-populate.dir/clean:
	$(CMAKE_COMMAND) -P CMakeFiles\sdl2-populate.dir\cmake_clean.cmake
.PHONY : CMakeFiles/sdl2-populate.dir/clean

CMakeFiles/sdl2-populate.dir/depend:
	$(CMAKE_COMMAND) -E cmake_depends "MinGW Makefiles" C:\tp_projekt_3\build\SDL2_gfx\sdl2-subbuild C:\tp_projekt_3\build\SDL2_gfx\sdl2-subbuild C:\tp_projekt_3\build\SDL2_gfx\sdl2-subbuild C:\tp_projekt_3\build\SDL2_gfx\sdl2-subbuild C:\tp_projekt_3\build\SDL2_gfx\sdl2-subbuild\CMakeFiles\sdl2-populate.dir\DependInfo.cmake --color=$(COLOR)
.PHONY : CMakeFiles/sdl2-populate.dir/depend

