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
CMAKE_SOURCE_DIR = C:\tp_projekt_3\build\_deps\sdl2_mixer-subbuild

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = C:\tp_projekt_3\build\_deps\sdl2_mixer-subbuild

# Utility rule file for sdl2_mixer-populate.

# Include any custom commands dependencies for this target.
include CMakeFiles/sdl2_mixer-populate.dir/compiler_depend.make

# Include the progress variables for this target.
include CMakeFiles/sdl2_mixer-populate.dir/progress.make

CMakeFiles/sdl2_mixer-populate: CMakeFiles/sdl2_mixer-populate-complete

CMakeFiles/sdl2_mixer-populate-complete: sdl2_mixer-populate-prefix/src/sdl2_mixer-populate-stamp/sdl2_mixer-populate-install
CMakeFiles/sdl2_mixer-populate-complete: sdl2_mixer-populate-prefix/src/sdl2_mixer-populate-stamp/sdl2_mixer-populate-mkdir
CMakeFiles/sdl2_mixer-populate-complete: sdl2_mixer-populate-prefix/src/sdl2_mixer-populate-stamp/sdl2_mixer-populate-download
CMakeFiles/sdl2_mixer-populate-complete: sdl2_mixer-populate-prefix/src/sdl2_mixer-populate-stamp/sdl2_mixer-populate-update
CMakeFiles/sdl2_mixer-populate-complete: sdl2_mixer-populate-prefix/src/sdl2_mixer-populate-stamp/sdl2_mixer-populate-patch
CMakeFiles/sdl2_mixer-populate-complete: sdl2_mixer-populate-prefix/src/sdl2_mixer-populate-stamp/sdl2_mixer-populate-configure
CMakeFiles/sdl2_mixer-populate-complete: sdl2_mixer-populate-prefix/src/sdl2_mixer-populate-stamp/sdl2_mixer-populate-build
CMakeFiles/sdl2_mixer-populate-complete: sdl2_mixer-populate-prefix/src/sdl2_mixer-populate-stamp/sdl2_mixer-populate-install
CMakeFiles/sdl2_mixer-populate-complete: sdl2_mixer-populate-prefix/src/sdl2_mixer-populate-stamp/sdl2_mixer-populate-test
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --blue --bold --progress-dir=C:\tp_projekt_3\build\_deps\sdl2_mixer-subbuild\CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Completed 'sdl2_mixer-populate'"
	"C:\Program Files\CMake\bin\cmake.exe" -E make_directory C:/tp_projekt_3/build/_deps/sdl2_mixer-subbuild/CMakeFiles
	"C:\Program Files\CMake\bin\cmake.exe" -E touch C:/tp_projekt_3/build/_deps/sdl2_mixer-subbuild/CMakeFiles/sdl2_mixer-populate-complete
	"C:\Program Files\CMake\bin\cmake.exe" -E touch C:/tp_projekt_3/build/_deps/sdl2_mixer-subbuild/sdl2_mixer-populate-prefix/src/sdl2_mixer-populate-stamp/sdl2_mixer-populate-done

sdl2_mixer-populate-prefix/src/sdl2_mixer-populate-stamp/sdl2_mixer-populate-build: sdl2_mixer-populate-prefix/src/sdl2_mixer-populate-stamp/sdl2_mixer-populate-configure
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --blue --bold --progress-dir=C:\tp_projekt_3\build\_deps\sdl2_mixer-subbuild\CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "No build step for 'sdl2_mixer-populate'"
	cd /d C:\tp_projekt_3\build\_deps\sdl2_mixer-build && "C:\Program Files\CMake\bin\cmake.exe" -E echo_append
	cd /d C:\tp_projekt_3\build\_deps\sdl2_mixer-build && "C:\Program Files\CMake\bin\cmake.exe" -E touch C:/tp_projekt_3/build/_deps/sdl2_mixer-subbuild/sdl2_mixer-populate-prefix/src/sdl2_mixer-populate-stamp/sdl2_mixer-populate-build

sdl2_mixer-populate-prefix/src/sdl2_mixer-populate-stamp/sdl2_mixer-populate-configure: sdl2_mixer-populate-prefix/tmp/sdl2_mixer-populate-cfgcmd.txt
sdl2_mixer-populate-prefix/src/sdl2_mixer-populate-stamp/sdl2_mixer-populate-configure: sdl2_mixer-populate-prefix/src/sdl2_mixer-populate-stamp/sdl2_mixer-populate-patch
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --blue --bold --progress-dir=C:\tp_projekt_3\build\_deps\sdl2_mixer-subbuild\CMakeFiles --progress-num=$(CMAKE_PROGRESS_3) "No configure step for 'sdl2_mixer-populate'"
	cd /d C:\tp_projekt_3\build\_deps\sdl2_mixer-build && "C:\Program Files\CMake\bin\cmake.exe" -E echo_append
	cd /d C:\tp_projekt_3\build\_deps\sdl2_mixer-build && "C:\Program Files\CMake\bin\cmake.exe" -E touch C:/tp_projekt_3/build/_deps/sdl2_mixer-subbuild/sdl2_mixer-populate-prefix/src/sdl2_mixer-populate-stamp/sdl2_mixer-populate-configure

sdl2_mixer-populate-prefix/src/sdl2_mixer-populate-stamp/sdl2_mixer-populate-download: sdl2_mixer-populate-prefix/src/sdl2_mixer-populate-stamp/download-sdl2_mixer-populate.cmake
sdl2_mixer-populate-prefix/src/sdl2_mixer-populate-stamp/sdl2_mixer-populate-download: sdl2_mixer-populate-prefix/src/sdl2_mixer-populate-stamp/sdl2_mixer-populate-urlinfo.txt
sdl2_mixer-populate-prefix/src/sdl2_mixer-populate-stamp/sdl2_mixer-populate-download: sdl2_mixer-populate-prefix/src/sdl2_mixer-populate-stamp/sdl2_mixer-populate-mkdir
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --blue --bold --progress-dir=C:\tp_projekt_3\build\_deps\sdl2_mixer-subbuild\CMakeFiles --progress-num=$(CMAKE_PROGRESS_4) "Performing download step (download, verify and extract) for 'sdl2_mixer-populate'"
	cd /d C:\tp_projekt_3\build\_deps && "C:\Program Files\CMake\bin\cmake.exe" -P C:/tp_projekt_3/build/_deps/sdl2_mixer-subbuild/sdl2_mixer-populate-prefix/src/sdl2_mixer-populate-stamp/download-sdl2_mixer-populate.cmake
	cd /d C:\tp_projekt_3\build\_deps && "C:\Program Files\CMake\bin\cmake.exe" -P C:/tp_projekt_3/build/_deps/sdl2_mixer-subbuild/sdl2_mixer-populate-prefix/src/sdl2_mixer-populate-stamp/verify-sdl2_mixer-populate.cmake
	cd /d C:\tp_projekt_3\build\_deps && "C:\Program Files\CMake\bin\cmake.exe" -P C:/tp_projekt_3/build/_deps/sdl2_mixer-subbuild/sdl2_mixer-populate-prefix/src/sdl2_mixer-populate-stamp/extract-sdl2_mixer-populate.cmake
	cd /d C:\tp_projekt_3\build\_deps && "C:\Program Files\CMake\bin\cmake.exe" -E touch C:/tp_projekt_3/build/_deps/sdl2_mixer-subbuild/sdl2_mixer-populate-prefix/src/sdl2_mixer-populate-stamp/sdl2_mixer-populate-download

sdl2_mixer-populate-prefix/src/sdl2_mixer-populate-stamp/sdl2_mixer-populate-install: sdl2_mixer-populate-prefix/src/sdl2_mixer-populate-stamp/sdl2_mixer-populate-build
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --blue --bold --progress-dir=C:\tp_projekt_3\build\_deps\sdl2_mixer-subbuild\CMakeFiles --progress-num=$(CMAKE_PROGRESS_5) "No install step for 'sdl2_mixer-populate'"
	cd /d C:\tp_projekt_3\build\_deps\sdl2_mixer-build && "C:\Program Files\CMake\bin\cmake.exe" -E echo_append
	cd /d C:\tp_projekt_3\build\_deps\sdl2_mixer-build && "C:\Program Files\CMake\bin\cmake.exe" -E touch C:/tp_projekt_3/build/_deps/sdl2_mixer-subbuild/sdl2_mixer-populate-prefix/src/sdl2_mixer-populate-stamp/sdl2_mixer-populate-install

sdl2_mixer-populate-prefix/src/sdl2_mixer-populate-stamp/sdl2_mixer-populate-mkdir:
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --blue --bold --progress-dir=C:\tp_projekt_3\build\_deps\sdl2_mixer-subbuild\CMakeFiles --progress-num=$(CMAKE_PROGRESS_6) "Creating directories for 'sdl2_mixer-populate'"
	"C:\Program Files\CMake\bin\cmake.exe" -Dcfgdir= -P C:/tp_projekt_3/build/_deps/sdl2_mixer-subbuild/sdl2_mixer-populate-prefix/tmp/sdl2_mixer-populate-mkdirs.cmake
	"C:\Program Files\CMake\bin\cmake.exe" -E touch C:/tp_projekt_3/build/_deps/sdl2_mixer-subbuild/sdl2_mixer-populate-prefix/src/sdl2_mixer-populate-stamp/sdl2_mixer-populate-mkdir

sdl2_mixer-populate-prefix/src/sdl2_mixer-populate-stamp/sdl2_mixer-populate-patch: sdl2_mixer-populate-prefix/src/sdl2_mixer-populate-stamp/sdl2_mixer-populate-update
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --blue --bold --progress-dir=C:\tp_projekt_3\build\_deps\sdl2_mixer-subbuild\CMakeFiles --progress-num=$(CMAKE_PROGRESS_7) "No patch step for 'sdl2_mixer-populate'"
	"C:\Program Files\CMake\bin\cmake.exe" -E echo_append
	"C:\Program Files\CMake\bin\cmake.exe" -E touch C:/tp_projekt_3/build/_deps/sdl2_mixer-subbuild/sdl2_mixer-populate-prefix/src/sdl2_mixer-populate-stamp/sdl2_mixer-populate-patch

sdl2_mixer-populate-prefix/src/sdl2_mixer-populate-stamp/sdl2_mixer-populate-test: sdl2_mixer-populate-prefix/src/sdl2_mixer-populate-stamp/sdl2_mixer-populate-install
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --blue --bold --progress-dir=C:\tp_projekt_3\build\_deps\sdl2_mixer-subbuild\CMakeFiles --progress-num=$(CMAKE_PROGRESS_8) "No test step for 'sdl2_mixer-populate'"
	cd /d C:\tp_projekt_3\build\_deps\sdl2_mixer-build && "C:\Program Files\CMake\bin\cmake.exe" -E echo_append
	cd /d C:\tp_projekt_3\build\_deps\sdl2_mixer-build && "C:\Program Files\CMake\bin\cmake.exe" -E touch C:/tp_projekt_3/build/_deps/sdl2_mixer-subbuild/sdl2_mixer-populate-prefix/src/sdl2_mixer-populate-stamp/sdl2_mixer-populate-test

sdl2_mixer-populate-prefix/src/sdl2_mixer-populate-stamp/sdl2_mixer-populate-update: sdl2_mixer-populate-prefix/src/sdl2_mixer-populate-stamp/sdl2_mixer-populate-download
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --blue --bold --progress-dir=C:\tp_projekt_3\build\_deps\sdl2_mixer-subbuild\CMakeFiles --progress-num=$(CMAKE_PROGRESS_9) "No update step for 'sdl2_mixer-populate'"
	"C:\Program Files\CMake\bin\cmake.exe" -E echo_append
	"C:\Program Files\CMake\bin\cmake.exe" -E touch C:/tp_projekt_3/build/_deps/sdl2_mixer-subbuild/sdl2_mixer-populate-prefix/src/sdl2_mixer-populate-stamp/sdl2_mixer-populate-update

sdl2_mixer-populate: CMakeFiles/sdl2_mixer-populate
sdl2_mixer-populate: CMakeFiles/sdl2_mixer-populate-complete
sdl2_mixer-populate: sdl2_mixer-populate-prefix/src/sdl2_mixer-populate-stamp/sdl2_mixer-populate-build
sdl2_mixer-populate: sdl2_mixer-populate-prefix/src/sdl2_mixer-populate-stamp/sdl2_mixer-populate-configure
sdl2_mixer-populate: sdl2_mixer-populate-prefix/src/sdl2_mixer-populate-stamp/sdl2_mixer-populate-download
sdl2_mixer-populate: sdl2_mixer-populate-prefix/src/sdl2_mixer-populate-stamp/sdl2_mixer-populate-install
sdl2_mixer-populate: sdl2_mixer-populate-prefix/src/sdl2_mixer-populate-stamp/sdl2_mixer-populate-mkdir
sdl2_mixer-populate: sdl2_mixer-populate-prefix/src/sdl2_mixer-populate-stamp/sdl2_mixer-populate-patch
sdl2_mixer-populate: sdl2_mixer-populate-prefix/src/sdl2_mixer-populate-stamp/sdl2_mixer-populate-test
sdl2_mixer-populate: sdl2_mixer-populate-prefix/src/sdl2_mixer-populate-stamp/sdl2_mixer-populate-update
sdl2_mixer-populate: CMakeFiles/sdl2_mixer-populate.dir/build.make
.PHONY : sdl2_mixer-populate

# Rule to build all files generated by this target.
CMakeFiles/sdl2_mixer-populate.dir/build: sdl2_mixer-populate
.PHONY : CMakeFiles/sdl2_mixer-populate.dir/build

CMakeFiles/sdl2_mixer-populate.dir/clean:
	$(CMAKE_COMMAND) -P CMakeFiles\sdl2_mixer-populate.dir\cmake_clean.cmake
.PHONY : CMakeFiles/sdl2_mixer-populate.dir/clean

CMakeFiles/sdl2_mixer-populate.dir/depend:
	$(CMAKE_COMMAND) -E cmake_depends "MinGW Makefiles" C:\tp_projekt_3\build\_deps\sdl2_mixer-subbuild C:\tp_projekt_3\build\_deps\sdl2_mixer-subbuild C:\tp_projekt_3\build\_deps\sdl2_mixer-subbuild C:\tp_projekt_3\build\_deps\sdl2_mixer-subbuild C:\tp_projekt_3\build\_deps\sdl2_mixer-subbuild\CMakeFiles\sdl2_mixer-populate.dir\DependInfo.cmake --color=$(COLOR)
.PHONY : CMakeFiles/sdl2_mixer-populate.dir/depend

