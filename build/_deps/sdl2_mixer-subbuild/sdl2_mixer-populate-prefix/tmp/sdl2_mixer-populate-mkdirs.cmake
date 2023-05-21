# Distributed under the OSI-approved BSD 3-Clause License.  See accompanying
# file Copyright.txt or https://cmake.org/licensing for details.

cmake_minimum_required(VERSION 3.5)

file(MAKE_DIRECTORY
  "C:/tp_projekt_3/build/_deps/sdl2_mixer-src"
  "C:/tp_projekt_3/build/_deps/sdl2_mixer-build"
  "C:/tp_projekt_3/build/_deps/sdl2_mixer-subbuild/sdl2_mixer-populate-prefix"
  "C:/tp_projekt_3/build/_deps/sdl2_mixer-subbuild/sdl2_mixer-populate-prefix/tmp"
  "C:/tp_projekt_3/build/_deps/sdl2_mixer-subbuild/sdl2_mixer-populate-prefix/src/sdl2_mixer-populate-stamp"
  "C:/tp_projekt_3/build/_deps/sdl2_mixer-subbuild/sdl2_mixer-populate-prefix/src"
  "C:/tp_projekt_3/build/_deps/sdl2_mixer-subbuild/sdl2_mixer-populate-prefix/src/sdl2_mixer-populate-stamp"
)

set(configSubDirs )
foreach(subDir IN LISTS configSubDirs)
    file(MAKE_DIRECTORY "C:/tp_projekt_3/build/_deps/sdl2_mixer-subbuild/sdl2_mixer-populate-prefix/src/sdl2_mixer-populate-stamp/${subDir}")
endforeach()
if(cfgdir)
  file(MAKE_DIRECTORY "C:/tp_projekt_3/build/_deps/sdl2_mixer-subbuild/sdl2_mixer-populate-prefix/src/sdl2_mixer-populate-stamp${cfgdir}") # cfgdir has leading slash
endif()
