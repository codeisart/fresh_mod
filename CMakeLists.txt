cmake_minimum_required(VERSION 3.10)

# set the project name
project(fresh_mod VERSION 1.0)

# specify the C++ standard
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED True)
set(CMAKE_BUILD_TYPE Debug)

SET( CMAKE_EXPORT_COMPILE_COMMANDS ON )

IF( EXISTS "${CMAKE_CURRENT_BINARY_DIR}/compile_commands.json" )
  EXECUTE_PROCESS( COMMAND ${CMAKE_COMMAND} -E copy_if_different
    ${CMAKE_CURRENT_BINARY_DIR}/compile_commands.json
    ${CMAKE_CURRENT_SOURCE_DIR}/compile_commands.json
  )
ENDIF()

# add the executable
add_executable(fresh_mod main.cpp mod.cpp plm.cpp)

find_library(fftw NAMES libfftw3.a)
find_library(ncurses NAMES libncurses.a)
find_library(portaudio NAMES libportaudio.a)
find_library(system NAMES System)
find_library(coreaudio NAMES CoreAudio)
find_library(corefoundation NAMES CoreFoundation)
find_library(audiounit NAMES AudioUnit)
find_library(audiotoolbox NAMES AudioToolbox)
find_library(coreservices NAMES CoreServices)
message(STATUS ${fftw})
message(STATUS ${ncurses})
message(STATUS ${portaudio})
message(STATUS ${system})
message(STATUS ${coreaudio})
message(STATUS ${corefoundation})
message(STATUS ${audiounit})
message(STATUS ${audiotoolbox})
message(STATUS ${coreservices})

target_include_directories(fresh_mod PUBLIC /usr/local/include)
target_link_libraries(fresh_mod ${fftw} ${ncurses} ${portaudio} ${system} ${coreaudio} ${corefoundation} ${audiounit} ${audiotoolbox} ${coreservices}) 

