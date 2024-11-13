cmake_minimum_required(VERSION 3.0)
project(MC-Calib VERSION 1.2.0 DESCRIPTION "MC-Calib: A generic and robust calibration toolbox for multi-camera systems")

add_definitions(-std=c++17)
set(CMAKE_CXX_FLAGS "-std=c++17") # required for Ceres https://github.com/colmap/colmap/issues/905#issuecomment-731138700

option(USE_SANITIZERS "Enable AddressSanitizer and LeakSanitizer" OFF)

set(CMAKE_CONFIGURATION_TYPES ${CMAKE_BUILD_TYPE} CACHE STRING "" FORCE)

find_package(OpenCV REQUIRED)
include_directories( ${OpenCV_INCLUDE_DIRS} )

find_package(Ceres REQUIRED)

# boost related setup
set(Boost_USE_STATIC_LIBS ON)
set(Boost_USE_MULTITHREADED ON)
find_package(Boost COMPONENTS log system REQUIRED)
message(STATUS "Boost version: ${Boost_VERSION}")

IF(SSSE3_FOUND)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -msse3")
ENDIF(SSSE3_FOUND)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -O3")

if(ENABLE_COVERAGE)
  # set compiler flags
  set(CMAKE_CXX_FLAGS "-O0 -coverage")
  # find required tools
  find_program(LCOV lcov REQUIRED)
  find_program(GENHTML genhtml REQUIRED)

  # add coverage target
  add_custom_target(coverage
    # gather data
    COMMAND ${LCOV} --directory . --capture --output-file coverage.info --exclude '/usr/*' 
      --ignore-errors mismatch,negative
    # generate report
    COMMAND ${GENHTML} --demangle-cpp -o coverage coverage.info
    WORKING_DIRECTORY ${CMAKE_BINARY_DIR})
endif()

add_compile_options(-Wall -Wextra -Wpedantic -Werror)

if(USE_SANITIZERS)
    # sanitizers https://www.jetbrains.com/help/clion/google-sanitizers.html#Configuration
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS_DEBUG} -fsanitize=address")
endif()

include_directories(
	include
	/usr/include/opencv
	/usr/include/eigen3
	/usr/local/lib
)

######################### Documentation related ##########################
find_package(Doxygen)

# check if Doxygen is installed
find_package(Doxygen)
if (DOXYGEN_FOUND)
    # set input and output files
    set(DOXYGEN_IN ${CMAKE_CURRENT_SOURCE_DIR}/docs/Doxyfile.in)
    set(DOXYGEN_OUT ${CMAKE_CURRENT_BINARY_DIR}/Doxyfile)

    # request to configure the file
    configure_file(${DOXYGEN_IN} ${DOXYGEN_OUT} @ONLY)
    message("Doxygen build started")

    # note the option ALL which allows to build the docs together with the application
    add_custom_target( doc_doxygen ALL
        COMMAND ${DOXYGEN_EXECUTABLE} ${DOXYGEN_OUT}
        WORKING_DIRECTORY ${CMAKE_CURRENT_BINARY_DIR}
        COMMENT "Generating API documentation with Doxygen"
        VERBATIM )
else (DOXYGEN_FOUND)
  message("Doxygen need to be installed to generate the doxygen documentation")
endif (DOXYGEN_FOUND)


##########################################################################
add_subdirectory(McCalib)
add_subdirectory(apps/create_charuco_boards)
add_subdirectory(apps/calibrate)
add_subdirectory(tests)
