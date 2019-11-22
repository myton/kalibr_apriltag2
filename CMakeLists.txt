cmake_minimum_required(VERSION 2.8.3)
project(apriltags)
SET(CMAKE_BUILD_TYPE Debug)
#add_definitions(-DLOG)
IF(NOT CMAKE_BUILD_TYPE)
  SET(CMAKE_BUILD_TYPE Release)
#ELSE()
#  add_definitions(-DLOG)
ENDIF()
#add_definitions(-DLOG)

MESSAGE("Build type: " ${CMAKE_BUILD_TYPE})

set(USE_MYNTEYE_SDK True)

set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS}  -Wall  -O3 -march=native ")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall   -O3 -march=native -g")

# Check C++11 or C++0x support
include(CheckCXXCompilerFlag)
CHECK_CXX_COMPILER_FLAG("-std=c++11" COMPILER_SUPPORTS_CXX11)
CHECK_CXX_COMPILER_FLAG("-std=c++0x" COMPILER_SUPPORTS_CXX0X)
if(COMPILER_SUPPORTS_CXX11)
   set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")
   add_definitions(-DCOMPILEDWITHC11)
   message(STATUS "Using flag -std=c++11.")
elseif(COMPILER_SUPPORTS_CXX0X)
   set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++0x")
   add_definitions(-DCOMPILEDWITHC0X)
   message(STATUS "Using flag -std=c++0x.")
else()
   message(FATAL_ERROR "The compiler ${CMAKE_CXX_COMPILER} has no C++11 support. Please use a different C++ compiler.")
endif()

LIST(APPEND CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/cmake_modules)
set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)

find_package(Eigen3 REQUIRED)
find_package(OpenCV REQUIRED)
#add_definitions(-fPIC -O3)
include_directories(
include
${EIGEN3_INCLUDE_DIR}
)

file(GLOB SOURCE_FILES "src/apriltags/*.cc")
add_library(${PROJECT_NAME} ${SOURCE_FILES})
target_link_libraries(${PROJECT_NAME} ${OpenCV_LIBS})

add_executable(mynt_demo ${PROJECT_SOURCE_DIR}/examples/mynt_demo.cc)
target_link_libraries(mynt_demo ${PROJECT_NAME})
