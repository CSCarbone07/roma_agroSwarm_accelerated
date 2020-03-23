cmake_minimum_required (VERSION 3.1 FATAL_ERROR)
project(MACPP)

# Enable C++14 and warnings
set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)
set(CMAKE_CXX_FLAGS  "${CMAKE_CXX_FLAGS} -Wall -Wextra")

# find modules
find_package(PkgConfig)
pkg_check_modules(YamlCpp yaml-cpp)
find_package(Boost 1.58 REQUIRED COMPONENTS program_options)

#include headers
include_directories(
  include
  Eigen
)

# add the executable
add_executable(
  MACPP
  src/macpp.cpp
  src/world.cpp
  src/agent.cpp
  src/randomwalk.cpp
  src/informationGain.cpp
  src/orca.cpp
)
target_link_libraries(
  MACPP
  yaml-cpp
  ${Boost_LIBRARIES}
)
