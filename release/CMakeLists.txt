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
find_package(OpenGL REQUIRED)


set(GLFW_BUILD_DOCS OFF CACHE BOOL "" FORCE)
set(GLFW_BUILD_TESTS OFF CACHE BOOL "" FORCE)
set(GLFW_BUILD_EXAMPLES OFF CACHE BOOL "" FORCE)

add_subdirectory(libraries/glew-2.2.0/build/cmake)
add_subdirectory(libraries/glfw-3.3.2)
add_subdirectory(libraries/glm-0.9.9.7)
add_subdirectory(libraries/freetype-2.10.1)

#include headers
include_directories(
  include
  Eigen
)

# add the executable
add_executable(
  MACPP
  src/engine.cpp
  src/macpp.cpp
  src/world.cpp
  src/WObject.cpp
  src/graphics/Window.cpp
  src/graphics/Camera.cpp
  src/graphics/Shader.cpp
  src/graphics/Mesh.cpp
  src/cell.cpp
  src/weed.cpp
  src/agent.cpp
  src/randomwalk.cpp
  src/informationGain.cpp
  src/orca.cpp
)
target_link_libraries(
  MACPP
  glfw
  ${GLFW_LIBRARIES}
  glew
  glm
  freetype
  yaml-cpp
  ${Boost_LIBRARIES}
)
