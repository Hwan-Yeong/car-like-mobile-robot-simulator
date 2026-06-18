cmake_minimum_required(VERSION 3.20)
project(CarLikeMobileRobotSimulator LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release)
endif()

include(FetchContent)

set(SFML_BUILD_AUDIO OFF CACHE BOOL "" FORCE)
set(SFML_BUILD_NETWORK OFF CACHE BOOL "" FORCE)
set(BUILD_SHARED_LIBS OFF CACHE BOOL "" FORCE)

FetchContent_Declare(
  SFML
  GIT_REPOSITORY https://github.com/SFML/SFML.git
  GIT_TAG 2.6.1
  GIT_SHALLOW ON
)
FetchContent_MakeAvailable(SFML)

file(GLOB_RECURSE SIM_SOURCES CONFIGURE_DEPENDS
  ${CMAKE_SOURCE_DIR}/src/*.cpp
)

add_executable(car_sim ${SIM_SOURCES})

target_include_directories(car_sim PRIVATE
  ${CMAKE_SOURCE_DIR}/include
)

target_link_libraries(car_sim PRIVATE
  sfml-graphics
  sfml-window
  sfml-system
)
