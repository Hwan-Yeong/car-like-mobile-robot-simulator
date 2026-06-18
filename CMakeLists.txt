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

# Dear ImGui has no CMakeLists.txt of its own, so only populate its source tree
# (imgui-sfml's CMakeLists picks the sources up via IMGUI_DIR below).
FetchContent_Declare(
  imgui
  GIT_REPOSITORY https://github.com/ocornut/imgui.git
  GIT_TAG v1.91.6
  GIT_SHALLOW ON
)
FetchContent_GetProperties(imgui)
if(NOT imgui_POPULATED)
  FetchContent_Populate(imgui)
endif()

set(IMGUI_DIR ${imgui_SOURCE_DIR} CACHE PATH "" FORCE)
set(IMGUI_SFML_FIND_SFML OFF CACHE BOOL "" FORCE) # SFML targets already exist from FetchContent above
set(IMGUI_SFML_IMGUI_DEMO OFF CACHE BOOL "" FORCE)

FetchContent_Declare(
  imgui_sfml
  GIT_REPOSITORY https://github.com/SFML/imgui-sfml.git
  GIT_TAG v2.6.1
  GIT_SHALLOW ON
)
FetchContent_MakeAvailable(imgui_sfml)

# ImPlot also has no CMakeLists.txt of its own -- populate-only, like ImGui above.
FetchContent_Declare(
  implot
  GIT_REPOSITORY https://github.com/epezent/implot.git
  GIT_TAG v0.17
  GIT_SHALLOW ON
)
FetchContent_GetProperties(implot)
if(NOT implot_POPULATED)
  FetchContent_Populate(implot)
endif()

find_package(Eigen3 3.4 REQUIRED)
find_package(yaml-cpp REQUIRED)

file(GLOB_RECURSE SIM_SOURCES CONFIGURE_DEPENDS
  ${CMAKE_SOURCE_DIR}/src/*.cpp
)
list(APPEND SIM_SOURCES
  ${implot_SOURCE_DIR}/implot.cpp
  ${implot_SOURCE_DIR}/implot_items.cpp
)

add_executable(car_sim ${SIM_SOURCES})

target_include_directories(car_sim PRIVATE
  ${CMAKE_SOURCE_DIR}/include
  ${implot_SOURCE_DIR}
)

target_link_libraries(car_sim PRIVATE
  sfml-graphics
  sfml-window
  sfml-system
  ImGui-SFML::ImGui-SFML
  Eigen3::Eigen
  yaml-cpp
)
