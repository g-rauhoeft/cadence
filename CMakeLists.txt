cmake_minimum_required (VERSION 3.5.1)
project (Cadence)
add_executable(cadence main.cpp)

list(APPEND CMAKE_MODULE_PATH "${CMAKE_CURRENT_LIST_DIR}/cmake")

find_package(assimp REQUIRED)
find_package(OpenMesh REQUIRED)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")
target_link_libraries(cadence assimp OpenMeshCore OpenMeshTools)