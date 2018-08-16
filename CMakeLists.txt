cmake_minimum_required (VERSION 3.10.2)
project (Cadence)
add_executable(cadence main.cpp)

list(APPEND CMAKE_MODULE_PATH "${CMAKE_CURRENT_LIST_DIR}/cmake")

find_package(assimp REQUIRED)
find_package(OpenMesh REQUIRED)

target_link_libraries(cadence assimp OpenMeshCore)