
cmake_minimum_required(VERSION 3.8)

project(hMesh LANGUAGES CXX)

add_compile_definitions(ROOT_PATH="${PROJECT_SOURCE_DIR}")

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

file(GLOB SOURCES test/*cpp)

add_executable(hMesh ${SOURCES})