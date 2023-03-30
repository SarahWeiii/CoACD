cmake_minimum_required(VERSION 3.1)
project(CoACD)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

list(APPEND CMAKE_MODULE_PATH "${CMAKE_CURRENT_LIST_DIR}/cmake")

set(CMAKE_CXX_STANDARD 17)


find_package(OpenVDB REQUIRED)
find_package(OpenMP)

file(GLOB_RECURSE MAIN_SRC "src/*.h" "src/*.cc" "src/*.hpp" "src/*.cpp")
add_library(coacd STATIC
    ${MAIN_SRC}
)

target_include_directories(coacd PUBLIC public)
target_include_directories(coacd PRIVATE src)
target_include_directories(coacd PRIVATE 3rd/eigen)
target_include_directories(coacd PRIVATE 3rd/glm)
target_include_directories(coacd PRIVATE 3rd/libigl/include)
target_include_directories(coacd PRIVATE 3rd/cdt/CDT/include)

target_link_libraries(coacd OpenVDB::openvdb)
set_property(TARGET coacd PROPERTY COMPILE_WARNING_AS_ERROR OFF)

if (MSVC)
	target_compile_options(coacd PRIVATE /openmp)
endif()


add_executable(coacdapp main.cpp)
target_include_directories(coacdapp PUBLIC public)
target_include_directories(coacdapp PRIVATE src)
target_include_directories(coacdapp PRIVATE 3rd/eigen)
target_include_directories(coacdapp PRIVATE 3rd/glm)
target_include_directories(coacdapp PRIVATE 3rd/libigl/include)
target_include_directories(coacdapp PRIVATE 3rd/cdt/CDT/include)

target_link_libraries(coacdapp OpenVDB::openvdb)
set_property(TARGET coacdapp PROPERTY COMPILE_WARNING_AS_ERROR OFF)

if (MSVC)
	target_compile_options(coacdapp PRIVATE /openmp)
endif()


target_link_libraries(coacdapp coacd)
