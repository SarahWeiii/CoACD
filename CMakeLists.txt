cmake_minimum_required(VERSION 3.1)
project(CoACD)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

list(APPEND CMAKE_MODULE_PATH "${CMAKE_CURRENT_LIST_DIR}/cmake")

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_INCLUDE_CURRENT_DIR ON)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall -pthread")
set(CMAKE_CXX_FLAGS_DEBUG "${CMAKE_CXX_FLAGS_DEBUG}")
set(CMAKE_CXX_FLAGS_RELEASE "-O3")
set(CMAKE_LINKER_FLAGS "${CMAKE_LINKER_FLAGS}")
set(CMAKE_LINKER_FLAGS_DEBUG "${CMAKE_LINKER_FLAGS_DEBUG}")

if ("${CMAKE_CXX_COMPILER_ID}" STREQUAL "GNU")
    set(CMAKE_CXX_FLAGS_DEBUG "${CMAKE_CXX_FLAGS_DEBUG} -fsanitize=address")
    set(CMAKE_CXX_FLAGS_RELEASE "${CMAKE_CXX_FLAGS_RELEASE} -Wno-int-in-bool-context -Wsign-compare")
    set(CMAKE_LINKER_FLAGS_DEBUG "${CMAKE_LINKER_FLAGS_DEBUG} -fsanitize=address")
endif()

find_package(OpenMP)
if (OPENMP_FOUND)
    set (CMAKE_C_FLAGS "${CMAKE_C_FLAGS} ${OpenMP_C_FLAGS}")
    set (CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${OpenMP_CXX_FLAGS}")
    set (CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} ${OpenMP_EXE_LINKER_FLAGS}")
endif()

file(GLOB_RECURSE MAIN_SRC "src/*.h" "src/*.cc" "src/*.hpp" "src/*.cpp")
add_library(_coacd STATIC
    ${MAIN_SRC}
)

if (WIN32)
    option(DISABLE_SPDLOG "Disable spdlog" ON)
else()
    option(DISABLE_SPDLOG "Disable spdlog" OFF)
endif()

if (DISABLE_SPDLOG)
    add_definitions(-DDISABLE_SPDLOG)
else()
    find_package(spdlog REQUIRED)
endif()


# use static if possible
find_library(OPENVDB_STATIC libopenvdb.a)
# find_library(Z_STATIC libz.a)
find_library(TBB_STATIC libtbb.a)
find_library(BLOSC_STATIC libblosc.a)
if (OPENVDB_STATIC AND TBB_STATIC AND BLOSC_STATIC)
    message("CoACD: using static libraries")
    set(OPENVDB_LIBRARIES openvdb.a blosc.a tbb.a z)
else()
    message("CoACD: using dynamic libraries")
    set(OPENVDB_LIBRARIES openvdb tbb)
endif()

include_directories("/usr/include/eigen3" "/usr/local/include/eigen3")

target_include_directories(_coacd PUBLIC public)
target_include_directories(_coacd PRIVATE src)
target_include_directories(_coacd PRIVATE 3rd/cdt/CDT)
target_link_libraries(_coacd ${OPENVDB_LIBRARIES} ${SPDLOG_LIBRARIES})
set_target_properties(_coacd PROPERTIES POSITION_INDEPENDENT_CODE TRUE)

find_library(HALF_LIBRARY libHalf.so)
if (HALF_LIBRARY)
    target_link_libraries(_coacd ${HALF_LIBRARY})
endif()

add_executable(main main.cpp)
target_link_libraries(main _coacd)
target_include_directories(main PRIVATE src)

add_subdirectory("3rd/pybind11")
include_directories("python")
pybind11_add_module(coacd python/coacd/coacd.cpp public/coacd.cpp)
target_link_libraries(coacd PRIVATE _coacd)
