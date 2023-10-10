cmake_minimum_required(VERSION 3.24)
project(CoACD LANGUAGES C CXX)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

list(APPEND CMAKE_MODULE_PATH "${CMAKE_CURRENT_LIST_DIR}/cmake")

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall -fpermissive")
set(CMAKE_CXX_FLAGS_DEBUG "${CMAKE_CXX_FLAGS}")
set(CMAKE_CXX_FLAGS_RELEASE "${CMAKE_CXX_FLAGS} -O3")

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -static-libstdc++")

if ("${CMAKE_CXX_COMPILER_ID}" STREQUAL "GNU")
    set(CMAKE_CXX_FLAGS_DEBUG "${CMAKE_CXX_FLAGS_DEBUG} -fsanitize=address")
    set(CMAKE_CXX_FLAGS_RELEASE "${CMAKE_CXX_FLAGS_RELEASE} -Wno-int-in-bool-context -Wsign-compare")
    set(CMAKE_LINKER_FLAGS_DEBUG "${CMAKE_LINKER_FLAGS_DEBUG} -fsanitize=address")
endif()

include(zlib)
include(boost)
include(openvdb)
include(eigen)
include(spdlog)

file(GLOB_RECURSE COACD_SRC "src/*.cc" "src/*.cpp")
add_library(coacd STATIC ${COACD_SRC})
target_include_directories(coacd PUBLIC public)
target_include_directories(coacd PRIVATE 3rd/cdt/CDT)
target_link_libraries(coacd PRIVATE openvdb_static spdlog::spdlog)

find_package(OpenMP)
if (OpenMP_CXX_FOUND)
    target_link_libraries(coacd PUBLIC OpenMP::OpenMP_CXX)
endif()

find_package(Threads)
target_link_libraries(coacd PRIVATE Threads::Threads)
set_target_properties(coacd PROPERTIES POSITION_INDEPENDENT_CODE TRUE)

add_library(_coacd SHARED "public/coacd.cpp")
target_link_libraries(_coacd PRIVATE coacd spdlog::spdlog openvdb_static)

add_executable(main main.cpp)
target_link_libraries(main coacd spdlog::spdlog openvdb_static)
