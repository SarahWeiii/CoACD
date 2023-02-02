cmake_minimum_required(VERSION 3.1)
project(CoACD)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

list(APPEND CMAKE_MODULE_PATH "${CMAKE_CURRENT_LIST_DIR}/cmake")

set(CMAKE_CXX_STANDARD 20)
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

file(GLOB_RECURSE MAIN_SRC "src/*.h" "src/*.cc" "src/*.hpp" "src/*.cpp" "public/coacd.cpp")
add_library(coacd STATIC
    ${MAIN_SRC}
    src/manifold/ManifoldPlus.h
)

target_include_directories(coacd PUBLIC public)
target_include_directories(coacd PRIVATE src)
target_include_directories(coacd PRIVATE 3rd/eigen)
target_include_directories(coacd PRIVATE 3rd/glm)
target_include_directories(coacd PRIVATE 3rd/libigl/include)
target_include_directories(coacd PRIVATE 3rd/cdt/CDT)

find_library(HALF_LIBRARY libHalf.so)
if (HALF_LIBRARY)
    target_link_libraries(coacd ${HALF_LIBRARY})
endif()

target_link_libraries(coacd -lopenvdb -lboost_iostreams -ltbb)

add_executable(main main.cpp)
target_link_libraries(main coacd)
target_include_directories(main PRIVATE src)
target_include_directories(main PRIVATE 3rd/eigen)
target_include_directories(main PRIVATE 3rd/glm)
