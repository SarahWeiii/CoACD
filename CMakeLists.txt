cmake_minimum_required(VERSION 3.1)
project(fast_clip)

list(APPEND CMAKE_MODULE_PATH "${CMAKE_CURRENT_LIST_DIR}/cmake")

set(CMAKE_INCLUDE_CURRENT_DIR ON)
set(CMAKE_CXX_STANDARD 11)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall -pthread -O3")
set(CMAKE_CXX_FLAGS_DEBUG "${CMAKE_CXX_FLAGS_DEBUG}")
set(CMAKE_CXX_FLAGS_RELEASE "-O3")  # enable assert
set(CMAKE_LINKER_FLAGS "${CMAKE_LINKER_FLAGS}")
set(CMAKE_LINKER_FLAGS_DEBUG "${CMAKE_LINKER_FLAGS_DEBUG}")

if ("${CMAKE_CXX_COMPILER_ID}" STREQUAL "GNU")
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wno-int-in-bool-context -Wsign-compare")
    set(CMAKE_CXX_FLAGS_DEBUG "${CMAKE_CXX_FLAGS_DEBUG} -fsanitize=address")
    set(CMAKE_LINKER_FLAGS "${CMAKE_LINKER_FLAGS}")
    set(CMAKE_LINKER_FLAGS_DEBUG "${CMAKE_LINKER_FLAGS_DEBUG} -fsanitize=address")
endif()


include_directories(src)
include_directories(3rd/glm)
include_directories(3rd/eigen/)
include_directories(3rd/libigl/include)

set(
    main_SRC
    src/hausdorff.h
    src/shape.cpp
    src/shape.h
    src/sobol.cpp
    src/sobol.h
    src/clip.cpp
    src/clip.h
    src/cost.cpp
    src/cost.h
    src/io.cpp
    src/io.h
    src/quickhull/QuickHull.cpp
    src/quickhull/QuickHull.hpp
    src/btConvexHull/btScalar.h
    src/btConvexHull/btVector3.h
    src/btConvexHull/btMinMax.h
    src/btConvexHull/btAlignedObjectArray.h
    src/btConvexHull/btAlignedAllocator.cpp
    src/btConvexHull/btAlignedAllocator.h
    src/btConvexHull/btConvexHullComputer.cpp
    src/btConvexHull/btConvexHullComputer.h
    src/manifold/BVH.cpp
    src/manifold/BVH.h
    src/manifold/Intersection.cpp
    src/manifold/Intersection.h
    src/manifold/Octree.h
    src/manifold/model.cpp
    src/manifold/model.h
    src/manifold/Manifold.h
    src/manifold/GridIndex_p.h
    src/manifold/IO_p.cc
    src/manifold/IO_p.h
    src/manifold/Manifold_p.cc
    src/manifold/Manifold_p.h
    src/manifold/Octree_p.cc
    src/manifold/Octree_p.h
    src/manifold/types_p.h
    src/manifold/ManifoldPlus.h
    src/model_obj.cpp 
    src/model_obj.h
    src/config.h
    src/mcts.h
    src/process.cpp
    src/process.h
    src/main.cpp 
)

add_executable(
    main
    ${main_SRC}
)