if(TARGET openvdb_static)
    return()
endif()

if(CMAKE_CXX_COMPILER_ID STREQUAL "GNU")
    if(CMAKE_CXX_COMPILER_VERSION VERSION_GREATER "12")
        message(WARNING "GCC VERSION < 12 is supported. Current version: ${CMAKE_CXX_COMPILER_VERSION}")
    endif()
endif()

set(BUILD_SHARED_LIBS OFF CACHE STRING "" FORCE)
set(USE_STATIC_DEPENDENCIES ON CACHE BOOL "" FORCE)
set(OPENVDB_ENABLE_UNINSTALL OFF CACHE BOOL "" FORCE)
set(OPENVDB_FUTURE_DEPRECATION OFF CACHE BOOL "" FORCE)
set(USE_BLOSC OFF CACHE STRING "" FORCE)
set(USE_ZLIB OFF CACHE STRING "" FORCE)

include(FetchContent)
FetchContent_Declare(
    openvdb
    GIT_REPOSITORY https://github.com/AcademySoftwareFoundation/openvdb.git
    GIT_TAG        v8.2.0
    PATCH_COMMAND cmake -E copy_if_different ${CMAKE_CURRENT_SOURCE_DIR}/cmake/openvdb_CMakeLists.txt <SOURCE_DIR>/openvdb/openvdb/CMakeLists.txt && cmake -E copy_if_different ${CMAKE_CURRENT_SOURCE_DIR}/cmake/openvdb_cmd_CMakeLists.txt <SOURCE_DIR>/openvdb/openvdb/cmd/CMakeLists.txt
)

FetchContent_Declare(
    tbb
    GIT_REPOSITORY https://github.com/oneapi-src/oneTBB.git
    GIT_TAG        v2021.8.0
)

set(CMAKE_CXX_FLAGS_OLD ${CMAKE_CXX_FLAGS})
set(CMAKE_CXX_FLAGS -DTBB_ALLOCATOR_TRAITS_BROKEN)
FetchContent_MakeAvailable(tbb)
set(CMAKE_CXX_FLAGS ${CMAKE_CXX_FLAGS_OLD})

if (WIN32)
    set(OPENVDB_DISABLE_BOOST_IMPLICIT_LINKING OFF CACHE BOOL "" FORCE)
endif()

set(OPENVDB_ENABLE_UNINSTALL OFF CACHE BOOL "" FORCE)
FetchContent_MakeAvailable(openvdb)
set_target_properties(openvdb_static PROPERTIES POSITION_INDEPENDENT_CODE ON)

target_include_directories(openvdb_static PUBLIC
    ${boost_SOURCE_DIR}/libs/numeric/conversion/include
    ${boost_SOURCE_DIR}/libs/any/include
    ${boost_SOURCE_DIR}/libs/algorithm/include
    ${boost_SOURCE_DIR}/libs/uuid/include
    ${boost_SOURCE_DIR}/libs/foreach/include
    ${boost_SOURCE_DIR}/libs/interprocess/include
    ${boost_SOURCE_DIR}/libs/intrusive/include
    ${boost_SOURCE_DIR}/libs/tti/include
)

add_library(Boost::disable_autolinking INTERFACE IMPORTED)

