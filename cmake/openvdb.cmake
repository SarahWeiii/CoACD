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
set(TBB_TEST OFF CACHE BOOL "" FORCE)

include(FetchContent)
if(CMAKE_VERSION VERSION_LESS 3.28)
  FetchContent_Declare(
    openvdb
    GIT_REPOSITORY https://github.com/AcademySoftwareFoundation/openvdb.git
    GIT_TAG v8.2.0
    PATCH_COMMAND
      cmake -E copy_if_different
      ${CMAKE_CURRENT_SOURCE_DIR}/cmake/openvdb_CMakeLists.txt
      <SOURCE_DIR>/openvdb/openvdb/CMakeLists.txt && cmake -E copy_if_different
      ${CMAKE_CURRENT_SOURCE_DIR}/cmake/openvdb_cmd_CMakeLists.txt
      <SOURCE_DIR>/openvdb/openvdb/cmd/CMakeLists.txt)
  FetchContent_Declare(
    tbb
    GIT_REPOSITORY https://github.com/oneapi-src/oneTBB.git
    GIT_TAG v2022.0.0)

else()

  FetchContent_Declare(
    openvdb
    GIT_REPOSITORY https://github.com/AcademySoftwareFoundation/openvdb.git
    GIT_TAG v8.2.0
    PATCH_COMMAND
      cmake -E copy_if_different
      ${CMAKE_CURRENT_SOURCE_DIR}/cmake/openvdb_CMakeLists.txt
      <SOURCE_DIR>/openvdb/openvdb/CMakeLists.txt && cmake -E copy_if_different
      ${CMAKE_CURRENT_SOURCE_DIR}/cmake/openvdb_cmd_CMakeLists.txt
      <SOURCE_DIR>/openvdb/openvdb/cmd/CMakeLists.txt EXCLUDE_FROM_ALL)
  FetchContent_Declare(
    tbb
    GIT_REPOSITORY https://github.com/oneapi-src/oneTBB.git
    GIT_TAG v2022.0.0
    EXCLUDE_FROM_ALL)

endif()

set(CMAKE_CXX_FLAGS_OLD ${CMAKE_CXX_FLAGS})
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -DTBB_ALLOCATOR_TRAITS_BROKEN")

# Make tbb available
if(CMAKE_VERSION VERSION_LESS 3.28)
  FetchContent_GetProperties(tbb)
  if(NOT tbb_POPULATED)
    FetchContent_Populate(tbb)
    add_subdirectory(${tbb_SOURCE_DIR} ${tbb_BINARY_DIR} EXCLUDE_FROM_ALL)
  endif()
else()
  FetchContent_MakeAvailable(tbb)
endif()

set(CMAKE_CXX_FLAGS ${CMAKE_CXX_FLAGS_OLD})

if(WIN32)
    set(OPENVDB_DISABLE_BOOST_IMPLICIT_LINKING OFF CACHE BOOL "" FORCE)
endif()

set(CMAKE_CXX_STANDARD 17)
set(OPENVDB_ENABLE_UNINSTALL OFF CACHE BOOL "" FORCE)

# Make openvdb available
if(CMAKE_VERSION VERSION_LESS 3.28)
  FetchContent_GetProperties(openvdb)
  if(NOT openvdb_POPULATED)
    FetchContent_Populate(openvdb)
    add_subdirectory(${openvdb_SOURCE_DIR} ${openvdb_BINARY_DIR} EXCLUDE_FROM_ALL)
  endif()
else()
  FetchContent_MakeAvailable(openvdb)
endif()

if(APPLE)
    message(STATUS "Patching OpenVDB NodeManager.h for AppleClang template bug")

    # Locate the header file relative to the fetched source
    set(NODEMANAGER_H "${openvdb_SOURCE_DIR}/openvdb/openvdb/tree/NodeManager.h")

    if(EXISTS "${NODEMANAGER_H}")
        execute_process(
            COMMAND sed -i "" -E "s/OpT::template[[:space:]]+eval/OpT::eval/g" "${NODEMANAGER_H}"
            RESULT_VARIABLE PATCH_RESULT
        )
        if(PATCH_RESULT EQUAL 0)
            message(STATUS "✅ Successfully patched ${NODEMANAGER_H}")
        else()
            message(WARNING "⚠️  Failed to patch ${NODEMANAGER_H}")
        endif()
    else()
        message(WARNING "⚠️  OpenVDB NodeManager.h not found at ${NODEMANAGER_H}")
    endif()
endif()

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
set(CMAKE_CXX_STANDARD 20)
