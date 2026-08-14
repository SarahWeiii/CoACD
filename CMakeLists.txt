cmake_minimum_required(VERSION 3.24)
project(CoACD LANGUAGES C CXX)

# ##############################################################################
# General Settings
# ##############################################################################

set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

list(APPEND CMAKE_MODULE_PATH "${CMAKE_CURRENT_LIST_DIR}/cmake")

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
# Global so FetchContent third-party static libs also build with PIC,
# as required when linking them into the shared _coacd library.
set(CMAKE_POSITION_INDEPENDENT_CODE ON)

option(WITH_3RD_PARTY_LIBS "Include 3rd party libraries" ON)
if(APPLE)
    set(WITH_OPENMP_DEFAULT OFF)
else()
    set(WITH_OPENMP_DEFAULT ON)
endif()
option(WITH_OPENMP "Enable OpenMP support" ${WITH_OPENMP_DEFAULT})
option(WITH_STD_THREADS "Enable standard threads parallel fallback" ON)

# --- Compiler-specific flags
if(MSVC)
    # For Microsoft Visual Studio (Windows)
    add_compile_options("/bigobj")
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} /W0 /D_USE_MATH_DEFINES")
    set(CMAKE_SHARED_LIBRARY_PREFIX "lib")
else()
    # For non-MSVC (Linux, macOS, etc.)
    set(CMAKE_CXX_FLAGS
      "${CMAKE_CXX_FLAGS} -Wall")
    if(CMAKE_CXX_COMPILER_ID MATCHES "GNU")
        # Linux/Unix: disable -Werror=stringop-overflow
        set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wno-error=stringop-overflow")
    endif()
endif()

set(CMAKE_CXX_FLAGS_DEBUG "${CMAKE_CXX_FLAGS}")
set(CMAKE_CXX_FLAGS_RELEASE "${CMAKE_CXX_FLAGS} -O3")

if("${CMAKE_CXX_COMPILER_ID}" STREQUAL "GNU")
    set(CMAKE_CXX_FLAGS_DEBUG "${CMAKE_CXX_FLAGS_DEBUG} -fsanitize=address")
    set(CMAKE_CXX_FLAGS_RELEASE
        "${CMAKE_CXX_FLAGS_RELEASE} -Wno-int-in-bool-context -Wsign-compare")
    set(CMAKE_LINKER_FLAGS_DEBUG "${CMAKE_LINKER_FLAGS_DEBUG} -fsanitize=address")
endif()

if(WITH_3RD_PARTY_LIBS)
    include(zlib)
    include(boost)
    include(openvdb)
    include(spdlog)
endif()

# Set default install postfix
if(UNIX)
    include(GNUInstallDirs)
else()
    set(CMAKE_INSTALL_LIBDIR lib)
    set(CMAKE_INSTALL_INCLUDEDIR include)
    set(CMAKE_INSTALL_BINDIR bin)
endif()

# ##############################################################################
# BUILD Program
# ##############################################################################

file(GLOB_RECURSE COACD_SRC "src/*.cc" "src/*.cpp")
if(NOT WITH_3RD_PARTY_LIBS)
    # Exclude files that match the pattern "preprocess*"
    list(FILTER COACD_SRC EXCLUDE REGEX ".*preprocess.*")
endif()

add_library(coacd_interface INTERFACE)
if(WITH_3RD_PARTY_LIBS)
    target_compile_definitions(coacd_interface INTERFACE WITH_3RD_PARTY_LIBS=1)
else()
    target_compile_definitions(coacd_interface INTERFACE WITH_3RD_PARTY_LIBS=0)
    target_compile_definitions(coacd_interface INTERFACE DISABLE_SPDLOG)
endif()

add_library(coacd_objects OBJECT ${COACD_SRC})
target_link_libraries(coacd_objects PRIVATE coacd_interface)
target_include_directories(coacd_objects
    PUBLIC $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/public>
           $<INSTALL_INTERFACE:${CMAKE_INSTALL_INCLUDEDIR}>
    PRIVATE 3rd/cdt/CDT/include)
set_target_properties(coacd_objects PROPERTIES POSITION_INDEPENDENT_CODE TRUE)
if(WITH_3RD_PARTY_LIBS)
    target_link_libraries(coacd_objects PRIVATE openvdb_static spdlog::spdlog)
endif()

set(USE_STD_THREADS OFF)
if(WITH_OPENMP)
    find_package(OpenMP)
    if(OpenMP_CXX_FOUND)
        target_link_libraries(coacd_objects PUBLIC OpenMP::OpenMP_CXX)
    elseif(WITH_STD_THREADS)
        # Fallback to standard library threads if OpenMP is missing
        message(STATUS "OpenMP requested but not found; falling back to C++17 std::thread")
        set(USE_STD_THREADS ON)
    else()
        message(STATUS "OpenMP requested but not found; building sequential (WITH_STD_THREADS=OFF)")
    endif()
else()
    if(WITH_STD_THREADS)
        set(USE_STD_THREADS ON)
    endif()
endif ()

if(USE_STD_THREADS)
    find_package(Threads)
    target_compile_definitions(coacd_objects PUBLIC WITH_STD_THREADS=1)
    target_link_libraries(coacd_objects PRIVATE Threads::Threads)
endif()

add_library(coacd STATIC "public/coacd.cpp")
target_link_libraries(coacd PRIVATE coacd_objects)
target_compile_definitions(coacd PUBLIC COACD_STATIC)
target_link_libraries(coacd PUBLIC coacd_interface)

add_library(_coacd SHARED "public/coacd.cpp")
target_link_libraries(_coacd PRIVATE coacd_objects)
target_compile_definitions(_coacd PRIVATE _coacd_EXPORTS)
target_link_libraries(_coacd PUBLIC coacd_interface)

if (WITH_3RD_PARTY_LIBS)
    target_link_libraries(coacd PRIVATE openvdb_static spdlog::spdlog)
    target_link_libraries(_coacd PRIVATE openvdb_static spdlog::spdlog)
endif()

add_executable(main main.cpp)
target_link_libraries(main PRIVATE coacd)
if (WITH_3RD_PARTY_LIBS)
    target_link_libraries(main PRIVATE openvdb_static spdlog::spdlog)
endif()

# ##############################################################################
# Install target
# ##############################################################################

install(
  TARGETS _coacd coacd_interface
  EXPORT CoACDTargets
  LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR}
  ARCHIVE DESTINATION ${CMAKE_INSTALL_LIBDIR}
  RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR}
  INCLUDES
  DESTINATION ${CMAKE_INSTALL_INCLUDEDIR})

install(
  EXPORT CoACDTargets
  FILE CoACDTargets.cmake
  NAMESPACE CoACD::
  DESTINATION ${CMAKE_INSTALL_LIBDIR}/cmake/CoACD)

install(
  FILES public/coacd.h
  DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}/CoACD)

# ##############################################################################
# Install Cmake Configuration
# ##############################################################################

include(CMakePackageConfigHelpers)

configure_package_config_file(
  ${CMAKE_SOURCE_DIR}/cmake/Config.cmake.in
  "${CMAKE_BINARY_DIR}/CoACDConfig.cmake"
  INSTALL_DESTINATION ${CMAKE_INSTALL_LIBDIR}/cmake/CoACD)

install(FILES "${CMAKE_BINARY_DIR}/CoACDConfig.cmake"
        DESTINATION ${CMAKE_INSTALL_LIBDIR}/cmake/CoACD)
