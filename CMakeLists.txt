cmake_minimum_required(VERSION 3.24)
project(CoACD LANGUAGES C CXX)

# ##############################################################################
# General Settings
# ##############################################################################

set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

list(APPEND CMAKE_MODULE_PATH "${CMAKE_CURRENT_LIST_DIR}/cmake")

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

option(WITH_3RD_PARTY_LIBS "Include 3rd party libraries" ON)

if(WITH_3RD_PARTY_LIBS)
    add_compile_definitions(WITH_3RD_PARTY_LIBS=1)
else()
    add_definitions(-DDISABLE_SPDLOG)
    add_compile_definitions(WITH_3RD_PARTY_LIBS=0)
endif()

# --- Compiler-specific flags
if(MSVC)
  # For Microsoft Visual Studio (Windows)
    add_compile_options("/bigobj")
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} /W0 /D_USE_MATH_DEFINES")
    set(CMAKE_SHARED_LIBRARY_PREFIX "lib")
else()
  # For non-MSVC (Linux, macOS, etc.)
    set(CMAKE_CXX_FLAGS
      "${CMAKE_CXX_FLAGS} -Wall -fpermissive -D_USE_MATH_DEFINES")
    if(APPLE)
        # macOS: add -fPIC
        set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -fPIC")
    elseif(UNIX)
        # Linux/Unix: add -fPIC and disable -Werror=stringop-overflow
        set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -fPIC -Wno-error=stringop-overflow")
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

add_library(coacd STATIC ${COACD_SRC})
target_include_directories(
  coacd PUBLIC $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/public>
               $<INSTALL_INTERFACE:${CMAKE_INSTALL_INCLUDEDIR}>)
target_include_directories(coacd PRIVATE 3rd/cdt/CDT)

if(WITH_3RD_PARTY_LIBS)
    target_link_libraries(coacd PRIVATE openvdb_static spdlog::spdlog)
endif()

find_package(OpenMP)
if(OpenMP_CXX_FOUND)
    target_link_libraries(coacd PUBLIC OpenMP::OpenMP_CXX)
endif()

find_package(Threads)
target_link_libraries(coacd PRIVATE Threads::Threads)
set_target_properties(coacd PROPERTIES POSITION_INDEPENDENT_CODE TRUE)

add_library(_coacd SHARED "public/coacd.cpp")

if(WITH_3RD_PARTY_LIBS)
    target_link_libraries(_coacd PRIVATE coacd spdlog::spdlog openvdb_static)
else()
    target_link_libraries(_coacd PRIVATE coacd)
endif()

add_executable(main main.cpp)

if(WITH_3RD_PARTY_LIBS)
    target_link_libraries(main coacd spdlog::spdlog openvdb_static)
else()
    target_link_libraries(main coacd)
endif()

# ##############################################################################
# Install target
# ##############################################################################

install(
  TARGETS _coacd
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
