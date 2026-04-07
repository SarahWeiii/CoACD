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
        set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wno-int-in-bool-context -Wsign-compare")
    endif()
endif()

option(WITH_ASAN "Use address sanitizer" ON)
if(WITH_ASAN AND UNIX AND CMAKE_CXX_COMPILER_ID MATCHES "GNU")
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -fsanitize=address")
    set(CMAKE_LINKER_FLAGS "${CMAKE_LINKER_FLAGS} -fsanitize=address")
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

file(GLOB_RECURSE COACD_SRC "src/*.cc" "src/*.cpp" "public/coacd.cpp")
if(NOT WITH_3RD_PARTY_LIBS)
    # Exclude files that match the pattern "preprocess*"
    list(FILTER COACD_SRC EXCLUDE REGEX ".*preprocess.*")
endif()

add_library(coacd STATIC ${COACD_SRC})
target_include_directories(
  coacd PUBLIC $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/public>
               $<INSTALL_INTERFACE:${CMAKE_INSTALL_INCLUDEDIR}>)
target_include_directories(coacd PRIVATE 3rd/cdt/CDT/include)
set_target_properties(coacd PROPERTIES POSITION_INDEPENDENT_CODE TRUE)

if(WITH_3RD_PARTY_LIBS)
    target_link_libraries(coacd PRIVATE openvdb_static spdlog::spdlog)
    target_compile_definitions(coacd PUBLIC WITH_3RD_PARTY_LIBS=1)
else()
    target_compile_definitions(coacd PUBLIC WITH_3RD_PARTY_LIBS=0)
    target_compile_definitions(coacd PUBLIC DISABLE_SPDLOG)
endif()

find_package(OpenMP)
if(OpenMP_CXX_FOUND)
    target_link_libraries(coacd PUBLIC OpenMP::OpenMP_CXX)
endif()

find_package(Threads)
target_link_libraries(coacd PRIVATE Threads::Threads)

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

# ##############################################################################
# Testing
# ##############################################################################
enable_testing()
add_test (NAME coacd_SnowFlake COMMAND main -i examples/SnowFlake.obj -o ${CMAKE_CURRENT_BINARY_DIR}/SnowFlake_cvx.wrl -np -t 0.02 WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR})
add_test (NAME coacd_Octocat COMMAND main -i examples/Octocat-v2.obj -o ${CMAKE_CURRENT_BINARY_DIR}/Octocat-v2_cvx.wrl -np -t 0.05 WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR})
add_test (NAME coacd_KitchenPot COMMAND main -i examples/KitchenPot.obj -o ${CMAKE_CURRENT_BINARY_DIR}/KitchenPot_cvx.wrl -t 0.05 WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR})
add_test (NAME coacd_Kettle COMMAND main -i examples/Kettle.obj -o ${CMAKE_CURRENT_BINARY_DIR}/Kettle_cvx.wrl -t 0.05 WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR})
add_test (NAME coacd_Bottle COMMAND main -i examples/Bottle.obj -o ${CMAKE_CURRENT_BINARY_DIR}/Bottle_cvx.wrl -t 0.05 WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR})
