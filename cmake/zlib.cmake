
include(FetchContent)
FetchContent_Declare(
    zlib
    GIT_REPOSITORY https://github.com/madler/zlib.git
    GIT_TAG        v1.2.11
    OVERRIDE_FIND_PACKAGE
)

FetchContent_MakeAvailable(zlib)

set_target_properties(zlibstatic PROPERTIES POSITION_INDEPENDENT_CODE TRUE)
add_library(ZLIB::ZLIB ALIAS zlibstatic)
set(ZLIB_LIBRARY ZLIB::ZLIB)
set(ZLIB_INCLUDE_DIR ${zlib_SOURCE_DIR})
set(ZLIB_FOUND TRUE)
