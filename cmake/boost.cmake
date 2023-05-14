set(BOOST_IOSTREAMS_ENABLE_ZSTD OFF CACHE BOOL "" FORCE)

set(BOOST_IOSTREAMS_ENABLE_LZMA OFF)
set(BOOST_IOSTREAMS_ENABLE_BZIP2 OFF)

include(FetchContent)
FetchContent_Declare(
    boost
    URL      https://github.com/boostorg/boost/releases/download/boost-1.81.0/boost-1.81.0.tar.gz
    URL_HASH MD5=ffac94fbdd92d6bc70a897052022eeba
    OVERRIDE_FIND_PACKAGE
)

FetchContent_MakeAvailable(boost)

if (zlib_SOURCE_DIR)
    target_include_directories(boost_iostreams PRIVATE ${zlib_SOURCE_DIR} ${zlib_BINARY_DIR})
endif()
