set(BOOST_IOSTREAMS_ENABLE_ZSTD OFF CACHE BOOL "" FORCE)

set(BOOST_IOSTREAMS_ENABLE_LZMA OFF)
set(BOOST_IOSTREAMS_ENABLE_BZIP2 OFF)

include(FetchContent)

if(WIN32)
    FetchContent_Declare(
        boost
        URL https://github.com/boostorg/boost/releases/download/boost-1.81.0/boost-1.81.0.zip
        URL_HASH MD5=375693214b89309d2003f5296422c0a8
        OVERRIDE_FIND_PACKAGE
    )
else()
    FetchContent_Declare(
        boost
        URL https://github.com/boostorg/boost/releases/download/boost-1.81.0/boost-1.81.0.tar.gz
        URL_HASH MD5=ffac94fbdd92d6bc70a897052022eeba
        OVERRIDE_FIND_PACKAGE
    )
endif()

FetchContent_MakeAvailable(boost)

if(zlib_SOURCE_DIR)
    target_include_directories(boost_iostreams PRIVATE ${zlib_SOURCE_DIR} ${zlib_BINARY_DIR})
endif()
