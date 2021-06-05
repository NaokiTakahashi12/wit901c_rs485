
cmake_minimum_required(VERSION 3.10)

if(${CMAKE_CURRENT_SOURCE_DIR} STREQUAL ${CMAKE_BINARY_DIR})
    message(FATAL_ERROR "Failed in source build\nPlease\n```$ mkdir build```")
endif()

project(wit901c_rs485 LANGUAGES CXX VERSION 0.1.0)

set(CMAKE_BUILD_TYPE Release
    CACHE STRING "Choose the type of build" FORCE
)
set_property(CACHE CMAKE_BUILD_TYPE
    PROPERTY
        STRINGS
            Debug
            Release
            MinSizeRel
            RelWithDebInfo
)
set(CXX_COMPILE_FEATURE_TYPE cxx_std_14
    CACHE STRING "Choose the type of c++ compile feature" FORCE
)
set_property(CACHE CXX_COMPILE_FEATURE_TYPE
    PROPERTY
        STRINGS
            cxx_std_23
            cxx_std_20
            cxx_std_17
            cxx_std_14
            cxx_std_11
            cxx_std_98
)

add_subdirectory(src)

