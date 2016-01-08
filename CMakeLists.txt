cmake_minimum_required(VERSION 2.8 FATAL_ERROR)

project(pclzf_simple_viewer)

find_package(PCL 1.7 REQUIRED)


include(CheckCXXCompilerFlag)
CHECK_CXX_COMPILER_FLAG("-std=c++11" COMPILER_SUPPORTS_CXX11)
CHECK_CXX_COMPILER_FLAG("-std=c++0x" COMPILER_SUPPORTS_CXX0X)
if(COMPILER_SUPPORTS_CXX11)
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")
elseif(COMPILER_SUPPORTS_CXX0X)
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++0x")
else()
        message(STATUS "The compiler ${CMAKE_CXX_COMPILER}
            has no C++11 support. Please use a different C++ compiler.")
endif()


set(CMAKE_BUILD_TYPE RelWithDebInfo)

include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

add_executable (pclzf_simple_viewer src/pclzf_simple_viewer.cpp)
target_link_libraries (pclzf_simple_viewer ${PCL_LIBRARIES})
