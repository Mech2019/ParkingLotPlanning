cmake_minimum_required(VERSION 3.5)
set(CMAKE_CXX_STANDARD 11)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

add_executable( test MACOSX_BUNDLE
                test.cpp
                collision_check.cpp
                map.cpp
                primitive.cpp
                state.cpp
                util.h)