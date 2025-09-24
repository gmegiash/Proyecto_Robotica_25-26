cmake_minimum_required(VERSION 4.0)
project(prueba1)

set(CMAKE_CXX_STANDARD 23)

add_definitions(-O3)

add_executable(prueba1 main.cpp
        main.cpp)
target_link_libraries(prueba1 PRIVATE tbb)
