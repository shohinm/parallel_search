cmake_minimum_required(VERSION 2.8)

project (epase)

find_package(OpenCV REQUIRED)
find_package(SBPL REQUIRED)

if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release)
endif()

add_compile_options(-std=c++17)

set(CMAKE_CXX_FLAGS "-g")
SET(CMAKE_CXX_FLAGS_RELEASE "-O3")
SET(CMAKE_C_FLAGS_RELEASE "-O3")

include_directories(include
                    ${SBPL_INCLUDE_DIRS})

link_directories(${SBPL_LIBRARY_DIRS})

set(SOURCES
    src/State.cpp 
    src/Edge.cpp
    src/Planner.cpp
    src/EpasePlanner.cpp
)

add_executable(run_point_robot_nav
        examples/point_robot_nav/run_point_robot_nav.cpp
        examples/point_robot_nav/PointRobotActions.cpp
        ${SOURCES})

target_link_libraries(run_point_robot_nav
        ${OpenCV_LIBRARIES}
        ${SBPL_LIBRARIES}
        pthread)