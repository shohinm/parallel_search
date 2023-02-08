cmake_minimum_required(VERSION 2.8)

project (epase)
add_compile_options(-std=c++17)

find_package(OpenCV REQUIRED)
find_package(SBPL REQUIRED)
find_package(Eigen3 REQUIRED)

if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release)
endif()

add_compile_options(-std=c++17)

set(CMAKE_CXX_FLAGS "-g")
SET(CMAKE_CXX_FLAGS_RELEASE "-O3")
SET(CMAKE_C_FLAGS_RELEASE "-O3")

include_directories(include
                    ${SBPL_INCLUDE_DIRS}
                    ${EIGEN3_INCLUDE_DIR})

link_directories(${SBPL_LIBRARY_DIRS})

set(SOURCES
    src/common/State.cpp 
    src/common/Edge.cpp
    src/planners/Planner.cpp
    src/planners/WastarPlanner.cpp
    src/planners/PwastarPlanner.cpp
    src/planners/PasePlanner.cpp
    src/planners/EpasePlanner.cpp
    src/planners/GepasePlanner.cpp
    src/planners/MplpPlanner.cpp
)

add_executable(run_robot_nav_2d
        examples/robot_nav_2d/run_robot_nav_2d.cpp
        examples/robot_nav_2d/RobotNav2dActions.cpp
        ${SOURCES})

target_link_libraries(run_robot_nav_2d
        ${OpenCV_LIBRARIES}
        ${SBPL_LIBRARIES}
        pthread)

add_executable(run_insat_nav_2d
        examples/robot_nav_2d/run_insat_nav_2d.cpp
        examples/robot_nav_2d/RobotNav2dActions.cpp
        examples/robot_nav_2d/InsatNav2dActions.cpp
        ${SOURCES})

target_link_libraries(run_insat_nav_2d
        ${OpenCV_LIBRARIES}
        ${SBPL_LIBRARIES}
        pthread)

add_executable(domain_template
        examples/domain_template/domain_template.cpp
        examples/domain_template/ActionsTemplate.cpp
        ${SOURCES})

target_link_libraries(domain_template
        pthread)