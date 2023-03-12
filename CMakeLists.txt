cmake_minimum_required(VERSION 2.8)

project (epase)
add_compile_options(-std=c++17)

list(APPEND CMAKE_PREFIX_PATH /opt/drake)
list(APPEND CMAKE_PREFIX_PATH /opt/ros/noetic)
set(MUJOCO_INCLUDE_DIR "/home/gaussian/cmu_ri_phd/phd_research/parallel_search/third_party/mujoco-2.3.2/include")
set(MUJOCO_LIB_DIR "/home/gaussian/cmu_ri_phd/phd_research/parallel_search/third_party/mujoco-2.3.2/lib")

find_package(OpenCV REQUIRED)
find_package(SBPL REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(drake REQUIRED)

if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release)
endif()

add_compile_options(-std=c++17)

set(CMAKE_CXX_FLAGS "-g")
SET(CMAKE_CXX_FLAGS_RELEASE "-O3")
SET(CMAKE_C_FLAGS_RELEASE "-O3")

include_directories(include
                    ${SBPL_INCLUDE_DIRS}
                    ${EIGEN3_INCLUDE_DIR}
                    ${drake_INCLUDE_DIRS}
                    ${MUJOCO_INCLUDE_DIR})
include_directories(/opt/drake/include)

link_directories(${SBPL_LIBRARY_DIRS})
link_directories(${MUJOCO_LIB_DIR})
#link_directories(/opt/drake/lib)

set(SOURCES
    src/common/State.cpp 
    src/common/Edge.cpp
    src/common/insat/InsatEdge.cpp
    src/planners/Planner.cpp
    src/planners/BFSPlanner.cpp
    src/planners/WastarPlanner.cpp
    src/planners/PwastarPlanner.cpp
    src/planners/PasePlanner.cpp
    src/planners/EpasePlanner.cpp
    src/planners/GepasePlanner.cpp
    src/planners/MplpPlanner.cpp
    src/planners/RrtPlanner.cpp
    src/planners/RrtConnectPlanner.cpp
    src/planners/insat/PinsatPlanner.cpp
    src/planners/insat/InsatPlanner.cpp
    src/planners/insat/opt/DummyOpt.cpp
    src/planners/insat/opt/BSplineOpt.cpp
)

# EXECUTABLES
add_executable(run_robot_nav_2d
        examples/robot_nav_2d/run_robot_nav_2d.cpp
        examples/robot_nav_2d/RobotNav2dActions.cpp
        ${SOURCES})

target_link_libraries(run_robot_nav_2d
        ${OpenCV_LIBRARIES}
        ${SBPL_LIBRARIES}
        ${drake_LIBRARIES}
        pthread)

add_executable(run_insat_nav_2d
        examples/insat/robot_nav_2d/run_insat_nav_2d.cpp
        examples/insat/robot_nav_2d/InsatNav2dActions.cpp
        ${SOURCES})

target_link_libraries(run_insat_nav_2d
        ${OpenCV_LIBRARIES}
        ${SBPL_LIBRARIES}
        ${drake_LIBRARIES}
        pthread)

add_executable(run_manipulation
        examples/manipulation/run_manipulation.cpp
        examples/manipulation/ManipulationActions.cpp
        ${SOURCES})

target_link_libraries(run_manipulation
        mujoco
        ${drake_LIBRARIES}
        pthread)

add_executable(drake_test
        examples/insat/opt/DrakeTest.cpp)

target_link_libraries(drake_test
        mujoco
        ${drake_LIBRARIES})

add_executable(domain_template
        examples/domain_template/domain_template.cpp
        examples/domain_template/ActionsTemplate.cpp
        ${SOURCES})

target_link_libraries(domain_template
        ${drake_LIBRARIES}
        pthread)