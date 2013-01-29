cmake_minimum_required(VERSION 2.8)
project(particles)

# Questions about Qt? Look here:
# http://developer.qt.nokia.com/quarterly/view/using_cmake_to_build_qt_projects

#
# If the user specifies -DCMAKE_BUILD_TYPE on the command line, take their
# definition and dump it in the cache along with proper documentation.
# Otherwise set CMAKE_BUILD_TYPE to Debug prior to calling project().
#
if (DEFINED CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE ${CMAKE_BUILD_TYPE} CACHE STRING "Choose the build type.  Options are: Debug, Release")
else()
    set(CMAKE_BUILD_TYPE Debug CACHE STRING "Choose the build type.  Options are: Debug, Release")
endif()

find_package(Qt4 REQUIRED)
if (QT_FOUND)
    include(${QT_USE_FILE})
else()
    message("FATAL ERROR: Qt4 not found.")
endif()

include_directories(
    .
    lib
    /opt/local/include/eigen3/Eigen
    /opt/local/include
    /usr/local/include
)
link_directories(
    /opt/local/lib
    /usr/local/lib
)

if (NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE Release)
endif()

set(CMAKE_CXX_FLAGS "-Qunused-arguments -std=c++11 -stdlib=libc++ -Wall -Wno-long-long -pedantic")
set(CMAKE_CXX_FLAGS_RELEASE "${CMAKE_CXX_FLAGS} -O2 -Werror")
set(CMAKE_CXX_FLAGS_DEBUG "${CMAKE_CXX_FLAGS} -O0 -g")

if (CMAKE_BUILD_TYPE MATCHES "Release")
    add_definitions(-DNDEBUG)
endif ()

set(CMAKE_AUTOMOC ON)
set(QT_USE_QTOPENGL TRUE)

set (GEOM_SRC
    lib/geom/BBox.cpp
    lib/geom/KdTree.cpp
    lib/geom/Mesh.cpp
    lib/geom/Neighbor.cpp
    lib/geom/Neighborhood.cpp
    lib/geom/Triangle.cpp
)
add_library(geom ${GEOM_SRC})

set (PHYSICS_SRC
    lib/physics/Obstacle.cpp
    lib/physics/PlaneObstacle.cpp
    lib/physics/SoftBody.cpp
)
add_library(physics ${PHYSICS_SRC})
target_link_libraries(physics geom)

set (SIM_SRC
    lib/simulator/FrameSaver.cpp
    lib/simulator/SimThread.cpp
    lib/simulator/World.cpp
)
add_library(simulator ${SIM_SRC})
target_link_libraries(simulator physics geom)

set (CORE_SRC
    Options.cpp
    Utils.cpp
)
add_executable(headless main.cpp ${CORE_SRC})
target_link_libraries(headless physics geom simulator)

add_subdirectory(gui)

add_executable(accordion accordion.cpp)
target_link_libraries(accordion geom ${LIBS})

message(STATUS)
message(STATUS "--------------------------------------------------------------------------------")
message(STATUS "Change a value with: cmake . -D<Variable>=<Value>")
message(STATUS "Example: cmake . -DCMAKE_BUILD_TYPE=Debug")
message(STATUS "--------------------------------------------------------------------------------")
message(STATUS "CMAKE_BUILD_TYPE = ${CMAKE_BUILD_TYPE}")
if (CMAKE_BUILD_TYPE MATCHES "Debug")
    message(STATUS "Debug compiler flags: ${CMAKE_CXX_FLAGS_DEBUG}")
else()
    message(STATUS "Release compiler flags: ${CMAKE_CXX_FLAGS_RELEASE}")
endif()
message(STATUS "--------------------------------------------------------------------------------")
message(STATUS)

