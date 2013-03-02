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
    set(CMAKE_BUILD_TYPE ${CMAKE_BUILD_TYPE} CACHE STRING "Choose the build type. Options are: Debug, Release")
else()
    set(CMAKE_BUILD_TYPE Debug CACHE STRING "Choose the build type. Options are: Debug, Release")
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
    build
    /opt/local/include/eigen3/Eigen
    /opt/local/include
    /usr/local/include
)
link_directories(
    /opt/local/lib
    /usr/local/lib
)

find_library(GL_LIBRARY OpenGL)
find_library(GLUT_LIBRARY GLUT)
set(GL_LIBS ${GL_LIBRARY} ${GLUT_LIBRARY})

if (NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE Release)
endif()

set(CMAKE_CXX_FLAGS "-Qunused-arguments -std=c++11 -stdlib=libc++ -Wall -Wno-long-long -pedantic")
set(CMAKE_CXX_FLAGS_RELEASE "${CMAKE_CXX_FLAGS} -O2")
set(CMAKE_CXX_FLAGS_DEBUG "${CMAKE_CXX_FLAGS} -O0 -g")

if (CMAKE_BUILD_TYPE MATCHES "Release")
    add_definitions(-DNDEBUG)
endif ()

set(QT_USE_QTOPENGL TRUE)

# Geometry
set(GEOM_SRC
    lib/geom/BBox.cpp
    lib/geom/KdTree.cpp
    lib/geom/Mesh.cpp
    lib/geom/Neighbor.cpp
    lib/geom/Neighborhood.cpp
    lib/geom/Octree.cpp
    lib/geom/Triangle.cpp
    lib/geom/VectorList.cpp
)
add_library(geom ${GEOM_SRC})

# Physics
set(PHYSICS_SRC
    lib/physics/Obstacle.cpp
    lib/physics/PlaneObstacle.cpp
    lib/physics/SoftBody.cpp
)
add_library(physics ${PHYSICS_SRC})
target_link_libraries(physics geom tbb)

# Basic simulation
set(SIM_SRC
    lib/simulator/FrameSaver.cpp
    lib/simulator/Options.cpp
    lib/simulator/SimThread.cpp
    lib/simulator/World.cpp
)
add_library(simulator ${SIM_SRC})
target_link_libraries(simulator physics geom)
set_target_properties(simulator PROPERTIES AUTOMOC TRUE)

# Generic GUI
set(GUI_SRC
    lib/gui/GlWidget.cpp
    lib/gui/MatrixViewer.cpp
)
set(GUI_FORMS lib/gui/MatrixViewer.ui)
set(RESOURCES lib/gui/images.qrc)
qt4_wrap_ui(GUI_FORMS_HEADERS ${GUI_FORMS})
qt4_add_resources(RESOURCES_RCC ${RESOURCES})

add_library(gui ${GUI_SRC} ${GUI_FORMS_HEADERS})
target_link_libraries(gui ${GL_LIBS} ${QT_LIBRARIES} ${QT_QTOPENGL_LIBRARY})
set_target_properties(gui PROPERTIES AUTOMOC TRUE)

# Live interactive simulator GUI
set(LIVE_SIM_SRC
    lib/gui/live_sim/SimMainWindow.cpp
    lib/gui/live_sim/SimGlWidget.cpp
)
set(LIVE_SIM_FORMS lib/gui/live_sim/SimMainWindow.ui)
qt4_wrap_ui(LIVE_SIM_FORMS_HEADERS ${LIVE_SIM_FORMS})

add_library(live_sim ${LIVE_SIM_SRC} ${LIVE_SIM_FORMS_HEADERS})
target_link_libraries(live_sim gui)
set_target_properties(live_sim PROPERTIES AUTOMOC TRUE)

# Obj File Playback GUI
set(PLAYBACK_SRC
    lib/gui/playback/MeshSet.cpp
    lib/gui/playback/ObjGlWidget.cpp
    lib/gui/playback/ObjLoadThread.cpp
    lib/gui/playback/ObjMainWindow.cpp
)
set(PLAYBACK_FORMS lib/gui/playback/ObjMainWindow.ui)
qt4_wrap_ui(PLAYBACK_FORMS_HEADERS ${PLAYBACK_FORMS})

add_library(playback ${PLAYBACK_SRC} ${PLAYBACK_FORMS_HEADERS})
target_link_libraries(playback gui)
set_target_properties(playback PROPERTIES AUTOMOC TRUE)

# Headless simulator cmd
set(CORE_SRC
    Utils.cpp
)
add_executable(headless cmd/headless/main.cpp ${CORE_SRC})
target_link_libraries(headless physics geom simulator ${QT_LIBRARIES})
set_target_properties(headless PROPERTIES AUTOMOC TRUE)

# Interactive GUI simulator cmd
add_executable(interactive cmd/interactive/main.cpp)
target_link_libraries(interactive gui live_sim simulator)
set_target_properties(interactive PROPERTIES AUTOMOC TRUE)

# OBJ playback GUI cmd
add_executable(obj_play cmd/obj_play/main.cpp)
target_link_libraries(obj_play playback geom)
set_target_properties(obj_play PROPERTIES AUTOMOC TRUE)

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

