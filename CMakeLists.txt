cmake_minimum_required(VERSION 3.1.0)
enable_language(CXX)
set(CMAKE_VERBOSE_MAKEFILE OFF)

project("animated-b-spline")

if (CMAKE_BUILD_TYPE STREQUAL "Release")
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -O3")
else ()
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -DDEBUG")
endif (CMAKE_BUILD_TYPE STREQUAL "Release")

###########
# Library #
###########
add_subdirectory(libs)

##############
# Executable #
##############
# Add includes directories for build
include_directories(${CMAKE_CURRENT_SOURCE_DIR}/libs)

# Instruct CMake to run moc automatically when needed
set(CMAKE_AUTOMOC ON)
# Create code from a list of Qt designer ui files
set(CMAKE_AUTOUIC ON)

# Find the QtWidgets library
find_package(Qt5 COMPONENTS Widgets REQUIRED)
find_package(Qt5 COMPONENTS Core REQUIRED)
find_package(Qt5 COMPONENTS Gui REQUIRED)

# Create executable
add_executable(BezierCurve
    main.cpp
    mainwindow.cpp
    movingellipseitem.cpp
    mainwindow.h
    movingellipseitem.h
    mainwindow.ui
    )

target_link_libraries(BezierCurve bezier)
target_link_libraries(BezierCurve geometry)
target_link_libraries(BezierCurve Qt5::Core)
target_link_libraries(BezierCurve Qt5::Widgets)
target_link_libraries(BezierCurve Qt5::Gui)




