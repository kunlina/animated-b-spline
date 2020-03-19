cmake_minimum_required(VERSION 3.1.0)
ENABLE_LANGUAGE(CXX)
SET(CMAKE_VERBOSE_MAKEFILE OFF)

PROJECT("animated-b-spline")

if (CMAKE_BUILD_TYPE STREQUAL "Release")
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -O3")
else ()
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -DDEBUG")
endif (CMAKE_BUILD_TYPE STREQUAL "Release")


# Find includes in corresponding build directories
set(CMAKE_INCLUDE_CURRENT_DIR ON)
# Instruct CMake to run moc automatically when needed
set(CMAKE_AUTOMOC ON)
# Create code from a list of Qt designer ui files
set(CMAKE_AUTOUIC ON)
# Find the QtWidgets library
find_package(Qt5 COMPONENTS Widgets REQUIRED)
find_package(Qt5 COMPONENTS Core REQUIRED)
find_package(Qt5 COMPONENTS Gui REQUIRED)

# Useful variable
SET(BSPLINE_SRC_PATH  "${CMAKE_CURRENT_SOURCE_DIR}")

# Include paths
INCLUDE_DIRECTORIES(${BSPLINE_SRC_PATH})

# Include src/headers/forms
INCLUDE(${BSPLINE_SRC_PATH}/SourcesLib.cmake)

###########
# Library #
###########

# Create UI header
QT5_WRAP_UI(BSPLINE_FORMS_HEADERS ${BSPLINE_FORMS})

# Create library file
ADD_LIBRARY(bspline SHARED ${BSPLINE_SRC} ${BSPLINE_HEADERS} ${BSPLINE_FORMS_HEADERS})
target_link_libraries(bspline Qt5::Core)
target_link_libraries(bspline Qt5::Widgets)
target_link_libraries(bspline Qt5::Gui)

##############
# Executable #
##############

# Create executable
ADD_EXECUTABLE(BezierCurve "${CMAKE_CURRENT_SOURCE_DIR}/main.cpp")
TARGET_LINK_LIBRARIES(BezierCurve bspline ${QT_LIBRARIES})
target_link_libraries(BezierCurve Qt5::Core)
target_link_libraries(BezierCurve Qt5::Widgets)
target_link_libraries(BezierCurve Qt5::Gui)



