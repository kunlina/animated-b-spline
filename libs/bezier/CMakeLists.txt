cmake_minimum_required(VERSION 3.1.0)
enable_language(CXX)

project(bezier)

find_package(Qt5 COMPONENTS Widgets REQUIRED)
find_package(Qt5 COMPONENTS Core REQUIRED)

set(BSPLINE_SRC
bezierinterpolator.cpp
)

set(BSPLINE_HEADERS
bezierinterpolator.h
)

set(CMAKE_INCLUDE_CURRENT_DIR ON)

# Create library file
add_library(${PROJECT_NAME} SHARED ${BSPLINE_SRC} ${BSPLINE_HEADERS})

target_link_libraries(${PROJECT_NAME} Qt5::Core)
target_link_libraries(${PROJECT_NAME} Qt5::Gui)
