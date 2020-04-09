cmake_minimum_required(VERSION 3.1.0)
enable_language(CXX)

project(kbspline)

add_library(${PROJECT_NAME} SHARED BSplineSubdivision.cpp BSplineSubdivision.hpp)
