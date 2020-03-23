cmake_minimum_required(VERSION 3.1.0)
enable_language(CXX)

project(geometry)

add_library(${PROJECT_NAME} SHARED GeometryCompute.cpp GeometryCompute.h)