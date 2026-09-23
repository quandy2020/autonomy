cmake_minimum_required(VERSION 3.20)

project(base)

find_package(autocmake CONFIG REQUIRED)

autocmake_project()
autocmake_build(QUIT)

autocmake_library(base SOURCES src/base.cpp)

autocmake_package()
