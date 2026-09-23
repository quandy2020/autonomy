cmake_minimum_required(VERSION 3.20)

project(hello VERSION 0.1.0)

find_package(autocmake CONFIG REQUIRED)

autocmake_project()
autocmake_build(QUIT)

autocmake_library(hello SOURCES src/hello.cpp)
autocmake_library(greet SOURCES greet/src/greet.cpp)
autocmake_binary(hello_main SOURCES src/hello_main.cpp DEPENDENCIES hello)
autocmake_test(hello_TEST SOURCES test/hello_TEST.cpp DEPENDENCIES hello greet)

autocmake_package()
