cmake_minimum_required(VERSION 3.20)

project(app)

find_package(autocmake CONFIG REQUIRED)

autocmake_project()
autocmake_dependencies(REQUIRED base)
autocmake_build(QUIT)

autocmake_binary(app_main SOURCES src/main.cpp DEPENDENCIES base::core)

autocmake_package()
