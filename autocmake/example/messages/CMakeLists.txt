cmake_minimum_required(VERSION 3.20)

project(messages VERSION 0.1.0)

find_package(autocmake CONFIG REQUIRED)

autocmake_project()
autocmake_build(QUIT)

autocmake_protobuf(messages SOURCES proto/greeting.proto proto/note.proto)
autocmake_binary(messages_main SOURCES src/main.cpp DEPENDENCIES messages)

autocmake_package()
