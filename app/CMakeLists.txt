cmake_minimum_required(VERSION 3.20.0)

set(SHIELD ssd1306_128x64)

find_package(Zephyr REQUIRED HINTS $ENV{ZEPHYR_BASE})

project(app)

FILE(GLOB app_sources src/*.c)
target_sources(app PRIVATE ${app_sources})