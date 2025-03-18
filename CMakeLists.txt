cmake_minimum_required(VERSION 3.20.0)

list(APPEND BOARD_ROOT ${CMAKE_CURRENT_SOURCE_DIR})

macro(app_set_runner_args)
  board_runner_args(canopen "--node-id=${CONFIG_CANOPEN_NODE_ID}")
endmacro()

find_package(Zephyr REQUIRED HINTS $ENV{ZEPHYR_BASE})
project(gartennode_zysterne)
FILE(GLOB app_sources src/*.c)
target_sources(app PRIVATE ${app_sources} objdict/CO_OD.c)
zephyr_include_directories(objdict src)