# SPDX-License-Identifier: Apache-2.0

cmake_minimum_required(VERSION 3.20.0)

set(AS5048_DIR ${CMAKE_CURRENT_LIST_DIR} CACHE PATH "AS5048 directory")
add_subdirectory(drivers)
zephyr_include_directories(drivers)