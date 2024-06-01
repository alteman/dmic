# SPDX-License-Identifier: Apache-2.0

cmake_minimum_required(VERSION 3.20.0)

set_property(GLOBAL PROPERTY CSTD c17)
find_package(Zephyr REQUIRED HINTS $ENV{ZEPHYR_BASE})
project(dmic)

target_sources(app PRIVATE src/main.c src/analyzer.c src/neopixel.c)
