################################################################################
#                                                                              #
# Copyright (C) 2020 Fondazione Istitito Italiano di Tecnologia (IIT)          #
# All Rights Reserved.                                                         #
#                                                                              #
################################################################################

cmake_minimum_required(VERSION 3.12)
project(web-contest-sandbox VERSION 1.0.0)

find_package(YCM REQUIRED)
find_package(ICUBcontrib REQUIRED)
list(APPEND CMAKE_MODULE_PATH ${ICUBCONTRIB_MODULE_PATH})
include(ICUBcontribOptions)
include(ICUBcontribHelpers)

icubcontrib_set_default_prefix()
add_subdirectory(gazebo)
include(AddUninstallTarget)
