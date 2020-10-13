################################################################################
#                                                                              #
# Copyright (C) 2020 Fondazione Istitito Italiano di Tecnologia (IIT)          #
# All Rights Reserved.                                                         #
#                                                                              #
################################################################################

cmake_minimum_required(VERSION 3.12)
project(web-contest-sandbox VERSION 1.0.0)

find_package(YCM REQUIRED)
find_package(YARP 3.3.0 REQUIRED COMPONENTS os dev sig math pcl)
find_package(ICUBcontrib REQUIRED)
find_package(PCL REQUIRED)
find_package(VTK REQUIRED)
message (STATUS "VTK_VERSION: ${VTK_VERSION}")
if (VTK_VERSION VERSION_LESS "8.90.0")
  include(${VTK_USE_FILE})
endif()
find_package(Gazebo REQUIRED)

list(APPEND CMAKE_MODULE_PATH ${ICUBCONTRIB_MODULE_PATH})
include(ICUBcontribOptions)
include(ICUBcontribHelpers)

icubcontrib_set_default_prefix()
add_subdirectory(gazebo)
include(AddUninstallTarget)

# world
add_library(${PROJECT_NAME}-world SHARED ${CMAKE_SOURCE_DIR}/src/world.cpp)
target_compile_definitions(${PROJECT_NAME}-world PRIVATE _USE_MATH_DEFINES)
target_include_directories(${PROJECT_NAME}-world PRIVATE ${GAZEBO_INCLUDE_DIRS})
target_link_libraries(${PROJECT_NAME}-world PRIVATE ${GAZEBO_LIBRARIES} ${YARP_LIBRARIES})
install(TARGETS ${PROJECT_NAME}-world LIBRARY DESTINATION "${CMAKE_INSTALL_LIBDIR}" COMPONENT shlib)

# assignment
add_executable(assignment_${PROJECT_NAME} ${CMAKE_SOURCE_DIR}/src/main.cpp)
target_compile_definitions(assignment_${PROJECT_NAME} PRIVATE _USE_MATH_DEFINES)
target_include_directories(assignment_${PROJECT_NAME} PRIVATE ${CMAKE_SOURCE_DIR}/src)
target_link_libraries(assignment_${PROJECT_NAME} ${YARP_LIBRARIES} ${PCL_LIBRARIES} ${VTK_LIBRARIES})

if (VTK_VERSION VERSION_GREATER_EQUAL "8.90.0")
  vtk_module_autoinit(TARGETS assignment_${PROJECT_NAME} MODULES ${VTK_LIBRARIES})
endif()

install(TARGETS assignment_${PROJECT_NAME} DESTINATION bin)

file(GLOB scripts ${CMAKE_SOURCE_DIR}/app/scripts/*.xml)
yarp_install(FILES ${scripts} DESTINATION ${ICUBCONTRIB_APPLICATIONS_INSTALL_DIR})
