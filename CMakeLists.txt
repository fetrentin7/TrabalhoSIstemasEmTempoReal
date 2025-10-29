cmake_minimum_required(VERSION 3.16.0)
set(PROJECT_NAME "hello_world")

# Add your custom component folder
set(EXTRA_COMPONENT_DIRS "components/network")

include($ENV{IDF_PATH}/tools/cmake/project.cmake)
project(${PROJECT_NAME})
