# The following lines of boilerplate have to be in your project's CMakeLists
# in this exact order for cmake to work correctly
cmake_minimum_required(VERSION 3.5)

if(NOT DEFINED ENV{RMAKER_PATH})
    set(ENV{RMAKER_PATH} ${CMAKE_CURRENT_LIST_DIR}/../../)
endif()

set(RMAKER_PATH $ENV{RMAKER_PATH})
set(EXTRA_COMPONENT_DIRS ${RMAKER_PATH}/components ${RMAKER_PATH}/examples/common)
set(PROJECT_VER "1.3")

include($ENV{IDF_PATH}/tools/cmake/project.cmake)
project(environmental_data_logger)

