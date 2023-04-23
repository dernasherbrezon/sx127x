# This is the project CMakeLists.txt file for the test subproject
cmake_minimum_required(VERSION 3.5)

# Include the components directory of the main application:
set(EXTRA_COMPONENT_DIRS "./components")

include($ENV{IDF_PATH}/tools/cmake/project.cmake)
project(test_app)