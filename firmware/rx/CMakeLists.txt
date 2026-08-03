cmake_minimum_required(VERSION 3.16.0)

# Версия — из общего заголовка. Подробности почему не из git — там же.
include(${CMAKE_CURRENT_LIST_DIR}/../common/version.cmake)
g920_read_version(PROJECT_VER)

include($ENV{IDF_PATH}/tools/cmake/project.cmake)
project(g920-rx)
