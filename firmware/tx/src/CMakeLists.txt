# Приложение. Общий код подключается не отсюда, а через lib_extra_dirs
# в platformio.ini — PlatformIO собирает его своим сборщиком и линкует.
file(GLOB_RECURSE app_sources ${CMAKE_CURRENT_SOURCE_DIR}/*.c)
idf_component_register(SRCS ${app_sources})
