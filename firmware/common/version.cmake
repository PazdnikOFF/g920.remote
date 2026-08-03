# Версия проекта для ESP-IDF — из общего заголовка, не из git.
#
# Две причины не отдавать это `git describe`:
#  1. Единственный источник версии — common/util/include/g920/version.h.
#     Версия в образе прошивки обязана совпадать с тем, что печатает
#     g920_firmware_version(), иначе журнал замеров начнёт врать.
#  2. Сборка не должна зависеть от состояния репозитория. Каталог проекта
#     лежит внутри ~/Documents, который сам оказался git-репозиторием без
#     единого коммита, и git_describe на нём падает с ошибкой CMake.

# Запоминаем каталог здесь, на верхнем уровне файла: внутри функции
# CMAKE_CURRENT_LIST_DIR указывает уже на вызывающий проект.
set(G920_COMMON_DIR "${CMAKE_CURRENT_LIST_DIR}")

function(g920_read_version out_var)
    set(header "${G920_COMMON_DIR}/util/include/g920/version.h")
    if(NOT EXISTS "${header}")
        message(FATAL_ERROR "g920: не найден ${header}")
    endif()
    file(READ "${header}" content)

    foreach(part MAJOR MINOR PATCH)
        unset(CMAKE_MATCH_1)
        string(REGEX MATCH "#define[ \t]+G920_FW_VERSION_${part}[ \t]+([0-9]+)"
               _unused "${content}")
        if(NOT DEFINED CMAKE_MATCH_1)
            message(FATAL_ERROR
                    "g920: в ${header} не найден G920_FW_VERSION_${part}")
        endif()
        set(_${part} "${CMAKE_MATCH_1}")
    endforeach()

    set(${out_var} "${_MAJOR}.${_MINOR}.${_PATCH}" PARENT_SCOPE)
endfunction()
