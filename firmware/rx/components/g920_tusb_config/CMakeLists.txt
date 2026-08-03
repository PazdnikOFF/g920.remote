# Компонент без единого исходника: он существует ради одного заголовка.
#
# Голый TinyUSB требует `tusb_config.h` от приложения, но сам его каталог в
# свои include-пути не добавляет — их добавляет тот, кто конфигурацию даёт.
# Обёртка `esp_tinyusb` делает ровно это же тремя строками ниже; здесь они
# повторены потому, что обёртка нам не подошла (см. src/idf_component.yml).
idf_component_register(INCLUDE_DIRS "include"
                       REQUIRES espressif__tinyusb)

# Отдать нашу конфигурацию самому TinyUSB.
idf_component_get_property(tusb_lib espressif__tinyusb COMPONENT_LIB)
target_include_directories(${tusb_lib} PRIVATE "${CMAKE_CURRENT_LIST_DIR}/include")
