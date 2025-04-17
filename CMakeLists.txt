# The following lines of boilerplate have to be in your project's CMakeLists
# in this exact order for cmake to work correctly
cmake_minimum_required(VERSION 3.16)

set(PARTITION_TABLE_CUSTOM_FILENAME "partitions.csv")
set(PARTITION_TABLE_CUSTOM_APP_BIN_OFFSET 0x10000)

#include(${IDF_PATH}/tools/cmake/spiffs.cmake)
# Criar imagem SPIFFS com os arquivos do diretório "spiffs"
#spiffs_create_partition_image(communication_spiffs ${CMAKE_CURRENT_SOURCE_DIR}/spiffs FLASH_IN_PROJECT)

# Habilitar SPIFFS e criar imagem
#idf_component_register(SRCS "")
#spiffs_create_partition_image(storage ../spiffs FLASH_IN_PROJECT)


include($ENV{IDF_PATH}/tools/cmake/project.cmake)
project(self-balanced-biclycle)

#include(spiffs)
#idf_build_get_property(project_dir PROJECT_DIR)
#spiffs_create_partition_image(spiffs ${project_dir}/spiffs FLASH_IN_PROJECT)


