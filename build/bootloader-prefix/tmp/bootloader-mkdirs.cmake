# Distributed under the OSI-approved BSD 3-Clause License.  See accompanying
# file Copyright.txt or https://cmake.org/licensing for details.

cmake_minimum_required(VERSION 3.5)

file(MAKE_DIRECTORY
  "C:/Espressif/v5.2.2/esp-idf/components/bootloader/subproject"
  "C:/Users/Breno Alencar/Documents/Insper/Estudos/Pesquisa/firmware/firmware_bicycle/self-balanced-bicycle/build/bootloader"
  "C:/Users/Breno Alencar/Documents/Insper/Estudos/Pesquisa/firmware/firmware_bicycle/self-balanced-bicycle/build/bootloader-prefix"
  "C:/Users/Breno Alencar/Documents/Insper/Estudos/Pesquisa/firmware/firmware_bicycle/self-balanced-bicycle/build/bootloader-prefix/tmp"
  "C:/Users/Breno Alencar/Documents/Insper/Estudos/Pesquisa/firmware/firmware_bicycle/self-balanced-bicycle/build/bootloader-prefix/src/bootloader-stamp"
  "C:/Users/Breno Alencar/Documents/Insper/Estudos/Pesquisa/firmware/firmware_bicycle/self-balanced-bicycle/build/bootloader-prefix/src"
  "C:/Users/Breno Alencar/Documents/Insper/Estudos/Pesquisa/firmware/firmware_bicycle/self-balanced-bicycle/build/bootloader-prefix/src/bootloader-stamp"
)

set(configSubDirs )
foreach(subDir IN LISTS configSubDirs)
    file(MAKE_DIRECTORY "C:/Users/Breno Alencar/Documents/Insper/Estudos/Pesquisa/firmware/firmware_bicycle/self-balanced-bicycle/build/bootloader-prefix/src/bootloader-stamp/${subDir}")
endforeach()
if(cfgdir)
  file(MAKE_DIRECTORY "C:/Users/Breno Alencar/Documents/Insper/Estudos/Pesquisa/firmware/firmware_bicycle/self-balanced-bicycle/build/bootloader-prefix/src/bootloader-stamp${cfgdir}") # cfgdir has leading slash
endif()
