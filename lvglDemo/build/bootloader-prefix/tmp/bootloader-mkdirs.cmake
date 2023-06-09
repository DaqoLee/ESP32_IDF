# Distributed under the OSI-approved BSD 3-Clause License.  See accompanying
# file Copyright.txt or https://cmake.org/licensing for details.

cmake_minimum_required(VERSION 3.5)

file(MAKE_DIRECTORY
  "D:/Espressif/frameworks/esp-idf-v4.4.4/components/bootloader/subproject"
  "D:/ESP32/learn/ESP32_IDF/lvglDemo/build/bootloader"
  "D:/ESP32/learn/ESP32_IDF/lvglDemo/build/bootloader-prefix"
  "D:/ESP32/learn/ESP32_IDF/lvglDemo/build/bootloader-prefix/tmp"
  "D:/ESP32/learn/ESP32_IDF/lvglDemo/build/bootloader-prefix/src/bootloader-stamp"
  "D:/ESP32/learn/ESP32_IDF/lvglDemo/build/bootloader-prefix/src"
  "D:/ESP32/learn/ESP32_IDF/lvglDemo/build/bootloader-prefix/src/bootloader-stamp"
)

set(configSubDirs )
foreach(subDir IN LISTS configSubDirs)
    file(MAKE_DIRECTORY "D:/ESP32/learn/ESP32_IDF/lvglDemo/build/bootloader-prefix/src/bootloader-stamp/${subDir}")
endforeach()
