# Distributed under the OSI-approved BSD 3-Clause License.  See accompanying
# file Copyright.txt or https://cmake.org/licensing for details.

cmake_minimum_required(VERSION 3.5)

file(MAKE_DIRECTORY
  "D:/Espressif/frameworks/esp-idf-v4.4.4/components/bootloader/subproject"
  "F:/Studio/Project/learn/ESP32/u8g2/build/bootloader"
  "F:/Studio/Project/learn/ESP32/u8g2/build/bootloader-prefix"
  "F:/Studio/Project/learn/ESP32/u8g2/build/bootloader-prefix/tmp"
  "F:/Studio/Project/learn/ESP32/u8g2/build/bootloader-prefix/src/bootloader-stamp"
  "F:/Studio/Project/learn/ESP32/u8g2/build/bootloader-prefix/src"
  "F:/Studio/Project/learn/ESP32/u8g2/build/bootloader-prefix/src/bootloader-stamp"
)

set(configSubDirs )
foreach(subDir IN LISTS configSubDirs)
    file(MAKE_DIRECTORY "F:/Studio/Project/learn/ESP32/u8g2/build/bootloader-prefix/src/bootloader-stamp/${subDir}")
endforeach()
