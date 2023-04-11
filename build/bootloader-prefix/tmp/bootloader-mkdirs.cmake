# Distributed under the OSI-approved BSD 3-Clause License.  See accompanying
# file Copyright.txt or https://cmake.org/licensing for details.

cmake_minimum_required(VERSION 3.5)

file(MAKE_DIRECTORY
  "/home/chris/esp/esp-idf/components/bootloader/subproject"
  "/home/chris/Documents/code/esp32/twin-servo/build/bootloader"
  "/home/chris/Documents/code/esp32/twin-servo/build/bootloader-prefix"
  "/home/chris/Documents/code/esp32/twin-servo/build/bootloader-prefix/tmp"
  "/home/chris/Documents/code/esp32/twin-servo/build/bootloader-prefix/src/bootloader-stamp"
  "/home/chris/Documents/code/esp32/twin-servo/build/bootloader-prefix/src"
  "/home/chris/Documents/code/esp32/twin-servo/build/bootloader-prefix/src/bootloader-stamp"
)

set(configSubDirs )
foreach(subDir IN LISTS configSubDirs)
    file(MAKE_DIRECTORY "/home/chris/Documents/code/esp32/twin-servo/build/bootloader-prefix/src/bootloader-stamp/${subDir}")
endforeach()
if(cfgdir)
  file(MAKE_DIRECTORY "/home/chris/Documents/code/esp32/twin-servo/build/bootloader-prefix/src/bootloader-stamp${cfgdir}") # cfgdir has leading slash
endif()
