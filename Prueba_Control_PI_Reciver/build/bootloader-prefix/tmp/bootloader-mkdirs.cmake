# Distributed under the OSI-approved BSD 3-Clause License.  See accompanying
# file Copyright.txt or https://cmake.org/licensing for details.

cmake_minimum_required(VERSION 3.5)

file(MAKE_DIRECTORY
  "C:/Users/Hale/esp/esp-idf/components/bootloader/subproject"
  "C:/Users/Hale/Documents/Proyectos_Personales_HAY/Prueba_Control_PI_Reciver/build/bootloader"
  "C:/Users/Hale/Documents/Proyectos_Personales_HAY/Prueba_Control_PI_Reciver/build/bootloader-prefix"
  "C:/Users/Hale/Documents/Proyectos_Personales_HAY/Prueba_Control_PI_Reciver/build/bootloader-prefix/tmp"
  "C:/Users/Hale/Documents/Proyectos_Personales_HAY/Prueba_Control_PI_Reciver/build/bootloader-prefix/src/bootloader-stamp"
  "C:/Users/Hale/Documents/Proyectos_Personales_HAY/Prueba_Control_PI_Reciver/build/bootloader-prefix/src"
  "C:/Users/Hale/Documents/Proyectos_Personales_HAY/Prueba_Control_PI_Reciver/build/bootloader-prefix/src/bootloader-stamp"
)

set(configSubDirs )
foreach(subDir IN LISTS configSubDirs)
    file(MAKE_DIRECTORY "C:/Users/Hale/Documents/Proyectos_Personales_HAY/Prueba_Control_PI_Reciver/build/bootloader-prefix/src/bootloader-stamp/${subDir}")
endforeach()
if(cfgdir)
  file(MAKE_DIRECTORY "C:/Users/Hale/Documents/Proyectos_Personales_HAY/Prueba_Control_PI_Reciver/build/bootloader-prefix/src/bootloader-stamp${cfgdir}") # cfgdir has leading slash
endif()
