# Distributed under the OSI-approved BSD 3-Clause License.  See accompanying
# file Copyright.txt or https://cmake.org/licensing for details.

cmake_minimum_required(VERSION ${CMAKE_VERSION}) # this file comes with cmake

# If CMAKE_DISABLE_SOURCE_CHANGES is set to true and the source directory is an
# existing directory in our source tree, calling file(MAKE_DIRECTORY) on it
# would cause a fatal error, even though it would be a no-op.
if(NOT EXISTS "D:/Keil_Project/Robocon2026_Chassis_Frame/MDK-ARM/tmp/chassis+chassis")
  file(MAKE_DIRECTORY "D:/Keil_Project/Robocon2026_Chassis_Frame/MDK-ARM/tmp/chassis+chassis")
endif()
file(MAKE_DIRECTORY
  "D:/Keil_Project/Robocon2026_Chassis_Frame/MDK-ARM/tmp/1"
  "D:/Keil_Project/Robocon2026_Chassis_Frame/MDK-ARM/tmp/chassis+chassis"
  "D:/Keil_Project/Robocon2026_Chassis_Frame/MDK-ARM/tmp/chassis+chassis/tmp"
  "D:/Keil_Project/Robocon2026_Chassis_Frame/MDK-ARM/tmp/chassis+chassis/src/chassis+chassis-stamp"
  "D:/Keil_Project/Robocon2026_Chassis_Frame/MDK-ARM/tmp/chassis+chassis/src"
  "D:/Keil_Project/Robocon2026_Chassis_Frame/MDK-ARM/tmp/chassis+chassis/src/chassis+chassis-stamp"
)

set(configSubDirs )
foreach(subDir IN LISTS configSubDirs)
    file(MAKE_DIRECTORY "D:/Keil_Project/Robocon2026_Chassis_Frame/MDK-ARM/tmp/chassis+chassis/src/chassis+chassis-stamp/${subDir}")
endforeach()
if(cfgdir)
  file(MAKE_DIRECTORY "D:/Keil_Project/Robocon2026_Chassis_Frame/MDK-ARM/tmp/chassis+chassis/src/chassis+chassis-stamp${cfgdir}") # cfgdir has leading slash
endif()
