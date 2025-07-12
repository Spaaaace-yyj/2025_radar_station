# Distributed under the OSI-approved BSD 3-Clause License.  See accompanying
# file Copyright.txt or https://cmake.org/licensing for details.

cmake_minimum_required(VERSION 3.5)

file(MAKE_DIRECTORY
  "/home/spaaaaace/Code/mid70/2025_radar_station/build/livox_sdk_vendor/livox-sdk-v2.3.1-prefix/src/livox-sdk-v2.3.1"
  "/home/spaaaaace/Code/mid70/2025_radar_station/build/livox_sdk_vendor/livox-sdk-v2.3.1-prefix/src/livox-sdk-v2.3.1-build"
  "/home/spaaaaace/Code/mid70/2025_radar_station/build/livox_sdk_vendor/livox-sdk-v2.3.1-prefix"
  "/home/spaaaaace/Code/mid70/2025_radar_station/build/livox_sdk_vendor/livox-sdk-v2.3.1-prefix/tmp"
  "/home/spaaaaace/Code/mid70/2025_radar_station/build/livox_sdk_vendor/livox-sdk-v2.3.1-prefix/src/livox-sdk-v2.3.1-stamp"
  "/home/spaaaaace/Code/mid70/2025_radar_station/build/livox_sdk_vendor/livox-sdk-v2.3.1-prefix/src"
  "/home/spaaaaace/Code/mid70/2025_radar_station/build/livox_sdk_vendor/livox-sdk-v2.3.1-prefix/src/livox-sdk-v2.3.1-stamp"
)

set(configSubDirs )
foreach(subDir IN LISTS configSubDirs)
    file(MAKE_DIRECTORY "/home/spaaaaace/Code/mid70/2025_radar_station/build/livox_sdk_vendor/livox-sdk-v2.3.1-prefix/src/livox-sdk-v2.3.1-stamp/${subDir}")
endforeach()
if(cfgdir)
  file(MAKE_DIRECTORY "/home/spaaaaace/Code/mid70/2025_radar_station/build/livox_sdk_vendor/livox-sdk-v2.3.1-prefix/src/livox-sdk-v2.3.1-stamp${cfgdir}") # cfgdir has leading slash
endif()
