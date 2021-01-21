file(REMOVE_RECURSE
  "config/sdkconfig.h"
  "config/sdkconfig.cmake"
  "bootloader/bootloader.elf"
  "bootloader/bootloader.bin"
  "bootloader/bootloader.map"
  "camera_web_server.map"
  "CMakeFiles/size-components"
)

# Per-language clean rules from dependency scanning.
foreach(lang )
  include(CMakeFiles/size-components.dir/cmake_clean_${lang}.cmake OPTIONAL)
endforeach()
