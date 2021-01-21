file(REMOVE_RECURSE
  "config/sdkconfig.h"
  "config/sdkconfig.cmake"
  "bootloader/bootloader.elf"
  "bootloader/bootloader.bin"
  "bootloader/bootloader.map"
  "camera_web_server.map"
  "dummy_main_src.c"
  "CMakeFiles/camera_web_server.elf.dir/dummy_main_src.c.obj"
  "camera_web_server.elf.pdb"
  "camera_web_server.elf"
)

# Per-language clean rules from dependency scanning.
foreach(lang C)
  include(CMakeFiles/camera_web_server.elf.dir/cmake_clean_${lang}.cmake OPTIONAL)
endforeach()
