set(
  COMPONENT_SRCS 
  "app_main.c" 
  "app_wifi.c" 
  "app_camera.c" 
  "app_cmd.c" 
  "app_weight.c" 
  "app_led.c"
  "app_key.c"
  "app_httpd.c" 
  "app_httpc.c" 
  "app_scale.c" 
  "blufi.c"
  "blufi_security.c" 
  "board_driver.c"
  "task_httpc.c"
  "timer_tick.c"
)
set(
  COMPONENT_ADD_INCLUDEDIRS 
  "include"
)

set(
  COMPONENT_REQUIRES
  esp32-camera
  esp-face
  nvs_flash
  esp_http_client
  esp_http_server
  fb_gfx
)

set(
  COMPONENT_EMBED_FILES
  "www/index_ov2640.html.gz"
  "www/index_ov3660.html.gz"
)

set(
  COMPONENT_EMBED_TXTFILES 
  lulupet_com_root_cert.pem
)

register_component()
