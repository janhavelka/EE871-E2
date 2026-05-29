idf_component_register(
  SRCS "main.cpp"
  INCLUDE_DIRS "." "../../common"
  REQUIRES "EE871-E2" esp_driver_gpio esp_timer freertos
)

target_compile_features(${COMPONENT_LIB} PUBLIC cxx_std_17)
