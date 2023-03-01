cmake_minimum_required(VERSION 3.15)

project(USART
  LANGUAGES CXX
  VERSION 1.2.0
)

add_executable(emulator)

target_sources(emulator PUBLIC main.cpp)

target_compile_features(emulator PRIVATE cxx_std_20)

set_target_properties(emulator PROPERTIES
   DEBUG_POSTFIX "d"
)

install(TARGETS emulator)
install(FILES CTRL_LSB.bin CTRL_MSB.bin ROM.bin DESTINATION bin)

configure_file(CTRL_LSB.bin CTRL_LSB.bin COPYONLY)
configure_file(CTRL_MSB.bin CTRL_MSB.bin COPYONLY)
configure_file(ROM.bin ROM.bin COPYONLY)