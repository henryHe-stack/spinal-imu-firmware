cmake_minimum_required(VERSION 3.22)
project(spinal_host_tests C)
set(CMAKE_C_STANDARD 11)
enable_testing()
# Compile production C sources on the host. Only peripheral boundaries are
# mocked; these tests do not model electrical timing or the FreeRTOS scheduler.
add_executable(firmware_tests test_firmware.c
  ../Services/Src/protocol_service.c ../Devices/Src/bno055.c
  ../Devices/Src/tca9548a.c ../BSP/Src/bsp_uart.c ../BSP/Src/bsp_time.c)
target_include_directories(firmware_tests PRIVATE mocks ../Common/Inc
  ../Config/Inc ../Services/Inc ../BSP/Inc ../Devices/Inc)
if(MSVC)
  target_compile_options(firmware_tests PRIVATE /W4 /WX)
else()
  target_compile_options(firmware_tests PRIVATE -Wall -Wextra -Werror)
endif()
add_test(NAME firmware_behavior COMMAND firmware_tests)
add_test(NAME firmware_wire_golden COMMAND ${CMAKE_COMMAND}
  -DPROGRAM=$<TARGET_FILE:firmware_tests>
  -DFIXTURE=${CMAKE_CURRENT_SOURCE_DIR}/../HostApp/tests/fixtures/firmware-v1.json
  -P ${CMAKE_CURRENT_SOURCE_DIR}/check_fixture.cmake)
