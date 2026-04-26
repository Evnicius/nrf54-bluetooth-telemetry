cmake_minimum_required(VERSION 3.20.0)
find_package(Zephyr REQUIRED HINTS $ENV{ZEPHYR_BASE})

project(nrf54l15-ble-telemetry)

target_sources(app PRIVATE 
                    src/main.c
                    drivers/HD44780/HD44780.c
)

zephyr_library_include_directories(.)

add_subdirectory(drivers)