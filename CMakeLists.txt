cmake_minimum_required(VERSION 3.22)

#
# This file is generated only once,
# and is not re-generated if converter is called multiple times.
#
# User is free to modify the file as much as necessary
#

# Setup compiler settings
set(CMAKE_C_STANDARD 11)
set(CMAKE_C_STANDARD_REQUIRED ON)
set(CMAKE_C_EXTENSIONS ON)


# Define the build type
if(NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE "Debug")
endif()

# Set the project name
set(CMAKE_PROJECT_NAME FOC_two)

# Enable compile command to ease indexing with e.g. clangd
set(CMAKE_EXPORT_COMPILE_COMMANDS TRUE)

# Core project settings
project(${CMAKE_PROJECT_NAME})
message("Build type: " ${CMAKE_BUILD_TYPE})

# Enable CMake support for ASM and C languages
enable_language(C ASM)

# Create an executable object type
add_executable(${CMAKE_PROJECT_NAME})

# Add STM32CubeMX generated sources
add_subdirectory(cmake/stm32cubemx)

# Keep debug symbols and useful stepping in application code. HAL and USB
# drivers use size optimization so the Debug image fits the real 128 KiB Flash.
target_compile_options(${CMAKE_PROJECT_NAME} PRIVATE $<$<CONFIG:Debug>:-Og>)
target_compile_options(STM32_Drivers PRIVATE $<$<CONFIG:Debug>:-Os>)
target_compile_options(USB_Device_Library PRIVATE $<$<CONFIG:Debug>:-Os>)
# The USB core and its existing LTO application glue can share constants and
# calls too; retain motor/HSO functions within the real 128KiB Flash limit.
target_compile_options(USB_Device_Library PRIVATE -flto)
# Share peripheral helpers with the existing LTO encoder/Flash/serial code.
# Keep HAL startup and the HSO fast path at their existing settings.
set_property(SOURCE Drivers/STM32G4xx_HAL_Driver/Src/stm32g4xx_hal_spi.c
    Drivers/STM32G4xx_HAL_Driver/Src/stm32g4xx_hal_gpio.c
    Drivers/STM32G4xx_HAL_Driver/Src/stm32g4xx_hal_uart.c
    Drivers/STM32G4xx_HAL_Driver/Src/stm32g4xx_hal_uart_ex.c
    TARGET_DIRECTORY STM32_Drivers APPEND PROPERTY COMPILE_OPTIONS -flto)

# Link directories setup
target_link_directories(${CMAKE_PROJECT_NAME} PRIVATE
    # Add user defined library search paths
)

# Add sources to executable
target_sources(${CMAKE_PROJECT_NAME} PRIVATE
    mycode/JerryFOC.c
    mycode/MT6701.c
    mycode/angle_src_encoder.c
    mycode/motor_params.c
    mycode/motor_sampling.c
    mycode/motor_control.c
    mycode/st_hso.c
    ThirdParty/ST_HSO/fixpmath.c
    mycode/ws2812.c
    mycode/protocol.c

    mycode/kk_oled/graphics/kk_oled.c
    mycode/kk_oled/graphics/kk_oled_bitmap.c
    mycode/kk_oled/graphics/kk_oled_draw.c
    mycode/kk_oled/graphics/kk_oled_font.c
    mycode/kk_oled/driver/kk_oled_driver.c

    mycode/kk_ui/src/kk_ui.c
    mycode/kk_ui/src/kk_ui_anim.c
    mycode/kk_ui/src/kk_ui_dialog.c
    mycode/kk_ui/src/kk_ui_draw.c
    mycode/kk_ui/src/kk_ui_home.c
    mycode/kk_ui/src/kk_ui_menu.c
    mycode/kk_ui/src/kk_ui_nav.c
    mycode/kk_ui/src/kk_ui_page.c

    mycode/ui_app.c
    mycode/ui_input.c
    mycode/ui_fonts.c
    mycode/w25q64.c
    mycode/app_storage.c
    # Add user sources here
)

# UI/图形/存储代码在 Debug 下也按 -Os 编译，给 FOC 调试构建留出 Flash 空间
set(FOC_UI_SOURCES
    mycode/kk_oled/graphics/kk_oled.c
    mycode/kk_oled/graphics/kk_oled_bitmap.c
    mycode/kk_oled/graphics/kk_oled_draw.c
    mycode/kk_oled/graphics/kk_oled_font.c
    mycode/kk_oled/driver/kk_oled_driver.c
    mycode/kk_ui/src/kk_ui.c
    mycode/kk_ui/src/kk_ui_anim.c
    mycode/kk_ui/src/kk_ui_dialog.c
    mycode/kk_ui/src/kk_ui_draw.c
    mycode/kk_ui/src/kk_ui_home.c
    mycode/kk_ui/src/kk_ui_menu.c
    mycode/kk_ui/src/kk_ui_nav.c
    mycode/kk_ui/src/kk_ui_page.c
    mycode/ui_app.c
    mycode/ui_input.c
    mycode/ui_fonts.c
    mycode/w25q64.c
    mycode/app_storage.c
    mycode/protocol.c
    mycode/ws2812.c
)
set_source_files_properties(${FOC_UI_SOURCES}
    PROPERTIES COMPILE_OPTIONS "-Os;-flto")
# Share the two-axis/UI code at link time while retaining Debug symbols.
target_link_options(${CMAKE_PROJECT_NAME} PRIVATE -flto=1 -flto-partition=one)
set_source_files_properties(mycode/motor_params.c PROPERTIES COMPILE_OPTIONS "-Os;-flto")
set_source_files_properties(mycode/motor_control.c mycode/motor_sampling.c
    mycode/MT6701.c mycode/JerryFOC.c mycode/angle_src_encoder.c PROPERTIES COMPILE_OPTIONS "-Os;-flto")

# Keep the growing main-loop orchestration within the 128 KiB device too.
# Debug symbols remain available; optimized local variables may not be visible.
set_source_files_properties(Core/Src/main.c PROPERTIES COMPILE_OPTIONS "-Os;-flto")
set_source_files_properties(Core/Src/adc.c Core/Src/tim.c Core/Src/spi.c
    Core/Src/gpio.c Core/Src/dma.c Core/Src/fdcan.c Core/Src/i2c.c
    Core/Src/usart.c Core/Src/stm32g4xx_hal_msp.c
    PROPERTIES COMPILE_OPTIONS "-Os;-flto")
# IRQ dispatch wrappers need no -Og expansion; keep the real-time control
# and protection paths intact while recovering space on the 128KiB device.
set_source_files_properties(Core/Src/stm32g4xx_it.c PROPERTIES COMPILE_OPTIONS "-Os")
# Generated USB application glue follows the same size policy as the USB
# library, with link-time optimization to fit hand-follow planning as well.
# Debug symbols remain, although optimized locals may not be inspectable.
set_source_files_properties(USB_Device/App/usb_device.c USB_Device/App/usbd_desc.c
    USB_Device/App/usbd_cdc_if.c USB_Device/Target/usbd_conf.c
    PROPERTIES COMPILE_OPTIONS "-Os;-flto")

# Add include paths
target_include_directories(${CMAKE_PROJECT_NAME} PRIVATE
    mycode
    mycode/st_hso_port
    ThirdParty/ST_HSO
    mycode/kk_oled/include
    mycode/kk_oled/graphics
    mycode/kk_oled/driver
    mycode/kk_ui/include
    mycode/kk_ui/src
    # Add user defined include paths
)

# Add project symbols (macros)
target_compile_definitions(${CMAKE_PROJECT_NAME} PRIVATE
    KK_UI_REFRESH_MODE=2
    # Add user defined symbols
)

# Remove wrong libob.a library dependency when using cpp files
list(REMOVE_ITEM CMAKE_C_IMPLICIT_LINK_LIBRARIES ob)

# Add linked libraries
target_link_libraries(${CMAKE_PROJECT_NAME}
    stm32cubemx
    ${CMAKE_CURRENT_SOURCE_DIR}/ThirdParty/ST_HSO/libHSO.a
    ${CMAKE_CURRENT_SOURCE_DIR}/ThirdParty/ST_HSO/libImpedCorr.a

    # Add user defined libraries
)

set_source_files_properties(mycode/st_hso.c ThirdParty/ST_HSO/fixpmath.c
    PROPERTIES COMPILE_OPTIONS "-O2")

# 让 IDE 的常规构建同步生成最新 BIN，避免只更新 ELF 后误用旧固件。
add_custom_command(TARGET ${CMAKE_PROJECT_NAME} POST_BUILD
    COMMAND ${CMAKE_OBJCOPY} -O binary
        $<TARGET_FILE:${CMAKE_PROJECT_NAME}>
        $<TARGET_FILE_DIR:${CMAKE_PROJECT_NAME}>/${CMAKE_PROJECT_NAME}.bin
    COMMENT "Generating ${CMAKE_PROJECT_NAME}.bin"
    VERBATIM)
