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
    PROPERTIES COMPILE_OPTIONS "-Os")

# 10 kHz FOC 在 Debug 中使用保留调试信息的 -Og；-O0 占用过多 Flash，
# 也会增加高频中断的执行时间，挤占按键轮询和 UI 动画的主循环时间。
set_source_files_properties(mycode/JerryFOC.c
    PROPERTIES COMPILE_OPTIONS "$<$<CONFIG:Debug>:-Og>")

# Add include paths
target_include_directories(${CMAKE_PROJECT_NAME} PRIVATE
    mycode
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

    # Add user defined libraries
)

# 让 IDE 的常规构建同步生成最新 BIN，避免只更新 ELF 后误用旧固件。
add_custom_command(TARGET ${CMAKE_PROJECT_NAME} POST_BUILD
    COMMAND ${CMAKE_OBJCOPY} -O binary
        $<TARGET_FILE:${CMAKE_PROJECT_NAME}>
        $<TARGET_FILE_DIR:${CMAKE_PROJECT_NAME}>/${CMAKE_PROJECT_NAME}.bin
    COMMENT "Generating ${CMAKE_PROJECT_NAME}.bin"
    VERBATIM)
