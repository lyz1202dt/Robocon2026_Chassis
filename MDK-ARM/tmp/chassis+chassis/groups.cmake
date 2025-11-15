# groups.cmake

# group Application/MDK-ARM
add_library(Group_Application_MDK-ARM OBJECT
  "${SOLUTION_ROOT}/startup_stm32f407xx.s"
)
target_include_directories(Group_Application_MDK-ARM PUBLIC
  $<TARGET_PROPERTY:${CONTEXT},INTERFACE_INCLUDE_DIRECTORIES>
)
target_compile_definitions(Group_Application_MDK-ARM PUBLIC
  $<TARGET_PROPERTY:${CONTEXT},INTERFACE_COMPILE_DEFINITIONS>
)
add_library(Group_Application_MDK-ARM_ABSTRACTIONS INTERFACE)
target_link_libraries(Group_Application_MDK-ARM_ABSTRACTIONS INTERFACE
  ${CONTEXT}_ABSTRACTIONS
)
target_compile_options(Group_Application_MDK-ARM PUBLIC
  $<TARGET_PROPERTY:${CONTEXT},INTERFACE_COMPILE_OPTIONS>
)
target_link_libraries(Group_Application_MDK-ARM PUBLIC
  Group_Application_MDK-ARM_ABSTRACTIONS
)
set(COMPILE_DEFINITIONS
  STM32F407xx
  _RTE_
)
cbuild_set_defines(AS_ARM COMPILE_DEFINITIONS)
set_source_files_properties("${SOLUTION_ROOT}/startup_stm32f407xx.s" PROPERTIES
  COMPILE_FLAGS "${COMPILE_DEFINITIONS}"
)

# group Drivers/STM32F4xx_HAL_Driver
add_library(Group_Drivers_STM32F4xx_HAL_Driver OBJECT
  "${SOLUTION_ROOT}/../Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_tim.c"
  "${SOLUTION_ROOT}/../Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_tim_ex.c"
  "${SOLUTION_ROOT}/../Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_pcd.c"
  "${SOLUTION_ROOT}/../Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_pcd_ex.c"
  "${SOLUTION_ROOT}/../Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_ll_usb.c"
  "${SOLUTION_ROOT}/../Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_rcc.c"
  "${SOLUTION_ROOT}/../Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_rcc_ex.c"
  "${SOLUTION_ROOT}/../Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_flash.c"
  "${SOLUTION_ROOT}/../Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_flash_ex.c"
  "${SOLUTION_ROOT}/../Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_flash_ramfunc.c"
  "${SOLUTION_ROOT}/../Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_gpio.c"
  "${SOLUTION_ROOT}/../Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_dma_ex.c"
  "${SOLUTION_ROOT}/../Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_dma.c"
  "${SOLUTION_ROOT}/../Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_pwr.c"
  "${SOLUTION_ROOT}/../Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_pwr_ex.c"
  "${SOLUTION_ROOT}/../Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_cortex.c"
  "${SOLUTION_ROOT}/../Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal.c"
  "${SOLUTION_ROOT}/../Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_exti.c"
  "${SOLUTION_ROOT}/../Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_can.c"
  "${SOLUTION_ROOT}/../Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_spi.c"
  "${SOLUTION_ROOT}/../Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_uart.c"
)
target_include_directories(Group_Drivers_STM32F4xx_HAL_Driver PUBLIC
  $<TARGET_PROPERTY:${CONTEXT},INTERFACE_INCLUDE_DIRECTORIES>
)
target_compile_definitions(Group_Drivers_STM32F4xx_HAL_Driver PUBLIC
  $<TARGET_PROPERTY:${CONTEXT},INTERFACE_COMPILE_DEFINITIONS>
)
add_library(Group_Drivers_STM32F4xx_HAL_Driver_ABSTRACTIONS INTERFACE)
target_link_libraries(Group_Drivers_STM32F4xx_HAL_Driver_ABSTRACTIONS INTERFACE
  ${CONTEXT}_ABSTRACTIONS
)
target_compile_options(Group_Drivers_STM32F4xx_HAL_Driver PUBLIC
  $<TARGET_PROPERTY:${CONTEXT},INTERFACE_COMPILE_OPTIONS>
)
target_link_libraries(Group_Drivers_STM32F4xx_HAL_Driver PUBLIC
  Group_Drivers_STM32F4xx_HAL_Driver_ABSTRACTIONS
)
set_source_files_properties("${SOLUTION_ROOT}/../Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_pcd.c" PROPERTIES
  COMPILE_OPTIONS ""
)
set_source_files_properties("${SOLUTION_ROOT}/../Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_pcd_ex.c" PROPERTIES
  COMPILE_OPTIONS ""
)
set_source_files_properties("${SOLUTION_ROOT}/../Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_ll_usb.c" PROPERTIES
  COMPILE_OPTIONS ""
)
set_source_files_properties("${SOLUTION_ROOT}/../Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_spi.c" PROPERTIES
  COMPILE_OPTIONS ""
)

# group Drivers/CMSIS
add_library(Group_Drivers_CMSIS OBJECT
  "${SOLUTION_ROOT}/../Core/Src/system_stm32f4xx.c"
)
target_include_directories(Group_Drivers_CMSIS PUBLIC
  $<TARGET_PROPERTY:${CONTEXT},INTERFACE_INCLUDE_DIRECTORIES>
)
target_compile_definitions(Group_Drivers_CMSIS PUBLIC
  $<TARGET_PROPERTY:${CONTEXT},INTERFACE_COMPILE_DEFINITIONS>
)
add_library(Group_Drivers_CMSIS_ABSTRACTIONS INTERFACE)
target_link_libraries(Group_Drivers_CMSIS_ABSTRACTIONS INTERFACE
  ${CONTEXT}_ABSTRACTIONS
)
target_compile_options(Group_Drivers_CMSIS PUBLIC
  $<TARGET_PROPERTY:${CONTEXT},INTERFACE_COMPILE_OPTIONS>
)
target_link_libraries(Group_Drivers_CMSIS PUBLIC
  Group_Drivers_CMSIS_ABSTRACTIONS
)

# group Middlewares/FreeRTOS
add_library(Group_Middlewares_FreeRTOS OBJECT
  "${SOLUTION_ROOT}/../Middlewares/Third_Party/FreeRTOS/Source/croutine.c"
  "${SOLUTION_ROOT}/../Middlewares/Third_Party/FreeRTOS/Source/event_groups.c"
  "${SOLUTION_ROOT}/../Middlewares/Third_Party/FreeRTOS/Source/list.c"
  "${SOLUTION_ROOT}/../Middlewares/Third_Party/FreeRTOS/Source/queue.c"
  "${SOLUTION_ROOT}/../Middlewares/Third_Party/FreeRTOS/Source/stream_buffer.c"
  "${SOLUTION_ROOT}/../Middlewares/Third_Party/FreeRTOS/Source/tasks.c"
  "${SOLUTION_ROOT}/../Middlewares/Third_Party/FreeRTOS/Source/timers.c"
  "${SOLUTION_ROOT}/../Middlewares/Third_Party/FreeRTOS/Source/CMSIS_RTOS/cmsis_os.c"
  "${SOLUTION_ROOT}/../Middlewares/Third_Party/FreeRTOS/Source/portable/MemMang/heap_4.c"
  "${SOLUTION_ROOT}/../Middlewares/Third_Party/FreeRTOS/Source/portable/RVDS/ARM_CM4F/port.c"
)
target_include_directories(Group_Middlewares_FreeRTOS PUBLIC
  $<TARGET_PROPERTY:${CONTEXT},INTERFACE_INCLUDE_DIRECTORIES>
)
target_compile_definitions(Group_Middlewares_FreeRTOS PUBLIC
  $<TARGET_PROPERTY:${CONTEXT},INTERFACE_COMPILE_DEFINITIONS>
)
add_library(Group_Middlewares_FreeRTOS_ABSTRACTIONS INTERFACE)
target_link_libraries(Group_Middlewares_FreeRTOS_ABSTRACTIONS INTERFACE
  ${CONTEXT}_ABSTRACTIONS
)
target_compile_options(Group_Middlewares_FreeRTOS PUBLIC
  $<TARGET_PROPERTY:${CONTEXT},INTERFACE_COMPILE_OPTIONS>
)
target_link_libraries(Group_Middlewares_FreeRTOS PUBLIC
  Group_Middlewares_FreeRTOS_ABSTRACTIONS
)

# group BaseLib
add_library(Group_BaseLib OBJECT
  "${SOLUTION_ROOT}/../BaseLib/CANDrive.c"
  "${SOLUTION_ROOT}/../BaseLib/motor.c"
  "${SOLUTION_ROOT}/../Bsp/bsp_dwt.c"
  "${SOLUTION_ROOT}/../BaseLib/Odrive.c"
  "${SOLUTION_ROOT}/../BaseLib/Canbuffer.c"
  "${SOLUTION_ROOT}/../BaseLib/slope.c"
  "${SOLUTION_ROOT}/../BaseLib/WatchDog2.c"
  "${SOLUTION_ROOT}/../BaseLib/list.c"
  "${SOLUTION_ROOT}/../BaseLib/PID_old.c"
  "${SOLUTION_ROOT}/../BaseLib/STP-23L.c"
  "${SOLUTION_ROOT}/../BaseLib/AutoPilot.c"
  "${SOLUTION_ROOT}/../BaseLib/ForceChassis.c"
  "${SOLUTION_ROOT}/../Matrix/svd.c"
  "${SOLUTION_ROOT}/drive_callback.c"
  "${SOLUTION_ROOT}/../BaseLib/vesc.c"
  "${SOLUTION_ROOT}/../BaseLib/Task_Init.c"
  "${SOLUTION_ROOT}/Pilot_callback.c"
  "${SOLUTION_ROOT}/../BaseLib/JY61.c"
  "${SOLUTION_ROOT}/../BaseLib/WatchDog.c"
  "${SOLUTION_ROOT}/../BaseLib/encoder.c"
  "${SOLUTION_ROOT}/../BaseLib/AS5047.c"
)
target_include_directories(Group_BaseLib PUBLIC
  $<TARGET_PROPERTY:${CONTEXT},INTERFACE_INCLUDE_DIRECTORIES>
  "${SOLUTION_ROOT}/../BaseLib"
  "${SOLUTION_ROOT}/../Bsp"
  "${SOLUTION_ROOT}/../Matrix"
  "${SOLUTION_ROOT}/."
)
target_compile_definitions(Group_BaseLib PUBLIC
  $<TARGET_PROPERTY:${CONTEXT},INTERFACE_COMPILE_DEFINITIONS>
)
add_library(Group_BaseLib_ABSTRACTIONS INTERFACE)
target_link_libraries(Group_BaseLib_ABSTRACTIONS INTERFACE
  ${CONTEXT}_ABSTRACTIONS
)
target_compile_options(Group_BaseLib PUBLIC
  $<TARGET_PROPERTY:${CONTEXT},INTERFACE_COMPILE_OPTIONS>
)
target_link_libraries(Group_BaseLib PUBLIC
  Group_BaseLib_ABSTRACTIONS
)

# group Application/User/Core
add_library(Group_Application_User_Core OBJECT
  "${SOLUTION_ROOT}/../Core/Src/main.c"
  "${SOLUTION_ROOT}/../Core/Src/gpio.c"
  "${SOLUTION_ROOT}/../Core/Src/freertos.c"
  "${SOLUTION_ROOT}/../Core/Src/can.c"
  "${SOLUTION_ROOT}/../Core/Src/dma.c"
  "${SOLUTION_ROOT}/../Core/Src/spi.c"
  "${SOLUTION_ROOT}/../Core/Src/tim.c"
  "${SOLUTION_ROOT}/../Core/Src/usart.c"
  "${SOLUTION_ROOT}/../Core/Src/stm32f4xx_it.c"
  "${SOLUTION_ROOT}/../Core/Src/stm32f4xx_hal_msp.c"
  "${SOLUTION_ROOT}/../Core/Src/stm32f4xx_hal_timebase_tim.c"
)
target_include_directories(Group_Application_User_Core PUBLIC
  $<TARGET_PROPERTY:${CONTEXT},INTERFACE_INCLUDE_DIRECTORIES>
)
target_compile_definitions(Group_Application_User_Core PUBLIC
  $<TARGET_PROPERTY:${CONTEXT},INTERFACE_COMPILE_DEFINITIONS>
)
add_library(Group_Application_User_Core_ABSTRACTIONS INTERFACE)
cbuild_set_options_flags(CC "none" "on" "" "" CC_OPTIONS_FLAGS_Group_Application_User_Core)
target_compile_options(Group_Application_User_Core_ABSTRACTIONS INTERFACE
  $<$<COMPILE_LANGUAGE:C>:
    "SHELL:${CC_OPTIONS_FLAGS_Group_Application_User_Core}"
  >
)
target_compile_options(Group_Application_User_Core PUBLIC
  $<TARGET_PROPERTY:${CONTEXT},INTERFACE_COMPILE_OPTIONS>
)
target_link_libraries(Group_Application_User_Core PUBLIC
  Group_Application_User_Core_ABSTRACTIONS
)
set_source_files_properties("${SOLUTION_ROOT}/../Core/Src/main.c" PROPERTIES
  COMPILE_OPTIONS ""
)
set_source_files_properties("${SOLUTION_ROOT}/../Core/Src/gpio.c" PROPERTIES
  COMPILE_OPTIONS ""
)
set_source_files_properties("${SOLUTION_ROOT}/../Core/Src/freertos.c" PROPERTIES
  COMPILE_OPTIONS ""
)
set_source_files_properties("${SOLUTION_ROOT}/../Core/Src/can.c" PROPERTIES
  COMPILE_OPTIONS ""
)
set_source_files_properties("${SOLUTION_ROOT}/../Core/Src/dma.c" PROPERTIES
  COMPILE_OPTIONS ""
)
set_source_files_properties("${SOLUTION_ROOT}/../Core/Src/spi.c" PROPERTIES
  COMPILE_OPTIONS ""
)
set_source_files_properties("${SOLUTION_ROOT}/../Core/Src/tim.c" PROPERTIES
  COMPILE_OPTIONS ""
)
set_source_files_properties("${SOLUTION_ROOT}/../Core/Src/usart.c" PROPERTIES
  COMPILE_OPTIONS ""
)
set_source_files_properties("${SOLUTION_ROOT}/../Core/Src/stm32f4xx_it.c" PROPERTIES
  COMPILE_OPTIONS ""
)
set_source_files_properties("${SOLUTION_ROOT}/../Core/Src/stm32f4xx_hal_msp.c" PROPERTIES
  COMPILE_OPTIONS ""
)
set_source_files_properties("${SOLUTION_ROOT}/../Core/Src/stm32f4xx_hal_timebase_tim.c" PROPERTIES
  COMPILE_OPTIONS ""
)

# group Matrix
add_library(Group_Matrix OBJECT
  "${SOLUTION_ROOT}/../Matrix/matrix.c"
)
target_include_directories(Group_Matrix PUBLIC
  $<TARGET_PROPERTY:${CONTEXT},INTERFACE_INCLUDE_DIRECTORIES>
  "${SOLUTION_ROOT}/../Matrix"
)
target_compile_definitions(Group_Matrix PUBLIC
  $<TARGET_PROPERTY:${CONTEXT},INTERFACE_COMPILE_DEFINITIONS>
)
add_library(Group_Matrix_ABSTRACTIONS INTERFACE)
target_link_libraries(Group_Matrix_ABSTRACTIONS INTERFACE
  ${CONTEXT}_ABSTRACTIONS
)
target_compile_options(Group_Matrix PUBLIC
  $<TARGET_PROPERTY:${CONTEXT},INTERFACE_COMPILE_OPTIONS>
)
target_link_libraries(Group_Matrix PUBLIC
  Group_Matrix_ABSTRACTIONS
)

# group Application/User/USB_DEVICE/App
add_library(Group_Application_User_USB_DEVICE_App OBJECT
  "${SOLUTION_ROOT}/../USB_DEVICE/App/usb_device.c"
  "${SOLUTION_ROOT}/../USB_DEVICE/App/usbd_desc.c"
  "${SOLUTION_ROOT}/../USB_DEVICE/App/usbd_cdc_if.c"
)
target_include_directories(Group_Application_User_USB_DEVICE_App PUBLIC
  $<TARGET_PROPERTY:${CONTEXT},INTERFACE_INCLUDE_DIRECTORIES>
)
target_compile_definitions(Group_Application_User_USB_DEVICE_App PUBLIC
  $<TARGET_PROPERTY:${CONTEXT},INTERFACE_COMPILE_DEFINITIONS>
)
add_library(Group_Application_User_USB_DEVICE_App_ABSTRACTIONS INTERFACE)
cbuild_set_options_flags(CC "none" "on" "" "" CC_OPTIONS_FLAGS_Group_Application_User_USB_DEVICE_App)
target_compile_options(Group_Application_User_USB_DEVICE_App_ABSTRACTIONS INTERFACE
  $<$<COMPILE_LANGUAGE:C>:
    "SHELL:${CC_OPTIONS_FLAGS_Group_Application_User_USB_DEVICE_App}"
  >
)
target_compile_options(Group_Application_User_USB_DEVICE_App PUBLIC
  $<TARGET_PROPERTY:${CONTEXT},INTERFACE_COMPILE_OPTIONS>
)
target_link_libraries(Group_Application_User_USB_DEVICE_App PUBLIC
  Group_Application_User_USB_DEVICE_App_ABSTRACTIONS
)
set_source_files_properties("${SOLUTION_ROOT}/../USB_DEVICE/App/usb_device.c" PROPERTIES
  COMPILE_OPTIONS ""
)
set_source_files_properties("${SOLUTION_ROOT}/../USB_DEVICE/App/usbd_desc.c" PROPERTIES
  COMPILE_OPTIONS ""
)
set_source_files_properties("${SOLUTION_ROOT}/../USB_DEVICE/App/usbd_cdc_if.c" PROPERTIES
  COMPILE_OPTIONS ""
)

# group Application/User/USB_DEVICE/Target
add_library(Group_Application_User_USB_DEVICE_Target OBJECT
  "${SOLUTION_ROOT}/../USB_DEVICE/Target/usbd_conf.c"
)
target_include_directories(Group_Application_User_USB_DEVICE_Target PUBLIC
  $<TARGET_PROPERTY:${CONTEXT},INTERFACE_INCLUDE_DIRECTORIES>
)
target_compile_definitions(Group_Application_User_USB_DEVICE_Target PUBLIC
  $<TARGET_PROPERTY:${CONTEXT},INTERFACE_COMPILE_DEFINITIONS>
)
add_library(Group_Application_User_USB_DEVICE_Target_ABSTRACTIONS INTERFACE)
cbuild_set_options_flags(CC "none" "on" "" "" CC_OPTIONS_FLAGS_Group_Application_User_USB_DEVICE_Target)
target_compile_options(Group_Application_User_USB_DEVICE_Target_ABSTRACTIONS INTERFACE
  $<$<COMPILE_LANGUAGE:C>:
    "SHELL:${CC_OPTIONS_FLAGS_Group_Application_User_USB_DEVICE_Target}"
  >
)
target_compile_options(Group_Application_User_USB_DEVICE_Target PUBLIC
  $<TARGET_PROPERTY:${CONTEXT},INTERFACE_COMPILE_OPTIONS>
)
target_link_libraries(Group_Application_User_USB_DEVICE_Target PUBLIC
  Group_Application_User_USB_DEVICE_Target_ABSTRACTIONS
)
set_source_files_properties("${SOLUTION_ROOT}/../USB_DEVICE/Target/usbd_conf.c" PROPERTIES
  COMPILE_OPTIONS ""
)

# group Middlewares/USB_Device_Library
add_library(Group_Middlewares_USB_Device_Library OBJECT
  "${SOLUTION_ROOT}/../Middlewares/ST/STM32_USB_Device_Library/Core/Src/usbd_core.c"
  "${SOLUTION_ROOT}/../Middlewares/ST/STM32_USB_Device_Library/Core/Src/usbd_ctlreq.c"
  "${SOLUTION_ROOT}/../Middlewares/ST/STM32_USB_Device_Library/Core/Src/usbd_ioreq.c"
  "${SOLUTION_ROOT}/../Middlewares/ST/STM32_USB_Device_Library/Class/CDC/Src/usbd_cdc.c"
)
target_include_directories(Group_Middlewares_USB_Device_Library PUBLIC
  $<TARGET_PROPERTY:${CONTEXT},INTERFACE_INCLUDE_DIRECTORIES>
)
target_compile_definitions(Group_Middlewares_USB_Device_Library PUBLIC
  $<TARGET_PROPERTY:${CONTEXT},INTERFACE_COMPILE_DEFINITIONS>
)
add_library(Group_Middlewares_USB_Device_Library_ABSTRACTIONS INTERFACE)
cbuild_set_options_flags(CC "none" "on" "" "" CC_OPTIONS_FLAGS_Group_Middlewares_USB_Device_Library)
target_compile_options(Group_Middlewares_USB_Device_Library_ABSTRACTIONS INTERFACE
  $<$<COMPILE_LANGUAGE:C>:
    "SHELL:${CC_OPTIONS_FLAGS_Group_Middlewares_USB_Device_Library}"
  >
)
target_compile_options(Group_Middlewares_USB_Device_Library PUBLIC
  $<TARGET_PROPERTY:${CONTEXT},INTERFACE_COMPILE_OPTIONS>
)
target_link_libraries(Group_Middlewares_USB_Device_Library PUBLIC
  Group_Middlewares_USB_Device_Library_ABSTRACTIONS
)
set_source_files_properties("${SOLUTION_ROOT}/../Middlewares/ST/STM32_USB_Device_Library/Core/Src/usbd_core.c" PROPERTIES
  COMPILE_OPTIONS ""
)
set_source_files_properties("${SOLUTION_ROOT}/../Middlewares/ST/STM32_USB_Device_Library/Core/Src/usbd_ctlreq.c" PROPERTIES
  COMPILE_OPTIONS ""
)
set_source_files_properties("${SOLUTION_ROOT}/../Middlewares/ST/STM32_USB_Device_Library/Core/Src/usbd_ioreq.c" PROPERTIES
  COMPILE_OPTIONS ""
)
set_source_files_properties("${SOLUTION_ROOT}/../Middlewares/ST/STM32_USB_Device_Library/Class/CDC/Src/usbd_cdc.c" PROPERTIES
  COMPILE_OPTIONS ""
)

# group Middlewares/Library/DSP Library/DSP Library
add_library(Group_Middlewares_Library_DSP_Library_DSP_Library INTERFACE)
target_include_directories(Group_Middlewares_Library_DSP_Library_DSP_Library INTERFACE
  $<TARGET_PROPERTY:${CONTEXT},INTERFACE_INCLUDE_DIRECTORIES>
)
target_compile_definitions(Group_Middlewares_Library_DSP_Library_DSP_Library INTERFACE
  $<TARGET_PROPERTY:${CONTEXT},INTERFACE_COMPILE_DEFINITIONS>
)
add_library(Group_Middlewares_Library_DSP_Library_DSP_Library_ABSTRACTIONS INTERFACE)
target_link_libraries(Group_Middlewares_Library_DSP_Library_DSP_Library INTERFACE
  ${SOLUTION_ROOT}/../Middlewares/ST/ARM/DSP/Lib/arm_cortexM4lf_math.lib
)
