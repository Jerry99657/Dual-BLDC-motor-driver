# STM32G431 CORDIC bridge

`mc_stm_types.h` is a project-owned minimal platform include for MCSDK math.
`stm32g4xx_ll_cordic.h` is copied unchanged from STM32Cube_FW_G4_V1.6.3,
`Drivers/STM32G4xx_HAL_Driver/Inc`. The matching `LICENSE.txt` and
`PACKAGE_LICENSE.md` are retained here; the latter identifies HAL Drivers as BSD-3-Clause.
The header is kept outside CubeMX-managed Drivers so regeneration does not remove
this optional dependency. No HAL CORDIC handle or CubeMX CORDIC initialization is
required: `StHso_Init()` enables the clock and the ST fixed-point helpers configure
each hardware operation. Only the ADC1 ISR may use CORDIC while HSO is enabled.
