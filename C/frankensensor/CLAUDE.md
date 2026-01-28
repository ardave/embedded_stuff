# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## Build & Flash Commands

```bash
make                # Build project (outputs to build/)
make clean          # Remove build artifacts

# Flash to Nucleo-144 board via ST-Link
st-flash write build/frankensensor.bin 0x8000000

# Serial monitor (ST-Link VCP on USART3 @ 115200 baud)
screen /dev/tty.usbmodem* 115200
```

## Target Hardware

- **MCU**: STM32F439ZIT6 (Cortex-M4F, 168 MHz, 2MB Flash, 256KB RAM)
- **Board**: NUCLEO-144 with onboard ST-Link/V2-1
- **Toolchain**: arm-none-eabi-gcc at `/Applications/ArmGNUToolchain/15.2.rel1/arm-none-eabi/bin/`

## Architecture

### FreeRTOS Task Model

The project uses FreeRTOS (v10.3.1) with CMSIS-RTOS V2 wrapper. Tasks are created in `Core/Src/freertos.c`:

- **defaultTask**: Main application loop
- **buttonTask**: Waits on `BUTTON_PRESSED_FLAG` task notification, toggled by EXTI ISR

ISR-to-task pattern: Hardware interrupts set flags via `osThreadFlagsSet()`, tasks process via `osThreadFlagsWait()`. Button debouncing uses 50ms timestamp check in the EXTI callback.

### Peripheral Configuration

| Peripheral | Pins | Purpose |
|-----------|------|---------|
| USART3 | PD8/PD9 | Debug output via ST-Link VCP |
| I2C1 | PB6/PB7 | I2C interface |
| User Button | PC13 | EXTI rising edge, priority 6 |
| LEDs | PB0/PB7/PB14 | LD1/LD2/LD3 status indicators |

UART handle `huart3` is declared in `usart.c`, exported via `usart.h`.

### Code Organization

STM32CubeMX-generated project with `/* USER CODE BEGIN/END */` sections for custom code preservation. The `.ioc` file can regenerate Drivers/ and Middlewares/ folders.

Key files:
- `Core/Src/main.c` - Peripheral init, system clock config, EXTI setup
- `Core/Src/freertos.c` - Task definitions and FreeRTOS initialization
- `Core/Inc/FreeRTOSConfig.h` - RTOS tuning (15KB heap, 1ms tick)
- `STM32F439XX_FLASH.ld` - Memory layout

### Interrupt Priorities

FreeRTOS-safe API calls require priority >= 6 (configLIBRARY_MAX_SYSCALL_INTERRUPT_PRIORITY = 5). The user button EXTI is set to priority 6 in main.c.
