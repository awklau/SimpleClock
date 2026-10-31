# Pinned vendor sources

The STM32C071 firmware profiles use the following reviewed upstream
sources. The copied files are kept under their upstream copyright and license
notices; the complete license text for each source is retained beside the
files.

| Source | Pinned revision | Upstream | License text |
| --- | --- | --- | --- |
| STMicroelectronics `cmsis-device-c0` | v1.4.1, `bcc8d94fecb767d1afb53d8c12a8f87ebeb503a2` | <https://github.com/STMicroelectronics/cmsis-device-c0> | [vendor/st/cmsis-device-c0/LICENSE.md](st/cmsis-device-c0/LICENSE.md) |
| Arm CMSIS-5 Core | 5.9.0, `2b7495b8535bdcb306dac29b9ded4cfb679d7e5c` | <https://github.com/ARM-software/CMSIS_5> | [vendor/st/cmsis/LICENSE.txt](st/cmsis/LICENSE.txt) |
| STMicroelectronics `stm32c0xx-hal-driver` | v1.4.1, `038bed9f00e0c07328627470328fe8abc184d071` | <https://github.com/STMicroelectronics/stm32c0xx-hal-driver> | [vendor/st/hal/LICENSE.md](st/hal/LICENSE.md) |
| STMicroelectronics `stm32-mw-usb-device` | `947131e000d1ecebc260f22010fa75486c50b423` (submodule pinned by `stm32c0-classic-coremw-apps` v1.1.0) | <https://github.com/STMicroelectronics/stm32-mw-usb-device> | [vendor/st/usb-device/LICENSE.md](st/usb-device/LICENSE.md) |
| STMicroelectronics STM32C0 classic CoreMW CDC application adapters | v1.1.0, `334655ed5eb719db18fa5990a5044332eeca59dc` | <https://github.com/STMicroelectronics/stm32c0-classic-coremw-apps> | [vendor/st/stm32c0-classic-coremw-apps/LICENSE.md](st/stm32c0-classic-coremw-apps/LICENSE.md) |

Only the files needed by the STM32C071 diagnostics are copied. The HAL PCD
source has one documented local workaround for STM32C071 erratum ES0618
2.11.1; see
[ES0618_PMA_READ_DELAY.md](st/hal/ES0618_PMA_READ_DELAY.md). Other HAL files
and the USB middleware files are intended to remain byte-identical to their
pinned upstream revisions. The USB application adapters retain SLA0044 terms
and execute only on the fitted STMicroelectronics MCU.
