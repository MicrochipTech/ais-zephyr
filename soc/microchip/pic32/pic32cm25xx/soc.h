#ifndef _SOC_MICROCHIP_PIC32_PIC32CM2564_SOC_H_
#define _SOC_MICROCHIP_PIC32_PIC32CM2564_SOC_H_

#ifndef _ASMLANGUAGE

#define DONT_USE_CMSIS_INIT

#include <zephyr/types.h>

#if defined(CONFIG_SOC_PIC32CM2564)
#include <pic32cm2564.h>
#include <lan8660.h>
#else
#error Library does not support the specified device.
#endif

#endif /* _ASMLANGUAGE */

#include "microchip_pic32_dt.h"

/** Processor Clock (HCLK) Frequency */
#define SOC_MICROCHIP_PIC32_HCLK_FREQ_HZ MICROCHIP_PIC32_DT_CPU_CLK_FREQ_HZ

#endif
