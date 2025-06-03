#ifndef _MICROCHIP_PIC32_DT_H_                                                       
#define _MICROCHIP_PIC32_DT_H_

/* Common macro for use to set HCLK_FREQ_HZ */                                  
#define MICROCHIP_PIC32_DT_CPU_CLK_FREQ_HZ \
	DT_PROP(DT_PATH(cpus, cpu_0), clock_frequency)

#endif
