#if 0
typedef enum IRQn
{
	PendSV_IRQn              = -2, /**< 14 Cortex-M0+ Pend SV Interrupt           */
	SysTick_IRQn             = -1, /**< 15 Cortex-M0+ System Tick Interrupt       */
} IRQn_Type;

#define __CM0PLUS_REV          1         /*!< Core revision r0p1 */             
#define __MPU_PRESENT          0         /*!< MPU present or not */             
#define __NVIC_PRIO_BITS       2         /*!< Number of bits used for Priority Levels */
#define __Vendor_SysTickConfig 0         /*!< Set to 1 if different SysTick Config is used */

#define __VTOR_PRESENT            CONFIG_CPU_CORTEX_M_HAS_VTOR

#include <core_cm0plus.h>
#endif
