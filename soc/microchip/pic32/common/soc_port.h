/*
 * Copyright (c) 2025 Microchip Technology Inc.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef MICROCHIP_PIC32_SOC_PORT_H_
#define MICROCHIP_PIC32_SOC_PORT_H_

#include <soc.h>

#define PINCFG_DIO_CFG_PU_ENC_Pos            _UINT32_(0)                                          /* (PINCFG_DIO0_CFG) 0b1: enable=pull up enabled; 0b0: disable=pull up disabled Position */
#define PINCFG_DIO_CFG_PU_ENC_Msk            (_UINT32_(0x1) << PINCFG_DIO_CFG_PU_ENC_Pos)        /* (PINCFG_DIO0_CFG) 0b1: enable=pull up enabled; 0b0: disable=pull up disabled Mask */
#define PINCFG_DIO_CFG_PU_ENC(value)         (PINCFG_DIO_CFG_PU_ENC_Msk & (_UINT32_(value) << PINCFG_DIO_CFG_PU_ENC_Pos)) /* Assignment of value for PU_ENC in the PINCFG_DIO0_CFG register */
#define   PINCFG_DIO_CFG_PU_ENC_enable_Val   _UINT32_(0x1)                                        /* (PINCFG_DIO0_CFG) pull up enabled  */
#define   PINCFG_DIO_CFG_PU_ENC_disable_Val  _UINT32_(0x0)                                        /* (PINCFG_DIO0_CFG) pull up disabled  */
#define PINCFG_DIO_CFG_PU_ENC_enable         (PINCFG_DIO_CFG_PU_ENC_enable_Val << PINCFG_DIO_CFG_PU_ENC_Pos) /* (PINCFG_DIO0_CFG) pull up enabled Position  */
#define PINCFG_DIO_CFG_PU_ENC_disable        (PINCFG_DIO_CFG_PU_ENC_disable_Val << PINCFG_DIO_CFG_PU_ENC_Pos) /* (PINCFG_DIO0_CFG) pull up disabled Position  */
#define PINCFG_DIO_CFG_PD_ENC_Pos            _UINT32_(1)                                          /* (PINCFG_DIO0_CFG) 0b1: enable=pull down enabled; 0b0: disable=pull down disabled Position */
#define PINCFG_DIO_CFG_PD_ENC_Msk            (_UINT32_(0x1) << PINCFG_DIO_CFG_PD_ENC_Pos)        /* (PINCFG_DIO0_CFG) 0b1: enable=pull down enabled; 0b0: disable=pull down disabled Mask */
#define PINCFG_DIO_CFG_PD_ENC(value)         (PINCFG_DIO_CFG_PD_ENC_Msk & (_UINT32_(value) << PINCFG_DIO_CFG_PD_ENC_Pos)) /* Assignment of value for PD_ENC in the PINCFG_DIO0_CFG register */
#define   PINCFG_DIO_CFG_PD_ENC_enable_Val   _UINT32_(0x1)                                        /* (PINCFG_DIO0_CFG) pull down enabled  */
#define   PINCFG_DIO_CFG_PD_ENC_disable_Val  _UINT32_(0x0)                                        /* (PINCFG_DIO0_CFG) pull down disabled  */
#define PINCFG_DIO_CFG_PD_ENC_enable         (PINCFG_DIO_CFG_PD_ENC_enable_Val << PINCFG_DIO_CFG_PD_ENC_Pos) /* (PINCFG_DIO0_CFG) pull down enabled Position  */
#define PINCFG_DIO_CFG_PD_ENC_disable        (PINCFG_DIO_CFG_PD_ENC_disable_Val << PINCFG_DIO_CFG_PD_ENC_Pos) /* (PINCFG_DIO0_CFG) pull down disabled Position  */
#define PINCFG_DIO_CFG_DIN_EN_Pos            _UINT32_(2)                                          /* (PINCFG_DIO0_CFG) 0b1: enable=input enabled; 0b0: disable=input disabled Position */
#define PINCFG_DIO_CFG_DIN_EN_Msk            (_UINT32_(0x1) << PINCFG_DIO_CFG_DIN_EN_Pos)        /* (PINCFG_DIO0_CFG) 0b1: enable=input enabled; 0b0: disable=input disabled Mask */
#define PINCFG_DIO_CFG_DIN_EN(value)         (PINCFG_DIO_CFG_DIN_EN_Msk & (_UINT32_(value) << PINCFG_DIO_CFG_DIN_EN_Pos)) /* Assignment of value for DIN_EN in the PINCFG_DIO0_CFG register */
#define   PINCFG_DIO_CFG_DIN_EN_enable_Val   _UINT32_(0x1)                                        /* (PINCFG_DIO0_CFG) input enabled  */
#define   PINCFG_DIO_CFG_DIN_EN_disable_Val  _UINT32_(0x0)                                        /* (PINCFG_DIO0_CFG) input disabled  */
#define PINCFG_DIO_CFG_DIN_EN_enable         (PINCFG_DIO_CFG_DIN_EN_enable_Val << PINCFG_DIO_CFG_DIN_EN_Pos) /* (PINCFG_DIO0_CFG) input enabled Position  */
#define PINCFG_DIO_CFG_DIN_EN_disable        (PINCFG_DIO_CFG_DIN_EN_disable_Val << PINCFG_DIO_CFG_DIN_EN_Pos) /* (PINCFG_DIO0_CFG) input disabled Position  */
#define PINCFG_DIO_CFG_OD_Pos                _UINT32_(3)                                          /* (PINCFG_DIO0_CFG) 0b1: enable=open drain enabled; 0b0: disable=open drain disabled Position */
#define PINCFG_DIO_CFG_OD_Msk                (_UINT32_(0x1) << PINCFG_DIO_CFG_OD_Pos)            /* (PINCFG_DIO0_CFG) 0b1: enable=open drain enabled; 0b0: disable=open drain disabled Mask */
#define PINCFG_DIO_CFG_OD(value)             (PINCFG_DIO_CFG_OD_Msk & (_UINT32_(value) << PINCFG_DIO_CFG_OD_Pos)) /* Assignment of value for OD in the PINCFG_DIO0_CFG register */
#define   PINCFG_DIO_CFG_OD_enable_Val       _UINT32_(0x1)                                        /* (PINCFG_DIO0_CFG) open drain enabled  */
#define   PINCFG_DIO_CFG_OD_disable_Val      _UINT32_(0x0)                                        /* (PINCFG_DIO0_CFG) open drain disabled  */
#define PINCFG_DIO_CFG_OD_enable             (PINCFG_DIO_CFG_OD_enable_Val << PINCFG_DIO_CFG_OD_Pos) /* (PINCFG_DIO0_CFG) open drain enabled Position  */
#define PINCFG_DIO_CFG_OD_disable            (PINCFG_DIO_CFG_OD_disable_Val << PINCFG_DIO_CFG_OD_Pos) /* (PINCFG_DIO0_CFG) open drain disabled Position  */
#define PINCFG_DIO_CFG_DRVSTR_Pos            _UINT32_(4)                                          /* (PINCFG_DIO0_CFG) 0b11:4mA=4mA drive; 0b10:3mA=3mA drive; 0b01:2mA=2mA drive; 0b00:1mA=1mA drive Position */
#define PINCFG_DIO_CFG_DRVSTR_Msk            (_UINT32_(0x3) << PINCFG_DIO_CFG_DRVSTR_Pos)        /* (PINCFG_DIO0_CFG) 0b11:4mA=4mA drive; 0b10:3mA=3mA drive; 0b01:2mA=2mA drive; 0b00:1mA=1mA drive Mask */
#define PINCFG_DIO_CFG_DRVSTR(value)         (PINCFG_DIO_CFG_DRVSTR_Msk & (_UINT32_(value) << PINCFG_DIO_CFG_DRVSTR_Pos)) /* Assignment of value for DRVSTR in the PINCFG_DIO0_CFG register */
#define   PINCFG_DIO_CFG_DRVSTR_4mA_Val      _UINT32_(0x3)                                        /* (PINCFG_DIO0_CFG) 4mA drive  */
#define   PINCFG_DIO_CFG_DRVSTR_3mA_Val      _UINT32_(0x2)                                        /* (PINCFG_DIO0_CFG) 3mA drive  */
#define   PINCFG_DIO_CFG_DRVSTR_2mA_Val      _UINT32_(0x1)                                        /* (PINCFG_DIO0_CFG) 2mA drive  */
#define   PINCFG_DIO_CFG_DRVSTR_1mA_Val      _UINT32_(0x0)                                        /* (PINCFG_DIO0_CFG) 1mA drive  */
#define PINCFG_DIO_CFG_DRVSTR_4mA            (PINCFG_DIO_CFG_DRVSTR_4mA_Val << PINCFG_DIO_CFG_DRVSTR_Pos) /* (PINCFG_DIO0_CFG) 4mA drive Position  */
#define PINCFG_DIO_CFG_DRVSTR_3mA            (PINCFG_DIO_CFG_DRVSTR_3mA_Val << PINCFG_DIO_CFG_DRVSTR_Pos) /* (PINCFG_DIO0_CFG) 3mA drive Position  */
#define PINCFG_DIO_CFG_DRVSTR_2mA            (PINCFG_DIO_CFG_DRVSTR_2mA_Val << PINCFG_DIO_CFG_DRVSTR_Pos) /* (PINCFG_DIO0_CFG) 2mA drive Position  */
#define PINCFG_DIO_CFG_DRVSTR_1mA            (PINCFG_DIO_CFG_DRVSTR_1mA_Val << PINCFG_DIO_CFG_DRVSTR_Pos) /* (PINCFG_DIO0_CFG) 1mA drive Position  */
#define PINCFG_DIO_CFG_SEL_Pos               _UINT32_(8)                                          /* (PINCFG_DIO0_CFG) 0b1000:Function=(PHY); 0b0111:Function=(CLKOUT); 0b0110:Function=(I2S); 0b0101:Function=(PDM); 0b0100:Function=(TS); 0b0011:Function=(EG); 0b0010:Function=(OASPI); 0b0001:Function=(SERCOM) : supported; 0b0000:GPIO=(default) : supported Position */
#define PINCFG_DIO_CFG_SEL_Msk               (_UINT32_(0xF) << PINCFG_DIO_CFG_SEL_Pos)           /* (PINCFG_DIO0_CFG) 0b1000:Function=(PHY); 0b0111:Function=(CLKOUT); 0b0110:Function=(I2S); 0b0101:Function=(PDM); 0b0100:Function=(TS); 0b0011:Function=(EG); 0b0010:Function=(OASPI); 0b0001:Function=(SERCOM) : supported; 0b0000:GPIO=(default) : supported Mask */
#define PINCFG_DIO_CFG_SEL(value)            (PINCFG_DIO_CFG_SEL_Msk & (_UINT32_(value) << PINCFG_DIO_CFG_SEL_Pos)) /* Assignment of value for SEL in the PINCFG_DIO0_CFG register */
#define   PINCFG_DIO_CFG_SEL_SERCOM_Val      _UINT32_(0x1)                                        /* (PINCFG_DIO0_CFG) SERCOM0_0  */
#define   PINCFG_DIO_CFG_SEL_GPIO_Val        _UINT32_(0x0)                                        /* (PINCFG_DIO0_CFG) default  */
#define PINCFG_DIO_CFG_SEL_SERCOM            (PINCFG_DIO_CFG_SEL_SERCOM_Val << PINCFG_DIO_CFG_SEL_Pos) /* (PINCFG_DIO0_CFG) SERCOM0_0 Position  */
#define PINCFG_DIO_CFG_SEL_GPIO              (PINCFG_DIO_CFG_SEL_GPIO_Val << PINCFG_DIO_CFG_SEL_Pos) /* (PINCFG_DIO0_CFG) default Position  */
#define PINCFG_DIO_CFG_ALT_SEL_Pos           _UINT32_(12)                                         /* (PINCFG_DIO0_CFG)  Position */
#define PINCFG_DIO_CFG_ALT_SEL_Msk           (_UINT32_(0x3) << PINCFG_DIO_CFG_ALT_SEL_Pos)       /* (PINCFG_DIO0_CFG)  Mask */
#define PINCFG_DIO_CFG_ALT_SEL(value)        (PINCFG_DIO_CFG_ALT_SEL_Msk & (_UINT32_(value) << PINCFG_DIO_CFG_ALT_SEL_Pos)) /* Assignment of value for ALT_SEL in the PINCFG_DIO0_CFG register */
#define PINCFG_DIO_CFG_Msk                   _UINT32_(0x00003F3F)

struct soc_port_pin {
	pincfg_registers_t *regs;   /** pointer to registers of the I/O Pin Controller */
	uint32_t pinum;    /** pin number */
	uint32_t flags;    /** pin flags/attributes */
};

/**
 * @brief Configure PORT pin.
 *
 * Configure one pin belonging to some PORT.
 * Example scenarios:
 * - configure pin(s) as input.
 * - connect pin(s) to a peripheral B and enable pull-up.
 *
 * @param pin  pin's configuration data such as pin mask, pin attributes, etc.
 */
void soc_port_configure(const struct soc_port_pin *pin);

/**
 * @brief Configure a list of PORT pin(s).
 *
 * Configure an arbitrary amount of pins in an arbitrary way.  Each
 * configuration entry is a single item in an array passed as an argument to
 * the function.
 *
 * @param pins an array where each item contains pin's configuration data.
 * @param size size of the pin list.
 */
void soc_port_list_configure(const struct soc_port_pin pins[],
			     unsigned int size);

#endif /* MICROCHIP_PIC32_SOC_PORT_H_ */
