/****************************************************************************
 * arch/arm/src/stm32h7/stm32_ethphy.h
 *
 *   Copyright (C) 2015 Gregory Nutt. All rights reserved.
 *   Author: Gregory Nutt <gnutt@nuttx.org>
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 * 3. Neither the name NuttX nor the names of its contributors may be
 *    used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ****************************************************************************/

#ifndef __ARCH_ARM_SRC_STM32H7_STM32_ETHPHY_H
#define __ARCH_ARM_SRC_STM32H7_STM32_ETHPHY_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#if STM32H7_NETHERNET > 0
#ifndef __ASSEMBLY__

#undef EXTERN
#if defined(__cplusplus)
#define EXTERN extern "C"
extern "C"
{
#else
#define EXTERN extern
#endif

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/****************************************************************************
 * Despite the fact that basic layout of MII PHY registers is standartized,
 * we still depend on registers that vary from chip to chip manufacturer.
 *
 * "Big" operating systems like Linux have dedicated drivers for different
 * chip families (plus a basic "generic" PHY driver). This is a bit of
 * overkill here, since NuttX mainly targets fixed hardware, so we don't
 * need dynamic switching between PHY drivers etc.
 *
 * So, for now we'll carry on with a simpler approach: in Kconfig user will
 * be presented with a list of PHY families, and here depending on the family
 * we'll define various macros which will help Ethernet driver code to
 * treat the PHY properly.
 ****************************************************************************/

#if defined CONFIG_ETH0_PHY_NONE

/* Define nothing, go the generic way */

#elif defined CONFIG_ETH0_PHY_KS8721 || defined CONFIG_ETH0_PHY_LAN8710 || \
      defined CONFIG_ETH0_PHY_LAN8742A

/* 1f.4:2 Operation Mode Indication
 *      [000] = Still in auto-negotiation
 *      [001] = 10BASE-T half-duplex
 *      [010] = 100BASE-TX half-duplex
 *      [011] = Reserved
 *      [101] = 10BASE-T full-duplex
 *      [110] = 100BASE-TX full-duplex
 *      [111] = PHY/MII isolate
 * 1f.7 Auto-Negotiation Complete
 *      1 = Auto-negotiation complete
 *      0 = Not complete
 */
#define STM32_PHYSR_N		31
#define STM32_PHYSR_MODE_MASK	0x001c
#define STM32_PHYSR_MODE_100FD	0x0018
#define STM32_PHYSR_MODE_100HD	0x0008
#define STM32_PHYSR_MODE_10FD	0x0014
#define STM32_PHYSR_MODE_10HD	0x0004
#define STM32_PHYSR_ANEG_MASK	0x0080
#define STM32_PHYSR_ANEG_OK	0x0080

#elif defined CONFIG_ETH0_PHY_TLK10x || defined ETH0_PHY_DP83848

/* 0x10 PHY Status Register
 * bit 1: Speed Status
 *      1 = 10 Mb/s mode
 *      0 = 100 Mb/s mode
 * bit 2: Duplex Status
 *      1 = Full duplex mode
 *      0 = Half duplex mode
 * bit 4: Auto-Neg Status
 *      1 = Auto-Negotiation complete
 *      0 = Auto-Negotiation not complete
 */
#define STM32_PHYSR_N		0x10
#define STM32_PHYSR_SPEED_MASK	0x0002
#define STM32_PHYSR_SPEED_100	0x0000
#define STM32_PHYSR_DUPLEX_MASK	0x0004
#define STM32_PHYSR_DUPLEX_FULL	0x0004
#define STM32_PHYSR_ANEG_MASK	0x0010
#define STM32_PHYSR_ANEG_OK	0x0010

#if defined CONFIG_ETH0_PHY_TLK10x && !defined STM32_PHY_INIT

/* Set up PHY LED to blink on link activity.
 * By default it just indicates "Good/No Link".
 */
#define STM32_PHY_INIT \
  do \
    { \
      stm32_phywrite(CONFIG_STM32H7_PHYADDR, 0x18, 0x0200, 0x0610); \
      stm32_phywrite(CONFIG_STM32H7_PHYADDR, 0x19, 0x0000, 0x0020); \
    } \
  while (0)

#endif

#elif defined ETH0_PHY_DM9161

/* Configuration and Status Register (DSCSR) - 17
 * 17.15: 100M Full Duplex Operation Mode
 * 17.14: 100M Half Duplex Operation Mode
 * 17.13: 10M Full Duplex Operation Mode
 * 17.12: 10M Half Duplex Operation Mode
 * 17.3: Auto-negotiation completed successfully
 */
#define STM32_PHYSR_N		17
#define STM32_PHYSR_MODE_MASK	0xf000
#define STM32_PHYSR_MODE_100FD	0x8000
#define STM32_PHYSR_MODE_100HD	0x4000
#define STM32_PHYSR_MODE_10FD	0x2000
#define STM32_PHYSR_MODE_10HD	0x1000
#define STM32_PHYSR_ANEG_MASK	0x0008
#define STM32_PHYSR_ANEG_OK	0x0008

#else
#error "Unsupported PHY chip selected!"
#endif

#undef EXTERN
#if defined(__cplusplus)
}
#endif

#endif /* __ASSEMBLY__ */
#endif /* STM32H7_NETHERNET > 0 */
#endif /* __ARCH_ARM_SRC_STM32H7_STM32_ETHPHY_H */
