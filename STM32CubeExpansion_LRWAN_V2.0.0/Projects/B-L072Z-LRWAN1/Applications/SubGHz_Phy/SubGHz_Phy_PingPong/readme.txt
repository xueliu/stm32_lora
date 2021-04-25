/**
  @page SubGHz_Phy_PingPong Readme file
 
  @verbatim
  ******************** (C) COPYRIGHT 2020 STMicroelectronics *******************
  * @file    Applications/SubGHz_Phy/SubGHz_Phy_PingPong/readme.txt 
  * @author  MCD Application Team
  * @brief   This application is a simple demo application software between two 
  *          LoRa Objects containing a STM32LXxx-Nucleo board and LoRA Radio expansion board.
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2020 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under Ultimate Liberty license
  * SLA0044, the "License"; You may not use this file except in compliance with
  * the License. You may obtain a copy of the License at:
  *                             www.st.com/SLA0044
  *
  ******************************************************************************
  @endverbatim

@par Description

This directory contains a set of source files that implements a Ping-Pong application 
between two PingPong Devices.

The PingPong Device can be 
   - either a STM32LXxx-Nucleo board and Lora Radio expansion board, optionnally a sensor board. 
   - or a B-L072Z-LRWAN1 board
It is a simple RX/TX RF link between the two PingPong Devices, one will be called Ping the other will be called Pong. 
By default, each PingPong Device starts as a master and will transmit a "Ping" message, and then wait for an answer. 
The first PingPong Device receiving a "Ping" message will become a slave and answers the master with a "Pong" message.
The Ping Pong is then started.
It means that both boards have their RX windows synchronised (and TX windows too). The RX window of the first board 
should NOT be perfectly synchronised with the second board, there should be one TX window in front of one RX window. 
You only have to reset one of the two boards, to get rid of this issue.

  ******************************************************************************

@par Keywords

Applications, SubGHz_Phy, PingPong

@par Directory contents 


  - SubGHz_Phy_PingPong/Core/Inc/dma.h                                          This file contains all the function prototypes for
                                                                                the dma.c file
  - SubGHz_Phy_PingPong/Core/Inc/main.h                                         Header for main.c file.
                                                                                This file contains the common defines of the application.
  - SubGHz_Phy_PingPong/Core/Inc/platform.h                                     Header for General HW instances configuration
  - SubGHz_Phy_PingPong/Core/Inc/rtc.h                                          This file contains all the function prototypes for
                                                                                the rtc.c file
  - SubGHz_Phy_PingPong/Core/Inc/rtc_if.h                                       configuration of the rtc_if.c instances
  - SubGHz_Phy_PingPong/Core/Inc/stm32l0xx_hal_conf.h                           HAL configuration file.
  - SubGHz_Phy_PingPong/Core/Inc/stm32l0xx_it.h                                 This file contains the headers of the interrupt handlers.
  - SubGHz_Phy_PingPong/Core/Inc/stm32_lpm_if.h                                 Header for Low Power Manager interface configuration
  - SubGHz_Phy_PingPong/Core/Inc/sys_app.h                                      Function prototypes for sys_app.c file
  - SubGHz_Phy_PingPong/Core/Inc/sys_conf.h                                     Applicative configuration, e.g. : debug, trace, low power, sensors
  - SubGHz_Phy_PingPong/Core/Inc/sys_debug.h                                    Configuration of the debug.c instances
  - SubGHz_Phy_PingPong/Core/Inc/usart.h                                        This file contains all the function prototypes for
                                                                                the usart.c file
  - SubGHz_Phy_PingPong/Core/Inc/usart_if.h                                     Header for USART interface configuration
  - SubGHz_Phy_PingPong/Core/Inc/utilities_conf.h                               Header for configuration file to utilities
  - SubGHz_Phy_PingPong/Core/Inc/utilities_def.h                                Definitions for modules requiring utilities
  - SubGHz_Phy_PingPong/SubGHz_Phy/App/app_subghz_phy.h                         Header of application of the SubGHz_Phy Middleware
  - SubGHz_Phy_PingPong/SubGHz_Phy/App/app_version.h                            Definition the version of the application
  - SubGHz_Phy_PingPong/SubGHz_Phy/App/subghz_phy_app.h                         Header of application of the SubGHz_Phy Middleware
  - SubGHz_Phy_PingPong/SubGHz_Phy/Target/b_l072z_lrwan1_bus.h                  header file for the BSP BUS IO driver
  - SubGHz_Phy_PingPong/SubGHz_Phy/Target/b_l072z_lrwan1_errno.h                Error Code
  - SubGHz_Phy_PingPong/SubGHz_Phy/Target/cmwx1zzabz_0xx_conf.h                 This file provides code for the configuration
                                                                                of the shield instances (pin mapping).
  - SubGHz_Phy_PingPong/SubGHz_Phy/Target/mw_log_conf.h                         Configure (enable/disable) traces
  - SubGHz_Phy_PingPong/SubGHz_Phy/Target/radio_board_if.h                      Header for Radio interface configuration
  - SubGHz_Phy_PingPong/SubGHz_Phy/Target/radio_conf.h                          Header of Radio configuration
  - SubGHz_Phy_PingPong/SubGHz_Phy/Target/timer.h                               Wrapper to timer server

  - SubGHz_Phy_PingPong/Core/Src/dma.c                                          This file provides code for the configuration
                                                                                of all the requested memory to memory DMA transfers.
  - SubGHz_Phy_PingPong/Core/Src/main.c                                         Main program body
  - SubGHz_Phy_PingPong/Core/Src/rtc.c                                          This file provides code for the configuration
                                                                                of the RTC instances.
  - SubGHz_Phy_PingPong/Core/Src/rtc_if.c                                       Configure RTC Alarm, Tick and Calendar manager
  - SubGHz_Phy_PingPong/Core/Src/stm32l0xx_hal_msp.c                            This file provides code for the MSP Initialization
                                                                                and de-Initialization codes.
  - SubGHz_Phy_PingPong/Core/Src/stm32l0xx_it.c                                 Interrupt Service Routines.
  - SubGHz_Phy_PingPong/Core/Src/stm32_lpm_if.c                                 Low layer function to enter/exit low power modes (stop, sleep)
  - SubGHz_Phy_PingPong/Core/Src/system_stm32l0xx.c                             CMSIS Cortex-M0+ Device Peripheral Access Layer System Source File.
  - SubGHz_Phy_PingPong/Core/Src/sys_app.c                                      Initializes HW and SW system entities (not related to the radio)
  - SubGHz_Phy_PingPong/Core/Src/sys_debug.c                                    Enables 4 debug pins for internal signals RealTime debugging
  - SubGHz_Phy_PingPong/Core/Src/usart.c                                        This file provides code for the configuration
                                                                                of the USART instances.
  - SubGHz_Phy_PingPong/Core/Src/usart_if.c                                     Configuration of UART MX driver interface for hyperterminal communication
  - SubGHz_Phy_PingPong/SubGHz_Phy/App/app_subghz_phy.c                         Application of the SubGHz_Phy Middleware
  - SubGHz_Phy_PingPong/SubGHz_Phy/App/subghz_phy_app.c                         Application of the SubGHz_Phy Middleware
  - SubGHz_Phy_PingPong/SubGHz_Phy/Target/b_l072z_lrwan1_bus.c                  source file for the BSP BUS IO driver


@par Hardware and Software environment 

  - This example runs on STM32L053R8, STM32L073RZ, STM32L152RE and STM32L476RG devices.
    
  - This application has been tested with STMicroelectronics:
    NUCLEO-L053R8 RevC
    NUCLEO-L073RZ RevC
    NUCLEO-L152RE RevC
    NUCLEO-L476RG RevC 
    B-L072Z-LRWAN1 RevC
    boards and can be easily tailored to any other supported device 
    and development board.

  - STM32LXxx-Nucleo Set-up    
    - Connect the Nucleo board to your PC with a USB cable type A to mini-B 
      to ST-LINK connector (CN1 / CN7 on B-L072Z-LRWAN1).
    - Please ensure that the ST-LINK connector CN2 (CN8 on B-L072Z-LRWAN1) jumpers are fitted.

  - Configure the software via the configuration files:
    - sys_conf.h, radio_conf.h, mw_log_conf.h, radio_board_if.h, main.h, etc

  -Set Up:

             --------------------------  V    V  --------------------------
             |   PingPong Device      |  |    |  |   PingPong Device      |
             |                        |  |    |  |                        |
   ComPort<--|                        |--|    |--|                        |-->ComPort
             |                        |          |                        |
             --------------------------          --------------------------

@par How to use it ? 
In order to make the program work, you must do the following :
  - Open your preferred toolchain 
  - Rebuild all files and load your image into target memory
  - Do the same for second board
  - Reset the two boards
  - Run the example on two boards
  - Open two Terminals, each connected to their respective PingPong Device
  - UART Config = 115200, 8b, 1 stopbit, no parity, no flow control

 * <h3><center>&copy; COPYRIGHT STMicroelectronics</center></h3>
 */
