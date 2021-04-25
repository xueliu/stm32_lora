/**
  @page LoRaWAN_End_Node Readme file
 
  @verbatim
  ******************** (C) COPYRIGHT 2020 STMicroelectronics *******************
  * @file    Applications/LoRaWAN/LoRaWAN_End_Node/readme.txt 
  * @author  MCD Application Team
  * @brief   This application is a simple demo application software of a LoRa
  *          modem connecting to Network server. Data sent can be checked on 
  *          Network server for eg Loriot. Traces are displayed over UART
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

This directory contains a set of source files that implements a simple demo of an end 
device also known as a LoRa Object connecting to a Lora Network. The LoRa Object can be 
   - either a NUCLEO-LXxxxx board and LoRa Radio expansion board, optionnally a sensor board. 
   - or a B-L072Z-LRWAN1 board
By setting the LoRa Ids in se-identity.h file according to the LoRa Network requirements, 
the end device will send periodically the sensor data to the LoRa network.
  ******************************************************************************

@par Keywords

Applications, SubGHz_Phy, LoRaWAN, End_Node

@par Directory contents 


  - LoRaWAN_End_Node/Core/Inc/adc.h                                             This file contains all the function prototypes for
                                                                                the adc.c file
  - LoRaWAN_End_Node/Core/Inc/adc_if.h                                          Header for ADC interface configuration
  - LoRaWAN_End_Node/Core/Inc/dma.h                                             This file contains all the function prototypes for
                                                                                the dma.c file
  - LoRaWAN_End_Node/Core/Inc/main.h                                            Header for main.c file.
                                                                                This file contains the common defines of the application.
  - LoRaWAN_End_Node/Core/Inc/platform.h                                        Header for General HW instances configuration
  - LoRaWAN_End_Node/Core/Inc/rtc.h                                             This file contains all the function prototypes for
                                                                                the rtc.c file
  - LoRaWAN_End_Node/Core/Inc/rtc_if.h                                          configuration of the rtc_if.c instances
  - LoRaWAN_End_Node/Core/Inc/stm32l0xx_hal_conf.h                              HAL configuration file.
  - LoRaWAN_End_Node/Core/Inc/stm32l0xx_it.h                                    This file contains the headers of the interrupt handlers.
  - LoRaWAN_End_Node/Core/Inc/stm32_lpm_if.h                                    Header for Low Power Manager interface configuration
  - LoRaWAN_End_Node/Core/Inc/sys_app.h                                         Function prototypes for sys_app.c file
  - LoRaWAN_End_Node/Core/Inc/sys_conf.h                                        Applicative configuration, e.g. : debug, trace, low power, sensors
  - LoRaWAN_End_Node/Core/Inc/sys_debug.h                                       Configuration of the debug.c instances
  - LoRaWAN_End_Node/Core/Inc/sys_sensors.h                                     Header for sensors application
  - LoRaWAN_End_Node/Core/Inc/usart.h                                           This file contains all the function prototypes for
                                                                                the usart.c file
  - LoRaWAN_End_Node/Core/Inc/usart_if.h                                        Header for USART interface configuration
  - LoRaWAN_End_Node/Core/Inc/utilities_conf.h                                  Header for configuration file to utilities
  - LoRaWAN_End_Node/Core/Inc/utilities_def.h                                   Definitions for modules requiring utilities
  - LoRaWAN_End_Node/LoRaWAN/App/app_lorawan.h                                  Header of application of the LRWAN Middleware
  - LoRaWAN_End_Node/LoRaWAN/App/CayenneLpp.h                                   Implements the Cayenne Low Power Protocol
  - LoRaWAN_End_Node/LoRaWAN/App/Commissioning.h                                End-device commissioning parameters
  - LoRaWAN_End_Node/LoRaWAN/App/lora_app.h                                     Header of application of the LRWAN Middleware
  - LoRaWAN_End_Node/LoRaWAN/App/lora_app_version.h                             Definition the version of the application
  - LoRaWAN_End_Node/LoRaWAN/App/lora_info.h                                    To give info to the application about LoRaWAN configuration
  - LoRaWAN_End_Node/LoRaWAN/App/se-identity.h                                  Secure Element identity and keys
  - LoRaWAN_End_Node/LoRaWAN/Target/iks01a2_conf.h                              This file contains definitions for the MEMS components bus interfaces
  - LoRaWAN_End_Node/LoRaWAN/Target/iks01a3_conf.h                              This file contains definitions for the MEMS components bus interfaces
  - LoRaWAN_End_Node/LoRaWAN/Target/lorawan_conf.h                              Header for LoRaWAN middleware instances
  - LoRaWAN_End_Node/LoRaWAN/Target/mw_log_conf.h                               Configure (enable/disable) traces
  - LoRaWAN_End_Node/LoRaWAN/Target/nucleo_l053r8_bus.h                         header file for the BSP BUS IO driver
  - LoRaWAN_End_Node/LoRaWAN/Target/nucleo_l053r8_errno.h                       Error Code
  - LoRaWAN_End_Node/LoRaWAN/Target/radio_board_if.h                            Header for Radio interface configuration
  - LoRaWAN_End_Node/LoRaWAN/Target/radio_conf.h                                Header of Radio configuration
  - LoRaWAN_End_Node/LoRaWAN/Target/stm32l0xx_nucleo_conf.h                     Configuration file
  - LoRaWAN_End_Node/LoRaWAN/Target/sx1261dvk1bas_conf.h                        Pin mapping sx126x-evb_e407 shield (daughter board)
  - LoRaWAN_End_Node/LoRaWAN/Target/sx1262dvk1cas_conf.h                        Pin mapping sx126x-evb_e407 shield (daughter board)
  - LoRaWAN_End_Node/LoRaWAN/Target/sx1262dvk1das_conf.h                        Pin mapping sx126x-evb_e407 shield (daughter board)
  - LoRaWAN_End_Node/LoRaWAN/Target/sx1272mb2das_conf.h                         This file provides code for the configuration
                                                                                of the shield instances (pin mapping).
  - LoRaWAN_End_Node/LoRaWAN/Target/sx1276mb1las_conf.h                         This file provides code for the configuration
                                                                                of the shield instances (pin mapping).
  - LoRaWAN_End_Node/LoRaWAN/Target/sx1276mb1mas_conf.h                         This file provides code for the configuration
                                                                                of the shield instances (pin mapping).
  - LoRaWAN_End_Node/LoRaWAN/Target/systime.h                                   Map middleware systime
  - LoRaWAN_End_Node/LoRaWAN/Target/timer.h                                     Wrapper to timer server

  - LoRaWAN_End_Node/Core/Src/adc.c                                             This file provides code for the configuration
                                                                                of the ADC instances.
  - LoRaWAN_End_Node/Core/Src/adc_if.c                                          Read status related to the chip (battery level, VREF, chip temperature)
  - LoRaWAN_End_Node/Core/Src/dma.c                                             This file provides code for the configuration
                                                                                of all the requested memory to memory DMA transfers.
  - LoRaWAN_End_Node/Core/Src/main.c                                            Main program body
  - LoRaWAN_End_Node/Core/Src/rtc.c                                             This file provides code for the configuration
                                                                                of the RTC instances.
  - LoRaWAN_End_Node/Core/Src/rtc_if.c                                          Configure RTC Alarm, Tick and Calendar manager
  - LoRaWAN_End_Node/Core/Src/stm32l0xx_hal_msp.c                               This file provides code for the MSP Initialization
                                                                                and de-Initialization codes.
  - LoRaWAN_End_Node/Core/Src/stm32l0xx_it.c                                    Interrupt Service Routines.
  - LoRaWAN_End_Node/Core/Src/stm32_lpm_if.c                                    Low layer function to enter/exit low power modes (stop, sleep)
  - LoRaWAN_End_Node/Core/Src/system_stm32l0xx.c                                CMSIS Cortex-M0+ Device Peripheral Access Layer System Source File.
  - LoRaWAN_End_Node/Core/Src/sys_app.c                                         Initializes HW and SW system entities (not related to the radio)
  - LoRaWAN_End_Node/Core/Src/sys_debug.c                                       Enables 4 debug pins for internal signals RealTime debugging
  - LoRaWAN_End_Node/Core/Src/sys_sensors.c                                     Manages the sensors on the application
  - LoRaWAN_End_Node/Core/Src/usart.c                                           This file provides code for the configuration
                                                                                of the USART instances.
  - LoRaWAN_End_Node/Core/Src/usart_if.c                                        Configuration of UART MX driver interface for hyperterminal communication
  - LoRaWAN_End_Node/LoRaWAN/App/app_lorawan.c                                  Application of the LRWAN Middleware
  - LoRaWAN_End_Node/LoRaWAN/App/CayenneLpp.c                                   Implements the Cayenne Low Power Protocol
  - LoRaWAN_End_Node/LoRaWAN/App/lora_app.c                                     Application of the LRWAN Middleware
  - LoRaWAN_End_Node/LoRaWAN/App/lora_info.c                                    To give info to the application about LoRaWAN configuration
  - LoRaWAN_End_Node/LoRaWAN/Target/nucleo_l053r8_bus.c                         source file for the BSP BUS IO driver
 
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
    - sys_conf.h, radio_conf.h, lorawan_conf.h, lora_app.h, se-identity.h, mw_log_conf.h, radio_board_if.h, main.h, etc
    - Careful: 
        - the region and class chosen on LoRaWAN/App/lora_app.h shall be compatible with LoRaWAN/Target/lorawan_conf.h list

  -Set Up:

             --------------------------  V    V  --------------------------
             |      LoRa Object       |  |    |  |      LoRa Network      |
             |                        |  |    |  |                        |
   ComPort<--|                        |--|    |--|                        |-->Web Server
             |                        |          |                        |
             --------------------------          --------------------------

@par How to use it ? 
In order to make the program work, you must do the following :
  - Open your preferred toolchain 
  - Rebuild all files and load your image into target memory
  - Run the example
  - Open a Terminal, connected the LoRa Object
  - UART Config = 115200, 8b, 1 stopbit, no parity, no flow control

 * <h3><center>&copy; COPYRIGHT STMicroelectronics</center></h3>
 */
