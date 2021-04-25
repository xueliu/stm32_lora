/**
  @page LoRaWAN_AT_Master Readme file
 
  @verbatim
  ******************** (C) COPYRIGHT 2020 STMicroelectronics *******************
  * @file    Applications/LoRaWAN/LoRaWAN_AT_Master/readme.txt 
  * @author  MCD Application Team
  * @brief   This application is  a simple AT_Master Finite State Machine demo
  *          that drives a LoRa modem which is controlled though AT command
  *          interface over UART
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2021 STMicroelectronics.
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

This directory contains a set of source files that implements a host AT_Master
application, featuring a complete set of AT commands to drive an external
LoRa modem.

The application provides a way to establish a LoRa Link, to read sensors
(temperature, humidity and pressure) and to transmit associated data to the
LoRa Network.

This application is targeting the NUCLEO-L053R8 and NUCLEO-L073RZ boards.
  ******************************************************************************

@par Keywords

Applications, SubGHz_Phy, LoRaWAN, AT_Master

@par Directory contents 


  - LoRaWAN_AT_Master/Core/Inc/adc.h                                            This file contains all the function prototypes for
                                                                                the adc.c file
  - LoRaWAN_AT_Master/Core/Inc/adc_if.h                                         Header for ADC interface configuration
  - LoRaWAN_AT_Master/Core/Inc/ee.h                                             Interface for ee driver API
  - LoRaWAN_AT_Master/Core/Inc/main.h                                           Header for main.c file.
                                                                                This file contains the common defines of the application.
  - LoRaWAN_AT_Master/Core/Inc/platform.h                                       Header for General HW instances configuration
  - LoRaWAN_AT_Master/Core/Inc/rtc.h                                            This file contains all the function prototypes for
                                                                                the rtc.c file
  - LoRaWAN_AT_Master/Core/Inc/rtc_if.h                                         configuration of the rtc_if.c instances
  - LoRaWAN_AT_Master/Core/Inc/stm32l0xx_hal_conf.h                             HAL configuration file.
  - LoRaWAN_AT_Master/Core/Inc/stm32l0xx_hw_conf.h                              contains hardaware configuration Macros and Constants
  - LoRaWAN_AT_Master/Core/Inc/stm32l0xx_it.h                                   This file contains the headers of the interrupt handlers.
  - LoRaWAN_AT_Master/Core/Inc/stm32_lpm_if.h                                   Header for Low Power Manager interface configuration
  - LoRaWAN_AT_Master/Core/Inc/sys_app.h                                        Function prototypes for sys_app.c file
  - LoRaWAN_AT_Master/Core/Inc/sys_conf.h                                       Applicative configuration, e.g. : debug, trace, low power, sensors
  - LoRaWAN_AT_Master/Core/Inc/sys_debug.h                                      Configuration of the debug.c instances
  - LoRaWAN_AT_Master/Core/Inc/sys_sensors.h                                    Header for sensors application
  - LoRaWAN_AT_Master/Core/Inc/usart.h                                          This file provides code for the configuration of the USART
                                                                                instances.
  - LoRaWAN_AT_Master/Core/Inc/utilities_conf.h                                 configuration for utilities
  - LoRaWAN_AT_Master/Core/Inc/utilities_def.h                                  Definitions for modules requiring utilities
  - LoRaWAN_AT_Master/LoRaWAN/App/app_master.h                                  Header of Entry point AT master application
  - LoRaWAN_AT_Master/LoRaWAN/App/lora_driver.h                                 Header for lora driver module
  - LoRaWAN_AT_Master/LoRaWAN/App/master_app.h                                  Application of the AT Master

  - LoRaWAN_AT_Master/Core/Src/adc.c                                            This file provides code for the configuration
                                                                                of the ADC instances.
  - LoRaWAN_AT_Master/Core/Src/adc_if.c                                         Read status related to the chip (battery level, VREF, chip temperature)
  - LoRaWAN_AT_Master/Core/Src/ee.c                                             This file contains a R/W function for EEPROM
  - LoRaWAN_AT_Master/Core/Src/main.c                                           Main program body
  - LoRaWAN_AT_Master/Core/Src/rtc.c                                            This file provides code for the configuration
                                                                                of the RTC instances.
  - LoRaWAN_AT_Master/Core/Src/rtc_if.c                                         Configure RTC Alarm, Tick and Calendar manager
  - LoRaWAN_AT_Master/Core/Src/stm32l0xx_hal_msp.c                              This file provides code for the MSP Initialization and
                                                                                de-Initialization codes.
  - LoRaWAN_AT_Master/Core/Src/stm32l0xx_it.c                                   Interrupt Service Routines.
  - LoRaWAN_AT_Master/Core/Src/stm32_lpm_if.c                                   Low layer function to enter/exit low power modes (stop, sleep)
  - LoRaWAN_AT_Master/Core/Src/sys_app.c                                        Initializes HW and SW system entities (not related to the radio)
  - LoRaWAN_AT_Master/Core/Src/sys_debug.c                                      Enables 4 debug pins for internal signals RealTime debugging
  - LoRaWAN_AT_Master/Core/Src/sys_sensors.c                                    Manages the sensors on the application
  - LoRaWAN_AT_Master/Core/Src/usart.c                                          This file provides code for the configuration of the USART
                                                                                instances.
  - LoRaWAN_AT_Master/LoRaWAN/App/app_master.c                                  Entry point AT master application
  - LoRaWAN_AT_Master/LoRaWAN/App/lora_driver.c                                 LoRa module API
  - LoRaWAN_AT_Master/LoRaWAN/App/master_app.c                                  Application of the AT Master


@par Hardware and Software environment 

  - This example runs on the STM32L053R8 and STM32L073RZ devices and has been tested with
    STMicroelectronics NUCLEO-L053R8 RevC and NUCLEO-L073RZ RevC boards.

  - The driven modem can be:
    - a I-NUCLEO-LRWAN1 USI® STM32™ Nucleo expansion board for LoRa™
    - a B-L072Z-LRWAN1 Discovery board running an AT_Slave application, identified as a MDM32L07X01 modem
    - a RisingHF LRWAN_NS1 sensor expansion board
    - a STM32WLxx Nucleo board running an AT_Slave application, identified as a MDM32WL modem

  - NUCLEO-L053R8/NUCLEO-L073RZ board Set-up 
    - When using MDM32L07X01 expansion board, plug the X_NUCLEO_IKS01A1 shield (sensors) 
    - Plug the  expansion board (I-NUCLEO-LRWAN1 or MDM32L07X01 or LRWAN_NS1 or MDM32WL) on the NUCLEO-L053R8 board   
    - Connect the Nucleo board to your PC with a USB cable type A to micro-B 
      to ST-LINK connector (CN1).
    - Please ensure that the ST-LINK connector CN2 jumpers are fitted.


  -Set Up:


     -----------------                      -----------------------  V    V  ----------------------
     |  AT-Master    |                      |      LoRa Modem     |  |    |  |      LoRa Network  |
     |  Application  |                      |                     |  |    |  |                    |
     |               |<--AT_CMD over UART-->|                     |--|    |--|                    |-->Web Server
     |               |                      |                     |          |                    |
     -----------------                      -----------------------          ----------------------

@par How to use it ? 
In order to make the program work, you must do the following :
  - Open your preferred toolchain 
  - Rebuild all files and load your image into target memory (select the right workspace following the expansion board I_NUCLEO_LRWAN1 or
    MDM32L07X01 or LRWAN_NS1 or MDM32WL)
  - Run the example
  - Note : If needed and in order to spy the communication between AT_MAster and LoRa modem 
    - Open an Hyperterminal Terminal and connect it (could be an FTDI cable) to the "D1:LPUART1_RX/D0:LPUART1_TX" of the LoRa Modem
    - UART Config = 115200, 8b, 1 stopbit, no parity, no flow control
    - Terminal Config: Select 'CR+LF' for Transmit New-Line and switch 'Local echo' on

 * <h3><center>&copy; COPYRIGHT STMicroelectronics</center></h3>
 */