#![allow(dead_code, mutable_transmutes, non_camel_case_types, non_snake_case,
         non_upper_case_globals, unused_assignments, unused_mut)]
#![register_tool(c2rust)]
#![feature(register_tool)]
extern "C" {
    #[no_mangle]
    fn rlDriverGetPlatformId() -> rlUInt8_t;
    #[no_mangle]
    fn rlDriverIsDeviceMapValid(deviceMap: rlUInt8_t) -> rlReturnVal_t;
    #[no_mangle]
    fn rlDriverCmdInvoke(deviceMap: rlUInt8_t, inMsg: rlDriverMsg_t,
                         outMsg: *mut rlDriverMsg_t) -> rlReturnVal_t;
    #[no_mangle]
    fn rlDriverFillPayload(msgId: rlUInt16_t, sbcID: rlUInt16_t,
                           payloadPtr: *mut rlPayloadSb_t,
                           data: *mut rlUInt8_t, inLen: rlUInt16_t);
    #[no_mangle]
    fn rlDriverConstructInMsg(msgId: rlUInt16_t, inMsg: *mut rlDriverMsg_t,
                              payloadPtr: *mut rlPayloadSb_t);
    #[no_mangle]
    fn rlDriverConstructOutMsg(numSblk: rlUInt16_t,
                               outMsg: *mut rlDriverMsg_t,
                               payloadPtr: *mut rlPayloadSb_t);
    #[no_mangle]
    fn rlDriverExecuteGetApi(deviceMap: rlUInt8_t, msgId: rlUInt16_t,
                             sbcID: rlUInt16_t, msgData: *mut rlUInt8_t,
                             inLen: rlUInt16_t) -> rlReturnVal_t;
    #[no_mangle]
    fn rlDriverExecuteSetApi(deviceMap: rlUInt8_t, msgId: rlUInt16_t,
                             sbcID: rlUInt16_t, msgData: *mut rlUInt8_t,
                             inLen: rlUInt16_t) -> rlReturnVal_t;
}
/* ***************************************************************************************
 * FileName     : rl_datatypes.h
 *
 * Description  : This file defines the datatypes.
 *
 ****************************************************************************************
 * (C) Copyright 2014, Texas Instruments Incorporated. - TI web address www.ti.com
 *---------------------------------------------------------------------------------------
 *
 *  Redistribution and use in source and binary forms, with or without modification,
 *  are permitted provided that the following conditions are met:
 *
 *    Redistributions of source code must retain the above copyright notice,
 *    this list of conditions and the following disclaimer.
 *
 *    Redistributions in binary form must reproduce the above copyright notice,
 *    this list of conditions and the following disclaimer in the documentation
 *    and/or other materials provided with the distribution.
 *
 *    Neither the name of Texas Instruments Incorporated nor the names of its
 *    contributors may be used to endorse or promote products derived from this
 *    software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 *  AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 *  THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR
 *  PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT  OWNER OR CONTRIBUTORS
 *  BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 *  CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 *  SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 *  INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 *  CONTRACT,  STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 *  ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *
 ****************************************************************************************
 * Revision History   :
 *---------------------------------------------------------------------------------------
 * Version  Date        Author    Defect No       Description
 *---------------------------------------------------------------------------------------
 * 0.1      12May2015   Kaushal    -               Initial Version
 *
 ****************************************************************************************
 */
/* ***************************************************************************************
 * FILE INCLUSION PROTECTION
 ******************************************************************************
 */
/* *****************************************************************************
 * INCLUDE FILES
 ******************************************************************************
 */
/* *****************************************************************************
 * TYPE-DEFINE STRUCT/ENUM/UNION DEFINITIONS
 *******************************************************************************
 */
/* ! \brief
 * Standard C data types typedef
 */
pub type rlUInt8_t = libc::c_uchar;
pub type rlUInt16_t = libc::c_ushort;
pub type rlUInt32_t = libc::c_uint;
pub type rlInt8_t = libc::c_char;
pub type rlInt16_t = libc::c_short;
pub type rlInt32_t = libc::c_int;
/* ************************************************************************************************
 * FileName    : mmwavelink.h
 *
 * Description : This file includes all the header files which needs to be included by application
 *
 *************************************************************************************************
 * (C) Copyright 2014, Texas Instruments Incorporated. - TI web address www.ti.com
 *------------------------------------------------------------------------------------------------
 *
 *  Redistribution and use in source and binary forms, with or without modification, are permitted
 *  provided that the following conditions are met:
 *
 *    Redistributions of source code must retain the above copyright notice, this list of
 *    conditions and the following disclaimer.
 *
 *    Redistributions in binary form must reproduce the above copyright notice, this list of
 *    conditions and the following disclaimer in the documentation and/or other materials provided
 *    with the distribution.
 *
 *    Neither the name of Texas Instruments Incorporated nor the names of its contributors may be
 *    used to endorse or promote products derived from this software without specific prior
 *    written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY EXPRESS OR
 *  IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY
 *  AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT  OWNER OR
 *  CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 *  CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 *  SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
 *  THEORY OF LIABILITY, WHETHER IN CONTRACT,  STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR
 *  OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *
 */
/* ************************************************************************************************
 * FILE INCLUSION PROTECTION
 *************************************************************************************************
 */
/*LDRA_NOANALYSIS*/
 /* !
 \mainpage mmWaveLink Framework

 \section intro_sec Introduction

 TI Automotive and Industrial mmWave Radar products are highly-integrated 77GHz CMOS millimeter
 wave devices.The devices integrate all of the RF and Analog functionality, including VCO, PLL,
 PA, LNA, Mixer and ADC for multiple TX/RX channels into a single chip.\n
 -# The AWR1243 is an RF transceiver device and it includes 4 receiver channels and 3 transmit
 channels in a single chip. AWR1243 also support multi-chip cascading. \n
 -# The AWR1443/IWR1443 is a mmwave radar-on-a-chip device, which includes 4 receive channels
 and 3 transmit channels and additionally an Cortex R4F and hardware FFT accelerator.\n
 -# AWR1642 and IWR1642 are mmwave radar-on-a-chip device, which includes 4 receive channels and 2
 transmit channels and additionally an Cortex R4F and C674x DSP for signal processing

 TI mmWave radar devices include a mmwave front end or BIST (Built-in Self-Test) processor, which
 is responsible to configure the RF/Analog and digital front-end in real-time, as well as to
 periodically schedule calibration  and functional safety monitoring.This enables the mmwave
 front-end(BIST processor) to be self-contained and capable of adapting itself to handle
 temperature and ageing effects, and to enable significant ease-of-use from an external host
 perspective.

 TI mmwave front end is a closed subsystem whose internal blocks are configurable using messages
 coming over mailbox.\n TI mmWaveLink provides APIs generates these message and sends it to mmwave
 front end over mailbox. It also includes acknowledement and CRC for each message to provide a
 reliable communication

 TI mmWaveLink Framework:
 - Is a link between application and mmwave front end.
 - Provides low level APIs to configure the front end and handles the communication with the front
   end.
 - Is platform and OS independent which means it can be ported into any processor which provides
   communication interface such as SPI and basic OS routines. The mmWaveLink framework can also
   run in single threaded environment
 - Is integrated into mmWave SDK and can run on R4F or DSP and communicates with mmwave front
   end over Mailbox interface

 *  @image html mmwl_block_diagram.png

  \section modules_sec Modules
  To make it simple, TI's mmWaveLink framework capabilities are divided into modules.\n
  These capabilities include device control, RF/Analog configuration, ADC configuration,
  Data path(LVDS/CSI2) cofiguration, FMCW chirp configuration and more.\n
  Listed below are the various modules in the mmWaveLink framework:
      -# \ref Device - Controls mmwave radar device which include:

            * Initialization, such as: mmwave device power On/Off, Firmware Patch download
            * Cascade device configuration such as Add/Connect multiple mmWave devices

      -# \ref Sensor - The RF/Sensor Configuration module controls the different HW blocks
             inside mmWave Front end.

       *  @image html mmwave_frontend.png

         * mmWave Front End has below key blocks
         -# Chirp sequencer (Radar Timing Engine) - This block is responsible for constructing
            the sequence of FMCW chirps or frames and programming the timing engine
         -# Rx/Tx Channel - This defines how many Rx and Tx channel needs to be enabled. Also it
            defines how to configure the mmWave front end in cascade mode for Imaging Radar
         -# Rx Analog Chain - This defines how the received signal is mixed and how different
            filters in the chain can be configured
         -# ADC and Digital Front End Configuration - This defines how the IF data is digitized
            and how it is sampled for further processing in the DSP or Hardware Accelerator.
            Same ADC data can be sent over LVDS/CSI2 interface to an extenal processor

       * The configuration APIs can further be categorized as.\n
       -# mmwave static configuration, such as: Tx and Rx channel, ADC configuration etc
       -# mmwave dynamic configuration, such as FMCW Chirp configuration, profile configuration
       -# mmwave advance configuration such as Binary phase modulation, Dynamic power save etc
       -# mmwave sensor control, such as: Frame start/stop

      -# \ref Data_Path - The Data path Configuration module controls the high speed interface
              in mmWave device.

       *  @image html data_path.png

       * The configuration APIs include.\n
       -# High Speed interface(LVDS/CSI2) selection
       -# Data format and rate configuration
       -# ADC, Chirp Profile(CP), Chirp Quality(CQ) data transfer sequence
       -# Lane configurations
       -# LVDS/CSI2 specific configuration

      -# \ref Monitoring - The Monitoring/Calibration module configures the calibration and
              functional safety monitoring in mmWave device

     * TI mmWave Front end includes built-in processor that is programmed by TI to handle RF
      calibrations and functional safety monitoring.  The RF calibrations ensure that the
      performance of the device is maintained across temperature and process corners

      -# \ref Communication_Protocol - The mmWave communication protocol ensures reliable
              communication between host(internal or external) and mmWave Front end.

        -# It is a simple stop and wait protocol. Each message needs to be acknowledged
            by receiver before next message can be sent.
        -# Messages are classifieds as "Command", "Response" and "Asynchronous Event"
        -# If Command can not be processed immediately, ACK response is sent immediately
            (If requested). "Asynchronous Event"  is sent upon completion of the command
            execution.

       *  @image html comm_prot.png

     \section         proting_sec     Porting Guide

 The porting of the mmWaveLink driver to any new platform is based on few simple steps.
 This guide takes you through this process step by step. Please follow the instructions
 carefully to avoid any problems during this process and to enable efficient and proper
 work with the device.
 Please notice that all modifications and porting adjustments of the driver should be
 made in the application only and driver should not be modified.
 Changes in the application file will ensure smoothly transaction to
 new versions of the driver at the future!

 \subsection     porting_step1   Step 1 - Define mmWaveLink client callback structure
The mmWaveLink framework is ported to different platforms using mmWaveLink client callbacks. These
callbacks are grouped as different structures such as OS callbacks, Communication Interface
callbacks and others. Application needs to define these callbacks and initialize the mmWaveLink
framework with the structure.

 *  @code
 *  rlClientCbs_t clientCtx = { 0 };
 *  rlReturnVal_t retVal;
 *  rlUInt32_t deviceMap = RL_DEVICE_MAP_CASCADED_1;
 *  @endcode

 Refer to \ref rlClientCbs_t for more details

 \subsection     porting_step2   Step 2 - Implement Communication Interface Callbacks
 The mmWaveLink device support several standard communication protocol among SPI and MailBox
  Depending on device variant, one need to choose the communication channel. For e.g
 xWR1443/xWR1642/xWR1843 requires Mailbox interface and AWR1243 supports SPI interface.
 The interface for this communication channel should include 4 simple access functions:
 -# rlComIfOpen
 -# rlComIfClose
 -# rlComIfRead
 -# rlComIfWrite

 *  @code
 *  clientCtx.comIfCb.rlComIfOpen = Host_spiOpen;
 *  clientCtx.comIfCb.rlComIfClose = Host_spiClose;
 *  clientCtx.comIfCb.rlComIfRead = Host_spiRead;
 *  clientCtx.comIfCb.rlComIfWrite = Host_spiWrite;
 *  @endcode

 Refer to \ref rlComIfCbs_t for interface details

  \subsection     porting_step3   Step 3 - Implement Device Control Interface
 The mmWaveLink driver internally powers on/off the mmWave radar device. The exact implementation
 of these interface is platform dependent, hence you need to implement below functions:
 -# rlDeviceEnable
 -# rlDeviceDisable
 -# rlRegisterInterruptHandler

 *  @code
 *  clientCtx.devCtrlCb.rlDeviceDisable = Host_disableDevice;
 *  clientCtx.devCtrlCb.rlDeviceEnable = Host_enableDevice;
 *  clientCtx.devCtrlCb.rlDeviceMaskHostIrq = Host_spiIRQMask;
 *  clientCtx.devCtrlCb.rlDeviceUnMaskHostIrq = Host_spiIRQUnMask;
 *  clientCtx.devCtrlCb.rlRegisterInterruptHandler = Host_registerInterruptHandler;
 *  clientCtx.devCtrlCb.rlDeviceWaitIrqStatus = Host_deviceWaitIrqStatus;
 *  @endcode

 Refer to \ref rlDeviceCtrlCbs_t for interface details

 \subsection     porting_step4     Step 4 - Implement Event Handlers
 The mmWaveLink driver reports asynchronous event indicating mmwave radar device status,
 exceptions etc. Application can register this callback to receive these notification and take
 appropriate actions

 *  @code
 *  clientCtx.eventCb.rlAsyncEvent = Host_asyncEventHandler;
 *  @endcode

 Refer to \ref rlEventCbs_t for interface details

 \subsection     porting_step5     Step 5 - Implement OS Interface
 The mmWaveLink driver can work in both OS and NonOS environment. If Application prefers to use
 operating system, it needs to implement basic OS routines such as tasks, mutex and Semaphore.
 In Case of Non-OS environment application needs to implement equivalent form of mutex & semaphore.

 *  @code
 *  clientCtx.osiCb.mutex.rlOsiMutexCreate = Host_osiLockObjCreate;
 *  clientCtx.osiCb.mutex.rlOsiMutexLock = Host_osiLockObjLock;
 *  clientCtx.osiCb.mutex.rlOsiMutexUnLock = Host_osiLockObjUnlock;
 *  clientCtx.osiCb.mutex.rlOsiMutexDelete = Host_osiLockObjDelete;
 *
 *  clientCtx.osiCb.sem.rlOsiSemCreate = Host_osiSyncObjCreate;
 *  clientCtx.osiCb.sem.rlOsiSemWait = Host_osiSyncObjWait;
 *  clientCtx.osiCb.sem.rlOsiSemSignal = Host_osiSyncObjSignal;
 *  clientCtx.osiCb.sem.rlOsiSemDelete = Host_osiSyncObjDelete;
 *
 *  clientCtx.osiCb.queue.rlOsiSpawn = Host_osiSpawn;
 *
 *  clientCtx.timerCb.rlDelay = Host_osiSleep;
 *  @endcode


 Refer to \ref rlOsiCbs_t for interface details

 \subsection     porting_step6     Step 6 - Implement CRC Interface and Type
 The mmWaveLink driver uses CRC for message integrity. If Application prefers to use
 CRC, it needs to implement CRC routine and provides the CRC type.

 *  @code
 *  clientCtx.crcCb.rlComputeCRC = Host_computeCRC;
 *  clientCtx.crcType = RL_CRC_TYPE_32BIT;
 *  @endcode

 Refer to \ref rlCrcCbs_t for interface details
 @note : Recommended CRC type is RL_CRC_TYPE_32BIT for AWR1243 device.

 \subsection     porting_step7     Step 7 - Implement Debug Interface
 The mmWaveLink driver can print the debug message. If Application prefers to enable
 debug messages, it needs to implement debug callback.

 Refer to \ref rlDbgCb_t for interface details

 \subsection     porting_final     Final Step - Initializing mmWaveLink Driver
 Once all the above Interfaces are implemented, Application need to fill these callbacks
 in \ref rlClientCbs_t and Initialize mmWaveLink by passing the client callbacks.
 Application also need to define where the mmWaveLink driver is running, for e.g,
 External Host in case of AWR1243 or MSS/DSS in case of xWR1642/xWR1843.

 *
 *  @code
 *  clientCtx.platform = RL_PLATFORM_HOST;
 *  clientCtx.arDevType = RL_AR_DEVICETYPE_12XX;
 *
 *  retVal = rlDevicePowerOn(deviceMap, clientCtx);
 *  @endcode
 *
  \subsection     porting_crc     CRC Type Implementation
  Device sets same CRC type as recieved command to response message. So change to command's
  CRC type will cause a change to response's CRC type as well. [Refer to \ref porting_step7].
  mmWave device(MasterSS & RadarSS) uses 16-bit CRC type by default for async-event messages.
  If Host needs to set different CRC type to Async-event then it must implement the following
  code snippet.
 *
 *  @code
 *  rlRfDevCfg_t rfDevCfg = {0x0};
 *  // set global and monitoring async event direction to Host
 *  rfDevCfg.aeDirection = 0x05;
 *  // Set the CRC type of Async event received from RadarSS
 *  rfDevCfg.aeCrcConfig = RL_CRC_TYPE_32BIT;
 *  retVal = rlRfSetDeviceCfg(deviceMap, &rfDevCfg);
 *
 *  rlDevMiscCfg_t devMiscCfg = {0};
 *  // Set the CRC Type for Async Event from MSS
 *  devMiscCfg.aeCrcConfig = RL_CRC_TYPE_32BIT;
 *  retVal = rlDeviceSetMiscConfig(deviceMap, &devMiscCfg);
 *  @endcode
 *
  \subsection     porting_be     Big Endian Support
 The mmWaveLink driver by default is enabled for Little Endian host. Support for
 Big Endian is provided as compile time option using a Pre-processor Macro MMWL_BIG_ENDIAN. \n
 For memory optimizations, mmWaveLink doesn't swap the data elements in structure buffer. It
 is the responsibility of the application to swap multi byte data elements before passing the
 structure buffer to mmWaveLink API. Since SPI word-size is 16bit, Swap of 32 bit fields such
 as integer needs to be done at 16bit boundary.
 @note 1: Please refer latest mmWave device DFP release notes for all known issues and de-featured
          APIs. \n
 @note 2: All reserved bits/bytes in API sub blocks shall be programmed with value zero. The
          functionality of radar device is not guaranteed if reserved bytes are not zero. \n
 @note 3: All reserved bits/bytes in API message reports shall be masked off. \n
 *
  \subsection     notes     General Notes
        -# Host should ensure that there is a delay of at least 2 SPI clocks between CS going low 
           and start of SPI clock.
        -# Host should ensure that CS is toggled for every 16 bits of transfer via SPI.
        -# There should be a delay of at least 2 SPI Clocks between consecutive CS.
        -# SPI needs to be operated at Mode 0 (Phase 1, Polarity 0).
        -# SPI word length should be 16 bit (Half word).
 *
  \subsection     appl_notes     Application Care Abouts
        -# Retry of RF Power up message is unsupported.
        -# HOST is recommended to wait for RF Power Async msg before any further APIs are issued. 
           Lack of RF Power up Async msg should be treated as bootup failure.
        -# It is recommended to wait for Async event for Latent fault injection API before the 
           next CMD is issued.
        -# HOST to ensure a delay of 30us in response to the HOST_IRQ interrupt, to allow for a 
           SPI DMA configuration in device post HOST_IRQ set high.
        -# It is recommended to use 232 as the chunk size in mmWavelink/HOST when firmware 
           download is done through SPI. 
 */
/*LDRA_ANALYSIS*/
/* ***************************************************************************************
 * INCLUDE FILES
 *****************************************************************************************
 */
/* ****************************************************************************************
 * MACRO DEFINITIONS
 *****************************************************************************************
 */
/* Export Macro for DLL */
/* ! mmWaveLink Version */
/* ! mmWaveLink Error Codes */
/* no-error */
/* mmWaveLink Protocol error */
/* invalid input from the application */
/* error in mmWaveLink itself */
/* Radar HW/SW interface error */
/* memory allocation error */
/* CRC value mismatched wrt
                                                                received data */
/* Checksum value mismatched wrt to
                                                                received data */
/* device failed to send response
                                                                within time */
/* Fatal error internal to
                                                                mmWaveLink APIs */
/* OS interface failure */
/* Invalid state within mmWaveLink */
/* API called is not supported */
/* Message-ID mismatched in
                                                                response data */
/* Null pointer error */
/* Interface callback passed as NULL */
/* If device sends NACK message */
/* If post writing CNYS HostIRQ is not
                                                          down within time limit and re-writing
                                                          CNYS also has same result */
/* ACK sequence number is not matching with
                                                          CMD sequence number */
/* ! RF Error Codes */
/* Incorrect opcode/Msg ID */
/* Incorrect no. of Sub-Block */
/* Incorrect Sub-Block ID */
/* Incorrect Sub-Block Length */
/* Incorrect Sub-Block Data */
/* Error in Sub Block processing */
/* Mismatch in File CRC */
/* Mismatch in File Type */
/* ! Frame start stop API */
/* Frames are already started when the
                                                                FRAME_START command was issued */
/* Frames are already stopped when the
                                                                FRAME_STOP command was issued */
/* No valid frame configuration API was
                                                                issued and frames are started */
/* START_STOP_CMD parameter is out of
                                                                range*/
/* ! Channel Config API */
/* RX_CHAN_EN parameter is out of range
                                                                may vary based on device */
/* TX_CHAN_EN parameter is out of range
                                                                may vary based on device */
/* CASCADING_CFG parameter is out of
                                                                range [0, 2] */
/* ! ADC out API */
/* NUM_ADC_BITS parameter is out of
                                                                range [0, 2] */
/* ADC_OUT_FMT parameter is out of
                                                                range [0, 3] */
/* ! Low power ADC API */
/* LP_ADC_MODE parameter is out of
                                                                range [0, 1] */
/* ! Dynamic power save API */
/* BLOCK_CFG parameter is out of
                                                                range [0, 7] */
/* ! HSI config API */
/* HSI clock rate code[1:0] is 0 */
/* HSI clock rate code[3:2] is 3  &
                                                                HSI clock rate code[1:0] is 2 */
/* HSI clock rate code[3:2] is 3  &
                                                                HSI clock rate code[1:0] is 2 */
/* ! Profile config API */
/* PF indx >= 4 */
/* PF freq const is not
                                                                with[76GHz,81GHz] in limit */
/* PF idle time const > 5.24ms */
/* Maximum DFE spill time (refer
                                                               rampgen calculator in mmWaveStudio 
                                                               for more details) > PF idle 
                                                               time const */
/* PF ADC start time const > 4095 */
/* PF ramp end time > 524287 */
/* PF ramp end time < PF ADC start
                                                                time const + ADC sampling time */
/* PF_TX_OUTPUT_POWER_BACKOFF for
                                                                TX0 > 30 */
/* PF_TX_OUTPUT_POWER_BACKOFF for
                                                                TX1 > 30 */
/* PF_TX_OUTPUT_POWER_BACKOFF for
                                                                TX2 > 30 */
/* Ramp end freq is not
                                                                with[76GHz,81GHz] in limits */
/* Absolute value of TX_START_TIME
                                                                is > 38.45us */
/* Number of ADC samples is not
                                                                within [2,8192] */
/* Output sampling rate is not
                                                                within [2, 37.5]Msps */
/* HPF1 corner frequency > 700 kHz */
/* HPF2 corner frequency > 2.8 MHz */
/* PF_RX_GAIN is not within [24, 52] dB
                                                                orPF_RX_GAIN is an odd number */
/* ! Chirp config API */
/* Chirp Start indx >= 512 */
/* Chirp End indx >= 512 */
/* Chirp Start indx > Chirp End indx */
/* PF indx >= 4 */
/* PF corresponding to PF indx is not
                                                                defined */
/* Chirp freq start > 8388607 */
/* Chirp freq slope > 63 */
/* Chirp start or end
                                                                freq[76GHz,81GHz] is outside */
/* Chirp Idle time > 4095 */
/* Chirp ADC start time > 4095 */
/* Ramp end time < ADC start time +
                                                                ADC sampling time */
/* Chirp TX enable > 7 */
/* Chirp TX enable indicates to enable
                                                                a TX which is not enabled in
                                                                Channel config */
/* ! Frame config API */
/* Chirp Start indx >= 512 */
/* Chirp End indx >= 512 */
/* Chirp Start indx > Chirp End indx */
/* Chirp used in frame is not
                                                                configured by Chirp config */
/* Profile used in frame is not
                                                                configured by PF config */
/* No. of loops is outside[1,255] */
/* Frame periodicity is
                                                                outside[100us,1.342s] */
/* Frame ON time > Frame periodicity */
/* Trigger select is outside[1,2] */
/* Frame Trigger delay > 100us */
/* API issued when frame is ongoing */
/* ! Advance Frame config API */
/* No. Sub Frames is outside[1,4] */
/* Force single Profile is
                                                                outside[1,4] */
/* Force single Profile >= 4 */
/* Profile defined by Force Single
                                                                Profile is not defined */
/* Sub Frame Chirp Start indx >= 512 */
/* Sub Frame NO. of unique chirps per
                                                                Burst is outside[1,512] */
/* Chirp used in frame is not
                                                                configured by Chirp config */
/* Profie used in the frame is not
                                                                configured by profile config */
/* Sub Frame No. of loops is
                                                                outside[1,225] */
/* Sub Frame burst period is
                                                                outside[100us,1.342s] */
/* Burst ON time > Burst period */
/* Sub Frame Chirp start indx
                                                                offset >= 512 */
/* Sub Frame Chirp start indx >= 512
                                                                or (Sub Frame Chirp start indx +
                                                                Sub Frame No. unique Chirps per
                                                                burst - 1) >= 512*/
/* Sub Frame No. bursts is
                                                                outside[1,512] */
/* Sub Frame No. outer loops is
                                                                outside[1,64] */
/* Sub Frame period is
                                                                outside[100us,1.342s] */
/* Sub Frame ontime > Sub Frame period
                                                                or when test source enabled, Sub
                                                                Frame idale time < 150us */
/* Trigger select is outside[1,2] */
/* Frame trigger delay is > 100us */
/* API issued when frame is ongoing */
/* ! Test source config API */
/* position vector x[y] < 0 */
/* position vector x[x] < 5000 or
                                                                position vector x[y] < 5000 or
                                                                position vector x[x] < 5000 */
/* SIG_LEV_VECx > 950 */
/* RX_ANT_POS_XZ[Bytex] > 120 */
/* ! Programmable filter config API */
/* Prog. Filter coefficient start
                                                                 indx is odd number */
/* Pro indx >= 4 */
/* API issued for non AWR1642 device*/
/* ! Per chirp phase shifter API */
/* API issued for non AWR1243 device*/
/* Chirp Start indx >= 512 */
/* Chirp End indx >= 512 */
/* Chirp Start indx > End indx */
/* ! Calibration config APIs */
/* Boot time calibrations are not
                                                         done so cannot run runtime calibrations */
/* Freq. is outside[76GHz,81GHz] or
                                                                 Freq. low limit > high limit */
/* CALIB_MON_TIME_UNIT <= 0 */
/* CALIBRATION_ PERIODICITY = 0 */
/* API is issued when continuous
                                                                streaming mode is on */
/* RX gain run time calibration was
                                                                requested but boot time calibration
                                                                was not performed */
/* LO distribution run time
                                                            calibration was requested but boot time
                                                            calibration was not performed */
/* TX power run time calibration was
                                                                requested but boot time calibration
                                                                was not performed */
/* DFE mode is pseudo real */
/* FULL_SCALE_REDUCTION_FACTOR is > 0
                                                                for 16 bit ADC, or > 2 for 14 bit
                                                             ADC mode or > 4 for 12 bit ADC mode */
/* NUM_OF_CASCADED_DEV <= 0. This
                                                                error code applicable only on
                                                                xWR6843 ES2.0 */
/* Minimum RF frequency is < 200MHz.
                                                                This error code applicable only on
                                                                xWR6843 ES2.0 */
/* ! Loopback burst error */
/* ! Inter Chirp Block Control Config */
/* RX02_RF_TURN_OFF_TIME is not
                                                                within the range [-1024, 1023] */
/* RX13_RF_TURN_OFF_TIME is not
                                                                within the range [-1024, 1023] */
/* RX02_BB_TURN_OFF_TIME is not
                                                                within the range [-1024, 1023] */
/* RX13_BB_TURN_OFF_TIME is not
                                                                within the range [-1024, 1023] */
/* RX02_RF_PREENABLE_TIME is not
                                                                within the range [-1024, 1023] */
/* RX13_RF_PREENABLE_TIME is not
                                                                within the range [-1024, 1023] */
/* RX02_BB_PREENABLE_TIME is not
                                                                within the range [-1024, 1023] */
/* RX13_BB_PREENABLE_TIME is not
                                                                within the range [-1024, 1023] */
/* RX02_RF_TURN_ON_TIME is not
                                                                within the range [-1024, 1023] */
/* RX13_RF_TURN_ON_TIME is not
                                                                within the range [-1024, 1023] */
/* RX02_BB_TURN_ON_TIME is not
                                                                within the range [-1024, 1023] */
/* RX13_BB_TURN_ON_TIME is not
                                                                within the range [-1024, 1023] */
/* RX_LO_TURN_OFF_TIME is not
                                                                within the range [-1024, 1023] */
/* TX_LO_TURN_OFF_TIME is not
                                                                within the range [-1024, 1023] */
/* RX_LO_TURN_ON_TIME is not
                                                                within the range [-1024, 1023] */
/* TX_LO_TURN_ON_TIME is not
                                                                within the range [-1024, 1023] */
/* Sub frame trigger option is not
                                                                enabled but sub frame trigger API
                                                                is issued or frame is configured
                                                                for software trigger mode and
                                                                sub-frame trigger API is issued */
/* Regular ADC mode is issued on a
                                                                5 MHz part variant */
/* Chirp row select is not with in
                                                                the range [0x00, 0x30] */
/* ! Monitoring config APIs */
/* Device type is not ASILB */
/* Fault injection API or Digital
                                                                 latent fault API is issued when
                                                                 frames are ongoing */
/* Invalid reporting mode */
/* Configured profile ID is not
                                                                 within [0,3] */
/* Monitoring profile ID is not
                                                                 configured yet */
/* Settling time is configured is
                                                                 more than 12us */
/* None of the RXs are enabled */
/* TX0 is not enabled */
/* TX1 is not enabled */
/* TX2 is not enabled */
/* Invalid RF bit mask */
/* Monitored TX is not enabled */
/* Monitored RX is not enabled */
/* TX selected for RX gain phase
                                                                 monitor is TX2 (Only TX0 or TX1 is
                                                                 allowed) */
/* SAT_MON_SEL is not in [0, 3] */
/* SAT_MON_PRIMARY_TIME_SLICE_DURATION
                                                                 is less than 0.64us or greater
                                                                 than ADC sampling time */
/* SAT_MON_NUM_SLICES is 0 or
                                                                 greater than 127 */
/* SIG_IMG_MON_NUM_SLICES is 0 or
                                                                 greater than 127 */
/* NUM_SAMPLES_ PER_PRIMARY_TIME_SLICE
                                                            is odd, or less than 4 in Complex1x
                                                            mode or less than 8 in non-Complex1x
                                                            modes or greater than NUM_ADC_SAMPLES*/
/* MONITOR_START_TIME is outside the 
                                                                specified range. */
/* LDO fault inject is requested but
                                                                  LDOs are bypassed */
/* Signal and image band monitor is
                                                                not supported */
/* Device variant does not allow
                                                                cascading but API is issued to
                                                                enable cascading mode */
/* Monitoring chirp error */
/* Loopback power measured by PD
                                                                is below -40 dBm */
/* ADC power is higher than 7 dBm */
/* Noise figure is less than 0 */
/* PD measurement with RF on is less
                                                                than with RF off */
/* Incorrect PGA gain for monitoring*/
/* The 20G monitor is not supported 
                                                                in single chip configuration */
/* MONITOR_CONFIG_MODE is invalid. */
/* Both Live and Non-live synth 
                                                                frequency monitors are cannot be 
                                                                enabled together. */
/* Invalid monitoring start 
                                                                frequency. [Only applicable to
                                                                xWR6x43 devices]. */
/* Invalid state transition command
                                                                received */
/* API is not supported in this 
                                                                device */
/* ADC Config API */
/* numADCBits out of Range */
/* rxChannelEn out of Range */
/* adcOutFormat out of Range */
/* sampleInterleave out of
                                                                      Range */
/* channelInterleave out of
                                                                      Range */
/* Data Path Config API */
/* dataIntfSel out of Range */
/* dataTransPkt0Format
                                                                      Unsupporetd */
/* dataTransPkt1Format
                                                                      Unsupporetd */
/* Lane Enable config API */
/* laneEnable is out of range */
/* laneEnable is not supported */
/* Lane Clock config API */
/* laneClkCfg is out of range */
/* laneClkCfg is not supported */
/* dataRate is out of range */
/* LVDS config API */
/* laneFmtMap is out of range */
/* laneParamCfg is out of range */
/* Continuous Streaming Mode API */
/* contStreamMode is out of
                                                                      range */
/* contStreamMode is already
                                                                      in requested mode */
/* CSI2 Lane Config API */
/* lane0 pos is out of range */
/* lane1 pos is out of range */
/* lane2 pos is out of range */
/* lane3 pos is out of range */
/* ClockPos is out of range */
/* Frame Config Apply API */
/* adcOutSize is out of range */
/* Advanced Frame Config API */
/* numSubFrames is out of range */
/* totNumChirps is out of range */
/* numADCSamplesInPkt is out
                                                                      of range */
/* numChirpsInPkt is out of
                                                                      range */
/* totNumChirps is out of
                                                                      range */
/* numADCSamplesInPkt is out
                                                                      of range */
/* numChirpsInPkt is out of
                                                                      range */
/* totNumChirps is out of
                                                                      range */
/* numADCSamplesInPkt is out of
                                                                      range */
/* numChirpsInPkt is out of
                                                                      range */
/* totNumChirps is out of
                                                                      range */
/* numADCSamplesInPkt is out of
                                                                      range */
/* numChirpsInPkt is out of range */
/* mcuClkOutEn is out of range */
/* mcuClkOutSrc is out of range */
/* pmicClkOutEn is out of range */
/* pmicClkOutSrc is out of range */
/* modeSel is out of range */
/* freqSlope is out of range */
/* clkDitherEn is out of range */
/* testPatternGenEn is out of
                                                                    range */
/* Data interface selected in
                                                                  RL_DEV_RX_DATA_PATH_CONF_SET_SB
                                                                  is SPI */
/* Unsupported Latent Fault test
                                                                  selected in
                                                                 RL_DEV_LATENTFAULT_TEST_CONF_SB */
/* ! \brief
 * mmwavelink MACRO to enable/disable logging.
 * To enable logging set this MACRO to '0' and set proper function pointer
 * dbgCb.rlPrint and debug level dbgCb.dbgLevel out of RL_DBG_LEVEL_* during rlDevicePowerOn
 */
/* mmwavelink MACROs for Error Checks */
/* build time MACRO to change message size */
/* !< This includes payload+HR+CRC */
/* * @note : \n
 * 1. When Running On MSS: \n
 *         RL_DEVICE_INDEX_INTERNAL_BSS: for radarSS communication \n
 *         RL_DEVICE_INDEX_INTERNAL_DSS_MSS: for DSS communication \n
 *         RL_DEVICE_INDEX_INTERNAL_HOST: for SPI communication \n
 * 2. When Running On DSS: \n
 *         RL_DEVICE_INDEX_INTERNAL_BSS: for radarSS communication \n
 *         RL_DEVICE_INDEX_INTERNAL_DSS_MSS: for MSS communication \n
 */
/* !< When calling native API */
/* !< AWR1243 Cascaded Device Id 1 */
/* !< AWR1243 Cascaded Device Id 2 */
/* !< AWR1243 Cascaded Device Id 3 */
/* !< AWR1243 Cascaded Device Id 4 */
/* AWR1243 Device Map - Max Cascading */
/* Device Index for SubSystem */
/* !< xWR1443/xWR1642/xWR1843 radarSS */
/* !< xWR1642/xWR1843 DSS/MSS */
/* !< xWR1443/xWR1642/xWR1843 External
                                                                 Host */
/* !< xWR1443/xWR1642/xWR1843
                                                                         radarSS */
/* !< xWR1642/xWR1843 DSS/MSS */
/* !< xWR1443/xWR1642/xWR1843
                                                                         External Host */
/* ! \brief
* mmWaveLink CRC Types
*/
/* !< CCITT */
/* !< ISO 3309, Ethernet */
/* !< 64 Bit ISO */
/* !< CRC Disabled */
/* ! \brief
* mmWaveLink Execution Home type
*/
/* !< running on Ext. Host */
/* !< running on MSS core */
/* !< running on DSS core */
/* ! \brief
* mmWave Device Types
*/
/* !< mmWavelink device 12XX */
/* !< mmWavelink device 14XX */
/* !< mmWavelink device 16XX */
/* !< mmWavelink device 18XX */
/* !< mmWavelink device 68XX */
/* ! \brief
* mmWaveLink Debug Levels
*/
/* ! \brief
* GPADC sensors macros
*/
/* ! \brief
* Swap 2 half words in 32 bit Integer. Required for Big Endian Host
*/
/* *****************************************************************************
 * TYPE-DEFINE STRUCT/ENUM/UNION DEFINITIONS
 ******************************************************************************
 */
/* DesignId : MMWL_DesignId_001 */
/* Requirements : AUTORADAR_REQ-697, AUTORADAR_REQ-698, AUTORADAR_REQ-699, AUTORADAR_REQ-700,
                  AUTORADAR_REQ-701, AUTORADAR_REQ-702, AUTORADAR_REQ-703, AUTORADAR_REQ-704,
                  AUTORADAR_REQ-705, AUTORADAR_REQ-706, AUTORADAR_REQ-830, AUTORADAR_REQ-831,
                  AUTORADAR_REQ-832, AUTORADAR_REQ-889, AUTORADAR_REQ-890, AUTORADAR_REQ-1002,
                  AUTORADAR_REQ-1006
*/
/* ! \brief
* mmWaveLink API return type
*/
pub type rlReturnVal_t = rlInt32_t;
/* ! \brief
 * Structure to hold the BSS ESM Fault data strucutre for event RL_RF_AE_ESMFAULT_SB
 * @note : The Programmable filter Parity error and double bit ECC errors are fatal errors but
 *         connected to ESM Group 1 lines, these shuld be treated as fatal errors on the Host .
 *         (MSS or DSS in xWR1642, xWR1843 and xWR6843 devices)
 */
#[derive(Copy, Clone)]
#[repr(C)]
pub struct rlBssEsmFault {
    pub esmGrp1Err: rlUInt32_t,
    pub esmGrp2Err: rlUInt32_t,
}
pub type rlBssEsmFault_t = rlBssEsmFault;
/* ! \brief
 * Structure to hold the MSS/radarSS CPU Fault data strucutre for
 * event RL_DEV_AE_MSS_CPUFAULT_SB and RL_RF_AE_CPUFAULT_SB
 * @note : All the Monitoring Async events will be sent out periodically at calibMonTimeUnit
 *         frame rate (FTTI). The RadarSS/BSS has a queue to hold max 8 transmit API messages
 *         (AEs or Responses), the host shall service all the AEs before start of the next FTTI
 *         epoch to avoid RadarSS Queue full CPU fault fatal error.
 */
#[derive(Copy, Clone)]
#[repr(C)]
pub struct rlCpuFault {
    pub faultType: rlUInt8_t,
    pub reserved0: rlUInt8_t,
    pub lineNum: rlUInt16_t,
    pub faultLR: rlUInt32_t,
    pub faultPrevLR: rlUInt32_t,
    pub faultSpsr: rlUInt32_t,
    pub faultSp: rlUInt32_t,
    pub faultAddr: rlUInt32_t,
    pub faultErrStatus: rlUInt16_t,
    pub faultErrSrc: rlUInt8_t,
    pub faultAxiErrType: rlUInt8_t,
    pub faultAccType: rlUInt8_t,
    pub faultRecovType: rlUInt8_t,
    pub reserved1: rlUInt16_t,
}
pub type rlCpuFault_t = rlCpuFault;
/* ***************************************************************************************
 * FileName     : rl_sensor.h
 *
 * Description  : This file defines the functions to configure RF/Sensor in mmwave radar device.
 *
 ****************************************************************************************
 * (C) Copyright 2014, Texas Instruments Incorporated. - TI web address www.ti.com
 *---------------------------------------------------------------------------------------
 *
 *  Redistribution and use in source and binary forms, with or without modification,
 *  are permitted provided that the following conditions are met:
 *
 *    Redistributions of source code must retain the above copyright notice,
 *    this list of conditions and the following disclaimer.
 *
 *    Redistributions in binary form must reproduce the above copyright notice,
 *    this list of conditions and the following disclaimer in the documentation
 *    and/or other materials provided with the distribution.
 *
 *    Neither the name of Texas Instruments Incorporated nor the names of its
 *    contributors may be used to endorse or promote products derived from this
 *    software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 *  AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 *  THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR
 *  PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT  OWNER OR CONTRIBUTORS
 *  BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 *  CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 *  SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 *  INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 *  CONTRACT,  STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 *  ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 */
/* ***************************************************************************************
* FILE INCLUSION PROTECTION
****************************************************************************************
*/
/* *****************************************************************************
 * INCLUDE FILES
 ******************************************************************************
 */
/* ***************************************************************************************
 * MACRO DEFINITIONS
 ****************************************************************************************
 */
/* Count of Test Sources*/
/* Number of chunks in Calibration data generated by radarSS */
/* ! \brief
* Supported maximum number of RX channels
*/
/* ! \brief
* Supported maximum number of TX channels
*/
/* ! \brief
* Enable/Disable RX/TX Channels
*/
/* ! \brief
* Start/Stop Frame Trigger constants
*/
/* ! \brief
* Supported maximum number of subframes in a frame
*/
/* ! \brief
* Supported maximum number of chirp profiles
*/
/* ! \brief
* Supported noise figure modes
*/
/* ! \brief
* Supported ADC data bitwidths
*/
/* ! \brief
* Supported ADC modes of operation
*/
/* ! \brief
* Supported ADC modes of operation
*/
/* ! \brief
* Supported Rx baseband bandwidths
*/
/* ! \brief
* Supported HPF-1 corner frequencies
*/
/* ! \brief
* Supported HPF-2 corner frequencies
*/
/* ! \brief
* Supported Rx gains
*/
/* ! \brief
* Supported frame sync modes
*/
/* ! \brief
* Supported frame start trigger modes
*/
/* ! \brief
* Supported Rx analog configurations
*/
/* ! \brief
* Supported Temperature Sensor Range in step of 10C
*/
/* *****************************************************************************
 * GLOBAL VARIABLES/DATA-TYPES DEFINITIONS
 ******************************************************************************
 */
/* ! \brief
* Rx/Tx Channel Configuration
*/
#[derive(Copy, Clone)]
#[repr(C)]
pub struct rlChanCfg {
    pub rxChannelEn: rlUInt16_t,
    pub txChannelEn: rlUInt16_t,
    pub cascading: rlUInt16_t,
    pub cascadingPinoutCfg: rlUInt16_t,
}
pub type rlChanCfg_t = rlChanCfg;
/* ! \brief
* ADC Bit and ADC Output format Configuration
*/
#[derive(Copy, Clone, BitfieldStruct)]
#[repr(C)]
pub struct rlAdcBitFormat {
    #[bitfield(name = "b2AdcBits", ty = "rlUInt32_t", bits = "0..=1")]
    #[bitfield(name = "b6Reserved0", ty = "rlUInt32_t", bits = "2..=7")]
    #[bitfield(name = "b8FullScaleReducFctr", ty = "rlUInt32_t", bits =
               "8..=15")]
    #[bitfield(name = "b2AdcOutFmt", ty = "rlUInt32_t", bits = "16..=17")]
    #[bitfield(name = "b14Reserved1", ty = "rlUInt32_t", bits = "18..=31")]
    pub b2AdcBits_b6Reserved0_b8FullScaleReducFctr_b2AdcOutFmt_b14Reserved1: [u8; 4],
}
pub type rlAdcBitFormat_t = rlAdcBitFormat;
/* ! \brief
* ADC format and payload justification Configuration
*/
#[derive(Copy, Clone)]
#[repr(C)]
pub struct rlAdcOutCfg {
    pub fmt: rlAdcBitFormat_t,
    pub reserved0: rlUInt16_t,
    pub reserved1: rlUInt16_t,
}
pub type rlAdcOutCfg_t = rlAdcOutCfg;
/* ! \brief
* Binary phase modulation mode configuration
*/
#[derive(Copy, Clone, BitfieldStruct)]
#[repr(C)]
pub struct rlBpmModeCfg {
    #[bitfield(name = "b2SrcSel", ty = "rlUInt16_t", bits = "0..=1")]
    #[bitfield(name = "b1Reserved0", ty = "rlUInt16_t", bits = "2..=2")]
    #[bitfield(name = "b13Reserved1", ty = "rlUInt16_t", bits = "3..=15")]
    pub b2SrcSel_b1Reserved0_b13Reserved1: [u8; 2],
}
pub type rlBpmModeCfg_t = rlBpmModeCfg;
/* ! \brief
* Binary phase modulation common configuration
*/
#[derive(Copy, Clone)]
#[repr(C)]
pub struct rlBpmCommonCfg {
    pub mode: rlBpmModeCfg_t,
    pub reserved0: rlUInt16_t,
    pub reserved1: rlUInt16_t,
    pub reserved2: rlUInt16_t,
    pub reserved3: rlUInt32_t,
    pub reserved4: rlUInt32_t,
}
pub type rlBpmCommonCfg_t = rlBpmCommonCfg;
/* ! \brief
* Binary phase modulation common configuration
*/
#[derive(Copy, Clone)]
#[repr(C)]
pub struct rlBpmChirpCfg {
    pub chirpStartIdx: rlUInt16_t,
    pub chirpEndIdx: rlUInt16_t,
    pub constBpmVal: rlUInt16_t,
    pub reserved: rlUInt16_t,
}
pub type rlBpmChirpCfg_t = rlBpmChirpCfg;
/* ! \brief
* Low Power mode configuration
*/
#[derive(Copy, Clone)]
#[repr(C)]
pub struct rlLowPowerModeCfg {
    pub reserved: rlUInt16_t,
    pub lpAdcMode: rlUInt16_t,
}
pub type rlLowPowerModeCfg_t = rlLowPowerModeCfg;
/* ! \brief
* Power saving mode configuration
*/
#[derive(Copy, Clone)]
#[repr(C)]
pub struct rlPowerSaveModeCfg {
    pub lowPwrStateTransCmd: rlUInt16_t,
    pub reserved0: rlUInt16_t,
    pub reserved: [rlUInt32_t; 4],
}
pub type rlPowerSaveModeCfg_t = rlPowerSaveModeCfg;
/* ! \brief
* Profile config API parameters. A profile contains coarse parameters of FMCW chirp such as
* start frequency, chirp slope, ramp time, idle time etc. Fine dithering values need
* to be programmed in chirp configuration \ref rlChirpCfg_t
* \note Maximum of 4 profiles can be configured.
*/
#[derive(Copy, Clone)]
#[repr(C)]
pub struct rlProfileCfg {
    pub profileId: rlUInt16_t,
    pub pfVcoSelect: rlUInt8_t,
    pub pfCalLutUpdate: rlUInt8_t,
    pub startFreqConst: rlUInt32_t,
    pub idleTimeConst: rlUInt32_t,
    pub adcStartTimeConst: rlUInt32_t,
    pub rampEndTime: rlUInt32_t,
    pub txOutPowerBackoffCode: rlUInt32_t,
    pub txPhaseShifter: rlUInt32_t,
    pub freqSlopeConst: rlInt16_t,
    pub txStartTime: rlInt16_t,
    pub numAdcSamples: rlUInt16_t,
    pub digOutSampleRate: rlUInt16_t,
    pub hpfCornerFreq1: rlUInt8_t,
    pub hpfCornerFreq2: rlUInt8_t,
    pub txCalibEnCfg: rlUInt16_t,
    pub rxGain: rlUInt16_t,
    pub reserved: rlUInt16_t,
}
pub type rlProfileCfg_t = rlProfileCfg;
/* ! \brief
* Chirp config API parameters. This structure contains fine dithering to coarse profile
* defined in \ref rlProfileCfg_t. It also includes the selection of Transmitter and
* binary phase modulation for a chirp.\n
* @note : One can define upto 512 unique chirps.These chirps need to be included in
*         frame configuration structure \ref rlFrameCfg_t to create FMCW frame
*/
#[derive(Copy, Clone)]
#[repr(C)]
pub struct rlChirpCfg {
    pub chirpStartIdx: rlUInt16_t,
    pub chirpEndIdx: rlUInt16_t,
    pub profileId: rlUInt16_t,
    pub reserved: rlUInt16_t,
    pub startFreqVar: rlUInt32_t,
    pub freqSlopeVar: rlUInt16_t,
    pub idleTimeVar: rlUInt16_t,
    pub adcStartTimeVar: rlUInt16_t,
    pub txEnable: rlUInt16_t,
}
pub type rlChirpCfg_t = rlChirpCfg;
/* ! \brief
* Chirp start, end Index parameters for rlGetChirpConfig
*/
#[derive(Copy, Clone)]
#[repr(C)]
pub struct rlWordParam {
    pub halfWordOne: rlUInt16_t,
    pub halfWordTwo: rlUInt16_t,
}
pub type rlWordParam_t = rlWordParam;
/* ! \brief
* Frame config API parameters
*/
#[derive(Copy, Clone)]
#[repr(C)]
pub struct rlFrameCfg {
    pub reserved0: rlUInt16_t,
    pub chirpStartIdx: rlUInt16_t,
    pub chirpEndIdx: rlUInt16_t,
    pub numLoops: rlUInt16_t,
    pub numFrames: rlUInt16_t,
    pub numAdcSamples: rlUInt16_t,
    pub framePeriodicity: rlUInt32_t,
    pub triggerSelect: rlUInt16_t,
    pub reserved1: rlUInt8_t,
    pub numDummyChirpsAtEnd: rlUInt8_t,
    pub frameTriggerDelay: rlUInt32_t,
}
pub type rlFrameCfg_t = rlFrameCfg;
/* ! \brief
 * Advance Frame config API Subframe configuration
 */
#[derive(Copy, Clone)]
#[repr(C)]
pub struct rlSubFrameCfg {
    pub forceProfileIdx: rlUInt16_t,
    pub chirpStartIdx: rlUInt16_t,
    pub numOfChirps: rlUInt16_t,
    pub numLoops: rlUInt16_t,
    pub burstPeriodicity: rlUInt32_t,
    pub chirpStartIdxOffset: rlUInt16_t,
    pub numOfBurst: rlUInt16_t,
    pub numOfBurstLoops: rlUInt16_t,
    pub reserved0: rlUInt16_t,
    pub subFramePeriodicity: rlUInt32_t,
    pub reserved1: rlUInt32_t,
    pub reserved2: rlUInt32_t,
}
pub type rlSubFrameCfg_t = rlSubFrameCfg;
/* ! \brief
 * Advance Frame Sequence config API parameters rlAdvFrameCfg, 148 bytes
 */
#[derive(Copy, Clone)]
#[repr(C)]
pub struct rlAdvFrameSeqCfg {
    pub numOfSubFrames: rlUInt8_t,
    pub forceProfile: rlUInt8_t,
    pub loopBackCfg: rlUInt8_t,
    pub subFrameTrigger: rlUInt8_t,
    pub subFrameCfg: [rlSubFrameCfg_t; 4],
    pub numFrames: rlUInt16_t,
    pub triggerSelect: rlUInt16_t,
    pub frameTrigDelay: rlUInt32_t,
    pub reserved0: rlUInt32_t,
    pub reserved1: rlUInt32_t,
}
pub type rlAdvFrameSeqCfg_t = rlAdvFrameSeqCfg;
/* ! \brief
* Frame config API parameters
*/
#[derive(Copy, Clone)]
#[repr(C)]
pub struct rlFrameApplyCfg {
    pub numChirps: rlUInt32_t,
    pub numAdcSamples: rlUInt16_t,
    pub reserved: rlUInt16_t,
}
pub type rlFrameApplyCfg_t = rlFrameApplyCfg;
/* ! \brief
* Sub Frame data config API parameters
*/
#[derive(Copy, Clone)]
#[repr(C)]
pub struct rlSubFrameDataCfg {
    pub totalChirps: rlUInt32_t,
    pub numAdcSamples: rlUInt16_t,
    pub numChirpsInDataPacket: rlUInt8_t,
    pub reserved: rlUInt8_t,
}
pub type rlSubFrameDataCfg_t = rlSubFrameDataCfg;
/* ! \brief
* Advance Frame data config API parameters.
* This structure is only applicable when mmWaveLink instance is running on
* External Host and connected to AWR1243 device.
*/
#[derive(Copy, Clone)]
#[repr(C)]
pub struct rlAdvFrameDataCfg {
    pub numSubFrames: rlUInt8_t,
    pub reserved0: rlUInt8_t,
    pub reserved1: rlUInt16_t,
    pub subframeDataCfg: [rlSubFrameDataCfg_t; 4],
}
pub type rlAdvFrameDataCfg_t = rlAdvFrameDataCfg;
/* ! \brief
 * Advance Frame Sequence config API parameters rlAdvFrameCfg, 148 bytes
 */
#[derive(Copy, Clone)]
#[repr(C)]
pub struct rlAdvFrameCfg {
    pub frameSeq: rlAdvFrameSeqCfg_t,
    pub frameData: rlAdvFrameDataCfg_t,
}
pub type rlAdvFrameCfg_t = rlAdvFrameCfg;
/* ! \brief
* Continous Mode config API parameters
*/
#[derive(Copy, Clone)]
#[repr(C)]
pub struct rlContModeCfg {
    pub startFreqConst: rlUInt32_t,
    pub txOutPowerBackoffCode: rlUInt32_t,
    pub txPhaseShifter: rlUInt32_t,
    pub digOutSampleRate: rlUInt16_t,
    pub hpfCornerFreq1: rlUInt8_t,
    pub hpfCornerFreq2: rlUInt8_t,
    pub rxGain: rlUInt8_t,
    pub vcoSelect: rlUInt8_t,
    pub reserved: rlUInt16_t,
}
pub type rlContModeCfg_t = rlContModeCfg;
/* ! \brief
* Continous Mode Enable API parameters
*/
#[derive(Copy, Clone)]
#[repr(C)]
pub struct rlContModeEn {
    pub contModeEn: rlUInt16_t,
    pub reserved: rlUInt16_t,
}
pub type rlContModeEn_t = rlContModeEn;
/* ! \brief
* Frame Trigger API parameters RL_RF_FRAMESTARTSTOP_CONF_SB
*/
#[derive(Copy, Clone)]
#[repr(C)]
pub struct rlFrameTrigger {
    pub startStop: rlUInt16_t,
    pub reserved: rlUInt16_t,
}
pub type rlFrameTrigger_t = rlFrameTrigger;
/* ! \brief
* The Object position and signal strength parameter structure
*/
#[derive(Copy, Clone)]
#[repr(C)]
pub struct rlTestSourceObject {
    pub posX: rlInt16_t,
    pub posY: rlInt16_t,
    pub posZ: rlInt16_t,
    pub velX: rlInt16_t,
    pub velY: rlInt16_t,
    pub velZ: rlInt16_t,
    pub sigLvl: rlUInt16_t,
    pub posXMin: rlInt16_t,
    pub posYMin: rlInt16_t,
    pub posZMin: rlInt16_t,
    pub posXMax: rlInt16_t,
    pub posYMax: rlInt16_t,
    pub posZMax: rlInt16_t,
}
pub type rlTestSourceObject_t = rlTestSourceObject;
/* ! \brief
* The Antenna position parameter structure
*/
#[derive(Copy, Clone)]
#[repr(C)]
pub struct rlTestSourceAntPos {
    pub antPosX: rlInt8_t,
    pub antPosZ: rlInt8_t,
}
pub type rlTestSourceAntPos_t = rlTestSourceAntPos;
/* ! \brief
* Test source config API parameters E_API_AR_TEST_SOURCE_CONF_SB
*/
#[derive(Copy, Clone)]
#[repr(C)]
pub struct rlTestSource {
    pub testObj: [rlTestSourceObject_t; 2],
    pub rxAntPos: [rlTestSourceAntPos_t; 4],
    pub reserved0: rlUInt32_t,
    pub reserved1: rlUInt32_t,
}
pub type rlTestSource_t = rlTestSource;
/* ! \brief
* Test source Enable API parameters RL_RF_TEST_SOURCE_ENABLE_SB
*/
#[derive(Copy, Clone)]
#[repr(C)]
pub struct rlTestSourceEn {
    pub tsEnable: rlUInt16_t,
    pub reserved: rlUInt16_t,
}
pub type rlTestSourceEnable_t = rlTestSourceEn;
/* ! \brief
* RF characterization Time and Temperature data structure
*/
#[derive(Copy, Clone)]
#[repr(C)]
pub struct rlRfTempData {
    pub time: rlUInt32_t,
    pub tmpRx0Sens: rlInt16_t,
    pub tmpRx1Sens: rlInt16_t,
    pub tmpRx2Sens: rlInt16_t,
    pub tmpRx3Sens: rlInt16_t,
    pub tmpTx0Sens: rlInt16_t,
    pub tmpTx1Sens: rlInt16_t,
    pub tmpTx2Sens: rlInt16_t,
    pub tmpPmSens: rlInt16_t,
    pub tmpDig0Sens: rlInt16_t,
    pub tmpDig1Sens: rlInt16_t,
}
pub type rlRfTempData_t = rlRfTempData;
/* ! \brief
* The DFE Statistics for Rx Channel for particular profile
*/
#[derive(Copy, Clone)]
#[repr(C)]
pub struct rlDfeRxdStatReport {
    pub iAvgDC: rlInt16_t,
    pub qAvgDC: rlInt16_t,
    pub iAvgPwr: rlUInt16_t,
    pub qAvgPwr: rlUInt16_t,
    pub iqAvgCroCorrel: rlInt32_t,
}
pub type rlDfeRxStatReport_t = rlDfeRxdStatReport;
/* ! \brief
* The DFE Statistics Report Contents
*/
#[derive(Copy, Clone)]
#[repr(C)]
pub struct rlDfeStatReport {
    pub dfeStatRepo: [[rlDfeRxStatReport_t; 4]; 4],
}
pub type rlDfeStatReport_t = rlDfeStatReport;
/* ! \brief
* Dynamic power saving API parameters
*/
#[derive(Copy, Clone)]
#[repr(C)]
pub struct rlDynPwrSave {
    pub blkCfg: rlUInt16_t,
    pub reserved: rlUInt16_t,
}
pub type rlDynPwrSave_t = rlDynPwrSave;
/* ! \brief
* API RF device Config SBC M_API_AR_RF_DEV_CONF_SBC
*/
#[derive(Copy, Clone)]
#[repr(C)]
pub struct rlRfDevCfg {
    pub aeDirection: rlUInt32_t,
    pub aeControl: rlUInt8_t,
    pub reserved0: rlUInt8_t,
    pub reserved1: rlUInt8_t,
    pub bssDigCtrl: rlUInt8_t,
    pub aeCrcConfig: rlUInt8_t,
    pub reserved2: rlUInt8_t,
    pub reserved3: rlUInt16_t,
}
pub type rlRfDevCfg_t = rlRfDevCfg;
/* ! \brief
*  Num of samples to collect for API GPADC sensors
*  sampleCnt    : Number of samples to collect @625KHz
*/
#[derive(Copy, Clone)]
#[repr(C)]
pub struct rlGpAdcSamples {
    pub sampleCnt: rlUInt8_t,
    pub settlingTime: rlUInt8_t,
}
pub type rlGpAdcSamples_t = rlGpAdcSamples;
/* ! \brief
* API radarSS GPADC API MEAS SET SBC M_API_AR_RF_GPADC_API_SET_SB
*/
#[derive(Copy, Clone)]
#[repr(C)]
pub struct rlGpAdcCfg {
    pub enable: rlUInt8_t,
    pub bufferEnable: rlUInt8_t,
    pub numOfSamples: [rlGpAdcSamples_t; 6],
    pub reserved0: rlUInt16_t,
    pub reserved1: [rlUInt32_t; 3],
}
pub type rlGpAdcCfg_t = rlGpAdcCfg;
/* ! \brief
* Radar RF LDO bypass enable/disable configuration
*/
#[derive(Copy, Clone)]
#[repr(C)]
pub struct rlRfLdoBypassCfg {
    pub ldoBypassEnable: rlUInt16_t,
    pub supplyMonIrDrop: rlUInt8_t,
    pub ioSupplyIndicator: rlUInt8_t,
}
pub type rlRfLdoBypassCfg_t = rlRfLdoBypassCfg;
/* ! \brief
* Radar RF Phase Shift enable/disable configuration
*/
#[derive(Copy, Clone)]
#[repr(C)]
pub struct rlRfPhaseShiftCfg {
    pub chirpStartIdx: rlUInt16_t,
    pub chirpEndIdx: rlUInt16_t,
    pub tx0PhaseShift: rlUInt8_t,
    pub tx1PhaseShift: rlUInt8_t,
    pub tx2PhaseShift: rlUInt8_t,
    pub reserved: rlUInt8_t,
}
pub type rlRfPhaseShiftCfg_t = rlRfPhaseShiftCfg;
/* ! \brief
* Radar RF PA loopback configuration
*/
#[derive(Copy, Clone)]
#[repr(C)]
pub struct rlRfPALoopbackCfg {
    pub paLoopbackFreq: rlUInt16_t,
    pub paLoopbackEn: rlUInt8_t,
    pub reserved: rlUInt8_t,
}
pub type rlRfPALoopbackCfg_t = rlRfPALoopbackCfg;
/* ! \brief
* Radar RF Phase shift loopback configuration
*/
#[derive(Copy, Clone)]
#[repr(C)]
pub struct rlRfPSLoopbackCfg {
    pub psLoopbackFreq: rlUInt16_t,
    pub reserved0: rlUInt16_t,
    pub psLoopbackEn: rlUInt8_t,
    pub psLoopbackTxId: rlUInt8_t,
    pub pgaGainIndex: rlUInt8_t,
    pub reserved1: rlUInt8_t,
}
pub type rlRfPSLoopbackCfg_t = rlRfPSLoopbackCfg;
/* ! \brief
* Radar RF IF loopback configuration.
*/
#[derive(Copy, Clone)]
#[repr(C)]
pub struct rlRfIFLoopbackCfg {
    pub ifLoopbackFreq: rlUInt16_t,
    pub ifLoopbackEn: rlUInt8_t,
    pub reserved: rlUInt8_t,
}
pub type rlRfIFLoopbackCfg_t = rlRfIFLoopbackCfg;
/* ! \brief
* Array of coefficients for the RF programmable filter
*/
#[derive(Copy, Clone)]
#[repr(C)]
pub struct rlRfProgFiltCoeff {
    pub coeffArray: [rlInt16_t; 104],
}
pub type rlRfProgFiltCoeff_t = rlRfProgFiltCoeff;
/* ! \brief
* Radar RF programmable filter configuration
*/
#[derive(Copy, Clone)]
#[repr(C)]
pub struct rlRfProgFiltConf {
    pub profileId: rlUInt8_t,
    pub coeffStartIdx: rlUInt8_t,
    pub progFiltLen: rlUInt8_t,
    pub progFiltFreqShift: rlUInt8_t,
}
pub type rlRfProgFiltConf_t = rlRfProgFiltConf;
/* ! \brief
* Radar RF Miscconfiguration
*/
#[derive(Copy, Clone)]
#[repr(C)]
pub struct rlRfMiscConf {
    pub miscCtl: rlUInt32_t,
    pub reserved: rlUInt32_t,
}
pub type rlRfMiscConf_t = rlRfMiscConf;
/* ! \brief
* Radar RF Calibration monitoring time unit configuration
*/
#[derive(Copy, Clone)]
#[repr(C)]
pub struct rlRfCalMonTimeUntConf {
    pub calibMonTimeUnit: rlUInt16_t,
    pub numOfCascadeDev: rlUInt8_t,
    pub devId: rlUInt8_t,
    pub reserved: rlUInt32_t,
}
pub type rlRfCalMonTimeUntConf_t = rlRfCalMonTimeUntConf;
/* ! \brief
* Radar RF Calibration monitoring Frequency Limit configuration
*/
#[derive(Copy, Clone)]
#[repr(C)]
pub struct rlRfCalMonFreqLimitConf {
    pub freqLimitLow: rlUInt16_t,
    pub freqLimitHigh: rlUInt16_t,
    pub reserved0: rlUInt32_t,
    pub reserved1: rlUInt32_t,
}
pub type rlRfCalMonFreqLimitConf_t = rlRfCalMonFreqLimitConf;
/* ! \brief
* Radar RF Init Calibration configuration
*/
#[derive(Copy, Clone)]
#[repr(C)]
pub struct rlRfInitCalConf {
    pub calibEnMask: rlUInt32_t,
    pub reserved0: rlUInt8_t,
    pub reserved1: rlUInt8_t,
    pub reserved2: rlUInt16_t,
    pub reserved3: rlUInt32_t,
}
pub type rlRfInitCalConf_t = rlRfInitCalConf;
/* ! \brief
* Radar RF Run time calibration configuration
*/
#[derive(Copy, Clone)]
#[repr(C)]
pub struct rlRunTimeCalibConf {
    pub oneTimeCalibEnMask: rlUInt32_t,
    pub periodicCalibEnMask: rlUInt32_t,
    pub calibPeriodicity: rlUInt32_t,
    pub reportEn: rlUInt8_t,
    pub reserved0: rlUInt8_t,
    pub txPowerCalMode: rlUInt8_t,
    pub reserved1: rlUInt8_t,
    pub reserved2: rlUInt32_t,
}
pub type rlRunTimeCalibConf_t = rlRunTimeCalibConf;
/* ! \brief
* RX gain temperature LUT read
*/
#[derive(Copy, Clone)]
#[repr(C)]
pub struct rlRxGainTempLutReadReq {
    pub profileIndx: rlUInt8_t,
    pub reserved0: rlUInt8_t,
    pub reserved1: rlUInt16_t,
}
pub type rlRxGainTempLutReadReq_t = rlRxGainTempLutReadReq;
/* ! \brief
* TX gain temperature LUT read
*/
#[derive(Copy, Clone)]
#[repr(C)]
pub struct rlTxGainTempLutReadReq {
    pub profileIndx: rlUInt8_t,
    pub reserved0: rlUInt8_t,
    pub reserved1: rlUInt16_t,
}
pub type rlTxGainTempLutReadReq_t = rlTxGainTempLutReadReq;
/* ! \brief
* RX gain temperature LUT inject
*/
#[derive(Copy, Clone)]
#[repr(C)]
pub struct rlRxGainTempLutData {
    pub profileIndx: rlUInt8_t,
    pub reserved0: rlUInt8_t,
    pub rxGainTempLut: [rlUInt8_t; 20],
    pub reserved1: rlUInt16_t,
}
pub type rlRxGainTempLutData_t = rlRxGainTempLutData;
/* ! \brief
* TX gain temperature LUT inject
*/
#[derive(Copy, Clone)]
#[repr(C)]
pub struct rlTxGainTempLutData {
    pub profileIndx: rlUInt8_t,
    pub reserved0: rlUInt8_t,
    pub txGainTempLut: [[rlUInt8_t; 20]; 3],
    pub reserved1: rlUInt16_t,
}
pub type rlTxGainTempLutData_t = rlTxGainTempLutData;
/* ! \brief
* Tx freq and power limit configuration
*/
#[derive(Copy, Clone)]
#[repr(C)]
pub struct rlRfTxFreqPwrLimitMonConf {
    pub freqLimitLowTx0: rlUInt16_t,
    pub freqLimitLowTx1: rlUInt16_t,
    pub freqLimitLowTx2: rlUInt16_t,
    pub freqLimitHighTx0: rlUInt16_t,
    pub freqLimitHighTx1: rlUInt16_t,
    pub freqLimitHighTx2: rlUInt16_t,
    pub tx0PwrBackOff: rlUInt8_t,
    pub tx1PwrBackOff: rlUInt8_t,
    pub tx2PwrBackOff: rlUInt8_t,
    pub reserved0: rlUInt8_t,
    pub reserved1: rlUInt16_t,
    pub reserved2: rlUInt16_t,
    pub reserved3: rlUInt16_t,
    pub reserved4: rlUInt16_t,
}
pub type rlRfTxFreqPwrLimitMonConf_t = rlRfTxFreqPwrLimitMonConf;
/* ! \brief
* Loopback burst set configuration
*/
#[derive(Copy, Clone)]
#[repr(C)]
pub struct rlLoopbackBurst {
    pub loopbackSel: rlUInt8_t,
    pub baseProfileIndx: rlUInt8_t,
    pub burstIndx: rlUInt8_t,
    pub reserved0: rlUInt8_t,
    pub freqConst: rlUInt32_t,
    pub slopeConst: rlInt16_t,
    pub reserved1: rlUInt16_t,
    pub txBackoff: rlUInt32_t,
    pub rxGain: rlUInt16_t,
    pub txEn: rlUInt8_t,
    pub reserved2: rlUInt8_t,
    pub bpmConfig: rlUInt16_t,
    pub digCorrDis: rlUInt16_t,
    pub ifLbFreq: rlUInt8_t,
    pub ifLbMag: rlUInt8_t,
    pub ps1PgaIndx: rlUInt8_t,
    pub ps2PgaIndx: rlUInt8_t,
    pub psLbFreq: rlUInt32_t,
    pub reserved3: rlUInt32_t,
    pub paLbFreq: rlUInt16_t,
    pub reserved4: [rlUInt16_t; 3],
}
pub type rlLoopbackBurst_t = rlLoopbackBurst;
/* ! \brief
* Chirp row configuration, radarSS stores each chirp config in memory in 3 rows.
*/
#[derive(Copy, Clone)]
#[repr(C)]
pub struct rlChirpRow {
    pub chirpNR1: rlUInt32_t,
    pub chirpNR2: rlUInt32_t,
    pub chirpNR3: rlUInt32_t,
}
pub type rlChirpRow_t = rlChirpRow;
/* ! \brief
* Dynamic chirp configuration for 16 chirp configurations.
*/
#[derive(Copy, Clone)]
#[repr(C)]
pub struct rlDynChirpCfg {
    pub chirpRowSelect: rlUInt8_t,
    pub chirpSegSel: rlUInt8_t,
    pub programMode: rlUInt16_t,
    pub chirpRow: [rlChirpRow_t; 16],
}
pub type rlDynChirpCfg_t = rlDynChirpCfg;
/* ! \brief
* Dynamic chirp enable configuration
*/
#[derive(Copy, Clone)]
#[repr(C)]
pub struct rlDynChirpEnCfg {
    pub reserved: rlUInt32_t,
}
pub type rlDynChirpEnCfg_t = rlDynChirpEnCfg;
/* ! \brief
* Dynamic per chirp phase shifter configuration for each TX
*/
#[derive(Copy, Clone)]
#[repr(C)]
pub struct rlChirpPhShiftPerTx {
    pub chirpNTx0PhaseShifter: rlUInt8_t,
    pub chirpNTx1PhaseShifter: rlUInt8_t,
    pub chirpNTx2PhaseShifter: rlUInt8_t,
}
pub type rlChirpPhShiftPerTx_t = rlChirpPhShiftPerTx;
/* ! \brief
* Dynamic per chirp phase shifter configuration
*/
#[derive(Copy, Clone)]
#[repr(C)]
pub struct rlDynPerChirpPhShftCfg {
    pub reserved: rlUInt8_t,
    pub chirpSegSel: rlUInt8_t,
    pub phShiftPerTx: [rlChirpPhShiftPerTx_t; 16],
    pub programMode: rlUInt16_t,
}
pub type rlDynPerChirpPhShftCfg_t = rlDynPerChirpPhShftCfg;
/* ! \brief
* Get calibration data configuration structure
*/
#[derive(Copy, Clone)]
#[repr(C)]
pub struct rlCalDataGetCfg {
    pub reserved: rlUInt16_t,
    pub chunkId: rlUInt16_t,
}
pub type rlCalDataGetCfg_t = rlCalDataGetCfg;
/* ! \brief
* Calibration data which application will receive from radarSS and will feed in to the Device
* in next power up to avoid calibration.
*/
#[derive(Copy, Clone)]
#[repr(C)]
pub struct rlCalDataStore {
    pub numOfChunk: rlUInt16_t,
    pub chunkId: rlUInt16_t,
    pub calData: [rlUInt8_t; 224],
}
pub type rlCalDataStore_t = rlCalDataStore;
/* ! \brief
* Structure to store all Calibration data chunks which device provides in response of
* rlRfCalibDataStore API. Applcation needs to provide same structure to rlRfCalibDataRestore API
* to restore calibration data to the device. \n
* Accumulative calData for 3 chunks (3 * 224 bytes) looks like as mentioned. \n
* For xWR1243/xWR1443/xWR1642/xWR1843 devices:
* Field Name     Num.of bytes  Description \n
* calValidStatus     4         This field indicates the status of each calibration (0 – FAIL, 
*                              1 – PASS). If a particular calibration was not enabled, then its
*                              corresponding field should be ignored. \n
*                              Bit   Definition (0 – FAIL, 1 – PASS) \n
*                              b0    RESERVED \n
*                              b1    APLL tuning (Ignore while restore) \n
*                              b2    SYNTH VCO1 tuning (Ignore while restore) \n
*                              b3    SYNTH VCO2 tuning (Ignore while restore) \n
*                              b4    LODIST calibration (Ignore while restore) \n
*                              b5    RX ADC DC offset calibration \n
*                              b6    HPF cutoff calibration \n
*                              b7    LPF cutoff calibration \n
*                              b8    Peak detector calibration (optional) \n
*                              b9    TX Power calibration (optional) \n
*                              b10   RX gain calibration \n
*                              b11   TX Phase calibration (Ignore while restore) \n
*                              b12   RX IQMM calibration \n
*                              b31:13 RESERVED
*                              The recommended Validity status bits while restoring
*                              is 0x000014E0, assuming only rxAdcDcCalData,
*                              hpfCalData, lpfCalData, rxRfGainCalData and IQMM
*                              iqmmCalData are stored and restored. \n
* calValidStatusCpy 4          Redundant calValidStatus value, this value should match with
*                              calValidStatus. \n
* ifStageCalStatus  4          This field indicates the status of IF stage calibration (0 – FAIL,
*                              1 – PASS). If a particular calibration was not enabled, then its
*                              corresponding field should be ignored. \n
*                              Bit Definition (0 – FAIL, 1 – PASS) \n
*                              b0  HPF1 \n
*                              b1  HPF2 \n
*                              b2  LPF1 \n
*                              b3  LPF2 \n
*                              b31:4 RESERVED \n
*                              This value shall be set to 0xF if HPF and LPF
*                              calibration validity status are PASS. \n
* reserved          4          Reserved for Future use \n
* calTemperature    2          Temperature at which boot calibration is done. \n
* reserved          14         Reserved for Future use \n
* rxAdcDcCalibData  16         RX chain ADC DC calibration data \n
* hpf1CalData       1          HPF1 calibration data \n
* hpf2CalData       1          HPF2 calibration data \n
* reserved          2          Reserved for Future use \n
* lpf1CalData       24         LPF1 calibration data \n
* lpf2CalData       24         LPF2 calibration data \n
* rxRfGainCalData   12         RX RF gain calibration data \n
* iqmmCalibData     72         RX IQMM calibration data \n
* txPowCalData      84         TX Power calibration data \n
* powDetCalData     348        Power detector calibration data \n
* reserved          56         Reserved for Future use \n
*
* For xWR6843 device:
* Field Name     Num.of bytes  Description \n
* calValidStatus     4         This field indicates the status of each calibration (0 – FAIL, 
*                              1 – PASS). If a particular calibration was not enabled, then its
*                              corresponding field should be ignored. \n
*                              Bit   Definition (0 – FAIL, 1 – PASS) \n
*                              b0    RESERVED \n
*                              b1    APLL tuning (Ignore while restore) \n
*                              b2    SYNTH VCO1 tuning (Ignore while restore) \n
*                              b3    SYNTH VCO2 tuning (Ignore while restore) \n
*                              b4    LODIST calibration (Ignore while restore) \n
*                              b5    RX ADC DC offset calibration \n
*                              b6    HPF cutoff calibration \n
*                              b7    LPF cutoff calibration \n
*                              b8    Peak detector calibration (optional) \n
*                              b9    TX Power calibration (optional) \n
*                              b10   RX gain calibration \n
*                              b11   TX Phase calibration (optional) \n
*                              b12   RX IQMM calibration \n
*                              b31:13 RESERVED
*                              The recommended Validity status bits while restoring
*                              is 0x000014E0, assuming only rxAdcDcCalData,
*                              hpfCalData, lpfCalData, rxRfGainCalData and IQMM
*                              iqmmCalData are stored and restored. \n
* calValidStatusCpy 4          Redundant calValidStatus value, this value should match with
*                              calValidStatus. \n
* reserved          8          Reserved for Future use \n
* calTemperature    2          Temperature at which boot calibration is done. \n
* reserved          14         Reserved for Future use \n
* rxAdcDcCalibData  16         RX chain ADC DC calibration data \n
* hpf1CalData       1          HPF1 calibration data \n
* hpf2CalData       1          HPF2 calibration data \n
* loDistBiasCode    1          LODIST calibration data \n
* reserved          1          Reserved for Future use \n
* rxRfGainCalData   8          RX RF gain calibration data \n
* iqmmCalibData     104        RX IQMM calibration data \n
* txPowCalData      122        TX Power calibration data \n
* powDetCalData     344        Power detector calibration data \n
* reserved          42         Reserved for Future use \n
*/
#[derive(Copy, Clone)]
#[repr(C)]
pub struct rlCalibrationData {
    pub calibChunk: [rlCalDataStore_t; 3],
}
pub type rlCalibrationData_t = rlCalibrationData;
/* ! \brief
* Inter-Rx gain and phase offset configuration
*/
#[derive(Copy, Clone)]
#[repr(C)]
pub struct rlInterRxGainPhConf {
    pub profileIndx: rlUInt8_t,
    pub reserved0: rlUInt8_t,
    pub reserved1: rlUInt16_t,
    pub digRxGain: [rlInt8_t; 4],
    pub digRxPhShift: [rlUInt16_t; 4],
    pub reserved2: rlUInt32_t,
    pub reserved3: rlUInt32_t,
}
pub type rlInterRxGainPhConf_t = rlInterRxGainPhConf;
/* ! \brief
 * BSS Bootup status data structure
 */
#[derive(Copy, Clone)]
#[repr(C)]
pub struct rlRfBootStatusCfg {
    pub bssSysStatus: rlUInt32_t,
    pub bssBootUpTime: rlUInt32_t,
    pub reserved0: rlUInt32_t,
    pub reserved1: rlUInt32_t,
}
pub type rlRfBootStatusCfg_t = rlRfBootStatusCfg;
/* ! \brief
* Inter Chirp block control configuration
*/
#[derive(Copy, Clone)]
#[repr(C)]
pub struct rlInterChirpBlkCtrlCfg {
    pub rx02RfTurnOffTime: rlInt16_t,
    pub rx13RfTurnOffTime: rlInt16_t,
    pub rx02BbTurnOffTime: rlInt16_t,
    pub rx12BbTurnOffTime: rlInt16_t,
    pub rx02RfPreEnTime: rlInt16_t,
    pub rx13RfPreEnTime: rlInt16_t,
    pub rx02BbPreEnTime: rlInt16_t,
    pub rx13BbPreEnTime: rlInt16_t,
    pub rx02RfTurnOnTime: rlInt16_t,
    pub rx13RfTurnOnTime: rlInt16_t,
    pub rx02BbTurnOnTime: rlInt16_t,
    pub rx13BbTurnOnTime: rlInt16_t,
    pub rxLoChainTurnOffTime: rlInt16_t,
    pub txLoChainTurnOffTime: rlInt16_t,
    pub rxLoChainTurnOnTime: rlInt16_t,
    pub txLoChainTurnOnTime: rlInt16_t,
    pub reserved0: rlInt32_t,
    pub reserved1: rlInt32_t,
}
pub type rlInterChirpBlkCtrlCfg_t = rlInterChirpBlkCtrlCfg;
/* ! \brief
* Sub-frame trigger API
*/
#[derive(Copy, Clone)]
#[repr(C)]
pub struct rlSubFrameStartCfg {
    pub startCmd: rlUInt16_t,
    pub reserved: rlUInt16_t,
}
pub type rlSubFrameStartCfg_t = rlSubFrameStartCfg;
/* ! \brief
* Get phase shift calibration data configuration structure
*/
#[derive(Copy, Clone)]
#[repr(C)]
pub struct rlPhShiftCalDataGetCfg {
    pub txIndex: rlUInt8_t,
    pub reserved0: rlUInt8_t,
    pub reserved1: rlUInt16_t,
}
pub type rlPhShiftCalDataGetCfg_t = rlPhShiftCalDataGetCfg;
/* ! \brief
* Phase shift calibration data which application will receive from radarSS and will feed in to the
* Device in next power up to avoid calibration.
*/
#[derive(Copy, Clone)]
#[repr(C)]
pub struct rlPhShiftCalibrationStore {
    pub txIndex: rlUInt8_t,
    pub calibApply: rlUInt8_t,
    pub observedPhShiftData: [rlUInt8_t; 128],
    pub reserved: rlUInt16_t,
}
pub type rlPhShiftCalibrationStore_t = rlPhShiftCalibrationStore;
/* ! \brief
* Structure to store all Phase shifter calibration data chunks which device provides in response of
* rlRfPhShiftCalibDataStore API. Applcation needs to provide same structure to
* rlRfPhShiftCalibDataRestore API to restore calibration data to the device.
*/
#[derive(Copy, Clone)]
#[repr(C)]
pub struct rlPhShiftCalibrationData {
    pub PhShiftcalibChunk: [rlPhShiftCalibrationStore_t; 3],
}
pub type rlPhShiftCalibrationData_t = rlPhShiftCalibrationData;
/* ! \brief
* Die ID data structure
*/
#[derive(Copy, Clone)]
#[repr(C)]
pub struct rlRfDieIdCfg {
    pub lotNo: rlUInt32_t,
    pub waferNo: rlUInt32_t,
    pub devX: rlUInt32_t,
    pub devY: rlUInt32_t,
    pub reserved0: rlUInt32_t,
    pub reserved1: rlUInt32_t,
    pub reserved2: rlUInt32_t,
    pub reserved3: rlUInt32_t,
}
pub type rlRfDieIdCfg_t = rlRfDieIdCfg;
/* ! \brief
* APLL Synthesizer Bandwidth Control
*/
#[derive(Copy, Clone)]
#[repr(C)]
pub struct rlRfApllSynthBwControl {
    pub synthIcpTrim: rlUInt8_t,
    pub synthRzTrim: rlUInt8_t,
    pub apllIcpTrim: rlUInt8_t,
    pub apllRzTrimLpf: rlUInt8_t,
    pub apllRzTrimVco: rlUInt8_t,
    pub reserved0: rlUInt8_t,
    pub reserved: [rlUInt16_t; 5],
}
pub type rlRfApllSynthBwControl_t = rlRfApllSynthBwControl;
/* ! \brief
* mmwave radar Driver Payload
*/
pub type rlDriverMsg_t = rlDriverMsg;
#[derive(Copy, Clone)]
#[repr(C)]
pub struct rlDriverMsg {
    pub opcode: rlDriverOpcode_t,
    pub subblocks: *mut rlPayloadSb_t,
    pub remChunks: rlUInt16_t,
}
/* ! \brief
* Payload Byte Alighment
*/
/* MSG must be multiple of this value */
/* MSG must be multiple of this value */
/* ! \brief
* RHCP Payload Structure
*/
pub type rlPayloadSb_t = rlPayloadSb;
#[derive(Copy, Clone)]
#[repr(C)]
pub struct rlPayloadSb {
    pub sbid: rlUInt16_t,
    pub len: rlUInt16_t,
    pub pSblkData: *mut rlUInt8_t,
}
/* ! \brief
* mmwave radar Driver Opcode
*/
pub type rlDriverOpcode_t = rlDriverOpcode;
#[derive(Copy, Clone)]
#[repr(C)]
pub struct rlDriverOpcode {
    pub dir: rlUInt8_t,
    pub msgType: rlUInt8_t,
    pub msgId: rlUInt16_t,
    pub nsbc: rlUInt16_t,
}
/* *
     * @brief  Message Direction
     */
/* *
     * @brief  Message Class
     */
/* *
     * @brief  Message Id
     */
/* *
     * @brief  Number of Sub Blocks in Payload
     */
/* ***************************************************************************************
 * FileName     : rl_sensor.c
 *
 * Description  : This file defines the functions to configure RF/Sensor in mmwave radar device.
 *
 ****************************************************************************************
 * (C) Copyright 2014, Texas Instruments Incorporated. - TI web address www.ti.com
 *---------------------------------------------------------------------------------------
 *
 *  Redistribution and use in source and binary forms, with or without modification,
 *  are permitted provided that the following conditions are met:
 *
 *    Redistributions of source code must retain the above copyright notice,
 *    this list of conditions and the following disclaimer.
 *
 *    Redistributions in binary form must reproduce the above copyright notice,
 *    this list of conditions and the following disclaimer in the documentation
 *    and/or other materials provided with the distribution.
 *
 *    Neither the name of Texas Instruments Incorporated nor the names of its
 *    contributors may be used to endorse or promote products derived from this
 *    software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 *  AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 *  THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR
 *  PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT  OWNER OR CONTRIBUTORS
 *  BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 *  CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 *  SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 *  INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 *  CONTRACT,  STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 *  ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 */
/*
 ****************************************************************************************
 * Revision History   :
 *---------------------------------------------------------------------------------------
 * Version  Date        Author             Defect No               Description
 *---------------------------------------------------------------------------------------
 * 0.1.0    12May2015   Kaushal Kukkar    -               Initial Version
 *
 * 0.5.2    23Sep2016   Kaushal Kukkar    AUTORADAR-540   Advance Frame APIs
 *
 * 0.6.0    15Nov2016   Kaushal Kukkar    MMWSDK-206      Dynamic Power Save API
 *                      Kaushal Kukkar    AUTORADAR-571   Cascade Feature Support
 *
 * 0.7.0    11May2017   Jitendra Gupta    MMWSDK-450      Calibration Config/Report APIs
 *                      Jitendra Gupta    MMWSDK-322      PS/PA/IF Loopback APIs
 *
 * 0.8.6    24Jul2017   Jitendra Gupta    MMWL-30         RF/Analog Monitoring APIs
 *                      Kaushal Kukkar    MMWL-23         Big Endian Support
 *
 * 0.9.1       -        Jitendra Gupta    MMWL-5          Code size optimization
 ****************************************************************************************
 */
/* *****************************************************************************
 * INCLUDE FILES
 ******************************************************************************
 */
/* *****************************************************************************
 * GLOBAL VARIABLES/DATA-TYPES DEFINITIONS
 ******************************************************************************
 */
/* *****************************************************************************
 * FUNCTION DEFINITIONS
 ******************************************************************************
 */
/* * @fn rlReturnVal_t rlSetChannelConfig(rlUInt8_t deviceMap, rlChanCfg_t* data)
*
*   @brief Sets the Rx and Tx Channel Configuration
*   @param[in] deviceMap - Bitmap of devices to send the message
*   @param[in] data - Container for Channel Configuration data
*
*   @return rlReturnVal_t Success - 0, Failure - Error Code
*
*   This function allows configuration of mmWave Front end for how many Receiver and
*   Transmit channels need to be enabled. It also defines whether to to enable
*   single mmWave device or multiple mmWave devices to realize a larger antenna array
*   (multiple is applicable only in AWR1243). This is applicable for given power cycle.

*   @note This is global configuration for transmit channels. Later one can chose
*   which transmit channel to be used for each chirp using Chirp configuaration API.
*   For e.g - If Chirp 0, uses TX0 and TX1, and Chirp 1 uses TX1 and TX2, One need
*   to enable TX0, TX1 and TX2 in this API. Based on the configuration, mmWave Front
*   would do necessary calibration before the transmit channel is used to transmit
*   chirps
*   @note : The cascade features are not supported in DFP 1.x (1st generation devices). Please
*           refer latest release note for more info.
*   @ref rlSetChirpConfig
*/
/* DesignId : MMWL_DesignId_009 */
/* Requirements : AUTORADAR_REQ-752, AUTORADAR_REQ-757 */
#[no_mangle]
pub unsafe extern "C" fn rlSetChannelConfig(mut deviceMap: rlUInt8_t,
                                            mut data: *mut rlChanCfg_t)
 -> rlReturnVal_t {
    let mut retVal: rlReturnVal_t = 0;
    /* check if deviceIndex is out of defined value */
    if rlDriverIsDeviceMapValid(deviceMap) != 0 as libc::c_int ||
           data.is_null() {
        /* set return error code */
        retVal = -(2 as libc::c_int)
    } else {
        /* Package the command with given data and send it to device */
        retVal =
            rlDriverExecuteSetApi(deviceMap,
                                  0x4 as libc::c_uint as rlUInt16_t,
                                  0 as libc::c_uint as rlUInt16_t,
                                  data as *mut rlUInt8_t,
                                  ::std::mem::size_of::<rlChanCfg_t>() as
                                      libc::c_ulong as rlUInt16_t)
    }
    return retVal;
}
/* * @fn rlReturnVal_t rlSetAdcOutConfig(rlUInt8_t deviceMap, rlAdcOutCfg_t* data)
*
*   @brief Sets ADC Output Configuration
*   @param[in] deviceMap - Bitmap of devices to send the message
*   @param[in] data - Container for ADC Configuration data
*
*   @return rlReturnVal_t Success - 0, Failure - Error Code
*
*   This function sets the static device configuration for the data format of the ADC
*   and digital front end output. This is RAW ADC samples of IF signal and needs to
*   be processed by HW accelerator or DSP. The ADC data can be sent to external Processor
*   over High Speed Interface such as LVDS or CSI2. The ADC data size supported are
*   12, 14 and 16 bits and supported formats are Real, Complex 1x and Complex 2x.
*   In Complex 1x, Image band is filtered out and only signal band is sampled in ADC.
*   Where as in Complex 2x, Both Image and Signal band is sampled.\n Complex baseband
*   architecture results in better noise figure and is recommended.

*   @note At the same sampling frequency(Fs), Complex 1x would support IF bandwidth of Fs,
*   where as real and complex 2x would provide IF bandwidth of upto Fs/2.
*/
/* DesignId :  MMWL_DesignId_010 */
/* Requirements : AUTORADAR_REQ-753, AUTORADAR_REQ-754 */
#[no_mangle]
pub unsafe extern "C" fn rlSetAdcOutConfig(mut deviceMap: rlUInt8_t,
                                           mut data: *mut rlAdcOutCfg_t)
 -> rlReturnVal_t {
    let mut retVal: rlReturnVal_t = 0;
    /* check if deviceIndex is out of defined value */
    if rlDriverIsDeviceMapValid(deviceMap) != 0 as libc::c_int ||
           data.is_null() {
        /* set return error code */
        retVal = -(2 as libc::c_int)
    } else {
        /* Package the command with given data and send it to device */
        retVal =
            rlDriverExecuteSetApi(deviceMap,
                                  0x4 as libc::c_uint as rlUInt16_t,
                                  0x2 as libc::c_uint as rlUInt16_t,
                                  data as *mut rlUInt8_t,
                                  ::std::mem::size_of::<rlAdcOutCfg_t>() as
                                      libc::c_ulong as rlUInt16_t)
    }
    return retVal;
}
/* * @fn rlReturnVal_t rlSetBpmCommonConfig(rlUInt8_t deviceMap, rlBpmCommonCfg_t* data)
*
*   @brief Sets Binary Phase Modulation Common Configuration
*   @param[in] deviceMap - Bitmap of devices to send the message
*   @param[in] data - Container for BPM common Configuration data
*
*   @return rlReturnVal_t Success - 0, Failure - Error Code
*
*   This API defines static configurations related to BPM (Binary Phase Modulation) feature in
*   each of the TXs. E.g. the source of the BPM pattern (one constant value for each chirp as
*   defined, or intra-chirp pseudo random BPM pattern as found by a programmable
*   LFSR or a programmable sequence inside each chirp), are defined here.
*
*   @note 1: Different source of BPM is currently not supported, hence this API is not required
*            to be called by application.
*/
/* DesignId : MMWL_DesignId_016 */
/* Requirements : AUTORADAR_REQ-724 */
#[no_mangle]
pub unsafe extern "C" fn rlSetBpmCommonConfig(mut deviceMap: rlUInt8_t,
                                              mut data: *mut rlBpmCommonCfg_t)
 -> rlReturnVal_t {
    let mut retVal: rlReturnVal_t = 0;
    /* check if deviceIndex is out of defined value */
    if rlDriverIsDeviceMapValid(deviceMap) != 0 as libc::c_int ||
           data.is_null() {
        /* set return error code */
        retVal = -(2 as libc::c_int)
    } else {
        /* Package the command with given data and send it to device */
        retVal =
            rlDriverExecuteSetApi(deviceMap,
                                  0xc as libc::c_uint as rlUInt16_t,
                                  0 as libc::c_uint as rlUInt16_t,
                                  data as *mut rlUInt8_t,
                                  ::std::mem::size_of::<rlBpmCommonCfg_t>() as
                                      libc::c_ulong as rlUInt16_t)
    }
    return retVal;
}
/* * @fn rlReturnVal_t rlSetBpmChirpConfig(rlUInt8_t deviceMap, rlBpmChirpCfg_t* data)
*
*   @brief Sets Binary Phase Modulation Chirp Configuration
*   @param[in] deviceMap - Bitmap of devices to send the message
*   @param[in] data - Container for BPM chirp configuration data
*
*   @return rlReturnVal_t Success - 0, Failure - Error Code
*
*   This API defines static configurations related to BPM (Binary Phase Modulation) feature in
*   each of the TXs
*
*   @note 1: BPM values are applied at TX start time.
*   @note 2: This API is not supported in IWR6843 ES1.0
*   @note 3: In xWR1843 device, If only BPM is used without phase shifter then recommended to
*            disable the boot time phase shifter calibration in RF init.
*   @note 4: In xWR1843 device, If both phase shifter and BPM is used then recommended to
*            achieve BPM logic using per chirp phase shifter API instead of using BPM API.
*/
/* DesignId : MMWL_DesignId_037 */
/* Requirements : AUTORADAR_REQ-727 */
#[no_mangle]
pub unsafe extern "C" fn rlSetBpmChirpConfig(mut deviceMap: rlUInt8_t,
                                             mut data: *mut rlBpmChirpCfg_t)
 -> rlReturnVal_t {
    let mut retVal: rlReturnVal_t = 0;
    /* check if deviceIndex is out of defined value */
    if rlDriverIsDeviceMapValid(deviceMap) != 0 as libc::c_int ||
           data.is_null() {
        /* set return error code */
        retVal = -(2 as libc::c_int)
    } else {
        /* Package the command with given data and send it to device */
        retVal =
            rlDriverExecuteSetApi(deviceMap,
                                  0xc as libc::c_uint as rlUInt16_t,
                                  0x1 as libc::c_uint as rlUInt16_t,
                                  data as *mut rlUInt8_t,
                                  ::std::mem::size_of::<rlBpmChirpCfg_t>() as
                                      libc::c_ulong as rlUInt16_t)
    }
    return retVal;
}
/* * @fn rlReturnVal_t rlSetMultiBpmChirpConfig(rlUInt8_t deviceMap, rlUInt16_t cnt,
                                               rlBpmChirpCfg_t** data)
*
*   @brief Sets Binary Phase Modulation configuration for multiple Chirp
*   @param[in] deviceMap - Bitmap of devices to send the message
*   @param[in] cnt - number of BPM chirp config data
*   @param[in] data - pointer to linked list/array of BPM chirp configuration data
*
*   @return rlReturnVal_t Success - 0, Failure - Error Code
*
*   Using this API application can configure multiple BPM chirp configuration
*   in a single call. This API defines static configurations related to BPM (Binary
*   Phase Modulation) feature in each of the TXs.
*/
/* DesignId : MMWL_DesignId_122 */
/* Requirements : AUTORADAR_REQ-727 */
#[no_mangle]
pub unsafe extern "C" fn rlSetMultiBpmChirpConfig(mut deviceMap: rlUInt8_t,
                                                  mut cnt: rlUInt16_t,
                                                  mut data:
                                                      *mut *mut rlBpmChirpCfg_t)
 -> rlReturnVal_t {
    let mut retVal: rlReturnVal_t = 0;
    /* check if deviceIndex is out of defined value */
    if rlDriverIsDeviceMapValid(deviceMap) != 0 as libc::c_int ||
           data.is_null() || cnt as libc::c_uint == 0 as libc::c_uint {
        /* set return error code */
        retVal = -(2 as libc::c_int)
    } else {
        /* Initialize Command and Response Sub Blocks */
        let mut inMsg: rlDriverMsg_t =
            rlDriverMsg_t{opcode:
                              rlDriverOpcode_t{dir: 0,
                                               msgType: 0,
                                               msgId: 0,
                                               nsbc: 0,},
                          subblocks: 0 as *mut rlPayloadSb_t,
                          remChunks: 0,};
        let mut outMsg: rlDriverMsg_t =
            {
                let mut init =
                    rlDriverMsg{opcode:
                                    {
                                        let mut init =
                                            rlDriverOpcode{dir:
                                                               0 as
                                                                   libc::c_int
                                                                   as
                                                                   rlUInt8_t,
                                                           msgType: 0,
                                                           msgId: 0,
                                                           nsbc: 0,};
                                        init
                                    },
                                subblocks: 0 as *mut rlPayloadSb_t,
                                remChunks: 0,};
                init
            };
        /* Index and other paramerters to send multiple sub blocks in one/more commands chunk */
        let mut sbLen: rlUInt16_t = 0;
        let mut maxSbCntInMsg: rlUInt16_t = 0;
        let mut sbCntInMsg: rlUInt16_t = 0;
        let mut numChnkOfMsg: rlUInt16_t = 0;
        let mut lastSbCntInMsg: rlUInt16_t = 0;
        let mut loopCnt: rlUInt16_t = 0;
        let mut indx: rlUInt16_t = 0;
        /* Initialize Command and Response Sub Blocks */
        let mut inPayloadSb: [rlPayloadSb_t; 19] =
            [rlPayloadSb_t{sbid: 0, len: 0, pSblkData: 0 as *mut rlUInt8_t,};
                19];
        /* single sub-block length for chirp config API */
        sbLen =
            (::std::mem::size_of::<rlBpmChirpCfg_t>() as
                 libc::c_ulong).wrapping_add((2 as
                                                  libc::c_uint).wrapping_add(2
                                                                                 as
                                                                                 libc::c_uint)
                                                 as libc::c_ulong) as
                rlUInt16_t;
        /* get Max Sub Block count */
        /* AR_CODE_REVIEW MR:D.4.1 <APPROVED> "sbLen can not be zero" */
        /*LDRA_INSPECTED 127 D */
        maxSbCntInMsg =
            (256 as
                 libc::c_uint).wrapping_sub((4 as
                                                 libc::c_uint).wrapping_add(12
                                                                                as
                                                                                libc::c_uint).wrapping_add(8
                                                                                                               as
                                                                                                               libc::c_uint)).wrapping_div(sbLen
                                                                                                                                               as
                                                                                                                                               libc::c_uint)
                as rlUInt16_t;
        retVal = 0 as libc::c_int;
        /* if requested count of chirpConfig is within one Message packet */
        if cnt as libc::c_int <= maxSbCntInMsg as libc::c_int {
            sbCntInMsg = cnt;
            numChnkOfMsg = 1 as libc::c_uint as rlUInt16_t;
            lastSbCntInMsg = 0 as libc::c_uint as rlUInt16_t
        } else {
            sbCntInMsg = maxSbCntInMsg;
            numChnkOfMsg =
                (cnt as libc::c_int / maxSbCntInMsg as libc::c_int) as
                    rlUInt16_t;
            lastSbCntInMsg =
                (cnt as libc::c_int % maxSbCntInMsg as libc::c_int) as
                    rlUInt16_t
        }
        /* Fill in-message packet */
        rlDriverConstructInMsg(0xc as libc::c_uint as rlUInt16_t, &mut inMsg,
                               &mut *inPayloadSb.as_mut_ptr().offset(0 as
                                                                         libc::c_int
                                                                         as
                                                                         isize));
        loopCnt =
            if lastSbCntInMsg as libc::c_uint == 0 as libc::c_uint {
                numChnkOfMsg as libc::c_uint
            } else {
                (numChnkOfMsg as libc::c_uint).wrapping_add(1 as libc::c_uint)
            } as rlUInt16_t;
        while retVal == 0 as libc::c_int {
            /* all full messages have been sent, then send last partial message */
            if loopCnt as libc::c_uint == 1 as libc::c_uint &&
                   lastSbCntInMsg as libc::c_uint != 0 as libc::c_uint {
                inMsg.opcode.nsbc = lastSbCntInMsg
            } else { inMsg.opcode.nsbc = sbCntInMsg }
            indx = 0 as libc::c_uint as rlUInt16_t;
            while (indx as libc::c_int) < inMsg.opcode.nsbc as libc::c_int {
                /* Set Command Sub Block*/
                /* AR_CODE_REVIEW MR:R.2.2  <APPROVED> "Values are used by called function. \
                LDRA Tool Issue" */
                /*LDRA_INSPECTED 105 D */
                inPayloadSb[indx as usize].sbid =
                    (0xc as
                         libc::c_uint).wrapping_mul(32 as
                                                        libc::c_uint).wrapping_add(0x1
                                                                                       as
                                                                                       libc::c_uint)
                        as rlUInt16_t;
                /* AR_CODE_REVIEW MR:R.2.2  <APPROVED> "Values are used by called function. \
                LDRA Tool Issue" */
                /*LDRA_INSPECTED 105 D */
                inPayloadSb[indx as usize].len =
                    ::std::mem::size_of::<rlBpmChirpCfg_t>() as libc::c_ulong
                        as rlUInt16_t;
                /* AR_CODE_REVIEW MR:R.2.2  <APPROVED> "Values are used by called function. \
                LDRA Tool Issue" */
                /*LDRA_INSPECTED 105 D */
                inPayloadSb[indx as usize].pSblkData =
                    *data as *mut rlUInt8_t;
                /*AR_CODE_REVIEW MR:R.18.1 <APPROVED> "require pointer increment to jump to
                  next profile Config */
                /*LDRA_INSPECTED 567 S */
                data = data.offset(1);
                indx = indx.wrapping_add(1)
            }
            /* Send Command to mmWave Radar Device */
            retVal += rlDriverCmdInvoke(deviceMap, inMsg, &mut outMsg);
            loopCnt = loopCnt.wrapping_sub(1);
            if loopCnt as libc::c_uint == 0 as libc::c_uint { break ; }
        }
    }
    return retVal;
}
/* * @fn rlReturnVal_t rlSetProfileConfig(rlUInt8_t deviceMap, rlUInt16_t cnt, rlProfileCfg_t* data)
*
*   @brief Sets Chirp profile Configuration
*   @param[in] deviceMap - Bitmap of devices to send the message
*   @param[in] cnt  - Number of Profiles
*   @param[in] data - Array of Profile Configuration data
*
*   @return rlReturnVal_t Success - 0, Failure - Error Code
*
*   This function sets the chirp profile in mmWave Front end. A profile is like a template which
*   contains coarse information about FMCW signal such as start frequency, chirp slope, chirp
*   duration, TX power etc. The API allows multiple profiles to be set together by passing the
*   array of profile data along with count of profiles.
*
*   @note 1: One can set upto 4 profiles. Each profile contains coarse inforamtion. Fine dithering
*            can be added using chirp configuration API
*   @note 2: This API can be issued dynamically to change profile parameters.
*            Few parameters which cannot be changed are
*            1. numAdcSamples
*            2. digOutSampleRate
*            3. Programmable filter coefficients in xWR1642/xWR1843
*   @note 3: Please refer Table \ref chirpCycleTime for details on minimum chirp duration.
*   @note 4: The max TX output power back-off only up to 20dB is supported.
*   @note 5: The RF band used in functional chirp profiles shall be within the limit set in
*            AWR_CAL_MON_FREQUENCY_TX_POWER_LIMITS_SB API.
*   @note 6: This API takes around 700us to execute in RadarSS sub System
*   @note 7: Phase shifter(PS) settings are applied at the knee of the ramp.
*   @note 8 : The mid frequency code of RF band used in functional chirp profiles + 200MHz shall
*             be within the max limit set in this API.
*
*   @ref rlSetChirpConfig
*
*/
/* DesignId : MMWL_DesignId_013 */
/* Requirements :  AUTORADAR_REQ-728, AUTORADAR_REQ-729, AUTORADAR_REQ-730, AUTORADAR_REQ-731,
                   AUTORADAR_REQ-732, AUTORADAR_REQ-733, AUTORADAR_REQ-734, AUTORADAR_REQ-735,
                   AUTORADAR_REQ-736, AUTORADAR_REQ-737, AUTORADAR_REQ-738, AUTORADAR_REQ-739,
                   AUTORADAR_REQ-740, AUTORADAR_REQ-741, AUTORADAR_REQ-742, AUTORADAR_REQ-743,
                   AUTORADAR_REQ-773
 */
#[no_mangle]
pub unsafe extern "C" fn rlSetProfileConfig(mut deviceMap: rlUInt8_t,
                                            mut cnt: rlUInt16_t,
                                            mut data: *mut rlProfileCfg_t)
 -> rlReturnVal_t {
    let mut retVal: rlReturnVal_t = 0;
    /* check if deviceIndex is out of defined value */
    if rlDriverIsDeviceMapValid(deviceMap) != 0 as libc::c_int ||
           data.is_null() || cnt as libc::c_uint == 0 as libc::c_uint {
        /* set return error code */
        retVal = -(2 as libc::c_int)
    } else {
        let mut inMsg: rlDriverMsg_t =
            {
                let mut init =
                    rlDriverMsg{opcode:
                                    {
                                        let mut init =
                                            rlDriverOpcode{dir:
                                                               0 as
                                                                   libc::c_int
                                                                   as
                                                                   rlUInt8_t,
                                                           msgType: 0,
                                                           msgId: 0,
                                                           nsbc: 0,};
                                        init
                                    },
                                subblocks: 0 as *mut rlPayloadSb_t,
                                remChunks: 0,};
                init
            };
        let mut outMsg: rlDriverMsg_t =
            {
                let mut init =
                    rlDriverMsg{opcode:
                                    {
                                        let mut init =
                                            rlDriverOpcode{dir:
                                                               0 as
                                                                   libc::c_int
                                                                   as
                                                                   rlUInt8_t,
                                                           msgType: 0,
                                                           msgId: 0,
                                                           nsbc: 0,};
                                        init
                                    },
                                subblocks: 0 as *mut rlPayloadSb_t,
                                remChunks: 0,};
                init
            };
        /* Initialize Command and Response Sub Blocks */
        let mut inPayloadSb: [rlPayloadSb_t; 4] =
            [rlPayloadSb_t{sbid: 0, len: 0, pSblkData: 0 as *mut rlUInt8_t,};
                4];
        /* Variable to count message chunks */
        let mut indx: rlUInt16_t = 0;
        let mut sbCntInAllChunk: rlUInt16_t = 0 as libc::c_uint as rlUInt16_t;
        let mut maxSbInMsg: rlUInt16_t =
            ((256 as
                  libc::c_uint).wrapping_sub((4 as
                                                  libc::c_uint).wrapping_add(12
                                                                                 as
                                                                                 libc::c_uint).wrapping_add(8
                                                                                                                as
                                                                                                                libc::c_uint))
                 as
                 libc::c_ulong).wrapping_div((::std::mem::size_of::<rlProfileCfg_t>()
                                                  as
                                                  libc::c_ulong).wrapping_add(2
                                                                                  as
                                                                                  libc::c_uint
                                                                                  as
                                                                                  libc::c_ulong).wrapping_add(2
                                                                                                                  as
                                                                                                                  libc::c_uint
                                                                                                                  as
                                                                                                                  libc::c_ulong))
                as rlUInt16_t;
        /* Construct command packet */
        rlDriverConstructInMsg(0x8 as libc::c_uint as rlUInt16_t, &mut inMsg,
                               &mut *inPayloadSb.as_mut_ptr().offset(0 as
                                                                         libc::c_int
                                                                         as
                                                                         isize));
        retVal = 0 as libc::c_int;
        /* Loop to copy all the command data to one message */
        indx = 0 as libc::c_uint as rlUInt16_t;
        while (indx as libc::c_int) < cnt as libc::c_int &&
                  retVal == 0 as libc::c_int {
            /* Fill in-message payload */
            /*AR_CODE_REVIEW MR:R.18.1 <APPROVED> "require pointer increment to jump to
              next profile Config */
            /*AR_CODE_REVIEW MR:R.18.4 <APPROVED> "require pointer increment to jump to
              next profile Config */
            /*LDRA_INSPECTED 87 S */
            /*LDRA_INSPECTED 567 S */
            rlDriverFillPayload(0x8 as libc::c_uint as rlUInt16_t,
                                0 as libc::c_uint as rlUInt16_t,
                                &mut *inPayloadSb.as_mut_ptr().offset(indx as
                                                                          isize),
                                data.offset(indx as libc::c_int as isize) as
                                    *mut rlUInt8_t,
                                ::std::mem::size_of::<rlProfileCfg_t>() as
                                    libc::c_ulong as rlUInt16_t);
            /* increment Sub-block count in one packet */
            sbCntInAllChunk = sbCntInAllChunk.wrapping_add(1);
            /* check if total payload length is going beyond defined limitation */
            if maxSbInMsg as libc::c_uint ==
                   (indx as libc::c_uint).wrapping_add(1 as libc::c_uint) ||
                   cnt as libc::c_int == sbCntInAllChunk as libc::c_int {
                /* setting of numSBC to inMsg */
                inMsg.opcode.nsbc =
                    (indx as libc::c_uint).wrapping_add(1 as libc::c_uint) as
                        rlUInt16_t;
                /* Send Command to mmWave Radar Device */
                retVal += rlDriverCmdInvoke(deviceMap, inMsg, &mut outMsg)
            }
            indx = indx.wrapping_add(1)
        }
    }
    return retVal;
}
/* * @fn rlReturnVal_t rlGetProfileConfig(rlUInt8_t deviceMap, rlUInt16_t profileId,
*                                        rlProfileCfg_t* data)
*
*   @brief Gets Chirp profile Configuration
*   @param[in] deviceMap - Bitmap of devices to send the message
*   @param[in] profileId - Profile Id
*   @param[out] data - Container for Profile Configuration data
*
*   @return rlReturnVal_t Success - 0, Failure - Error Code
*
*   This function gets the FMCW radar chirp properties like FMCW slope, chirp duration,
*   TX power etc. from the device.
*/
/* DesignId : MMWL_DesignId_118 */
/* Requirements :  AUTORADAR_REQ-728, AUTORADAR_REQ-729, AUTORADAR_REQ-730, AUTORADAR_REQ-731,
AUTORADAR_REQ-732, AUTORADAR_REQ-733, AUTORADAR_REQ-734, AUTORADAR_REQ-735,
AUTORADAR_REQ-736, AUTORADAR_REQ-737, AUTORADAR_REQ-738, AUTORADAR_REQ-739,
AUTORADAR_REQ-740, AUTORADAR_REQ-741, AUTORADAR_REQ-742, AUTORADAR_REQ-743,
AUTORADAR_REQ-773
*/
#[no_mangle]
pub unsafe extern "C" fn rlGetProfileConfig(mut deviceMap: rlUInt8_t,
                                            mut profileId: rlUInt16_t,
                                            mut data: *mut rlProfileCfg_t)
 -> rlReturnVal_t {
    let mut retVal: rlReturnVal_t = 0;
    /* check if deviceIndex is out of defined value */
    if rlDriverIsDeviceMapValid(deviceMap) != 0 as libc::c_int ||
           data.is_null() {
        /* set return error code */
        retVal = -(2 as libc::c_int)
    } else {
        /* Initialize Command and Response Sub Blocks */
        let mut inMsg: rlDriverMsg_t =
            {
                let mut init =
                    rlDriverMsg{opcode:
                                    {
                                        let mut init =
                                            rlDriverOpcode{dir:
                                                               0 as
                                                                   libc::c_int
                                                                   as
                                                                   rlUInt8_t,
                                                           msgType: 0,
                                                           msgId: 0,
                                                           nsbc: 0,};
                                        init
                                    },
                                subblocks: 0 as *mut rlPayloadSb_t,
                                remChunks: 0,};
                init
            };
        let mut outMsg: rlDriverMsg_t =
            {
                let mut init =
                    rlDriverMsg{opcode:
                                    {
                                        let mut init =
                                            rlDriverOpcode{dir:
                                                               0 as
                                                                   libc::c_int
                                                                   as
                                                                   rlUInt8_t,
                                                           msgType: 0,
                                                           msgId: 0,
                                                           nsbc: 0,};
                                        init
                                    },
                                subblocks: 0 as *mut rlPayloadSb_t,
                                remChunks: 0,};
                init
            };
        let mut inPayloadSb: rlPayloadSb_t =
            {
                let mut init =
                    rlPayloadSb{sbid: 0 as libc::c_int as rlUInt16_t,
                                len: 0,
                                pSblkData: 0 as *mut rlUInt8_t,};
                init
            };
        let mut outPayloadSb: rlPayloadSb_t =
            {
                let mut init =
                    rlPayloadSb{sbid: 0 as libc::c_int as rlUInt16_t,
                                len: 0,
                                pSblkData: 0 as *mut rlUInt8_t,};
                init
            };
        let mut wordParam: rlWordParam_t =
            rlWordParam_t{halfWordOne: 0, halfWordTwo: 0,};
        /* combine half words into WORD structure */
        wordParam.halfWordOne = profileId;
        wordParam.halfWordTwo = 0 as libc::c_uint as rlUInt16_t;
        /* Construct command packet */
        rlDriverConstructInMsg(0x9 as libc::c_uint as rlUInt16_t, &mut inMsg,
                               &mut inPayloadSb);
        /* Fill in-message Payload */
        rlDriverFillPayload(0x9 as libc::c_uint as rlUInt16_t,
                            0 as libc::c_uint as rlUInt16_t, &mut inPayloadSb,
                            &mut wordParam as *mut rlWordParam_t as
                                *mut rlUInt8_t,
                            ::std::mem::size_of::<rlWordParam_t>() as
                                libc::c_ulong as rlUInt16_t);
        /* Construct response packet */
        rlDriverConstructOutMsg(1 as libc::c_uint as rlUInt16_t, &mut outMsg,
                                &mut outPayloadSb);
        /* Fill in-message Payload */
        rlDriverFillPayload(0 as libc::c_uint as rlUInt16_t,
                            0 as libc::c_uint as rlUInt16_t,
                            &mut outPayloadSb, data as *mut rlUInt8_t,
                            0 as libc::c_uint as rlUInt16_t);
        /* Send Command to mmWave Radar Device */
        retVal = rlDriverCmdInvoke(deviceMap, inMsg, &mut outMsg)
    }
    return retVal;
}
/* * @fn rlReturnVal_t rlSetChirpConfig(rlUInt8_t deviceMap, rlUInt16_t cnt, rlChirpCfg_t* data)
*
*   @brief Sets Chirp Configuration
*   @param[in] deviceMap - Bitmap of devices to send the message
*   @param[in] cnt  - Number of configurations
*   @param[in] data - Array of Chirp Configuration data
*
*   @return rlReturnVal_t Success - 0, Failure - Error Code
*
*   This function sets the chirp to chirp variations on top of the chirp profile. The
*   User should first define a profile using rlSetProfileConfig. This function then configures
*   the chirp by associating it with a particular profile defined in rlSetProfileConfig API.
*   In addition to that user can define fine dither to the profile parameters using this API.
*   The dithers used in this configuration are only additive on top of programmed
*   parameters in rlSetProfileConfig. This API allows configuration of 1 or upto 512 chirps.
*   Also it allows configuraiton of which Transmit channels to be used for each chirp.

*   @note 1: One can set upto 512 unique chirps which can be stored in dedicated memory inside
*            mmWave front end. Hence user doesn't need to program the chirps during run time. Also
*            these chirps can be sequenced in a frame using rlSetFrameConfig to create a larger
*            FMCW signal.
*
*   @ref rlSetFrameConfig
*/
/* DesignId : MMWL_DesignId_014 */
/* Requirements : AUTORADAR_REQ-744, AUTORADAR_REQ-745, AUTORADAR_REQ-746, AUTORADAR_REQ-747,
                  AUTORADAR_REQ-748, AUTORADAR_REQ-749, AUTORADAR_REQ-750, AUTORADAR_REQ-751,
                  AUTORADAR_REQ-773
 */
#[no_mangle]
pub unsafe extern "C" fn rlSetChirpConfig(mut deviceMap: rlUInt8_t,
                                          mut cnt: rlUInt16_t,
                                          mut data: *mut rlChirpCfg_t)
 -> rlReturnVal_t {
    let mut retVal: rlReturnVal_t = 0;
    /* check if deviceIndex is out of defined value */
    if rlDriverIsDeviceMapValid(deviceMap) != 0 as libc::c_int ||
           data.is_null() || cnt as libc::c_uint == 0 as libc::c_uint {
        /* set return error code */
        retVal = -(2 as libc::c_int)
    } else {
        /* Initialize Command and Response Sub Blocks */
        let mut inMsg: rlDriverMsg_t =
            rlDriverMsg_t{opcode:
                              rlDriverOpcode_t{dir: 0,
                                               msgType: 0,
                                               msgId: 0,
                                               nsbc: 0,},
                          subblocks: 0 as *mut rlPayloadSb_t,
                          remChunks: 0,};
        let mut outMsg: rlDriverMsg_t =
            {
                let mut init =
                    rlDriverMsg{opcode:
                                    {
                                        let mut init =
                                            rlDriverOpcode{dir:
                                                               0 as
                                                                   libc::c_int
                                                                   as
                                                                   rlUInt8_t,
                                                           msgType: 0,
                                                           msgId: 0,
                                                           nsbc: 0,};
                                        init
                                    },
                                subblocks: 0 as *mut rlPayloadSb_t,
                                remChunks: 0,};
                init
            };
        /* Index and other paramerters to send multiple sub blocks in one/more commands chunk */
        let mut sbLen: rlUInt16_t = 0;
        let mut maxSbCntInMsg: rlUInt16_t = 0;
        let mut sbCntInMsg: rlUInt16_t = 0;
        let mut numChnkOfMsg: rlUInt16_t = 0;
        let mut lastSbCntInMsg: rlUInt16_t = 0;
        let mut loopCnt: rlUInt16_t = 0;
        let mut indx: rlUInt16_t = 0;
        /* Initialize Command and Response Sub Blocks */
        let mut inPayloadSb: [rlPayloadSb_t; 9] =
            [rlPayloadSb_t{sbid: 0, len: 0, pSblkData: 0 as *mut rlUInt8_t,};
                9];
        /* single sub-block length for chirp config API */
        sbLen =
            (::std::mem::size_of::<rlChirpCfg_t>() as
                 libc::c_ulong).wrapping_add((2 as
                                                  libc::c_uint).wrapping_add(2
                                                                                 as
                                                                                 libc::c_uint)
                                                 as libc::c_ulong) as
                rlUInt16_t;
        /* get Max Sub Block count */
        /* AR_CODE_REVIEW MR:D.4.1 <APPROVED> "sbLen can not be zero" */
        /*LDRA_INSPECTED 127 D */
        maxSbCntInMsg =
            (256 as
                 libc::c_uint).wrapping_sub((4 as
                                                 libc::c_uint).wrapping_add(12
                                                                                as
                                                                                libc::c_uint).wrapping_add(8
                                                                                                               as
                                                                                                               libc::c_uint)).wrapping_div(sbLen
                                                                                                                                               as
                                                                                                                                               libc::c_uint)
                as rlUInt16_t;
        retVal = 0 as libc::c_int;
        /* if requested count of chirpConfig is within one Message packet */
        if cnt as libc::c_int <= maxSbCntInMsg as libc::c_int {
            sbCntInMsg = cnt;
            numChnkOfMsg = 1 as libc::c_uint as rlUInt16_t;
            lastSbCntInMsg = 0 as libc::c_uint as rlUInt16_t
        } else {
            sbCntInMsg = maxSbCntInMsg;
            numChnkOfMsg =
                (cnt as libc::c_int / maxSbCntInMsg as libc::c_int) as
                    rlUInt16_t;
            lastSbCntInMsg =
                (cnt as libc::c_int % maxSbCntInMsg as libc::c_int) as
                    rlUInt16_t
        }
        /* Fill in-message packet */
        rlDriverConstructInMsg(0x8 as libc::c_uint as rlUInt16_t, &mut inMsg,
                               &mut *inPayloadSb.as_mut_ptr().offset(0 as
                                                                         libc::c_int
                                                                         as
                                                                         isize));
        /* get the loop count in form of number of msg chunks */
        loopCnt =
            if lastSbCntInMsg as libc::c_uint == 0 as libc::c_uint {
                numChnkOfMsg as libc::c_uint
            } else {
                (numChnkOfMsg as libc::c_uint).wrapping_add(1 as libc::c_uint)
            } as rlUInt16_t;
        while retVal == 0 as libc::c_int {
            /* all full messages have been sent, then send last partial message */
            if loopCnt as libc::c_uint == 1 as libc::c_uint &&
                   lastSbCntInMsg as libc::c_uint != 0 as libc::c_uint {
                inMsg.opcode.nsbc = lastSbCntInMsg
            } else { inMsg.opcode.nsbc = sbCntInMsg }
            indx = 0 as libc::c_uint as rlUInt16_t;
            while (indx as libc::c_int) < inMsg.opcode.nsbc as libc::c_int {
                /* Set Command Sub Block*/
                /* AR_CODE_REVIEW MR:R.2.2  <APPROVED> "Values are used by called function. \
                LDRA Tool Issue" */
                /*LDRA_INSPECTED 105 D */
                inPayloadSb[indx as usize].sbid =
                    (0x8 as
                         libc::c_uint).wrapping_mul(32 as
                                                        libc::c_uint).wrapping_add(0x1
                                                                                       as
                                                                                       libc::c_uint)
                        as rlUInt16_t;
                /* AR_CODE_REVIEW MR:R.2.2  <APPROVED> "Values are used by called function. \
                LDRA Tool Issue" */
                /*LDRA_INSPECTED 105 D */
                inPayloadSb[indx as usize].len =
                    ::std::mem::size_of::<rlChirpCfg_t>() as libc::c_ulong as
                        rlUInt16_t;
                /* AR_CODE_REVIEW MR:R.2.2  <APPROVED> "Values are used by called function. \
                LDRA Tool Issue" */
                /*LDRA_INSPECTED 105 D */
                inPayloadSb[indx as usize].pSblkData = data as *mut rlUInt8_t;
                /*AR_CODE_REVIEW MR:R.18.1 <APPROVED> "require pointer increment to jump to \
                  next chirp Config */
                /*LDRA_INSPECTED 567 S */
                data = data.offset(1);
                indx = indx.wrapping_add(1)
            }
            /* Send Command to mmWave Radar Device */
            /* AR_CODE_REVIEW MR:R.2.2  <APPROVED> "Values are used by called function. \
            LDRA Tool Issue" */
            /*LDRA_INSPECTED 105 D */
            retVal += rlDriverCmdInvoke(deviceMap, inMsg, &mut outMsg);
            /* decrement the loop count */
            loopCnt = loopCnt.wrapping_sub(1);
            /* if all chirp configurations have been sent to device then terminate the loop */
            if loopCnt as libc::c_uint == 0 as libc::c_uint { break ; }
        }
    }
    return retVal;
}
/* * @fn rlReturnVal_t rlGetChirpConfig(rlUInt8_t deviceMap, rlUInt16_t chirpStartIdx,
*                                      rlUInt16_t chirpEndIdx, rlChirpCfg_t* data)
*
*   @brief Gets Chirp Configuration
*   @param[in] deviceMap - Bitmap of devices to send the message
*   @param[in] chirpStartIdx - Chirp Start Index
*   @param[in] chirpEndIdx - Chirp End Index
*   @param[out] data - Container for Chirp Configuration data
*
*   @return rlReturnVal_t Success - 0, Failure - Error Code
*
*   This function gets the chirp configuration from the device.
*/
/* DesignId : MMWL_DesignId_119 */
/* Requirements : AUTORADAR_REQ-744, AUTORADAR_REQ-745, AUTORADAR_REQ-746, AUTORADAR_REQ-747,
AUTORADAR_REQ-748, AUTORADAR_REQ-749, AUTORADAR_REQ-750, AUTORADAR_REQ-751,
AUTORADAR_REQ-773
*/
#[no_mangle]
pub unsafe extern "C" fn rlGetChirpConfig(mut deviceMap: rlUInt8_t,
                                          mut chirpStartIdx: rlUInt16_t,
                                          mut chirpEndIdx: rlUInt16_t,
                                          mut data: *mut rlChirpCfg_t)
 -> rlReturnVal_t {
    let mut retVal: rlReturnVal_t = 0;
    let mut chirpCfgData: *mut rlChirpCfg_t = data;
    /* check if deviceIndex is out of defined value */
    if rlDriverIsDeviceMapValid(deviceMap) != 0 as libc::c_int ||
           chirpCfgData.is_null() {
        /* set return error code */
        retVal = -(2 as libc::c_int)
    } else {
        /* Initialize Command and Response Sub Blocks */
        let mut inMsg: rlDriverMsg_t =
            {
                let mut init =
                    rlDriverMsg{opcode:
                                    {
                                        let mut init =
                                            rlDriverOpcode{dir:
                                                               0 as
                                                                   libc::c_int
                                                                   as
                                                                   rlUInt8_t,
                                                           msgType: 0,
                                                           msgId: 0,
                                                           nsbc: 0,};
                                        init
                                    },
                                subblocks: 0 as *mut rlPayloadSb_t,
                                remChunks: 0,};
                init
            };
        let mut outMsg: rlDriverMsg_t =
            {
                let mut init =
                    rlDriverMsg{opcode:
                                    {
                                        let mut init =
                                            rlDriverOpcode{dir:
                                                               0 as
                                                                   libc::c_int
                                                                   as
                                                                   rlUInt8_t,
                                                           msgType: 0,
                                                           msgId: 0,
                                                           nsbc: 0,};
                                        init
                                    },
                                subblocks: 0 as *mut rlPayloadSb_t,
                                remChunks: 0,};
                init
            };
        /* Index and other paramerters to send multiple sub blocks in one/more commands chunk */
        let mut sbLen: rlUInt16_t = 0;
        let mut maxSbCntOutMsg: rlUInt16_t = 0;
        let mut sbCntOutMsg: rlUInt16_t = 0;
        let mut numChnkOfMsg: rlUInt16_t = 0;
        let mut lastSbCntOutMsg: rlUInt16_t = 0;
        let mut loopCnt: rlUInt16_t = 0;
        /* file chirp start and end index */
        let mut wordParam: rlWordParam_t =
            rlWordParam_t{halfWordOne: 0, halfWordTwo: 0,};
        let mut inPayloadSb: rlPayloadSb_t =
            {
                let mut init =
                    rlPayloadSb{sbid: 0 as libc::c_int as rlUInt16_t,
                                len: 0,
                                pSblkData: 0 as *mut rlUInt8_t,};
                init
            };
        let mut outPayloadSb: rlPayloadSb_t =
            rlPayloadSb_t{sbid: 0, len: 0, pSblkData: 0 as *mut rlUInt8_t,};
        /* single sub-block length for chirp config API */
        sbLen =
            ::std::mem::size_of::<rlChirpCfg_t>() as libc::c_ulong as
                rlUInt16_t;
        /* get Max Sub Block count */
        /* AR_CODE_REVIEW MR:D.4.1 <APPROVED> "sbLen can not be zero" */
        /*LDRA_INSPECTED 127 D */
        maxSbCntOutMsg =
            (256 as
                 libc::c_uint).wrapping_sub((4 as
                                                 libc::c_uint).wrapping_add(12
                                                                                as
                                                                                libc::c_uint).wrapping_add(8
                                                                                                               as
                                                                                                               libc::c_uint)).wrapping_sub((2
                                                                                                                                                as
                                                                                                                                                libc::c_uint).wrapping_add(2
                                                                                                                                                                               as
                                                                                                                                                                               libc::c_uint)).wrapping_div(sbLen
                                                                                                                                                                                                               as
                                                                                                                                                                                                               libc::c_uint)
                as rlUInt16_t;
        retVal = 0 as libc::c_int;
        /* if requested count of chirpConfig is within one Message packet */
        if ((chirpEndIdx as libc::c_int - chirpStartIdx as libc::c_int) as
                libc::c_uint).wrapping_add(1 as libc::c_uint) <=
               maxSbCntOutMsg as libc::c_uint {
            sbCntOutMsg =
                ((chirpEndIdx as libc::c_int - chirpStartIdx as libc::c_int)
                     as libc::c_uint).wrapping_add(1 as libc::c_uint) as
                    rlUInt16_t;
            numChnkOfMsg = 0 as libc::c_uint as rlUInt16_t;
            lastSbCntOutMsg = 0 as libc::c_uint as rlUInt16_t
        } else {
            sbCntOutMsg = maxSbCntOutMsg;
            numChnkOfMsg =
                ((chirpEndIdx as libc::c_int - chirpStartIdx as libc::c_int)
                     as
                     libc::c_uint).wrapping_add(1 as
                                                    libc::c_uint).wrapping_div(maxSbCntOutMsg
                                                                                   as
                                                                                   libc::c_uint)
                    as rlUInt16_t;
            lastSbCntOutMsg =
                ((chirpEndIdx as libc::c_int - chirpStartIdx as libc::c_int)
                     as
                     libc::c_uint).wrapping_add(1 as
                                                    libc::c_uint).wrapping_rem(maxSbCntOutMsg
                                                                                   as
                                                                                   libc::c_uint)
                    as rlUInt16_t
        }
        /* set chirp start index to inMsg field */
        /* AR_CODE_REVIEW MR:R.2.2 <APPROVED> "required to intial with startIdx" */
        /*LDRA_INSPECTED 8 D */
        wordParam.halfWordTwo = chirpStartIdx;
        /* get the loop count in form of number of msg chunks */
        loopCnt =
            (numChnkOfMsg as libc::c_uint).wrapping_add(1 as libc::c_uint) as
                rlUInt16_t;
        /* setting num of sub-block to one in outMsg */
        outMsg.opcode.nsbc = 1 as libc::c_uint as rlUInt16_t;
        while retVal == 0 as libc::c_int {
            /* combine half words into WORD structure */
            wordParam.halfWordOne = wordParam.halfWordTwo;
            /* all full messages have been sent, then send last partial message */
            if loopCnt as libc::c_uint == 1 as libc::c_uint &&
                   lastSbCntOutMsg as libc::c_uint != 0 as libc::c_uint {
                wordParam.halfWordTwo =
                    (wordParam.halfWordOne as
                         libc::c_uint).wrapping_add((lastSbCntOutMsg as
                                                         libc::c_uint).wrapping_sub(1
                                                                                        as
                                                                                        libc::c_uint))
                        as rlUInt16_t
            } else {
                wordParam.halfWordTwo =
                    (wordParam.halfWordOne as
                         libc::c_uint).wrapping_add((sbCntOutMsg as
                                                         libc::c_uint).wrapping_sub(1
                                                                                        as
                                                                                        libc::c_uint))
                        as rlUInt16_t
            }
            /* Construct command packet */
            rlDriverConstructInMsg(0x9 as libc::c_uint as rlUInt16_t,
                                   &mut inMsg, &mut inPayloadSb);
            /* Fill in-message Payload */
            rlDriverFillPayload(0x9 as libc::c_uint as rlUInt16_t,
                                0x1 as libc::c_uint as rlUInt16_t,
                                &mut inPayloadSb,
                                &mut wordParam as *mut rlWordParam_t as
                                    *mut rlUInt8_t,
                                ::std::mem::size_of::<rlWordParam_t>() as
                                    libc::c_ulong as rlUInt16_t);
            /* Set Command Sub Block*/
            outPayloadSb.sbid =
                (0x9 as
                     libc::c_uint).wrapping_mul(32 as
                                                    libc::c_uint).wrapping_add(0x1
                                                                                   as
                                                                                   libc::c_uint)
                    as rlUInt16_t;
            outPayloadSb.len =
                ::std::mem::size_of::<rlChirpCfg_t>() as libc::c_ulong as
                    rlUInt16_t;
            outPayloadSb.pSblkData = chirpCfgData as *mut rlUInt8_t;
            /* Construct response packet */
            rlDriverConstructOutMsg(outMsg.opcode.nsbc, &mut outMsg,
                                    &mut outPayloadSb);
            /* Send Command to mmWave Radar Device */
            retVal += rlDriverCmdInvoke(deviceMap, inMsg, &mut outMsg);
            /* increment data pointer for next chunk of reponse */
            chirpCfgData =
                chirpCfgData.offset(sbCntOutMsg as libc::c_int as isize);
            /* decrement the loop count */
            loopCnt = loopCnt.wrapping_sub(1);
            wordParam.halfWordTwo = wordParam.halfWordTwo.wrapping_add(1);
            /* if all chirp configurations have been sent to device then terminate the loop */
            if loopCnt as libc::c_uint == 0 as libc::c_uint { break ; }
        }
    }
    return retVal;
}
/* * @fn rlReturnVal_t rlSetMultiChirpCfg(rlUInt8_t deviceMap, rlUInt16_t cnt,
                                         rlChirpCfg_t **data)
*
*   @brief Injects chirp configuration to be programmed dynamically
*   @param[in] deviceMap - Bitmap of devices to send the message
*   @param[in] cnt - number of chirps
*   @param[in] data - Pointer to Chirp configuration linked list/array
*
*   @return rlReturnVal_t Success - 0, Failure - Error Code
*
*   This function sets the chirp to chirp variations on top of the chirp profile. The
*   User should first define a profile using rlSetProfileConfig.\n This function then configures
*   the chirp by associating it with a particular profile defined in rlSetProfileConfig API.
*   In addition to that user can define fine dither to the profile parameters using this API \n
*   This API allows configuration of 1 or upto 512 chirps. Also it allows configuraiton of
*   which Transmit channels to be used for each chirp.
*
*   @note One can set upto 512 unique chirps which can be stored in dedicated memory inside
*   mmWave front end. Hence user doesn't need to program the chirps during run time. Also these
*   chirps can be sequenced in a frame using rlSetFrameConfig to create a larger FMCW signal\n
*   This API is similar to rlSetChirpConfig but gives the flexibility to pass the array of
*   chirp configuration pointers, so chirp configuration memory need not be contiguous.
*
*/
#[no_mangle]
pub unsafe extern "C" fn rlSetMultiChirpCfg(mut deviceMap: rlUInt8_t,
                                            mut cnt: rlUInt16_t,
                                            mut data: *mut *mut rlChirpCfg_t)
 -> rlReturnVal_t {
    let mut retVal: rlReturnVal_t = 0;
    /* check if deviceIndex is out of defined value */
    if rlDriverIsDeviceMapValid(deviceMap) != 0 as libc::c_int ||
           data.is_null() || cnt as libc::c_uint == 0 as libc::c_uint {
        /* set return error code */
        retVal = -(2 as libc::c_int)
    } else {
        /* Initialize Command and Response Sub Blocks */
        let mut inMsg: rlDriverMsg_t =
            rlDriverMsg_t{opcode:
                              rlDriverOpcode_t{dir: 0,
                                               msgType: 0,
                                               msgId: 0,
                                               nsbc: 0,},
                          subblocks: 0 as *mut rlPayloadSb_t,
                          remChunks: 0,};
        let mut outMsg: rlDriverMsg_t =
            {
                let mut init =
                    rlDriverMsg{opcode:
                                    {
                                        let mut init =
                                            rlDriverOpcode{dir:
                                                               0 as
                                                                   libc::c_int
                                                                   as
                                                                   rlUInt8_t,
                                                           msgType: 0,
                                                           msgId: 0,
                                                           nsbc: 0,};
                                        init
                                    },
                                subblocks: 0 as *mut rlPayloadSb_t,
                                remChunks: 0,};
                init
            };
        /* Index and other paramerters to send multiple sub blocks in one/more commands chunk */
        let mut sbLen: rlUInt16_t = 0;
        let mut maxSbCntInMsg: rlUInt16_t = 0;
        let mut sbCntInMsg: rlUInt16_t = 0;
        let mut numChnkOfMsg: rlUInt16_t = 0;
        let mut lastSbCntInMsg: rlUInt16_t = 0;
        let mut loopCnt: rlUInt16_t = 0;
        let mut indx: rlUInt16_t = 0;
        /* Initialize Command and Response Sub Blocks */
        let mut inPayloadSb: [rlPayloadSb_t; 9] =
            [rlPayloadSb_t{sbid: 0, len: 0, pSblkData: 0 as *mut rlUInt8_t,};
                9];
        /* single sub-block length for chirp config API */
        sbLen =
            (::std::mem::size_of::<rlChirpCfg_t>() as
                 libc::c_ulong).wrapping_add((2 as
                                                  libc::c_uint).wrapping_add(2
                                                                                 as
                                                                                 libc::c_uint)
                                                 as libc::c_ulong) as
                rlUInt16_t;
        /* get Max Sub Block count */
        /* AR_CODE_REVIEW MR:D.4.1 <APPROVED> "sbLen can not be zero" */
        /*LDRA_INSPECTED 127 D */
        maxSbCntInMsg =
            (256 as
                 libc::c_uint).wrapping_sub((4 as
                                                 libc::c_uint).wrapping_add(12
                                                                                as
                                                                                libc::c_uint).wrapping_add(8
                                                                                                               as
                                                                                                               libc::c_uint)).wrapping_div(sbLen
                                                                                                                                               as
                                                                                                                                               libc::c_uint)
                as rlUInt16_t;
        retVal = 0 as libc::c_int;
        /* if requested count of chirpConfig is within one Message packet */
        if cnt as libc::c_int <= maxSbCntInMsg as libc::c_int {
            sbCntInMsg = cnt;
            numChnkOfMsg = 1 as libc::c_uint as rlUInt16_t;
            lastSbCntInMsg = 0 as libc::c_uint as rlUInt16_t
        } else {
            sbCntInMsg = maxSbCntInMsg;
            numChnkOfMsg =
                (cnt as libc::c_int / maxSbCntInMsg as libc::c_int) as
                    rlUInt16_t;
            lastSbCntInMsg =
                (cnt as libc::c_int % maxSbCntInMsg as libc::c_int) as
                    rlUInt16_t
        }
        /* Fill in-message packet */
        rlDriverConstructInMsg(0x8 as libc::c_uint as rlUInt16_t, &mut inMsg,
                               &mut *inPayloadSb.as_mut_ptr().offset(0 as
                                                                         libc::c_int
                                                                         as
                                                                         isize));
        loopCnt =
            if lastSbCntInMsg as libc::c_uint == 0 as libc::c_uint {
                numChnkOfMsg as libc::c_uint
            } else {
                (numChnkOfMsg as libc::c_uint).wrapping_add(1 as libc::c_uint)
            } as rlUInt16_t;
        while retVal == 0 as libc::c_int {
            /* all full messages have been sent, then send last partial message */
            if loopCnt as libc::c_uint == 1 as libc::c_uint &&
                   lastSbCntInMsg as libc::c_uint != 0 as libc::c_uint {
                inMsg.opcode.nsbc = lastSbCntInMsg
            } else { inMsg.opcode.nsbc = sbCntInMsg }
            indx = 0 as libc::c_uint as rlUInt16_t;
            while (indx as libc::c_int) < inMsg.opcode.nsbc as libc::c_int {
                /* Set Command Sub Block*/
                /* AR_CODE_REVIEW MR:R.2.2  <APPROVED> "Values are used by called function. \
                LDRA Tool Issue" */
                /*LDRA_INSPECTED 105 D */
                inPayloadSb[indx as usize].sbid =
                    (0x8 as
                         libc::c_uint).wrapping_mul(32 as
                                                        libc::c_uint).wrapping_add(0x1
                                                                                       as
                                                                                       libc::c_uint)
                        as rlUInt16_t;
                /* AR_CODE_REVIEW MR:R.2.2  <APPROVED> "Values are used by called function. \
                LDRA Tool Issue" */
                /*LDRA_INSPECTED 105 D */
                inPayloadSb[indx as usize].len =
                    ::std::mem::size_of::<rlChirpCfg_t>() as libc::c_ulong as
                        rlUInt16_t;
                /* AR_CODE_REVIEW MR:R.2.2  <APPROVED> "Values are used by called function. \
                LDRA Tool Issue" */
                /*LDRA_INSPECTED 105 D */
                inPayloadSb[indx as usize].pSblkData =
                    *data as *mut rlUInt8_t;
                /*AR_CODE_REVIEW MR:R.18.1 <APPROVED> "require pointer increment to jump to
                  next profile Config */
                /*LDRA_INSPECTED 567 S */
                data = data.offset(1);
                indx = indx.wrapping_add(1)
            }
            /* Send Command to mmWave Radar Device */
            retVal += rlDriverCmdInvoke(deviceMap, inMsg, &mut outMsg);
            loopCnt = loopCnt.wrapping_sub(1);
            if loopCnt as libc::c_uint == 0 as libc::c_uint { break ; }
        }
    }
    return retVal;
}
/* * @fn rlReturnVal_t rlSetFrameConfig(rlUInt8_t deviceMap, rlFrameCfg_t* data)
*
*   @brief Sets Frame Configuration
*   @param[in] deviceMap - Bitmap of devices to send the message
*   @param[in] data - Container for Frame Configuration data
*
*   @return rlReturnVal_t Success - 0, Failure - Error Code
*
*   This function allows configuration of FMCW frame in mmWave Front end. A Frame is basically
*   a sequence of chirps and how this sequnece needs to be repeated over time. User first need
*   to define a profile and set of chirps(associated with a profile). \n This function then
*   defines how to sequence these chirps. The same chirp can be simply looped to create a large
*   FMCW frame or multiple unique chirps cane be sequenced to create the frame. Chirp Start and
*   end Index defines how to sequence them in a frame. \n The API also allows configuration of
*   number of frames to be transmitted, periodicity of the frame and the trigger method. The
*   trigger method could be SW API based trigger or HW SYNC IN based trigger. \n This API calls
*   internally two APIs, one to RadraSS for sensor configuraion and another to MasterSS for
*   datapath configuraion. \n
*
*   @note 1: If hardware triggered mode is used, the SYNC_IN pulse width should be less than 1us.
*            Also, the minimum pulse width of SYNC_IN should be 25 ns.
*   @note 2: If frame trigger delay is used with hardware triggered mode, then external SYNC_IN
*            pulse periodicity should take care of the configured frame trigger delay and frame
*            periodicity. The external pulse should be issued only after the sum total of frame
*            trigger delay and frame periodicity.
*   @note 3: If dummy chirp is used then programmer should make sure the idle time of dummy
*            chirp >= 4us + DFE spill over time of previous chirp (calculate from rampgen
*            calculator). The first chirp of frame cannot be a dummy chirp.
*   @note 4: In Hw triggered mode, the Hw pulse should be issued or periodicity of pulse is
*            configured such that, the pulse is generated only 150us after the completion of
*            previous frame/burst (The pulse should not be issued before end of previous frame/
*            burst). The time delta between end of previous frame/burst and raising edge of HW
*            pulse recommended to be < 300us.
*   @note 5: Frame could have multiple chirps associated with different profile, but number of
*            samples need to be same in all the profiles. \n
*   @note 6: The PF_NUM_ADC_SAMPLES parameter should be identical across chirps in a frame,
*            when multiple profiles are used in a frame. \n
*   @note 7: The PF_DIGITAL_OUTPUT_SAMPLING_RATE impacts the LVDS/CSI2 data rate in a frame, so it
*            is recommended to analyze timing impact if different sample rate is used across
*            chirps in a frame. \n
*   @note 8: Please refer Table \ref interSubFrameTime for details on minimum inter-frame blank
*            time requirements. \n
*/
/* DesignId : MMWL_DesignId_015 */
/* Requirements : AUTORADAR_REQ-715, AUTORADAR_REQ-716, AUTORADAR_REQ-717,
   AUTORADAR_REQ-718, AUTORADAR_REQ-719, AUTORADAR_REQ-720, AUTORADAR_REQ-1045 */
#[no_mangle]
pub unsafe extern "C" fn rlSetFrameConfig(mut deviceMap: rlUInt8_t,
                                          mut data: *mut rlFrameCfg_t)
 -> rlReturnVal_t {
    let mut retVal: rlReturnVal_t = 0;
    /* check if deviceIndex is out of defined value */
    if rlDriverIsDeviceMapValid(deviceMap) != 0 as libc::c_int ||
           data.is_null() {
        /* set return error code */
        retVal = -(2 as libc::c_int)
    } else {
        /* Package the command with given data and send it to device */
        retVal =
            rlDriverExecuteSetApi(deviceMap,
                                  0x8 as libc::c_uint as rlUInt16_t,
                                  0x2 as libc::c_uint as rlUInt16_t,
                                  data as *mut rlUInt8_t,
                                  ::std::mem::size_of::<rlFrameCfg_t>() as
                                      libc::c_ulong as rlUInt16_t);
        /* If only mmWaveLink instance is running on Host */
        if retVal == 0 as libc::c_int &&
               0 as libc::c_uint == rlDriverGetPlatformId() as libc::c_uint {
            /* Initialize Command and Response Sub Blocks */
            let mut inMsg: rlDriverMsg_t =
                {
                    let mut init =
                        rlDriverMsg{opcode:
                                        {
                                            let mut init =
                                                rlDriverOpcode{dir:
                                                                   0 as
                                                                       libc::c_int
                                                                       as
                                                                       rlUInt8_t,
                                                               msgType: 0,
                                                               msgId: 0,
                                                               nsbc: 0,};
                                            init
                                        },
                                    subblocks: 0 as *mut rlPayloadSb_t,
                                    remChunks: 0,};
                    init
                };
            let mut outMsg: rlDriverMsg_t =
                {
                    let mut init =
                        rlDriverMsg{opcode:
                                        {
                                            let mut init =
                                                rlDriverOpcode{dir:
                                                                   0 as
                                                                       libc::c_int
                                                                       as
                                                                       rlUInt8_t,
                                                               msgType: 0,
                                                               msgId: 0,
                                                               nsbc: 0,};
                                            init
                                        },
                                    subblocks: 0 as *mut rlPayloadSb_t,
                                    remChunks: 0,};
                    init
                };
            let mut inPayloadSb: rlPayloadSb_t =
                {
                    let mut init =
                        rlPayloadSb{sbid: 0 as libc::c_int as rlUInt16_t,
                                    len: 0,
                                    pSblkData: 0 as *mut rlUInt8_t,};
                    init
                };
            let mut frameApplyCfgArgs: rlFrameApplyCfg_t =
                rlFrameApplyCfg_t{numChirps: 0,
                                  numAdcSamples: 0,
                                  reserved: 0,};
            let mut tempVar: rlUInt16_t = 0;
            frameApplyCfgArgs.reserved = 0 as libc::c_uint as rlUInt16_t;
            frameApplyCfgArgs.numAdcSamples = (*data).numAdcSamples;
            tempVar =
                (((*data).chirpEndIdx as libc::c_int -
                      (*data).chirpStartIdx as libc::c_int) as
                     libc::c_uint).wrapping_add(1 as
                                                    libc::c_uint).wrapping_mul((*data).numLoops
                                                                                   as
                                                                                   libc::c_uint).wrapping_sub((*data).numDummyChirpsAtEnd
                                                                                                                  as
                                                                                                                  libc::c_uint)
                    as rlUInt16_t;
            frameApplyCfgArgs.numChirps = tempVar as rlUInt32_t;
            /* Construct command packet */
            rlDriverConstructInMsg(0x206 as libc::c_uint as rlUInt16_t,
                                   &mut inMsg, &mut inPayloadSb);
            /* Fill in-message Payload */
            rlDriverFillPayload(0x206 as libc::c_uint as rlUInt16_t,
                                0 as libc::c_uint as rlUInt16_t,
                                &mut inPayloadSb,
                                &mut frameApplyCfgArgs as
                                    *mut rlFrameApplyCfg_t as *mut rlUInt8_t,
                                ::std::mem::size_of::<rlFrameApplyCfg_t>() as
                                    libc::c_ulong as rlUInt16_t);
            /* Send Command to mmWave Radar Device */
            retVal += rlDriverCmdInvoke(deviceMap, inMsg, &mut outMsg)
        }
    }
    return retVal;
}
/* * @fn rlReturnVal_t rlGetFrameConfig(rlUInt8_t deviceMap, rlFrameCfg_t* data)
*
*   @brief Gets Frame Configuration
*   @param[in] deviceMap - Bitmap of devices to send the message
*   @param[out] data - Container for Frame Configuration data
*
*   @return rlReturnVal_t Success - 0, Failure - Error Code
*
*   This function reads the frame properties of the device. This API calls internally two APIs,
*   one to RadraSS for sensor configuraion and another to MasterSS for datapath configuraion. \n
*/
/* DesignId : MMWL_DesignId_120 */
/* Requirements : AUTORADAR_REQ-715, AUTORADAR_REQ-716, AUTORADAR_REQ-717,
AUTORADAR_REQ-718, AUTORADAR_REQ-719, AUTORADAR_REQ-720, AUTORADAR_REQ-1045 */
#[no_mangle]
pub unsafe extern "C" fn rlGetFrameConfig(mut deviceMap: rlUInt8_t,
                                          mut data: *mut rlFrameCfg_t)
 -> rlReturnVal_t {
    let mut retVal: rlReturnVal_t = 0;
    /* check if deviceIndex is out of defined value */
    if rlDriverIsDeviceMapValid(deviceMap) != 0 as libc::c_int ||
           data.is_null() {
        /* set return error code */
        retVal = -(2 as libc::c_int)
    } else {
        /* Package the command with given data and send it to device */
        retVal =
            rlDriverExecuteGetApi(deviceMap,
                                  0x9 as libc::c_uint as rlUInt16_t,
                                  0x2 as libc::c_uint as rlUInt16_t,
                                  data as *mut rlUInt8_t,
                                  0 as libc::c_uint as rlUInt16_t);
        /* If only mmWaveLink instance is running on Host */
        if retVal == 0 as libc::c_int &&
               0 as libc::c_uint == rlDriverGetPlatformId() as libc::c_uint {
            /* Initialize Command and Response Sub Blocks */
            let mut inMsg: rlDriverMsg_t =
                {
                    let mut init =
                        rlDriverMsg{opcode:
                                        {
                                            let mut init =
                                                rlDriverOpcode{dir:
                                                                   0 as
                                                                       libc::c_int
                                                                       as
                                                                       rlUInt8_t,
                                                               msgType: 0,
                                                               msgId: 0,
                                                               nsbc: 0,};
                                            init
                                        },
                                    subblocks: 0 as *mut rlPayloadSb_t,
                                    remChunks: 0,};
                    init
                };
            let mut outMsg: rlDriverMsg_t =
                {
                    let mut init =
                        rlDriverMsg{opcode:
                                        {
                                            let mut init =
                                                rlDriverOpcode{dir:
                                                                   0 as
                                                                       libc::c_int
                                                                       as
                                                                       rlUInt8_t,
                                                               msgType: 0,
                                                               msgId: 0,
                                                               nsbc: 0,};
                                            init
                                        },
                                    subblocks: 0 as *mut rlPayloadSb_t,
                                    remChunks: 0,};
                    init
                };
            let mut inPayloadSb: rlPayloadSb_t =
                {
                    let mut init =
                        rlPayloadSb{sbid: 0 as libc::c_int as rlUInt16_t,
                                    len: 0,
                                    pSblkData: 0 as *mut rlUInt8_t,};
                    init
                };
            let mut outPayloadSb: rlPayloadSb_t =
                {
                    let mut init =
                        rlPayloadSb{sbid: 0 as libc::c_int as rlUInt16_t,
                                    len: 0,
                                    pSblkData: 0 as *mut rlUInt8_t,};
                    init
                };
            let mut frameApplyCfgArgs: rlFrameApplyCfg_t =
                {
                    let mut init =
                        rlFrameApplyCfg{numChirps:
                                            0 as libc::c_int as rlUInt32_t,
                                        numAdcSamples: 0,
                                        reserved: 0,};
                    init
                };
            /* Construct command packet */
            rlDriverConstructInMsg(0x207 as libc::c_uint as rlUInt16_t,
                                   &mut inMsg, &mut inPayloadSb);
            /* Fill in-message Payload */
            /*AR_CODE_REVIEW MR:R.11.2 <APPROVED> "Need to pass NULL pointer" */
            /*LDRA_INSPECTED 95 S */
            rlDriverFillPayload(0x207 as libc::c_uint as rlUInt16_t,
                                0x3 as libc::c_uint as rlUInt16_t,
                                &mut inPayloadSb,
                                0 as *mut libc::c_void as *mut rlUInt8_t,
                                0 as libc::c_uint as rlUInt16_t);
            /* Construct response packet */
            rlDriverConstructOutMsg(1 as libc::c_uint as rlUInt16_t,
                                    &mut outMsg, &mut outPayloadSb);
            /* Fill out-message Payload */
            rlDriverFillPayload(0 as libc::c_uint as rlUInt16_t,
                                0 as libc::c_uint as rlUInt16_t,
                                &mut outPayloadSb,
                                &mut frameApplyCfgArgs as
                                    *mut rlFrameApplyCfg_t as *mut rlUInt8_t,
                                0 as libc::c_uint as rlUInt16_t);
            /* Send Command to mmWave Radar Device */
            retVal += rlDriverCmdInvoke(deviceMap, inMsg, &mut outMsg);
            /* update parameter received by Get Command */
            (*data).numAdcSamples = frameApplyCfgArgs.numAdcSamples
        }
    }
    return retVal;
}
/* * @fn rlReturnVal_t rlSetAdvFrameConfig(rlUInt8_t deviceMap, rlAdvFrameCfg_t* data)
*
*   @brief Sets Advance Frame Configuration
*   @param[in] deviceMap - Bitmap of devices to send the message
*   @param[in] data - Container for Advance Frame Configuration data
*
*   @return rlReturnVal_t Success - 0, Failure - Error Code
*
*   This function allows configuration of advance frame in mmWave Front end. Advance Frame is
*   a sequence of chirps and how this sequnece needs to be repeated over time. User first need
*   to define a profile and set of chirps(associated with a profile).\n This function then defines
*   how to sequence these chirps. Multiple chirps can be looped to create a burst. Multiple bursts
*   can be grouped to create a sub-frame. Multiple sub-frames(Upto 4) can be grouped to create
*   advance frame. \n This function defines the advance frame properties like the number of burst
*   in subframe, number of chirps and loops in a burst, sequence of subframes to be transmitted,
*   number of frames to be transmitted, periodicity of the frame and the trigger method. This API
*   calls internally two APIs, one to RadraSS for sensor configuraion and another to MasterSS for
*   datapath configuraion. \n
*
*   @note 1: If hardware trigger mode is used with subFrameTrigger = 0, then the trigger should be
*            issued for each burst. If subFrameTrigger = 1, then the trigger needs to be issued
*            for each sub-frame.
*   @note 2: If hardware triggered mode is used, the SYNC_IN pulse width should be less than 1us.
*            Also, the minimum pulse width of SYNC_IN should be 25 ns.
*   @note 3: If frame trigger delay is used with hardware triggered mode, then external SYNC IN
*            pulse periodicity should take care of the configured frame trigger delay and frame
*            periodicity. The external pulse should be issued only after the sum total of frame
*            trigger delay and frame periodicity.
*   @note 4: In Hw triggered mode, the Hw pulse should be issued or periodicity of pulse is
*            configured such that, the pulse is generated only 150us after the completion of
*            previous frame/burst (The pulse should not be issued before end of previous frame/
*            burst). The time delta between end of previous frame/burst and raising edge of Hw
*            pulse recommended to be < 300us.
*   @note 5: The PF_NUM_ADC_SAMPLES parameter should be identical across chirps in a sub-frame,
*            when multiple profiles are used in a sub-frame. \n
*   @note 6: The PF_DIGITAL_OUTPUT_SAMPLING_RATE impacts the LVDS/CSI2 data rate in a sub-frame,
*            so it is recommended to analyze timing impact if different sample rate is used across
*            chirps in a sub-frame. \n
*   @note 7: Please refer Table \ref interBurstTime and \ref interSubFrameTime for details on
*            minimum inter-frame blank time requirements. \n
*   @note 8: The loop-back configuration in this API is not supported in DFP 1.x (1st generation
*            devices). Please refer latest DFP release note for more info.
*/
/* DesignId : MMWL_DesignId_046 */
/* Requirements : AUTORADAR_REQ-795 */
#[no_mangle]
pub unsafe extern "C" fn rlSetAdvFrameConfig(mut deviceMap: rlUInt8_t,
                                             mut data: *mut rlAdvFrameCfg_t)
 -> rlReturnVal_t {
    let mut retVal: rlReturnVal_t = 0;
    /* check if deviceIndex is out of defined value */
    if rlDriverIsDeviceMapValid(deviceMap) != 0 as libc::c_int ||
           data.is_null() {
        /* set return error code */
        retVal = -(2 as libc::c_int)
    } else {
        /* Package the command with given data and send it to device */
        retVal =
            rlDriverExecuteSetApi(deviceMap,
                                  0x8 as libc::c_uint as rlUInt16_t,
                                  0x5 as libc::c_uint as rlUInt16_t,
                                  &mut (*data).frameSeq as
                                      *mut rlAdvFrameSeqCfg_t as
                                      *mut rlUInt8_t,
                                  ::std::mem::size_of::<rlAdvFrameSeqCfg_t>()
                                      as libc::c_ulong as rlUInt16_t);
        /* If only mmWaveLink instance is running on Host */
        if 0 as libc::c_int == retVal &&
               0 as libc::c_uint == rlDriverGetPlatformId() as libc::c_uint {
            /* Initialize Command and Response Sub Blocks */
            let mut inMsg: rlDriverMsg_t =
                {
                    let mut init =
                        rlDriverMsg{opcode:
                                        {
                                            let mut init =
                                                rlDriverOpcode{dir:
                                                                   0 as
                                                                       libc::c_int
                                                                       as
                                                                       rlUInt8_t,
                                                               msgType: 0,
                                                               msgId: 0,
                                                               nsbc: 0,};
                                            init
                                        },
                                    subblocks: 0 as *mut rlPayloadSb_t,
                                    remChunks: 0,};
                    init
                };
            let mut outMsg: rlDriverMsg_t =
                {
                    let mut init =
                        rlDriverMsg{opcode:
                                        {
                                            let mut init =
                                                rlDriverOpcode{dir:
                                                                   0 as
                                                                       libc::c_int
                                                                       as
                                                                       rlUInt8_t,
                                                               msgType: 0,
                                                               msgId: 0,
                                                               nsbc: 0,};
                                            init
                                        },
                                    subblocks: 0 as *mut rlPayloadSb_t,
                                    remChunks: 0,};
                    init
                };
            let mut inPayloadSb: rlPayloadSb_t =
                {
                    let mut init =
                        rlPayloadSb{sbid: 0 as libc::c_int as rlUInt16_t,
                                    len: 0,
                                    pSblkData: 0 as *mut rlUInt8_t,};
                    init
                };
            /* Construct command packet */
            rlDriverConstructInMsg(0x206 as libc::c_uint as rlUInt16_t,
                                   &mut inMsg, &mut inPayloadSb);
            /* Fill in-message Payload */
            rlDriverFillPayload(0x206 as libc::c_uint as rlUInt16_t,
                                0x1 as libc::c_uint as rlUInt16_t,
                                &mut inPayloadSb,
                                &mut (*data).frameData as
                                    *mut rlAdvFrameDataCfg_t as
                                    *mut rlUInt8_t,
                                ::std::mem::size_of::<rlAdvFrameDataCfg_t>()
                                    as libc::c_ulong as rlUInt16_t);
            /* Send Command to mmWave Radar Device */
            retVal += rlDriverCmdInvoke(deviceMap, inMsg, &mut outMsg)
        }
    }
    return retVal;
}
/* * @fn rlReturnVal_t rlGetAdvFrameConfig(rlUInt8_t deviceMap, rlAdvFrameCfg_t* data)
*
*   @brief Gets Advance Frame Configuration
*   @param[in] deviceMap - Bitmap of devices to send the message
*   @param[out] data - Container for Advance Frame Configuration data
*
*   @return rlReturnVal_t Success - 0, Failure - Error Code
*
*   This function reads the advance frame properties of the device. This API calls internally two
*   APIs, one to RadraSS for sensor configuraion and another to MasterSS for datapath
*   configuraion. \n
*/
/* DesignId : MMWL_DesignId_047 */
/* Requirements : AUTORADAR_REQ-796 */
#[no_mangle]
pub unsafe extern "C" fn rlGetAdvFrameConfig(mut deviceMap: rlUInt8_t,
                                             mut data: *mut rlAdvFrameCfg_t)
 -> rlReturnVal_t {
    let mut retVal: rlReturnVal_t = 0;
    /* check if deviceIndex is out of defined value */
    if rlDriverIsDeviceMapValid(deviceMap) != 0 as libc::c_int ||
           data.is_null() {
        /* set return error code */
        retVal = -(2 as libc::c_int)
    } else {
        /* Package the command with given data and send it to device */
        retVal =
            rlDriverExecuteGetApi(deviceMap,
                                  0x9 as libc::c_uint as rlUInt16_t,
                                  0x5 as libc::c_uint as rlUInt16_t,
                                  &mut (*data).frameSeq as
                                      *mut rlAdvFrameSeqCfg_t as
                                      *mut rlUInt8_t,
                                  0 as libc::c_uint as rlUInt16_t);
        /* If only mmWaveLink instance is running on Host */
        if 0 as libc::c_int == retVal &&
               0 as libc::c_uint == rlDriverGetPlatformId() as libc::c_uint {
            /* Initialize Command and Response Sub Blocks */
            let mut inMsg: rlDriverMsg_t =
                {
                    let mut init =
                        rlDriverMsg{opcode:
                                        {
                                            let mut init =
                                                rlDriverOpcode{dir:
                                                                   0 as
                                                                       libc::c_int
                                                                       as
                                                                       rlUInt8_t,
                                                               msgType: 0,
                                                               msgId: 0,
                                                               nsbc: 0,};
                                            init
                                        },
                                    subblocks: 0 as *mut rlPayloadSb_t,
                                    remChunks: 0,};
                    init
                };
            let mut outMsg: rlDriverMsg_t =
                {
                    let mut init =
                        rlDriverMsg{opcode:
                                        {
                                            let mut init =
                                                rlDriverOpcode{dir:
                                                                   0 as
                                                                       libc::c_int
                                                                       as
                                                                       rlUInt8_t,
                                                               msgType: 0,
                                                               msgId: 0,
                                                               nsbc: 0,};
                                            init
                                        },
                                    subblocks: 0 as *mut rlPayloadSb_t,
                                    remChunks: 0,};
                    init
                };
            let mut inPayloadSb: rlPayloadSb_t =
                {
                    let mut init =
                        rlPayloadSb{sbid: 0 as libc::c_int as rlUInt16_t,
                                    len: 0,
                                    pSblkData: 0 as *mut rlUInt8_t,};
                    init
                };
            let mut outPayloadSb: rlPayloadSb_t =
                {
                    let mut init =
                        rlPayloadSb{sbid: 0 as libc::c_int as rlUInt16_t,
                                    len: 0,
                                    pSblkData: 0 as *mut rlUInt8_t,};
                    init
                };
            /* Construct command packet */
            rlDriverConstructInMsg(0x207 as libc::c_uint as rlUInt16_t,
                                   &mut inMsg, &mut inPayloadSb);
            /* Fill in-message Payload */
            /*AR_CODE_REVIEW MR:R.11.2 <APPROVED> "Need to pass NULL pointer" */
            /*LDRA_INSPECTED 95 S */
            rlDriverFillPayload(0x207 as libc::c_uint as rlUInt16_t,
                                0x4 as libc::c_uint as rlUInt16_t,
                                &mut inPayloadSb,
                                0 as *mut libc::c_void as *mut rlUInt8_t,
                                0 as libc::c_uint as rlUInt16_t);
            /* Construct response packet */
            rlDriverConstructOutMsg(1 as libc::c_uint as rlUInt16_t,
                                    &mut outMsg, &mut outPayloadSb);
            /* Fill out-message Payload */
            rlDriverFillPayload(0 as libc::c_uint as rlUInt16_t,
                                0 as libc::c_uint as rlUInt16_t,
                                &mut outPayloadSb,
                                &mut (*data).frameData as
                                    *mut rlAdvFrameDataCfg_t as
                                    *mut rlUInt8_t,
                                0 as libc::c_uint as rlUInt16_t);
            /* Send Command to mmWave Radar Device */
            retVal += rlDriverCmdInvoke(deviceMap, inMsg, &mut outMsg)
        }
    }
    return retVal;
}
/* * @fn rlReturnVal_t rlSetContModeConfig(rlUInt8_t deviceMap, rlContModeCfg_t* data)
*
*   @brief Sets Continous mode Configuration
*   @param[in] deviceMap - Bitmap of devices to send the message
*   @param[in] data - Continous mode Configuration data
*
*   @return rlReturnVal_t Success - 0, Failure - Error Code
*
*   This function sets the FMCW radar continous mode properties like Start Freq, TX power
*   etc. In continuous mode, the signal is not frequency modulated but has the same frequency
*   over time.
*
*   @note : The continuous streaming mode configuration APIs are supported only for debug purpose.
*           Please refer latest DFP release note for more info.
*/
/* DesignId : MMWL_DesignId_038 */
/* Requirements : AUTORADAR_REQ-827 */
#[no_mangle]
pub unsafe extern "C" fn rlSetContModeConfig(mut deviceMap: rlUInt8_t,
                                             mut data: *mut rlContModeCfg_t)
 -> rlReturnVal_t {
    let mut retVal: rlReturnVal_t = 0;
    /* check if deviceIndex is out of defined value */
    if rlDriverIsDeviceMapValid(deviceMap) != 0 as libc::c_int ||
           data.is_null() {
        /* set return error code */
        retVal = -(2 as libc::c_int)
    } else {
        /* Package the command with given data and send it to device */
        retVal =
            rlDriverExecuteSetApi(deviceMap,
                                  0x8 as libc::c_uint as rlUInt16_t,
                                  0x3 as libc::c_uint as rlUInt16_t,
                                  data as *mut rlUInt8_t,
                                  ::std::mem::size_of::<rlContModeCfg_t>() as
                                      libc::c_ulong as rlUInt16_t)
    }
    return retVal;
}
/* * @fn rlReturnVal_t rlEnableContMode(rlUInt8_t deviceMap, rlContModeEn_t* data)
*
*   @brief Enable/Disable Continous mode
*   @param[in] deviceMap - Bitmap of devices to send the message
*   @param[in] data - Continous Mode enable/disable
*
*   @return rlReturnVal_t Success - 0, Failure - Error Code
*
*   This function enables/disables the FMCW radar continous mode
*/
/* DesignId : MMWL_DesignId_039 */
/* Requirements : AUTORADAR_REQ-828 */
#[no_mangle]
pub unsafe extern "C" fn rlEnableContMode(mut deviceMap: rlUInt8_t,
                                          mut data: *mut rlContModeEn_t)
 -> rlReturnVal_t {
    let mut retVal: rlReturnVal_t = 0;
    /* check if deviceIndex is out of defined value */
    if rlDriverIsDeviceMapValid(deviceMap) != 0 as libc::c_int ||
           data.is_null() {
        /* set return error code */
        retVal = -(2 as libc::c_int)
    } else {
        /* Package the command with given data and send it to device */
        retVal =
            rlDriverExecuteSetApi(deviceMap,
                                  0x8 as libc::c_uint as rlUInt16_t,
                                  0x4 as libc::c_uint as rlUInt16_t,
                                  data as *mut rlUInt8_t,
                                  ::std::mem::size_of::<rlContModeEn_t>() as
                                      libc::c_ulong as rlUInt16_t)
    }
    return retVal;
}
/* * @fn rlReturnVal_t rlSetLowPowerModeConfig(rlUInt8_t deviceMap, rlLowPowerModeCfg_t* data)
*
*   @brief Sets Low Power Mode Configuration
*   @param[in] deviceMap - Bitmap of devices to send the message
*   @param[in] data - Container for Low power mode Configuration data
*
*   @return rlReturnVal_t Success - 0, Failure - Error Code
*
*   This function sets the static device configurations of low power options. In xWR1xxx devices,
*   this allows the user to modify the Sigma Delta ADC root sampling clock rate (reducing rate to
*   half to save power in small IF bandwidth applications). In xWR6x43 devices, this API doesn't
*   modify the ADC root sampling rate, but reduces the power by reducing the bias currents to
*   some of the IF analog blocks.
*
*   @note : In xWR1xxx devices, Low power ADC mode is mandatory on 5 MHz part variant(for e.g.
*           xWR1642), Normally if IF band width <= 7.5MHz then low power mode setting is
*           recommended.
*/
/* DesignId :  MMWL_DesignId_011 */
/* Requirements : AUTORADAR_REQ-755 */
#[no_mangle]
pub unsafe extern "C" fn rlSetLowPowerModeConfig(mut deviceMap: rlUInt8_t,
                                                 mut data:
                                                     *mut rlLowPowerModeCfg_t)
 -> rlReturnVal_t {
    let mut retVal: rlReturnVal_t = 0;
    /* check if deviceIndex is out of defined value */
    if rlDriverIsDeviceMapValid(deviceMap) != 0 as libc::c_int ||
           data.is_null() {
        /* set return error code */
        retVal = -(2 as libc::c_int)
    } else {
        /* Package the command with given data and send it to device */
        retVal =
            rlDriverExecuteSetApi(deviceMap,
                                  0x4 as libc::c_uint as rlUInt16_t,
                                  0x3 as libc::c_uint as rlUInt16_t,
                                  data as *mut rlUInt8_t,
                                  ::std::mem::size_of::<rlLowPowerModeCfg_t>()
                                      as libc::c_ulong as rlUInt16_t)
    }
    return retVal;
}
/* * @fn rlReturnVal_t rlSensorStart(rlUInt8_t deviceMap)
*
*   @brief Triggers Transmission of Frames
*   @param[in] deviceMap - Bitmap of devices to send the message
*
*   @return rlReturnVal_t Success - 0, Failure - Error Code
*
*   This function triggers the transmission of the frames as per the frame and chirp configuration
*   If trigger mode is selected as SW API based trigger, mmWaveFront end would start chirp
*   immediately after receiving this API. If trigger mode is HW SYNC IN pulse, it would wait for
*   SYNC pulse
*
*   @note : Once the chirping starts, mmWave Front end would send asynchronous event
*           RL_RF_AE_FRAME_TRIGGER_RDY_SB indicating the start of frame
*/
/* DesignId : MMWL_DesignId_017 */
/* Requirements : AUTORADAR_REQ-759 */
#[no_mangle]
pub unsafe extern "C" fn rlSensorStart(mut deviceMap: rlUInt8_t)
 -> rlReturnVal_t {
    let mut retVal: rlReturnVal_t = 0;
    let mut frameTriggerArgs: rlFrameTrigger_t =
        rlFrameTrigger_t{startStop: 0, reserved: 0,};
    frameTriggerArgs.startStop = 0x1 as libc::c_uint as rlUInt16_t;
    frameTriggerArgs.reserved = 0 as libc::c_uint as rlUInt16_t;
    /* check if deviceIndex is out of defined value */
    if rlDriverIsDeviceMapValid(deviceMap) != 0 as libc::c_int {
        /* set return error code */
        retVal = -(2 as libc::c_int)
    } else {
        /* Package the command with given data and send it to device */
        retVal =
            rlDriverExecuteSetApi(deviceMap,
                                  0xa as libc::c_uint as rlUInt16_t,
                                  0 as libc::c_uint as rlUInt16_t,
                                  &mut frameTriggerArgs as
                                      *mut rlFrameTrigger_t as *mut rlUInt8_t,
                                  ::std::mem::size_of::<rlFrameTrigger_t>() as
                                      libc::c_ulong as rlUInt16_t)
    }
    return retVal;
}
/* * @fn rlReturnVal_t rlSensorStop(rlUInt8_t deviceMap)
*
*   @brief Stops Transmission of Frames
*   @param[in] deviceMap - Bitmap of devices to send the message
*
*   @return rlReturnVal_t Success - 0, Failure - Error Code
*
*   This function stops the transmission of the frames.
*
*   @note 1: Once the chirping stops, mmWave Front end would send asynchronous event
*            RL_RF_AE_FRAME_END_SB indicating the stop of frame
*   @note 2: When Frame Stop command is sent to RadarSS, the frame will be stopped after
*            completing all the chirps of a Frame/Advance frame. In non periodic HW triggered
*            mode if frame stop command is issued when HW pulses are paused (i.e after completing
*            previous frame) then a HW pulse is required to trigger next frame/bursts and frame
*            will be stopped at the end of this triggered frame. In HW triggered mode, the forced
*            frame stop is not supported, the frame end command is honored only if frames are
*            actively running.
*/
/* DesignId : MMWL_DesignId_018 */
/* Requirements : AUTORADAR_REQ-760 */
#[no_mangle]
pub unsafe extern "C" fn rlSensorStop(mut deviceMap: rlUInt8_t)
 -> rlReturnVal_t {
    let mut retVal: rlReturnVal_t = 0;
    let mut frameTriggerArgs: rlFrameTrigger_t =
        rlFrameTrigger_t{startStop: 0, reserved: 0,};
    frameTriggerArgs.startStop = 0 as libc::c_uint as rlUInt16_t;
    frameTriggerArgs.reserved = 0 as libc::c_uint as rlUInt16_t;
    /* check if deviceIndex is out of defined value */
    if rlDriverIsDeviceMapValid(deviceMap) != 0 as libc::c_int {
        /* set return error code */
        retVal = -(2 as libc::c_int)
    } else {
        /* Package the command with given data and send it to device */
        retVal =
            rlDriverExecuteSetApi(deviceMap,
                                  0xa as libc::c_uint as rlUInt16_t,
                                  0 as libc::c_uint as rlUInt16_t,
                                  &mut frameTriggerArgs as
                                      *mut rlFrameTrigger_t as *mut rlUInt8_t,
                                  ::std::mem::size_of::<rlFrameTrigger_t>() as
                                      libc::c_ulong as rlUInt16_t)
    }
    return retVal;
}
/* * @fn rlReturnVal_t rlRfInit(rlUInt8_t deviceMap)
*
*   @brief Initializes the RF/Analog Subsystem
*
*   @param[in] deviceMap - Bitmap of devices to send the message
*   @return rlReturnVal_t Success - 0, Failure - Error Code
*
*   Initializes the RF/Analog Subsystem. This triggers one time calibrations for APLL
*   and synthesizer. Calibration can be enabled/disabled using Calibraton configuration APIs.
*   Device will turn on Tx/Rx and components for the calibration which will cause higher curent
*   consumption momenterily.
*
*   @note 1: Once the calibration is complete, mmWave Front end would send asynchronous event
*            RL_RF_AE_INITCALIBSTATUS_SB indicating the result of the initialization/calibrations.
*            Application needs to wait for this Async event message before calling next APIs.
*   @note 2: The following boot-time calibrations are susceptible to corruption by interference.
*            The calibrations may result in false configuration of the RF analog sections due to
*            corruption by interference during the calibration measurements. \n
*            a. RX gain calibration (susceptible to interference) \n
*            b. RX IQMM calibration (susceptible to interference) \n
*            c. TX Phase calibration (susceptible to interference) \n
*            In the 60G band (supported by xWR6x43 devices), it is mandated by regulatory 
*            standards that transmissions in non-ISM band are capped to -10dBm. The following 
*            calibrations could violate these standards if executed in the field. \n
*            a. TX power calibration \n
*            b. TX phase shifter calibration \n
*            c. RX IQMM calibration \n
*            It is recommended to perform factory calibration and store the calibration data in
*            non volatile memory using rlRfCalibDataStore. This data can be restored to radar
*            device using rlRfCalibDataRestore API.
*   @note 3: In xWR1843 device, If only BPM is used without phase shifter then recommended to
*            disable the boot time phase shifter calibration in RF init.
*   @note 4: In xWR1843 device, If both phase shifter and BPM is used then recommended to
*            achieve BPM logic using per chirp phase shifter API instead of using BPM API.
*   @note 5: In xWR1xxx devices, it is not recommended to issue this API in runtime multiple 
*            times. This API shall be issued only once after power cycle with or without 
*            calibration data restore operation.
*/
/* DesignId : MMWL_DesignId_012 */
/* Requirements : AUTORADAR_REQ-758 */
#[no_mangle]
pub unsafe extern "C" fn rlRfInit(mut deviceMap: rlUInt8_t) -> rlReturnVal_t {
    /* Initialize Command and Response Sub Blocks */
    let mut retVal: rlReturnVal_t = 0;
    /* check if deviceIndex is out of defined value */
    if rlDriverIsDeviceMapValid(deviceMap) != 0 as libc::c_int {
        /* set return error code */
        retVal = -(2 as libc::c_int)
    } else {
        /* Package the command with given data and send it to device */
        retVal =
            rlDriverExecuteSetApi(deviceMap,
                                  0x6 as libc::c_uint as rlUInt16_t,
                                  0 as libc::c_uint as rlUInt16_t,
                                  0 as *mut rlUInt8_t,
                                  0 as libc::c_uint as rlUInt16_t)
    }
    return retVal;
}
/* * @fn rlReturnVal_t rlSetTestSourceConfig(rlUInt8_t deviceMap, rlTestSource_t* data)
*
*   @brief Configures the Test Source
*   @param[in] deviceMap - Bitmap of devices to send the message
*   @param[in] data - Container for Test source configuration
*
*   @return rlReturnVal_t Success - 0, Failure - Error Code
*
*   This API allows configuration of the Test Source in mmWave Front end. A Test source simulates
*   2 objects at certain position relative to the mmWave device and generates the RAW ADC data.
*   It also simulates velocity of objects, relative position of TX and RX antennas.

*   @note 1: This helps in checking the integrity of control and data path during development
*            phase. API is meant to be used in development phase only and doesn't relate to any
*            real use case.
*   @note 2: Test source is not characterized and tuned to 60GHz in xWR6843 devices.
*   @note 3: The test source configuration APIs are supported only for debug purpose. Please refer
*            latest DFP release note for more info.
*   @note 4: After test source usage, it is recommend to disable the test source and issue
*            profile configuration API again for normal functionality of radar.
*/
/* DesignId : MMWL_DesignId_035 */
/* Requirements : AUTORADAR_REQ-790 */
#[no_mangle]
pub unsafe extern "C" fn rlSetTestSourceConfig(mut deviceMap: rlUInt8_t,
                                               mut data: *mut rlTestSource_t)
 -> rlReturnVal_t {
    let mut retVal: rlReturnVal_t = 0;
    /* check if deviceIndex is out of defined value */
    if rlDriverIsDeviceMapValid(deviceMap) != 0 as libc::c_int ||
           data.is_null() {
        /* set return error code */
        retVal = -(2 as libc::c_int)
    } else {
        /* Package the command with given data and send it to device */
        retVal =
            rlDriverExecuteSetApi(deviceMap,
                                  0x16 as libc::c_uint as rlUInt16_t,
                                  0x2 as libc::c_uint as rlUInt16_t,
                                  data as *mut rlUInt8_t,
                                  ::std::mem::size_of::<rlTestSource_t>() as
                                      libc::c_ulong as rlUInt16_t)
    }
    return retVal;
}
/* * @fn rlReturnVal_t rlTestSourceEnable(rlUInt8_t deviceMap, rlTestSourceEnable_t* data)
*
*   @brief Enables the Test Source
*   @param[in] deviceMap - Bitmap of devices to send the message
*   @param[in] data - Container for Test source enable parameters
*
*   @return rlReturnVal_t Success - 0, Failure - Error Code
*
*   Enables the Test Source that is configured using \ref rlSetTestSourceConfig API
*
*   @note : Test source is not characterized and tuned to 60GHz in xWR6843 devices
*/
/* DesignId : MMWL_DesignId_036 */
/* Requirements : AUTORADAR_REQ-791 */
#[no_mangle]
pub unsafe extern "C" fn rlTestSourceEnable(mut deviceMap: rlUInt8_t,
                                            mut data:
                                                *mut rlTestSourceEnable_t)
 -> rlReturnVal_t {
    let mut retVal: rlReturnVal_t = 0;
    /* check if deviceIndex is out of defined value */
    if rlDriverIsDeviceMapValid(deviceMap) != 0 as libc::c_int ||
           data.is_null() {
        /* set return error code */
        retVal = -(2 as libc::c_int)
    } else {
        /* Package the command with given data and send it to device */
        retVal =
            rlDriverExecuteSetApi(deviceMap,
                                  0x16 as libc::c_uint as rlUInt16_t,
                                  0x3 as libc::c_uint as rlUInt16_t,
                                  data as *mut rlUInt8_t,
                                  ::std::mem::size_of::<rlTestSourceEnable_t>()
                                      as libc::c_ulong as rlUInt16_t)
    }
    return retVal;
}
/* * @fn rlReturnVal_t rlRfGetTemperatureReport(rlUInt8_t deviceMap, rlRfTempData_t* data)
*
*   @brief Gets Time and Temperature information report
*   @param[in] deviceMap - Bitmap of devices to send the message
*   @param[out] data - Structure to store temperature report from all the temp sensors
*
*   @return rlReturnVal_t Success - 0, Failure - Error Code
*
*   This function reads Temperature information from all temperature sensors in the device
*/
/* DesignId : MMWL_DesignId_029 */
/* Requirements : AUTORADAR_REQ-789 */
#[no_mangle]
pub unsafe extern "C" fn rlRfGetTemperatureReport(mut deviceMap: rlUInt8_t,
                                                  mut data:
                                                      *mut rlRfTempData_t)
 -> rlReturnVal_t {
    let mut retVal: rlReturnVal_t = 0;
    /* check if deviceIndex is out of defined value */
    if rlDriverIsDeviceMapValid(deviceMap) != 0 as libc::c_int ||
           data.is_null() {
        /* set return error code */
        retVal = -(2 as libc::c_int)
    } else {
        /* Package the command with given data and send it to device */
        retVal =
            rlDriverExecuteGetApi(deviceMap,
                                  0x17 as libc::c_uint as rlUInt16_t,
                                  0xa as libc::c_uint as rlUInt16_t,
                                  data as *mut rlUInt8_t,
                                  ::std::mem::size_of::<rlRfTempData_t>() as
                                      libc::c_ulong as rlUInt16_t)
    }
    return retVal;
}
/* * @fn rlReturnVal_t rlRfDfeRxStatisticsReport(rlUInt8_t deviceMap, rlDfeStatReport_t* data)
*
*   @brief Gets Digital Front end statistics such as Residual DC, RMS power in I and
*          Q chains for different Receive channels for different selected profiles.
*          It also includes Cross correlation between I and Q chains
*   @param[in] deviceMap - Bitmap of devices to send the message
*   @param[in] data      - Container of dfe receiver status report
*
*   @return rlReturnVal_t Success - 0, Failure - Error Code
*
*   Gets Digital Front end statistics such as Residual DC, RMS power in I and
*   Q chains for different Receive channels for different selected profiles.
*   It also includes Cross correlation between I and Q chains
*
*   @note : The DFE statistics GET API is not supported in this release. Please refer latest DFP
*           release note for more info.
*/
/* DesignId : MMWL_DesignId_041 */
/* Requirements : AUTORADAR_REQ-788 */
#[no_mangle]
pub unsafe extern "C" fn rlRfDfeRxStatisticsReport(mut deviceMap: rlUInt8_t,
                                                   mut data:
                                                       *mut rlDfeStatReport_t)
 -> rlReturnVal_t {
    let mut retVal: rlReturnVal_t = 0;
    /* check if deviceIndex is out of defined value */
    if rlDriverIsDeviceMapValid(deviceMap) != 0 as libc::c_int ||
           data.is_null() {
        /* set return error code */
        retVal = -(2 as libc::c_int)
    } else {
        /* Package the command with given data and send it to device */
        retVal =
            rlDriverExecuteGetApi(deviceMap,
                                  0x13 as libc::c_uint as rlUInt16_t,
                                  0 as libc::c_uint as rlUInt16_t,
                                  data as *mut rlUInt8_t,
                                  0 as libc::c_uint as rlUInt16_t)
    }
    return retVal;
}
/* * @fn rlReturnVal_t rlRfDynamicPowerSave(rlUInt8_t deviceMap, rlDynPwrSave_t* data)
*
*   @brief : Configure dynamic power saving feature.
*   @param[in] deviceMap - Bitmap of devices to send the message
*   @param[in] data      - Container of dynamic power save information
*
*   @return rlReturnVal_t Success - 0, Failure - Error Code
*
*   Configure dynamic power saving feature during Inter chirp Idle time by
*   turning off various circuits such as Transmitter, Receiver and LO distribution
*   blocks
*   @note 1: whether to enable dynamic power saving during inter-chirp IDLE times by turning off
*            various circuits e.g. TX, RX, LO Distribution blocks. If Idle time + Tx start
*            time < 10us or Idle time < 3.5us then inter-chirp dynamic power save option will be
*            disabled, in that case, 15us of inter-burst idle time will be utilized to configure
*            sequencer LO, TX and RX signal timings by firmware. \n
*   @note 2: All the 3 configuration bits (TX, RX and LO) should have same value, i.e. user
*            should program value 0x7 to enable power save or 0x0 to disable the power save in
*            BLOCK_CFG.
*/
/* DesignId : MMWL_DesignId_019 */
/* Requirements : AUTORADAR_REQ-797 */
#[no_mangle]
pub unsafe extern "C" fn rlRfDynamicPowerSave(mut deviceMap: rlUInt8_t,
                                              mut data: *mut rlDynPwrSave_t)
 -> rlReturnVal_t {
    let mut retVal: rlReturnVal_t = 0;
    /* check if deviceIndex is out of defined value */
    if rlDriverIsDeviceMapValid(deviceMap) != 0 as libc::c_int ||
           data.is_null() {
        /* set return error code */
        retVal = -(2 as libc::c_int)
    } else {
        /* Package the command with given data and send it to device */
        retVal =
            rlDriverExecuteSetApi(deviceMap,
                                  0x4 as libc::c_uint as rlUInt16_t,
                                  0x4 as libc::c_uint as rlUInt16_t,
                                  data as *mut rlUInt8_t,
                                  ::std::mem::size_of::<rlDynPwrSave_t>() as
                                      libc::c_ulong as rlUInt16_t)
    }
    return retVal;
}
/* * @fn rlReturnVal_t rlRfSetDeviceCfg(rlUInt8_t deviceMap, rlRfDevCfg_t* data)
*
*   @brief : Set different RadarSS device configurations
*   @param[in] deviceMap - Bitmap of devices to send the message
*   @param[in] data      - Configuration parameter for AE.
*
*   @return rlReturnVal_t Success - 0, Failure - Error Code
*
*   Set different RadarSS device configurations. Enable and Configure asynchronous event
*   direction for device. By default all asynchronous event are enabled and sent to
*   the platform which issued the API. Below events can be configured to be received
*   on different platform by using this API:\n
*   [1.] CPU_FAULT [2.] ESM_FAULT [3.] ANALOG_FAULT
*   Similarly all monitoring events can be configured to be received on specific platform
*   using this API
*   Below events can be disabled using this API:\n
*   [1.] FRAME_START_ASYNC_EVENT [2.] FRAME_STOP_ASYNC_EVENT \n
*   Enable[1]/Disable[0] RadarSS Watchdog, where by default it is disable. Configure CRC type
*   for asynchronous event from RadarSS  [0] 16Bit, [1] 32Bit, [2] 64Bit.
*/
/* DesignId : MMWL_DesignId_042 */
/* Requirements : AUTORADAR_REQ-794 */
#[no_mangle]
pub unsafe extern "C" fn rlRfSetDeviceCfg(mut deviceMap: rlUInt8_t,
                                          mut data: *mut rlRfDevCfg_t)
 -> rlReturnVal_t {
    let mut retVal: rlReturnVal_t = 0;
    /* check if deviceIndex is out of defined value */
    if rlDriverIsDeviceMapValid(deviceMap) != 0 as libc::c_int ||
           data.is_null() {
        /* set return error code */
        retVal = -(2 as libc::c_int)
    } else {
        /* Package the command with given data and send it to device */
        retVal =
            rlDriverExecuteSetApi(deviceMap,
                                  0x4 as libc::c_uint as rlUInt16_t,
                                  0x6 as libc::c_uint as rlUInt16_t,
                                  data as *mut rlUInt8_t,
                                  ::std::mem::size_of::<rlRfDevCfg_t>() as
                                      libc::c_ulong as rlUInt16_t)
    }
    return retVal;
}
/* * @fn rlReturnVal_t rlSetGpAdcConfig(rlUInt8_t deviceMap, rlGpAdcCfg_t* data)
*
*   @brief : Configure GP ADC data parameters
*   @param[in] deviceMap - Bitmap of devices to send the message
*   @param[in] data      - Configuration parameter for GP ADC
*
*   @return rlReturnVal_t Success - 0, Failure - Error Code
*
*   This API enables the GPADC reads for external inputs (available only in xWR1642/xWR6843/
*   xWR1843). xWR1642/xWR1843 sends GP-ADC measurement data in async event
*   RL_RF_AE_GPADC_MEAS_DATA_SB
*
*   @note : The actual measurement of these GPADC signal are done in inter-burst or frame idle
*           time and the result AE sub block will be sent only after completing all the
*           measurements.
*/
/* DesignId : MMWL_DesignId_044 */
/* Requirements : AUTORADAR_REQ-792 */
#[no_mangle]
pub unsafe extern "C" fn rlSetGpAdcConfig(mut deviceMap: rlUInt8_t,
                                          mut data: *mut rlGpAdcCfg_t)
 -> rlReturnVal_t {
    let mut retVal: rlReturnVal_t = 0;
    /* check if deviceIndex is out of defined value */
    if rlDriverIsDeviceMapValid(deviceMap) != 0 as libc::c_int ||
           data.is_null() {
        /* set return error code */
        retVal = -(2 as libc::c_int)
    } else {
        /* Package the command with given data and send it to device */
        retVal =
            rlDriverExecuteSetApi(deviceMap,
                                  0x16 as libc::c_uint as rlUInt16_t,
                                  0x10 as libc::c_uint as rlUInt16_t,
                                  data as *mut rlUInt8_t,
                                  ::std::mem::size_of::<rlGpAdcCfg_t>() as
                                      libc::c_ulong as rlUInt16_t)
    }
    return retVal;
}
/* * @fn rlReturnVal_t rlRfSetLdoBypassConfig(rlUInt8_t deviceMap, rlRfCalibDisableCfg_t* data)
*
*   @brief Enables/Disables LDO bypass mode. By default Internal LDO is enabled
*          and 1.3V supply is required for RF. If external PMIC supplies 1.0V RF
*          Supply directly, Internal LDO has to be bypassed using this API.
*   @param[out] data - LDO enable/disable configuration
*
*   @return rlReturnVal_t Success - 0, Failure - Error Code
*
*   This function Enables/Disables LDO bypass mode.
*   @note : Refer to EVM/Board user guide to understand the Power Supply scheme before using this
*           API. If 1.3 V RF supply is provided and this API is called to enable LDO bypass mode,
*           devive could burn. Typically in TI EVMs, PMIC is configured to supply 1.3V to the RF
*           supplies, so in that case don't bypass LDO.
*/
/* DesignId : MMWL_DesignId_043 */
/* Requirements : AUTORADAR_REQ-793 */
#[no_mangle]
pub unsafe extern "C" fn rlRfSetLdoBypassConfig(mut deviceMap: rlUInt8_t,
                                                mut data:
                                                    *mut rlRfLdoBypassCfg_t)
 -> rlReturnVal_t {
    let mut retVal: rlReturnVal_t = 0;
    /* check if deviceIndex is out of defined value */
    if rlDriverIsDeviceMapValid(deviceMap) != 0 as libc::c_int ||
           data.is_null() {
        /* set return error code */
        retVal = -(2 as libc::c_int)
    } else {
        /* Package the command with given data and send it to device */
        retVal =
            rlDriverExecuteSetApi(deviceMap,
                                  0x16 as libc::c_uint as rlUInt16_t,
                                  0xc as libc::c_uint as rlUInt16_t,
                                  data as *mut rlUInt8_t,
                                  ::std::mem::size_of::<rlRfLdoBypassCfg_t>()
                                      as libc::c_ulong as rlUInt16_t)
    }
    return retVal;
}
/* * @fn rlReturnVal_t rlRfSetPhaseShiftConfig(rlUInt8_t deviceMap, rlUInt16_t cnt,
                                                rlRfPhaseShiftCfg_t* data)
*
*   @brief Enable/Disable phase shift configurations per chirp in each of the TXs
*   @param[in] deviceMap - Bitmap of devices to send the message
*   @param[in] cnt  - Number of configurations
*   @param[in] data - phase shift enable/disable configuration
*
*   @return rlReturnVal_t Success - 0, Failure - Error Code
*
*   This function configures the static phase shift configurations per chirp in each of the
*   TXs. This API is applicable only in certain devices (please refer data sheet). This API
*   will be honored after enabling per chirp phase shifter in rlRfSetMiscConfig.
*
*   @note 1 : Phase shifters are applied at the knee of the ramp.
*/
/* DesignId : MMWL_DesignId_045 */
/* Requirements : AUTORADAR_REQ-798 */
#[no_mangle]
pub unsafe extern "C" fn rlRfSetPhaseShiftConfig(mut deviceMap: rlUInt8_t,
                                                 mut cnt: rlUInt16_t,
                                                 mut data:
                                                     *mut rlRfPhaseShiftCfg_t)
 -> rlReturnVal_t {
    let mut retVal: rlReturnVal_t = 0;
    /* check if deviceIndex is out of defined value */
    if rlDriverIsDeviceMapValid(deviceMap) != 0 as libc::c_int ||
           data.is_null() || cnt as libc::c_uint == 0 as libc::c_uint {
        /* set return error code */
        retVal = -(2 as libc::c_int)
    } else {
        /* Initialize Command and Response Sub Blocks */
        let mut inMsg: rlDriverMsg_t =
            rlDriverMsg_t{opcode:
                              rlDriverOpcode_t{dir: 0,
                                               msgType: 0,
                                               msgId: 0,
                                               nsbc: 0,},
                          subblocks: 0 as *mut rlPayloadSb_t,
                          remChunks: 0,};
        let mut outMsg: rlDriverMsg_t =
            {
                let mut init =
                    rlDriverMsg{opcode:
                                    {
                                        let mut init =
                                            rlDriverOpcode{dir:
                                                               0 as
                                                                   libc::c_int
                                                                   as
                                                                   rlUInt8_t,
                                                           msgType: 0,
                                                           msgId: 0,
                                                           nsbc: 0,};
                                        init
                                    },
                                subblocks: 0 as *mut rlPayloadSb_t,
                                remChunks: 0,};
                init
            };
        /* Index and other paramerters to send multiple sub blocks in one/more commands chunk */
        let mut sbLen: rlUInt16_t = 0;
        let mut maxSbCntInMsg: rlUInt16_t = 0;
        let mut sbCntInMsg: rlUInt16_t = 0;
        let mut numChnkOfMsg: rlUInt16_t = 0;
        let mut lastSbCntInMsg: rlUInt16_t = 0;
        let mut loopCnt: rlUInt16_t = 0;
        let mut indx: rlUInt16_t = 0;
        /* Initialize Command and Response Sub Blocks */
        let mut inPayloadSb: [rlPayloadSb_t; 19] =
            [rlPayloadSb_t{sbid: 0, len: 0, pSblkData: 0 as *mut rlUInt8_t,};
                19];
        /* single sub-block length for chirp config API */
        sbLen =
            (::std::mem::size_of::<rlRfPhaseShiftCfg_t>() as
                 libc::c_ulong).wrapping_add((2 as
                                                  libc::c_uint).wrapping_add(2
                                                                                 as
                                                                                 libc::c_uint)
                                                 as libc::c_ulong) as
                rlUInt16_t;
        /* get Max Sub Block count */
        /* AR_CODE_REVIEW MR:D.4.1 <APPROVED> "sbLen can not be zero" */
        /*LDRA_INSPECTED 127 D */
        maxSbCntInMsg =
            (256 as
                 libc::c_uint).wrapping_sub((4 as
                                                 libc::c_uint).wrapping_add(12
                                                                                as
                                                                                libc::c_uint).wrapping_add(8
                                                                                                               as
                                                                                                               libc::c_uint)).wrapping_div(sbLen
                                                                                                                                               as
                                                                                                                                               libc::c_uint)
                as rlUInt16_t;
        retVal = 0 as libc::c_int;
        /* if requested count of chirpConfig is within one Message packet */
        if cnt as libc::c_int <= maxSbCntInMsg as libc::c_int {
            sbCntInMsg = cnt;
            numChnkOfMsg = 1 as libc::c_uint as rlUInt16_t;
            lastSbCntInMsg = 0 as libc::c_uint as rlUInt16_t
        } else {
            sbCntInMsg = maxSbCntInMsg;
            numChnkOfMsg =
                (cnt as libc::c_int / maxSbCntInMsg as libc::c_int) as
                    rlUInt16_t;
            lastSbCntInMsg =
                (cnt as libc::c_int % maxSbCntInMsg as libc::c_int) as
                    rlUInt16_t
        }
        /* Fill in-message packet */
        rlDriverConstructInMsg(0x8 as libc::c_uint as rlUInt16_t, &mut inMsg,
                               &mut *inPayloadSb.as_mut_ptr().offset(0 as
                                                                         libc::c_int
                                                                         as
                                                                         isize));
        /* get the loop count in form of number of msg chunks */
        loopCnt =
            if lastSbCntInMsg as libc::c_uint == 0 as libc::c_uint {
                numChnkOfMsg as libc::c_uint
            } else {
                (numChnkOfMsg as libc::c_uint).wrapping_add(1 as libc::c_uint)
            } as rlUInt16_t;
        while retVal == 0 as libc::c_int {
            /* all full messages have been sent, then send last partial message */
            if loopCnt as libc::c_uint == 1 as libc::c_uint &&
                   lastSbCntInMsg as libc::c_uint != 0 as libc::c_uint {
                inMsg.opcode.nsbc = lastSbCntInMsg
            } else { inMsg.opcode.nsbc = sbCntInMsg }
            indx = 0 as libc::c_uint as rlUInt16_t;
            while (indx as libc::c_int) < inMsg.opcode.nsbc as libc::c_int {
                /* Set Command Sub Block*/
                /* AR_CODE_REVIEW MR:R.2.2  <APPROVED> "Values are used by called function. \
                LDRA Tool Issue" */
                /*LDRA_INSPECTED 105 D */
                inPayloadSb[indx as usize].sbid =
                    (0x8 as
                         libc::c_uint).wrapping_mul(32 as
                                                        libc::c_uint).wrapping_add(0x6
                                                                                       as
                                                                                       libc::c_uint)
                        as rlUInt16_t;
                /* AR_CODE_REVIEW MR:R.2.2  <APPROVED> "Values are used by called function. \
                LDRA Tool Issue" */
                /*LDRA_INSPECTED 105 D */
                inPayloadSb[indx as usize].len =
                    ::std::mem::size_of::<rlRfPhaseShiftCfg_t>() as
                        libc::c_ulong as rlUInt16_t;
                /* AR_CODE_REVIEW MR:R.2.2  <APPROVED> "Values are used by called function. \
                LDRA Tool Issue" */
                /*LDRA_INSPECTED 105 D */
                inPayloadSb[indx as usize].pSblkData = data as *mut rlUInt8_t;
                /*AR_CODE_REVIEW MR:R.18.1 <APPROVED> "require pointer increment to jump to \
                  next chirp Config */
                /*LDRA_INSPECTED 567 S */
                data = data.offset(1);
                indx = indx.wrapping_add(1)
            }
            /* Send Command to mmWave Radar Device */
            /* AR_CODE_REVIEW MR:R.2.2  <APPROVED> "Values are used by called function. \
            LDRA Tool Issue" */
            /*LDRA_INSPECTED 105 D */
            retVal += rlDriverCmdInvoke(deviceMap, inMsg, &mut outMsg);
            /* decrement the loop count */
            loopCnt = loopCnt.wrapping_sub(1);
            /* if all phase shift configs have been sent to device then terminate the loop */
            if loopCnt as libc::c_uint == 0 as libc::c_uint { break ; }
        }
    }
    return retVal;
}
/* * @fn rlReturnVal_t rlRfSetPALoopbackConfig(rlUInt8_t deviceMap, rlRfPALoopbackCfg_t* data)
*
*   @brief Enable/Disable PA loopback for all enabled profiles
*   @param[in] deviceMap - Bitmap of devices to send the message
*   @param[in] data - PA loopback configuration
*
*   @return rlReturnVal_t Success - 0, Failure - Error Code
*
*   This function Enables/Disables PA loopback for all enabled profiles. This is
*   used for debug purpose that both the TX and RX paths are working properly
*
*   @note 1: The PA loop-back configuration API is supported only for debug purpose. Please refer
*            latest DFP release note for more info.
*/
/* DesignId :  MMWL_DesignId_098 */
/* Requirements : AUTORADAR_REQ-801 */
#[no_mangle]
pub unsafe extern "C" fn rlRfSetPALoopbackConfig(mut deviceMap: rlUInt8_t,
                                                 mut data:
                                                     *mut rlRfPALoopbackCfg_t)
 -> rlReturnVal_t {
    let mut retVal: rlReturnVal_t = 0;
    /* check if deviceIndex is out of defined value */
    if rlDriverIsDeviceMapValid(deviceMap) != 0 as libc::c_int ||
           data.is_null() {
        /* set return error code */
        retVal = -(2 as libc::c_int)
    } else {
        /* Package the command with given data and send it to device */
        retVal =
            rlDriverExecuteSetApi(deviceMap,
                                  0x16 as libc::c_uint as rlUInt16_t,
                                  0xd as libc::c_uint as rlUInt16_t,
                                  data as *mut rlUInt8_t,
                                  ::std::mem::size_of::<rlRfPALoopbackCfg_t>()
                                      as libc::c_ulong as rlUInt16_t)
    }
    return retVal;
}
/* * @fn rlReturnVal_t rlRfSetPSLoopbackConfig(rlUInt8_t deviceMap, rlRfPSLoopbackCfg_t* data)
*
*   @brief Enable/Disable Phase shift loopback for all enabled profiles
*   @param[in] deviceMap - Bitmap of devices to send the message
*   @param[in] data - Phase shift loopback configuration
*
*   @return rlReturnVal_t Success - 0, Failure - Error Code
*
*   This function Enables/Disables Phase shift loopback for all enabled profiles.This is used
*   to debug the TX (before the PA) and RX chains.
*
*   @note 1: The PS loop-back configuration API is supported only for debug purpose. Please refer
*            latest DFP release note for more info.
*   @note 2: The expected signal strength change with change in index value is only approximately 
*            indicated for PS<n>_PGA_GAIN_INDEX. Typically, the loopback path is the dominant 
*            path only in top ~10 indices (highest PGA gain values). For lower indices (lower PGA 
*            gain values), parasitic paths in the RF system can start dominating the loop-back 
*            measurements, and under such conditions, inter channel imbalances measured using 
*            such LB path, and LB signal SNR etc. can show degraded performance, with the 
*            degradation attributed to the loop-back path and not the functional 
*            path/circuits/system.
*/
/* DesignId : MMWL_DesignId_099 */
/* Requirements : AUTORADAR_REQ-802 */
#[no_mangle]
pub unsafe extern "C" fn rlRfSetPSLoopbackConfig(mut deviceMap: rlUInt8_t,
                                                 mut data:
                                                     *mut rlRfPSLoopbackCfg_t)
 -> rlReturnVal_t {
    let mut retVal: rlReturnVal_t = 0;
    /* check if deviceIndex is out of defined value */
    if rlDriverIsDeviceMapValid(deviceMap) != 0 as libc::c_int ||
           data.is_null() {
        /* set return error code */
        retVal = -(2 as libc::c_int)
    } else {
        /* Package the command with given data and send it to device */
        retVal =
            rlDriverExecuteSetApi(deviceMap,
                                  0x16 as libc::c_uint as rlUInt16_t,
                                  0xe as libc::c_uint as rlUInt16_t,
                                  data as *mut rlUInt8_t,
                                  ::std::mem::size_of::<rlRfPSLoopbackCfg_t>()
                                      as libc::c_ulong as rlUInt16_t)
    }
    return retVal;
}
/* * @fn rlReturnVal_t rlRfSetIFLoopbackConfig(rlUInt8_t deviceMap, rlRfIFLoopbackCfg_t* data)
*
*   @brief Enable/Disable RF IF loopback for all enabled profiles.
*    This is used for debug to check if both TX and RX chains are working correctly.
*   @param[in] deviceMap - Bitmap of devices to send the message
*   @param[in] data - IF loopback configuration
*
*   @return rlReturnVal_t Success - 0, Failure - Error Code
*
*   This function Enables/Disables RF IF loopback for all enabled profiles. This is used to
*   debug the  RX IF chain.
*
*   @note 1: The IF loop-back configuration API is supported only for debug purpose. Please refer
*            latest DFP release note for more info.
*/
/* DesignId :  MMWL_DesignId_100 */
/* Requirements : AUTORADAR_REQ-803 */
#[no_mangle]
pub unsafe extern "C" fn rlRfSetIFLoopbackConfig(mut deviceMap: rlUInt8_t,
                                                 mut data:
                                                     *mut rlRfIFLoopbackCfg_t)
 -> rlReturnVal_t {
    let mut retVal: rlReturnVal_t = 0;
    /* check if deviceIndex is out of defined value */
    if rlDriverIsDeviceMapValid(deviceMap) != 0 as libc::c_int ||
           data.is_null() {
        /* set return error code */
        retVal = -(2 as libc::c_int)
    } else {
        /* Package the command with given data and send it to device */
        retVal =
            rlDriverExecuteSetApi(deviceMap,
                                  0x16 as libc::c_uint as rlUInt16_t,
                                  0xf as libc::c_uint as rlUInt16_t,
                                  data as *mut rlUInt8_t,
                                  ::std::mem::size_of::<rlRfIFLoopbackCfg_t>()
                                      as libc::c_ulong as rlUInt16_t)
    }
    return retVal;
}
/* * @fn rlReturnVal_t rlRfSetProgFiltCoeffRam(rlUInt8_t deviceMap, rlRfProgFiltCoeff_t* data)
*
*   @brief Set Programmable Filter coefficient RAM
*   @param[in] deviceMap - Bitmap of devices to send the message
*   @param[in] data - array of coefficients for the programmable filter
*
*   @return rlReturnVal_t Success - 0, Failure - Error Code
*
*   This function is used to program the coefficients for the external programmable filter.
*   This API is applicable only in xWR1642/xWR1843.
*
*   @note 1: The programmable filter is applicable in Complex 1X and Real-only output modes for
*            sampling rates under 6.25 Msps (Complex 1X) and under 12.5 Msps (Real). This is to
*            allow for a trade-off between digital filter chain setting time and close-in anti-
*            alias attenuation. \n A real-coefficient FIR with up to 26 taps (16-bit coefficients)
*            is supported in the Complex 1X output mode, and a real-coefficient FIR with up to 20
*            taps (16-bit coefficients) in supported in the Real output mode.
*   @note 2: This API should be issued before rlSetProfileConfig.
*   @note 3: This API should not be issued when frames are ongoing.
*   @note 4: The programmable filter configuration APIs are not supported in DFP 1.x (1st
*            generation devices). Please refer latest DFP release note for more info.
*/
/* DesignId :  MMWL_DesignId_050 */
/* Requirements : AUTORADAR_REQ-799 */
#[no_mangle]
pub unsafe extern "C" fn rlRfSetProgFiltCoeffRam(mut deviceMap: rlUInt8_t,
                                                 mut data:
                                                     *mut rlRfProgFiltCoeff_t)
 -> rlReturnVal_t {
    let mut retVal: rlReturnVal_t = 0;
    /* check if deviceIndex is out of defined value */
    if rlDriverIsDeviceMapValid(deviceMap) != 0 as libc::c_int ||
           data.is_null() {
        /* set return error code */
        retVal = -(2 as libc::c_int)
    } else {
        /* Package the command with given data and send it to device */
        retVal =
            rlDriverExecuteSetApi(deviceMap,
                                  0x8 as libc::c_uint as rlUInt16_t,
                                  0x7 as libc::c_uint as rlUInt16_t,
                                  data as *mut rlUInt8_t,
                                  ::std::mem::size_of::<rlRfProgFiltCoeff_t>()
                                      as libc::c_ulong as rlUInt16_t)
    }
    return retVal;
}
/* * @fn rlReturnVal_t rlRfSetProgFiltConfig(rlUInt8_t deviceMap, rlRfProgFiltConf_t* data)
*
*   @brief Set Programmable Filter configuration
*   @param[in] deviceMap - Bitmap of devices to send the message
*   @param[in] data - programmable filter configuration
*
*   @return rlReturnVal_t Success - 0, Failure - Error Code
*
*   This function selects programmable filter cofficient RAM and
*   map it to configured profile ID.
*
*   @note 1: This API is applicable only in xWR1642/xWR1843
*   @note 2: This API should not be issued when frames are ongoing.
*   @note 3: The programmable filter configuration APIs are not supported in this release. Please
*            refer latest DFP release note for more info.
*/
/* DesignId : MMWL_DesignId_051 */
/* Requirements : AUTORADAR_REQ-800 */
#[no_mangle]
pub unsafe extern "C" fn rlRfSetProgFiltConfig(mut deviceMap: rlUInt8_t,
                                               mut data:
                                                   *mut rlRfProgFiltConf_t)
 -> rlReturnVal_t {
    let mut retVal: rlReturnVal_t = 0;
    /* check if deviceIndex is out of defined value */
    if rlDriverIsDeviceMapValid(deviceMap) != 0 as libc::c_int ||
           data.is_null() {
        /* set return error code */
        retVal = -(2 as libc::c_int)
    } else {
        /* Package the command with given data and send it to device */
        retVal =
            rlDriverExecuteSetApi(deviceMap,
                                  0x8 as libc::c_uint as rlUInt16_t,
                                  0x8 as libc::c_uint as rlUInt16_t,
                                  data as *mut rlUInt8_t,
                                  ::std::mem::size_of::<rlRfProgFiltConf_t>()
                                      as libc::c_ulong as rlUInt16_t)
    }
    return retVal;
}
/* * @fn rlReturnVal_t rlRfSetMiscConfig(rlUInt8_t deviceMap, rlRfMiscConf_t* data)
*
*   @brief Sets misc feature such as per chirp phase shifter.
*   @param[in] deviceMap - Bitmap of devices to send the message
*   @param[in] data - Misc configuration such as per chirp phase shifter
*
*   @return rlReturnVal_t Success - 0, Failure - Error Code
*
*   This function enables misc feature such as per chirp phase shifter.This API
*   is valid for devices for which phase shifter is enabled(refer data sheet).
*
*   @note : Issue this API first in the sequence if rlRfSetPhaseShiftConfig and
*           rlSetDynPerChirpPhShifterCfg are issued down in the sequence.
*/
/* DesignId :  MMWL_DesignId_052 */
/* Requirements : AUTORADAR_REQ-891 */
#[no_mangle]
pub unsafe extern "C" fn rlRfSetMiscConfig(mut deviceMap: rlUInt8_t,
                                           mut data: *mut rlRfMiscConf_t)
 -> rlReturnVal_t {
    let mut retVal: rlReturnVal_t = 0;
    /* check if deviceIndex is out of defined value */
    if rlDriverIsDeviceMapValid(deviceMap) != 0 as libc::c_int ||
           data.is_null() {
        /* set return error code */
        retVal = -(2 as libc::c_int)
    } else {
        /* Package the command with given data and send it to device */
        retVal =
            rlDriverExecuteSetApi(deviceMap,
                                  0x4 as libc::c_uint as rlUInt16_t,
                                  0x7 as libc::c_uint as rlUInt16_t,
                                  data as *mut rlUInt8_t,
                                  ::std::mem::size_of::<rlRfMiscConf_t>() as
                                      libc::c_ulong as rlUInt16_t)
    }
    return retVal;
}
/* * @fn rlReturnVal_t rlRfSetCalMonTimeUnitConfig(rlUInt8_t deviceMap,
                                                  rlRfCalMonTimeUntConf_t* data)
*
*   @brief Set Calibration monitoring time unit
*   @param[in] deviceMap - Bitmap of devices to send the message
*   @param[in] data - RF Calib Monitoring Time unit config
*
*   @return rlReturnVal_t Success - 0, Failure - Error Code
*
*   This function configures calibration monitoring time unit
*
*   @note 1: The Minimum total blank time in a calibMonTimeUnit shall be 1ms to run internal APLL
*           and SYNTH calibrations + ~12.5% of calibMonTimeUnit for  WDT clearing time if WDT is
*           enabled. \n
*   @note 2: Refer to \ref AnalogMonitoringDuration, \ref DigitalMonitoringDuration for
*            the duration of run time monitors and \ref SoftwareOverheads for software overheads.
*   @note 3: The CALIB_MON_TIME_UNIT is applicable for one frame trigger API. Once frame is
*            stopped then FTTI will reset, CALIB_MON_TIME_UNIT is not applicable across
*            multiple SW frame trigger API.
*   @note 4: In case of single frame configured in frame config API, then set CALIB_MON_TIME_UNIT
*            to one to run all monitors. It is recommended to use ONE_TIME_CALIB_ENABLE_MASK in
*            AWR_RUN_TIME_CALIBRATION_CONF_AND_TRIGGER_SB API to run one shot calibrations before
*            frame trigger in single frame case.
*/
/* DesignId :  MMWL_DesignId_053 */
/* Requirements : AUTORADAR_REQ-892 */
#[no_mangle]
pub unsafe extern "C" fn rlRfSetCalMonTimeUnitConfig(mut deviceMap: rlUInt8_t,
                                                     mut data:
                                                         *mut rlRfCalMonTimeUntConf_t)
 -> rlReturnVal_t {
    let mut retVal: rlReturnVal_t = 0;
    /* check if deviceIndex is out of defined value */
    if rlDriverIsDeviceMapValid(deviceMap) != 0 as libc::c_int ||
           data.is_null() {
        /* set return error code */
        retVal = -(2 as libc::c_int)
    } else {
        /* Package the command with given data and send it to device */
        retVal =
            rlDriverExecuteSetApi(deviceMap,
                                  0x8 as libc::c_uint as rlUInt16_t,
                                  0x9 as libc::c_uint as rlUInt16_t,
                                  data as *mut rlUInt8_t,
                                  ::std::mem::size_of::<rlRfCalMonTimeUntConf_t>()
                                      as libc::c_ulong as rlUInt16_t)
    }
    return retVal;
}
/* * @fn rlReturnVal_t rlRfSetCalMonFreqLimitConfig(rlUInt8_t deviceMap,
                                                    rlRfCalMonFreqLimitConf_t* data)
*
*   @brief Set Calibration monitoring Frequency Limit
*   @param[in] deviceMap - Bitmap of devices to send the message
*   @param[in] data - RF Calib Frequency Limit config
*
*   @return rlReturnVal_t Success - 0, Failure - Error Code
*
*   This function configures limits on RF frequency transmission during calibration
*   and monitoring
*
*   @note 1 : The minimum RF bandwidth shall be set to 200MHz, this is to perform internal
*             calibration and monitoring \n
*   @note 2 : The limit set in this API is not applicable for functional chirps and loop-back
*             chirps used in advanced frame config API. \n
*   @note 3 : The TX0 frequency limit is used by default in calibrations and monitors where TX is
*             not relevant or enabled. \n
*   @note 4 : The RF band used in functional chirp profiles shall be within the limit set in this
*             API. \n
*   @note 5 : This API is deprecated as a new API AWR_CAL_MON_FREQUENCY_TX_POWER_LIMITS_SB is
*             added to limit frequency for each TX channels. \n
*   @note 6 : The mid frequency code of RF band used in functional chirp profiles + 200MHz shall
*             be within the max limit set in this API.
*/
/* DesignId :  MMWL_DesignId_054 */
/* Requirements : AUTORADAR_REQ-893 */
#[no_mangle]
pub unsafe extern "C" fn rlRfSetCalMonFreqLimitConfig(mut deviceMap:
                                                          rlUInt8_t,
                                                      mut data:
                                                          *mut rlRfCalMonFreqLimitConf_t)
 -> rlReturnVal_t {
    let mut retVal: rlReturnVal_t = 0;
    /* check if deviceIndex is out of defined value */
    if rlDriverIsDeviceMapValid(deviceMap) != 0 as libc::c_int ||
           data.is_null() {
        /* set return error code */
        retVal = -(2 as libc::c_int)
    } else {
        /* Package the command with given data and send it to device */
        retVal =
            rlDriverExecuteSetApi(deviceMap,
                                  0x4 as libc::c_uint as rlUInt16_t,
                                  0x8 as libc::c_uint as rlUInt16_t,
                                  data as *mut rlUInt8_t,
                                  ::std::mem::size_of::<rlRfCalMonFreqLimitConf_t>()
                                      as libc::c_ulong as rlUInt16_t)
    }
    return retVal;
}
/* * @fn rlReturnVal_t rlRfInitCalibConfig(rlUInt8_t deviceMap, rlRfInitCalConf_t* data)
*
*   @brief Set RF Init Calibration Mask bits and report type
*   @param[in] deviceMap - Bitmap of devices to send the message
*   @param[in] data - RF Init calib config
*
*   @return rlReturnVal_t Success - 0, Failure - Error Code
*
*   This function configures RF Init calibration mask bits and report type.
*   Normally, upon receiving rlRfInit API, the Radar SS performs all relevant initial calibrations.
*   This step can be disabled by setting the corresponding bit in \ref rlRfInitCalConf_t
*   field to 0x0.If disabled, the host needs to send the calibration data using
*   \ref rlRfCalibDataRestore so that the RadarSS can operate using the injected calibration data
*
*   @note 1 : Each of these calibrations can be selectively disabled by issuing this
*             message before rlRfInit API.
*   @note 2 : The APLL, SYNTH1 and SYNTH2 calibrations are always triggred by default on RF init
*             command.
*/
/* DesignId :  MMWL_DesignId_055 */
/* Requirements : AUTORADAR_REQ-894 */
#[no_mangle]
pub unsafe extern "C" fn rlRfInitCalibConfig(mut deviceMap: rlUInt8_t,
                                             mut data: *mut rlRfInitCalConf_t)
 -> rlReturnVal_t {
    let mut retVal: rlReturnVal_t = 0;
    /* check if deviceIndex is out of defined value */
    if rlDriverIsDeviceMapValid(deviceMap) != 0 as libc::c_int ||
           data.is_null() {
        /* set return error code */
        retVal = -(2 as libc::c_int)
    } else {
        /* Package the command with given data and send it to device */
        retVal =
            rlDriverExecuteSetApi(deviceMap,
                                  0x4 as libc::c_uint as rlUInt16_t,
                                  0x9 as libc::c_uint as rlUInt16_t,
                                  data as *mut rlUInt8_t,
                                  ::std::mem::size_of::<rlRfInitCalConf_t>()
                                      as libc::c_ulong as rlUInt16_t)
    }
    return retVal;
}
/* * @fn rlReturnVal_t rlRfRunTimeCalibConfig(rlUInt8_t deviceMap, rlRunTimeCalibConf_t* data)
*
*   @brief Set RF one time & periodic calibration of various RF/analog aspects and trigger
*   @param[in] deviceMap - Bitmap of devices to send the message
*   @param[in] data - Runtime calibration config
*
*   @return rlReturnVal_t Success - 0, Failure - Error Code
*
*  This function configures RF one time & periodic calibration of various RF/analog aspects
*  and trigger. The response is in the form of an asynchronous event. The calibration would
*  be performed by Radar SS during while framing during inter-burst idle time slot of 250uS.
*
*  @note 1: This API must be called after rlSetProfileConfig
*  @note 2: This API should be issued when the device is not framing.
*/
/* DesignId : MMWL_DesignId_056 */
/* Requirements : AUTORADAR_REQ-895 */
#[no_mangle]
pub unsafe extern "C" fn rlRfRunTimeCalibConfig(mut deviceMap: rlUInt8_t,
                                                mut data:
                                                    *mut rlRunTimeCalibConf_t)
 -> rlReturnVal_t {
    let mut retVal: rlReturnVal_t = 0;
    /* check if deviceIndex is out of defined value */
    if rlDriverIsDeviceMapValid(deviceMap) != 0 as libc::c_int ||
           data.is_null() {
        /* set return error code */
        retVal = -(2 as libc::c_int)
    } else {
        /* Package the command with given data and send it to device */
        retVal =
            rlDriverExecuteSetApi(deviceMap,
                                  0x8 as libc::c_uint as rlUInt16_t,
                                  0xa as libc::c_uint as rlUInt16_t,
                                  data as *mut rlUInt8_t,
                                  ::std::mem::size_of::<rlRunTimeCalibConf_t>()
                                      as libc::c_ulong as rlUInt16_t)
    }
    return retVal;
}
/* * @fn rlReturnVal_t rlRxGainTempLutSet(rlUInt8_t deviceMap, rlRxGainTempLutData_t *data)
*
*   @brief Overwrite RX gain temperature Lookup Table(LUT) in Radar SS
*   @param[in] deviceMap - Bitmap of devices to send the message
*   @param[in] data - RX gain Temperature LUT config
*
*   @return rlReturnVal_t Success - 0, Failure - Error Code
*
*   This API can be used to overwrite the RX gain Lookup Table(LUT) for different temperature
*   used in RadarSS.
*
*   @note 1 : This API should be issued after profile configuration API.
*/
/* DesignId : MMWL_DesignId_057 */
/* Requirements : AUTORADAR_REQ-896 */
#[no_mangle]
pub unsafe extern "C" fn rlRxGainTempLutSet(mut deviceMap: rlUInt8_t,
                                            mut data:
                                                *mut rlRxGainTempLutData_t)
 -> rlReturnVal_t {
    let mut retVal: rlReturnVal_t = 0;
    /* check if deviceIndex is out of defined value */
    if rlDriverIsDeviceMapValid(deviceMap) != 0 as libc::c_int ||
           data.is_null() {
        /* set return error code */
        retVal = -(2 as libc::c_int)
    } else {
        /* Package the command with given data and send it to device */
        retVal =
            rlDriverExecuteSetApi(deviceMap,
                                  0x8 as libc::c_uint as rlUInt16_t,
                                  0xc as libc::c_uint as rlUInt16_t,
                                  data as *mut rlUInt8_t,
                                  ::std::mem::size_of::<rlRxGainTempLutData_t>()
                                      as libc::c_ulong as rlUInt16_t)
    }
    return retVal;
}
/* * @fn rlReturnVal_t rlRxGainTempLutGet(rlUInt8_t deviceMap, rlRxGainTempLutRead_t* inData,
                                                               rlRxGainTempLutInject_t *outData)
*
*   @brief Gets RX gain temperature Lookup Table(LUT) in Radar SS
*   @param[in] deviceMap - Bitmap of devices to send the message
*   @param[in] inData - RX gain Temperature LUT request  config
*   @param[out] outData - RX gain Temperature LUT read config
*
*   @return rlReturnVal_t Success - 0, Failure - Error Code
*
*   This API is to read the temperature based RX gain LUT used by the Radar SS.
*   This API should be issued after the profile configuration API.
*/
/* DesignId : MMWL_DesignId_058 */
/* Requirements : AUTORADAR_REQ-897 */
#[no_mangle]
pub unsafe extern "C" fn rlRxGainTempLutGet(mut deviceMap: rlUInt8_t,
                                            mut inData:
                                                *mut rlRxGainTempLutReadReq_t,
                                            mut outData:
                                                *mut rlRxGainTempLutData_t)
 -> rlReturnVal_t {
    let mut retVal: rlReturnVal_t = 0;
    /* check if deviceIndex is out of defined value */
    if rlDriverIsDeviceMapValid(deviceMap) != 0 as libc::c_int ||
           inData.is_null() {
        /* set return error code */
        retVal = -(2 as libc::c_int)
    } else {
        /* Initialize Command and Response Sub Blocks */
        let mut inMsg: rlDriverMsg_t =
            {
                let mut init =
                    rlDriverMsg{opcode:
                                    {
                                        let mut init =
                                            rlDriverOpcode{dir:
                                                               0 as
                                                                   libc::c_int
                                                                   as
                                                                   rlUInt8_t,
                                                           msgType: 0,
                                                           msgId: 0,
                                                           nsbc: 0,};
                                        init
                                    },
                                subblocks: 0 as *mut rlPayloadSb_t,
                                remChunks: 0,};
                init
            };
        let mut outMsg: rlDriverMsg_t =
            {
                let mut init =
                    rlDriverMsg{opcode:
                                    {
                                        let mut init =
                                            rlDriverOpcode{dir:
                                                               0 as
                                                                   libc::c_int
                                                                   as
                                                                   rlUInt8_t,
                                                           msgType: 0,
                                                           msgId: 0,
                                                           nsbc: 0,};
                                        init
                                    },
                                subblocks: 0 as *mut rlPayloadSb_t,
                                remChunks: 0,};
                init
            };
        /* Initialize Command and Response Sub Blocks */
        let mut inPayloadSb: rlPayloadSb_t =
            {
                let mut init =
                    rlPayloadSb{sbid: 0 as libc::c_int as rlUInt16_t,
                                len: 0,
                                pSblkData: 0 as *mut rlUInt8_t,};
                init
            };
        let mut outPayloadSb: rlPayloadSb_t =
            {
                let mut init =
                    rlPayloadSb{sbid: 0 as libc::c_int as rlUInt16_t,
                                len: 0,
                                pSblkData: 0 as *mut rlUInt8_t,};
                init
            };
        /* Fill in-message Payload */
        rlDriverFillPayload(0x9 as libc::c_uint as rlUInt16_t,
                            0xc as libc::c_uint as rlUInt16_t,
                            &mut inPayloadSb, inData as *mut rlUInt8_t,
                            ::std::mem::size_of::<rlRxGainTempLutReadReq_t>()
                                as libc::c_ulong as rlUInt16_t);
        /* Construct command packet */
        rlDriverConstructInMsg(0x9 as libc::c_uint as rlUInt16_t, &mut inMsg,
                               &mut inPayloadSb);
        /* Fill out-message Payload */
        rlDriverFillPayload(0 as libc::c_uint as rlUInt16_t,
                            0 as libc::c_uint as rlUInt16_t,
                            &mut outPayloadSb, outData as *mut rlUInt8_t,
                            ::std::mem::size_of::<rlRxGainTempLutData_t>() as
                                libc::c_ulong as rlUInt16_t);
        /* Construct response packet */
        rlDriverConstructOutMsg(1 as libc::c_uint as rlUInt16_t, &mut outMsg,
                                &mut outPayloadSb);
        /* Send Command to mmWave Radar Device */
        retVal = rlDriverCmdInvoke(deviceMap, inMsg, &mut outMsg)
    }
    return retVal;
}
/* * @fn rlReturnVal_t rlTxGainTempLutSet(rlUInt8_t deviceMap, rlTxGainTempLutData_t *data)
*
*   @brief Overwrites TX gain temperature based Lookup table (LUT)
*   @param[in] deviceMap - Bitmap of devices to send the message
*   @param[in] data - TX gain Temperature LUT config
*
*   @return rlReturnVal_t Success - 0, Failure - Error Code
*
*   This API can be used to overwrite the TX gain temperature LUT used in Radar SS.
*   This API should be issued after profile configuration API.
*/
/* DesignId : MMWL_DesignId_059 */
/* Requirements : AUTORADAR_REQ-898 */
#[no_mangle]
pub unsafe extern "C" fn rlTxGainTempLutSet(mut deviceMap: rlUInt8_t,
                                            mut data:
                                                *mut rlTxGainTempLutData_t)
 -> rlReturnVal_t {
    let mut retVal: rlReturnVal_t = 0;
    /* check if deviceIndex is out of defined value */
    if rlDriverIsDeviceMapValid(deviceMap) != 0 as libc::c_int ||
           data.is_null() {
        /* set return error code */
        retVal = -(2 as libc::c_int)
    } else {
        /* Package the command with given data and send it to device */
        retVal =
            rlDriverExecuteSetApi(deviceMap,
                                  0x8 as libc::c_uint as rlUInt16_t,
                                  0xd as libc::c_uint as rlUInt16_t,
                                  data as *mut rlUInt8_t,
                                  ::std::mem::size_of::<rlTxGainTempLutData_t>()
                                      as libc::c_ulong as rlUInt16_t)
    }
    return retVal;
}
/* * @fn rlReturnVal_t rlRxGainTempLutGet(rlUInt8_t deviceMap, rlTxGainTempLutRead_t* inData,
                                                               rlTxGainTempLutInject_t *outData)
*
*   @brief Gets TX gain temperature Lookup table (LUT)
*   @param[in] deviceMap - Bitmap of devices to send the message
*   @param[in] inData - TX gain Temperature LUT request config
*   @param[out] outData - TX gain Temperature LUT read config
*
*   @return rlReturnVal_t Success - 0, Failure - Error Code
*
*   This API is to read the temperature based TX gain LUT used by the firmware.
*   This API should be issued after the rlSetProfileConfig API.
*/
/* DesignId : MMWL_DesignId_060 */
/* Requirements : AUTORADAR_REQ-899 */
#[no_mangle]
pub unsafe extern "C" fn rlTxGainTempLutGet(mut deviceMap: rlUInt8_t,
                                            mut inData:
                                                *mut rlTxGainTempLutReadReq_t,
                                            mut outData:
                                                *mut rlTxGainTempLutData_t)
 -> rlReturnVal_t {
    let mut retVal: rlReturnVal_t = 0;
    /* check if deviceIndex is out of defined value */
    if rlDriverIsDeviceMapValid(deviceMap) != 0 as libc::c_int ||
           inData.is_null() {
        /* set return error code */
        retVal = -(2 as libc::c_int)
    } else {
        /* Initialize Command and Response Sub Blocks */
        let mut inMsg: rlDriverMsg_t =
            {
                let mut init =
                    rlDriverMsg{opcode:
                                    {
                                        let mut init =
                                            rlDriverOpcode{dir:
                                                               0 as
                                                                   libc::c_int
                                                                   as
                                                                   rlUInt8_t,
                                                           msgType: 0,
                                                           msgId: 0,
                                                           nsbc: 0,};
                                        init
                                    },
                                subblocks: 0 as *mut rlPayloadSb_t,
                                remChunks: 0,};
                init
            };
        let mut outMsg: rlDriverMsg_t =
            {
                let mut init =
                    rlDriverMsg{opcode:
                                    {
                                        let mut init =
                                            rlDriverOpcode{dir:
                                                               0 as
                                                                   libc::c_int
                                                                   as
                                                                   rlUInt8_t,
                                                           msgType: 0,
                                                           msgId: 0,
                                                           nsbc: 0,};
                                        init
                                    },
                                subblocks: 0 as *mut rlPayloadSb_t,
                                remChunks: 0,};
                init
            };
        /* Initialize Command and Response Sub Blocks */
        let mut inPayloadSb: rlPayloadSb_t =
            {
                let mut init =
                    rlPayloadSb{sbid: 0 as libc::c_int as rlUInt16_t,
                                len: 0,
                                pSblkData: 0 as *mut rlUInt8_t,};
                init
            };
        let mut outPayloadSb: rlPayloadSb_t =
            {
                let mut init =
                    rlPayloadSb{sbid: 0 as libc::c_int as rlUInt16_t,
                                len: 0,
                                pSblkData: 0 as *mut rlUInt8_t,};
                init
            };
        /* Fill in-message Payload */
        rlDriverFillPayload(0x9 as libc::c_uint as rlUInt16_t,
                            0xd as libc::c_uint as rlUInt16_t,
                            &mut inPayloadSb, inData as *mut rlUInt8_t,
                            ::std::mem::size_of::<rlTxGainTempLutReadReq_t>()
                                as libc::c_ulong as rlUInt16_t);
        /* Construct command packet */
        rlDriverConstructInMsg(0x9 as libc::c_uint as rlUInt16_t, &mut inMsg,
                               &mut inPayloadSb);
        /* Fill out-message Payload */
        rlDriverFillPayload(0 as libc::c_uint as rlUInt16_t,
                            0 as libc::c_uint as rlUInt16_t,
                            &mut outPayloadSb, outData as *mut rlUInt8_t,
                            ::std::mem::size_of::<rlTxGainTempLutData_t>() as
                                libc::c_ulong as rlUInt16_t);
        /* Construct response packet */
        rlDriverConstructOutMsg(1 as libc::c_uint as rlUInt16_t, &mut outMsg,
                                &mut outPayloadSb);
        /* Send Command to mmWave Radar Device */
        retVal = rlDriverCmdInvoke(deviceMap, inMsg, &mut outMsg)
    }
    return retVal;
}
/* * @fn rlReturnVal_t rlRfTxFreqPwrLimitConfig(rlUInt8_t deviceMap,
                                               rlRfTxFreqPwrLimitMonConf_t* data)
*
*   @brief Sets the limits for RF frequency transmission for each TX and also TX power limits
*   @param[in] deviceMap - Bitmap of devices to send the message
*   @param[in] data - Tx Rf freq and power limit config data
*
*   @return rlReturnVal_t Success - 0, Failure - Error Code
*
*   This API sets the limits for RF frequency transmission for each TX and also TX power limits.
*
*   @note 1 : The minimum RF bandwidth shall be set to 200MHz, this is to perform internal
*             calibration and monitoring \n
*   @note 2 : The limit set in this API is not applicable for functional chirps and loop-back
*             chirps used in advanced frame config API. \n
*   @note 3 : The TX0 frequency limit is used by default in calibrations and monitors where TX is
*             not relevant or enabled. \n
*   @note 4 : The RF band used in functional chirp profiles shall be within the limit set in this
*             API. \n
*   @note 5 : The mid frequency code of RF band used in functional chirp profiles + 200MHz shall
*             be within the max limit set in this API.
*/
/* DesignId : MMWL_DesignId_066 */
/* Requirements : AUTORADAR_REQ-905 */
#[no_mangle]
pub unsafe extern "C" fn rlRfTxFreqPwrLimitConfig(mut deviceMap: rlUInt8_t,
                                                  mut data:
                                                      *mut rlRfTxFreqPwrLimitMonConf_t)
 -> rlReturnVal_t {
    let mut retVal: rlReturnVal_t = 0;
    /* check if deviceIndex is out of defined value */
    if rlDriverIsDeviceMapValid(deviceMap) != 0 as libc::c_int ||
           data.is_null() {
        /* set return error code */
        retVal = -(2 as libc::c_int)
    } else {
        /* Package the command with given data and send it to device */
        retVal =
            rlDriverExecuteSetApi(deviceMap,
                                  0x4 as libc::c_uint as rlUInt16_t,
                                  0xa as libc::c_uint as rlUInt16_t,
                                  data as *mut rlUInt8_t,
                                  ::std::mem::size_of::<rlRfTxFreqPwrLimitMonConf_t>()
                                      as libc::c_ulong as rlUInt16_t)
    }
    return retVal;
}
/* * @fn rlReturnVal_t rlSetLoopBckBurstCfg(rlUInt8_t deviceMap, rlLoopbackBurst_t *data)
*
*   @brief This API is used to introduce loopback chirps within the functional frames.
*   @param[in] deviceMap - Bitmap of devices to send the message
*   @param[in] data - Loopback chirp config
*
*   @return rlReturnVal_t Success - 0, Failure - Error Code
*
*   This API can be used to introduce loopback chirps within the functional frames. This loopback
*   chirps will be introduced only if advanced frame configuration is used where user can define
*   which sub-frame contains loopback chirps. The following loopback configuration will apply to
*   one burst and user can program up to 16 different loopback configurations in 16 different
*   bursts of a given sub-frame. User has to ensure that the corresponding sub-frame is defined in
*   rlSetAdvFrameConfig and sufficient time is given to allow the loopback bursts to be
*   transmitted.
*
*   @note 1: If user desires to enable loopback chirps within functional frames, then this API
*            should be issued before rlSetProfileConfig
*   @note 2: Only profile based phase shifter is supported in loopback configuration. Per-chirp
*            phase shifter if enabled will not be reflected in loopback chirps.
*   @note 3: For the sub-frame in which loopback is desired, user should set numOfChirps per burst
*            as 1 and can use numLoops per burst for multiple chirps in the burst.
*   @note 4: The loop-back configuration API is not supported in DFP 1.x (1st gen devices).
*            Please refer latest DFP release note for more info.
*   @note 5: The expected signal strength change with change in index value is only approximately 
*            indicated for PS<n>_PGA_GAIN_INDEX. Typically, the loopback path is the dominant 
*            path only in top ~10 indices (highest PGA gain values). For lower indices (lower PGA 
*            gain values), parasitic paths in the RF system can start dominating the loop-back 
*            measurements, and under such conditions, inter channel imbalances measured using 
*            such LB path, and LB signal SNR etc. can show degraded performance, with the 
*            degradation attributed to the loop-back path and not the functional 
*            path/circuits/system.
*/
/* DesignId : MMWL_DesignId_062 */
/* Requirements : AUTORADAR_REQ-901 */
#[no_mangle]
pub unsafe extern "C" fn rlSetLoopBckBurstCfg(mut deviceMap: rlUInt8_t,
                                              mut data:
                                                  *mut rlLoopbackBurst_t)
 -> rlReturnVal_t {
    let mut retVal: rlReturnVal_t = 0;
    /* check if deviceIndex is out of defined value */
    if rlDriverIsDeviceMapValid(deviceMap) != 0 as libc::c_int ||
           data.is_null() {
        /* set return error code */
        retVal = -(2 as libc::c_int)
    } else {
        /* Package the command with given data and send it to device */
        retVal =
            rlDriverExecuteSetApi(deviceMap,
                                  0x8 as libc::c_uint as rlUInt16_t,
                                  0xe as libc::c_uint as rlUInt16_t,
                                  data as *mut rlUInt8_t,
                                  ::std::mem::size_of::<rlLoopbackBurst_t>()
                                      as libc::c_ulong as rlUInt16_t)
    }
    return retVal;
}
/* * @fn rlReturnVal_t rlSetDynChirpCfg(rlUInt8_t deviceMap, rlUInt16_t segCnt,
                                          rlDynChirpCfg_t **data)
*
*   @brief Injects chirp configuration to be programmed dynamically
*   @param[in] deviceMap - Bitmap of devices to send the message
*   @param[in] segCnt - number of segments for which application sends array of data.
*   @param[in] data - Dynamic chirp configuration
*
*   @return rlReturnVal_t Success - 0, Failure - Error Code
*
*   This API can be used to dynamically change the chirp configuration while frames are on-going.
*   The configuration will be stored in software and at rlDynChirpEnCfg API invocation radarSS
*   copies these chirp configurations from SW RAM to HW RAM at the end of current on-going frame.
*
*   @note : The new feature of dynamic chirp configuaration to configuare 48 chirps in one API
*           is not applicable in xWR6843.
*/
/* DesignId : MMWL_DesignId_063 */
/* Requirements : AUTORADAR_REQ-902 */
#[no_mangle]
pub unsafe extern "C" fn rlSetDynChirpCfg(mut deviceMap: rlUInt8_t,
                                          mut segCnt: rlUInt16_t,
                                          mut data: *mut *mut rlDynChirpCfg_t)
 -> rlReturnVal_t {
    let mut retVal: rlReturnVal_t = 0;
    /* check if deviceIndex is out of defined value */
    if rlDriverIsDeviceMapValid(deviceMap) != 0 as libc::c_int ||
           data.is_null() || segCnt as libc::c_uint == 0 as libc::c_uint {
        /* set return error code */
        retVal = -(2 as libc::c_int)
    } else {
        /* Initialize Command and Response Sub Blocks */
        let mut inMsg: rlDriverMsg_t =
            rlDriverMsg_t{opcode:
                              rlDriverOpcode_t{dir: 0,
                                               msgType: 0,
                                               msgId: 0,
                                               nsbc: 0,},
                          subblocks: 0 as *mut rlPayloadSb_t,
                          remChunks: 0,};
        let mut outMsg: rlDriverMsg_t =
            {
                let mut init =
                    rlDriverMsg{opcode:
                                    {
                                        let mut init =
                                            rlDriverOpcode{dir:
                                                               0 as
                                                                   libc::c_int
                                                                   as
                                                                   rlUInt8_t,
                                                           msgType: 0,
                                                           msgId: 0,
                                                           nsbc: 0,};
                                        init
                                    },
                                subblocks: 0 as *mut rlPayloadSb_t,
                                remChunks: 0,};
                init
            };
        /* Index and other paramerters to send multiple sub blocks in one/more commands chunk */
        let mut sbLen: rlUInt16_t = 0;
        let mut maxSbCntInMsg: rlUInt16_t = 0;
        let mut sbCntInMsg: rlUInt16_t = 0;
        let mut numChnkOfMsg: rlUInt16_t = 0;
        let mut lastSbCntInMsg: rlUInt16_t = 0;
        let mut loopCnt: rlUInt16_t = 0;
        let mut indx: rlUInt16_t = 0;
        /* Initialize Command and Response Sub Blocks */
        let mut inPayloadSb: [rlPayloadSb_t; 1] =
            [rlPayloadSb_t{sbid: 0, len: 0, pSblkData: 0 as *mut rlUInt8_t,};
                1];
        /* single sub-block length for chirp config API */
        sbLen =
            (::std::mem::size_of::<rlDynChirpCfg_t>() as
                 libc::c_ulong).wrapping_add((2 as
                                                  libc::c_uint).wrapping_add(2
                                                                                 as
                                                                                 libc::c_uint)
                                                 as libc::c_ulong) as
                rlUInt16_t;
        /* get Max Sub Block count */
        /* AR_CODE_REVIEW MR:D.4.1 <APPROVED> "sbLen can not be zero" */
        /*LDRA_INSPECTED 127 D */
        maxSbCntInMsg =
            (256 as
                 libc::c_uint).wrapping_sub((4 as
                                                 libc::c_uint).wrapping_add(12
                                                                                as
                                                                                libc::c_uint).wrapping_add(8
                                                                                                               as
                                                                                                               libc::c_uint)).wrapping_div(sbLen
                                                                                                                                               as
                                                                                                                                               libc::c_uint)
                as rlUInt16_t;
        retVal = 0 as libc::c_int;
        /* if requested count of chirpConfig is within one Message packet */
        if segCnt as libc::c_int <= maxSbCntInMsg as libc::c_int {
            sbCntInMsg = segCnt;
            numChnkOfMsg = 1 as libc::c_uint as rlUInt16_t;
            lastSbCntInMsg = 0 as libc::c_uint as rlUInt16_t
        } else {
            sbCntInMsg = maxSbCntInMsg;
            numChnkOfMsg =
                (segCnt as libc::c_int / maxSbCntInMsg as libc::c_int) as
                    rlUInt16_t;
            lastSbCntInMsg =
                (segCnt as libc::c_int % maxSbCntInMsg as libc::c_int) as
                    rlUInt16_t
        }
        /* Fill in-message packet */
        rlDriverConstructInMsg(0x8 as libc::c_uint as rlUInt16_t, &mut inMsg,
                               &mut *inPayloadSb.as_mut_ptr().offset(0 as
                                                                         libc::c_int
                                                                         as
                                                                         isize));
        loopCnt =
            if lastSbCntInMsg as libc::c_uint == 0 as libc::c_uint {
                numChnkOfMsg as libc::c_uint
            } else {
                (numChnkOfMsg as libc::c_uint).wrapping_add(1 as libc::c_uint)
            } as rlUInt16_t;
        while retVal == 0 as libc::c_int {
            /* all full messages have been sent, then send last partial message */
            if loopCnt as libc::c_uint == 1 as libc::c_uint &&
                   lastSbCntInMsg as libc::c_uint != 0 as libc::c_uint {
                inMsg.opcode.nsbc = lastSbCntInMsg
            } else { inMsg.opcode.nsbc = sbCntInMsg }
            indx = 0 as libc::c_uint as rlUInt16_t;
            while (indx as libc::c_int) < inMsg.opcode.nsbc as libc::c_int {
                /* Set Command Sub Block*/
                /* AR_CODE_REVIEW MR:R.2.2  <APPROVED> "Values are used by called function. \
                LDRA Tool Issue" */
                /*LDRA_INSPECTED 105 D */
                inPayloadSb[indx as usize].sbid =
                    (0x8 as
                         libc::c_uint).wrapping_mul(32 as
                                                        libc::c_uint).wrapping_add(0xf
                                                                                       as
                                                                                       libc::c_uint)
                        as rlUInt16_t;
                /* AR_CODE_REVIEW MR:R.2.2  <APPROVED> "Values are used by called function. \
                LDRA Tool Issue" */
                /*LDRA_INSPECTED 105 D */
                inPayloadSb[indx as usize].len =
                    ::std::mem::size_of::<rlDynChirpCfg_t>() as libc::c_ulong
                        as rlUInt16_t;
                /* AR_CODE_REVIEW MR:R.2.2  <APPROVED> "Values are used by called function. \
                LDRA Tool Issue" */
                /*LDRA_INSPECTED 105 D */
                inPayloadSb[indx as usize].pSblkData =
                    *data as *mut rlUInt8_t;
                /*AR_CODE_REVIEW MR:R.18.1 <APPROVED> "require pointer increment to jump to \
                  next Config */
                /*LDRA_INSPECTED 567 S */
                data = data.offset(1);
                indx = indx.wrapping_add(1)
            }
            /* Send Command to mmWave Radar Device */
            retVal += rlDriverCmdInvoke(deviceMap, inMsg, &mut outMsg);
            loopCnt = loopCnt.wrapping_sub(1);
            if loopCnt as libc::c_uint == 0 as libc::c_uint { break ; }
        }
    }
    return retVal;
}
/* * @fn rlReturnVal_t rlSetDynChirpEn(rlUInt8_t deviceMap, rlDynChirpEnCfg_t *data)
*
*   @brief Triggers copy of chirp config from SW to HW RAM.
*   @param[in] deviceMap - Bitmap of devices to send the message
*   @param[in] data - Dynamic chirp enable configuration
*
*   @return rlReturnVal_t Success - 0, Failure - Error Code
*
*   This API can be used to trigger the copy of chirp configuration from software to hardware RAM.
*   The copy will be performed at the end of the ongoing frame active window (start of the frame
*   idle time).
*   @note User needs to invoke this API within inter-frame idle time, not at boundary of frame end.
*         Since dynamic chirps are configured at run time, there is not error checks done on the
*         input data. If input data is out of range or invalid, device might misbehave.
*/
/* DesignId : MMWL_DesignId_064 */
/* Requirements : AUTORADAR_REQ-903 */
#[no_mangle]
pub unsafe extern "C" fn rlSetDynChirpEn(mut deviceMap: rlUInt8_t,
                                         mut data: *mut rlDynChirpEnCfg_t)
 -> rlReturnVal_t {
    let mut retVal: rlReturnVal_t = 0;
    /* check if deviceIndex is out of defined value */
    if rlDriverIsDeviceMapValid(deviceMap) != 0 as libc::c_int ||
           data.is_null() {
        /* set return error code */
        retVal = -(2 as libc::c_int)
    } else {
        /* Package the command with given data and send it to device */
        retVal =
            rlDriverExecuteSetApi(deviceMap,
                                  0x8 as libc::c_uint as rlUInt16_t,
                                  0x11 as libc::c_uint as rlUInt16_t,
                                  data as *mut rlUInt8_t,
                                  ::std::mem::size_of::<rlDynChirpEnCfg_t>()
                                      as libc::c_ulong as rlUInt16_t)
    }
    return retVal;
}
/* * @fn rlReturnVal_t rlSetDynPerChirpPhShifterCfg(rlUInt8_t deviceMap, rlUInt16_t segCnt,
                                                   rlDynPerChirpPhShftCfg_t **data)
*
*   @brief Injects per-chirp phase shifter configuration to be applied dynamically
*   @param[in] deviceMap - Bitmap of devices to send the message
*   @param[in] segCnt - number of segments for which application sends array of data.
*   @param[in] data - Dynamic chirp configuration
*
*   @return rlReturnVal_t Success - 0, Failure - Error Code
*
*   This API can be used to dynamically change the per-chirp phase shifter configuration while
*   frames are on-going. The configuration will be stored in software and the new configuration
*   will be applied at the end of the on-going frame. Note that the configuration should be
*   received by the firmware 10 ms before the end of the current frame.

*   @note 1: This API is valid only for AWR1243P mmWave device when mmWaveLink instance is running
*            on External Host Processor.
*   @note 2: Phase shifters are applied at the knee of the ramp.
*/
/* DesignId : MMWL_DesignId_065 */
/* Requirements : AUTORADAR_REQ-904 */
#[no_mangle]
pub unsafe extern "C" fn rlSetDynPerChirpPhShifterCfg(mut deviceMap:
                                                          rlUInt8_t,
                                                      mut segCnt: rlUInt16_t,
                                                      mut data:
                                                          *mut *mut rlDynPerChirpPhShftCfg_t)
 -> rlReturnVal_t {
    let mut retVal: rlReturnVal_t = 0;
    /* check if deviceIndex is out of defined value */
    if rlDriverIsDeviceMapValid(deviceMap) != 0 as libc::c_int ||
           data.is_null() || segCnt as libc::c_uint == 0 as libc::c_uint {
        /* set return error code */
        retVal = -(2 as libc::c_int)
    } else {
        /* Initialize Command and Response Sub Blocks */
        let mut inMsg: rlDriverMsg_t =
            rlDriverMsg_t{opcode:
                              rlDriverOpcode_t{dir: 0,
                                               msgType: 0,
                                               msgId: 0,
                                               nsbc: 0,},
                          subblocks: 0 as *mut rlPayloadSb_t,
                          remChunks: 0,};
        let mut outMsg: rlDriverMsg_t =
            {
                let mut init =
                    rlDriverMsg{opcode:
                                    {
                                        let mut init =
                                            rlDriverOpcode{dir:
                                                               0 as
                                                                   libc::c_int
                                                                   as
                                                                   rlUInt8_t,
                                                           msgType: 0,
                                                           msgId: 0,
                                                           nsbc: 0,};
                                        init
                                    },
                                subblocks: 0 as *mut rlPayloadSb_t,
                                remChunks: 0,};
                init
            };
        /* Index and other paramerters to send multiple sub blocks in one/more commands chunk */
        let mut sbLen: rlUInt16_t = 0;
        let mut maxSbCntInMsg: rlUInt16_t = 0;
        let mut sbCntInMsg: rlUInt16_t = 0;
        let mut numChnkOfMsg: rlUInt16_t = 0;
        let mut lastSbCntInMsg: rlUInt16_t = 0;
        let mut loopCnt: rlUInt16_t = 0;
        let mut indx: rlUInt16_t = 0;
        /* Initialize Command and Response Sub Blocks */
        let mut inPayloadSb: [rlPayloadSb_t; 4] =
            [rlPayloadSb_t{sbid: 0, len: 0, pSblkData: 0 as *mut rlUInt8_t,};
                4];
        /* single sub-block length for chirp config API */
        sbLen =
            (::std::mem::size_of::<rlDynPerChirpPhShftCfg_t>() as
                 libc::c_ulong).wrapping_add((2 as
                                                  libc::c_uint).wrapping_add(2
                                                                                 as
                                                                                 libc::c_uint)
                                                 as libc::c_ulong) as
                rlUInt16_t;
        /* get Max Sub Block count */
        /* AR_CODE_REVIEW MR:D.4.1 <APPROVED> "sbLen can not be zero" */
        /*LDRA_INSPECTED 127 D */
        maxSbCntInMsg =
            (256 as
                 libc::c_uint).wrapping_sub((4 as
                                                 libc::c_uint).wrapping_add(12
                                                                                as
                                                                                libc::c_uint).wrapping_add(8
                                                                                                               as
                                                                                                               libc::c_uint)).wrapping_div(sbLen
                                                                                                                                               as
                                                                                                                                               libc::c_uint)
                as rlUInt16_t;
        retVal = 0 as libc::c_int;
        /* if requested count of chirpConfig is within one Message packet */
        if segCnt as libc::c_int <= maxSbCntInMsg as libc::c_int {
            sbCntInMsg = segCnt;
            numChnkOfMsg = 1 as libc::c_uint as rlUInt16_t;
            lastSbCntInMsg = 0 as libc::c_uint as rlUInt16_t
        } else {
            sbCntInMsg = maxSbCntInMsg;
            numChnkOfMsg =
                (segCnt as libc::c_int / maxSbCntInMsg as libc::c_int) as
                    rlUInt16_t;
            lastSbCntInMsg =
                (segCnt as libc::c_int % maxSbCntInMsg as libc::c_int) as
                    rlUInt16_t
        }
        /* Fill in-message packet */
        rlDriverConstructInMsg(0x8 as libc::c_uint as rlUInt16_t, &mut inMsg,
                               &mut *inPayloadSb.as_mut_ptr().offset(0 as
                                                                         libc::c_int
                                                                         as
                                                                         isize));
        loopCnt =
            if lastSbCntInMsg as libc::c_uint == 0 as libc::c_uint {
                numChnkOfMsg as libc::c_uint
            } else {
                (numChnkOfMsg as libc::c_uint).wrapping_add(1 as libc::c_uint)
            } as rlUInt16_t;
        while retVal == 0 as libc::c_int {
            /* all full messages have been sent, then send last partial message */
            if loopCnt as libc::c_uint == 1 as libc::c_uint &&
                   lastSbCntInMsg as libc::c_uint != 0 as libc::c_uint {
                inMsg.opcode.nsbc = lastSbCntInMsg
            } else { inMsg.opcode.nsbc = sbCntInMsg }
            indx = 0 as libc::c_uint as rlUInt16_t;
            while (indx as libc::c_int) < inMsg.opcode.nsbc as libc::c_int {
                /* Set Command Sub Block*/
                /* AR_CODE_REVIEW MR:R.2.2  <APPROVED> "Values are used by called function.\
                LDRA Tool Issue" */
                /*LDRA_INSPECTED 105 D */
                inPayloadSb[indx as usize].sbid =
                    (0x8 as
                         libc::c_uint).wrapping_mul(32 as
                                                        libc::c_uint).wrapping_add(0x10
                                                                                       as
                                                                                       libc::c_uint)
                        as rlUInt16_t;
                /* AR_CODE_REVIEW MR:R.2.2  <APPROVED> "Values are used by called function. \
                LDRA Tool Issue" */
                /*LDRA_INSPECTED 105 D */
                inPayloadSb[indx as usize].len =
                    ::std::mem::size_of::<rlDynPerChirpPhShftCfg_t>() as
                        libc::c_ulong as rlUInt16_t;
                /* AR_CODE_REVIEW MR:R.2.2  <APPROVED> "Values are used by called function. \
                LDRA Tool Issue" */
                /*LDRA_INSPECTED 105 D */
                inPayloadSb[indx as usize].pSblkData =
                    *data as *mut rlUInt8_t;
                /*AR_CODE_REVIEW MR:R.18.1 <APPROVED> "require pointer increment to jump to \
                next Config */
                /*LDRA_INSPECTED 567 S */
                data = data.offset(1);
                indx = indx.wrapping_add(1)
            }
            /* Send Command to mmWave Radar Device */
            retVal += rlDriverCmdInvoke(deviceMap, inMsg, &mut outMsg);
            loopCnt = loopCnt.wrapping_sub(1);
            if loopCnt as libc::c_uint == 0 as libc::c_uint { break ; }
        }
    }
    return retVal;
}
/* * @fn rlReturnVal_t rlRfCalibDataRestore(rlUInt8_t deviceMap, rlCalibrationData_t *data)
*
*   @brief Injects calibration data to the device
*   @param[in] deviceMap - Bitmap of devices to send the message
*   @param[in] data - Calibration data of 3 chunks stored at application space
*
*   @return rlReturnVal_t Success - 0, Failure - Error Code
*
*   This API restores the calibration data which was stored previously using the
*   rlCalibDataStore command. Application needs to feed in 3 chunks of calibration data.
*   recommended API sequence for calibration data restore is: \n
*   1. rlRfCalibDataRestore (To restore factory calibration data to avoid on field
*      RF interference during calibration) \n
*   2. Wait for RL_RF_AE_INITCALIBSTATUS_SB event. \n
*   3. rlRfInitCalibConfig (Enable only required calibration to run) \n
*   4. rlRfInit: This triggers very basic calibrations and RF initializations. \n
*   5. Wait for RL_RF_AE_INITCALIBSTATUS_SB event. \n
*
*   @note 1: Once the calibration data is restored properly in radarSS SW RAM and validated,
*            mmWave Front end would send asynchronous event RL_RF_AE_INITCALIBSTATUS_SB indicating
*            the result of the calibrations based on Calib data sent by the application, this
*            indicates success of the calibration data restore.
*   @note 2: All 3 chunks of 224 bytes each shall be sent to radar device to complete the restore
*            process and to generate RL_RF_AE_INITCALIBSTATUS_SB.
*/
/* DesignId : MMWL_DesignId_067 */
/* Requirements : AUTORADAR_REQ-906 */
#[no_mangle]
pub unsafe extern "C" fn rlRfCalibDataRestore(mut deviceMap: rlUInt8_t,
                                              mut data:
                                                  *mut rlCalibrationData_t)
 -> rlReturnVal_t {
    let mut retVal: rlReturnVal_t = 0;
    let mut idx: rlUInt8_t = 0;
    /* check if deviceIndex is out of defined value */
    if rlDriverIsDeviceMapValid(deviceMap) != 0 as libc::c_int ||
           data.is_null() {
        /* set return error code */
        retVal = -(2 as libc::c_int)
    } else {
        /* Initialize Command and Response Sub Blocks */
        let mut inMsg: rlDriverMsg_t =
            {
                let mut init =
                    rlDriverMsg{opcode:
                                    {
                                        let mut init =
                                            rlDriverOpcode{dir:
                                                               0 as
                                                                   libc::c_int
                                                                   as
                                                                   rlUInt8_t,
                                                           msgType: 0,
                                                           msgId: 0,
                                                           nsbc: 0,};
                                        init
                                    },
                                subblocks: 0 as *mut rlPayloadSb_t,
                                remChunks: 0,};
                init
            };
        let mut outMsg: rlDriverMsg_t =
            {
                let mut init =
                    rlDriverMsg{opcode:
                                    {
                                        let mut init =
                                            rlDriverOpcode{dir:
                                                               0 as
                                                                   libc::c_int
                                                                   as
                                                                   rlUInt8_t,
                                                           msgType: 0,
                                                           msgId: 0,
                                                           nsbc: 0,};
                                        init
                                    },
                                subblocks: 0 as *mut rlPayloadSb_t,
                                remChunks: 0,};
                init
            };
        let mut inPayloadSb: rlPayloadSb_t =
            {
                let mut init =
                    rlPayloadSb{sbid: 0 as libc::c_int as rlUInt16_t,
                                len: 0,
                                pSblkData: 0 as *mut rlUInt8_t,};
                init
            };
        let mut outPayloadSb: rlPayloadSb_t =
            {
                let mut init =
                    rlPayloadSb{sbid: 0 as libc::c_int as rlUInt16_t,
                                len: 0,
                                pSblkData: 0 as *mut rlUInt8_t,};
                init
            };
        /* set return value to zero by default */
        retVal = 0 as libc::c_int;
        /* Invoke Set Calibration data command for RL_MAX_CALIB_DATA_CHUNK chunks */
        idx = 0 as libc::c_uint as rlUInt8_t;
        while (idx as libc::c_uint) < 3 as libc::c_uint {
            /* Construct command packet */
            rlDriverConstructInMsg(0x4 as libc::c_uint as rlUInt16_t,
                                   &mut inMsg, &mut inPayloadSb);
            /* Fill in-message Payload */
            /*AR_CODE_REVIEW MR:R.18.1 <APPROVED> "require pointer increment to jump to
              next calibData */
            /*AR_CODE_REVIEW MR:R.18.1 <APPROVED> "require pointer increment to jump to
                          next calibData */
            /*LDRA_INSPECTED 87 S */
            /*LDRA_INSPECTED 567 S */
            rlDriverFillPayload(0x4 as libc::c_uint as rlUInt16_t,
                                0xb as libc::c_uint as rlUInt16_t,
                                &mut inPayloadSb,
                                &mut *(*data).calibChunk.as_mut_ptr().offset(idx
                                                                                 as
                                                                                 isize)
                                    as *mut rlCalDataStore_t as
                                    *mut rlUInt8_t,
                                ::std::mem::size_of::<rlCalDataStore_t>() as
                                    libc::c_ulong as rlUInt16_t);
            /* Construct response packet */
            rlDriverConstructOutMsg(1 as libc::c_uint as rlUInt16_t,
                                    &mut outMsg, &mut outPayloadSb);
            /* Fill in-message Payload */
            rlDriverFillPayload(0 as libc::c_uint as rlUInt16_t,
                                0 as libc::c_uint as rlUInt16_t,
                                &mut outPayloadSb, 0 as *mut rlUInt8_t,
                                0 as libc::c_uint as rlUInt16_t);
            /* Send Command to mmWave Radar Device */
            retVal += rlDriverCmdInvoke(deviceMap, inMsg, &mut outMsg);
            /* check for return value */
            if 0 as libc::c_int != retVal { break ; }
            idx = idx.wrapping_add(1)
        }
    }
    return retVal;
}
/* * @fn rlReturnVal_t rlRfCalibDataStore(rlUInt8_t deviceMap, rlCalibrationData_t *data)
*
*   @brief Read calibration data from the device
*   @param[in] deviceMap - Bitmap of devices to send the message.
*   @param[in] data - Calibration data of 3 chunks which will filled by device.
*
*   @return rlReturnVal_t Success - 0, Failure - Error Code
*
*   This API reads the calibration data from the device which can be injected later using the
*   rlCalibDataRestore command. RadarSS will return 3 chunks of calibration data.
*
*   @note 1: This API is not supported in IWR6843 ES1.0
*   @note 2: The total size of the calibration data is 672 bytes, this has been split into
*            3 chunks (numOfChunk)of 224 bytes each due to SPI limitation. The Host should
*            receive all these 3 chunks from radar device, later host can store only relevant
*            data in non volatile memory.
*   @note 3: Before storing the calibration data in non volatile memory, the host shall make sure
*            validity status of all enabled calibrations are SET to value 1 including APLL, VCO1,
*            VCO2 and LODIST calibration validity in rlRfInit of radar device.
*   @note 4: Host can store only relevant calibration data in non volatile memory and
*            corresponding validity bits shall be set to 1 in rlRfCalibDataRestore and rest
*            of the validity bits should be clear to 0 before restoring the data to radar device.
*   @note 5: Host shall ignore APLL, VCO1, VCO2 and LODIST calibration validity bits while
*            restoring, these calibrations will be done in each device power-up.
*/
/* DesignId : MMWL_DesignId_068 */
/* Requirements : AUTORADAR_REQ-906 */
#[no_mangle]
pub unsafe extern "C" fn rlRfCalibDataStore(mut deviceMap: rlUInt8_t,
                                            mut data:
                                                *mut rlCalibrationData_t)
 -> rlReturnVal_t {
    let mut retVal: rlReturnVal_t = 0;
    /* check if deviceIndex is out of defined value */
    if rlDriverIsDeviceMapValid(deviceMap) != 0 as libc::c_int ||
           data.is_null() {
        /* set return error code */
        retVal = -(2 as libc::c_int)
    } else {
        /* Initialize Command and Response Sub Blocks */
        let mut inMsg: rlDriverMsg_t =
            {
                let mut init =
                    rlDriverMsg{opcode:
                                    {
                                        let mut init =
                                            rlDriverOpcode{dir:
                                                               0 as
                                                                   libc::c_int
                                                                   as
                                                                   rlUInt8_t,
                                                           msgType: 0,
                                                           msgId: 0,
                                                           nsbc: 0,};
                                        init
                                    },
                                subblocks: 0 as *mut rlPayloadSb_t,
                                remChunks: 0,};
                init
            };
        let mut outMsg: rlDriverMsg_t =
            {
                let mut init =
                    rlDriverMsg{opcode:
                                    {
                                        let mut init =
                                            rlDriverOpcode{dir:
                                                               0 as
                                                                   libc::c_int
                                                                   as
                                                                   rlUInt8_t,
                                                           msgType: 0,
                                                           msgId: 0,
                                                           nsbc: 0,};
                                        init
                                    },
                                subblocks: 0 as *mut rlPayloadSb_t,
                                remChunks: 0,};
                init
            };
        let mut inPayloadSb: rlPayloadSb_t =
            {
                let mut init =
                    rlPayloadSb{sbid: 0 as libc::c_int as rlUInt16_t,
                                len: 0,
                                pSblkData: 0 as *mut rlUInt8_t,};
                init
            };
        let mut outPayloadSb: rlPayloadSb_t =
            {
                let mut init =
                    rlPayloadSb{sbid: 0 as libc::c_int as rlUInt16_t,
                                len: 0,
                                pSblkData: 0 as *mut rlUInt8_t,};
                init
            };
        let mut caldataGetCfg: rlCalDataGetCfg_t =
            {
                let mut init =
                    rlCalDataGetCfg{reserved: 0 as libc::c_uint as rlUInt16_t,
                                    chunkId: 0,};
                init
            };
        let mut idx: rlUInt8_t = 0;
        /* set return value to zero by default */
        retVal = 0 as libc::c_int;
        /* Invoke Get Calibration data command for RL_MAX_CALIB_DATA_CHUNK chunks */
        idx = 0 as libc::c_uint as rlUInt8_t;
        while (idx as libc::c_uint) < 3 as libc::c_uint {
            /* Construct command packet */
            rlDriverConstructInMsg(0x5 as libc::c_uint as rlUInt16_t,
                                   &mut inMsg, &mut inPayloadSb);
            /* Fill in-message Payload */
            rlDriverFillPayload(0x5 as libc::c_uint as rlUInt16_t,
                                0xb as libc::c_uint as rlUInt16_t,
                                &mut inPayloadSb,
                                &mut caldataGetCfg as *mut rlCalDataGetCfg_t
                                    as *mut rlUInt8_t,
                                ::std::mem::size_of::<rlCalDataGetCfg_t>() as
                                    libc::c_ulong as rlUInt16_t);
            /* Construct response packet */
            rlDriverConstructOutMsg(1 as libc::c_uint as rlUInt16_t,
                                    &mut outMsg, &mut outPayloadSb);
            /* Fill in-message Payload */
            /*AR_CODE_REVIEW MR:R.18.1 <APPROVED> "require pointer increment to jump to
              next calibData */
            /*AR_CODE_REVIEW MR:R.18.4 <APPROVED> "require pointer increment to jump to
              next calibData */
            /*LDRA_INSPECTED 87 S */
            /*LDRA_INSPECTED 567 S */
            rlDriverFillPayload(0 as libc::c_uint as rlUInt16_t,
                                0 as libc::c_uint as rlUInt16_t,
                                &mut outPayloadSb,
                                &mut *(*data).calibChunk.as_mut_ptr().offset(idx
                                                                                 as
                                                                                 isize)
                                    as *mut rlCalDataStore_t as
                                    *mut rlUInt8_t,
                                0 as libc::c_uint as rlUInt16_t);
            /* Send Command to mmWave Radar Device */
            retVal += rlDriverCmdInvoke(deviceMap, inMsg, &mut outMsg);
            /* check for return value */
            if 0 as libc::c_int != retVal { break ; }
            /* increment Chunk ID for next Get Command */
            caldataGetCfg.chunkId = caldataGetCfg.chunkId.wrapping_add(1);
            idx = idx.wrapping_add(1)
        }
    }
    return retVal;
}
/* * @fn rlReturnVal_t rlRfInterRxGainPhaseConfig(rlUInt8_t deviceMap, rlInterRxGainPhConf_t* data)
*
*   @brief Sets different Rx gain/phase offset
*   @param[in] deviceMap - Bitmap of devices to send the message
*   @param[in] data - Inter RX gain, phase offset config
*
*   @return rlReturnVal_t Success - 0, Failure - Error Code
*
*   This API can be used to induce different gain/phase offsets on the different RXs, for
*   inter-RX mismatch compensation.
*
*   @note : The Inter RX gain phase control configuration API is not supported in this release.
*           Please refer latest DFP release note for more info.
*/
/* DesignId : MMWL_DesignId_061 */
/* Requirements : AUTORADAR_REQ-900 */
#[no_mangle]
pub unsafe extern "C" fn rlRfInterRxGainPhaseConfig(mut deviceMap: rlUInt8_t,
                                                    mut data:
                                                        *mut rlInterRxGainPhConf_t)
 -> rlReturnVal_t {
    let mut retVal: rlReturnVal_t = 0;
    /* check if deviceIndex is out of defined value */
    if rlDriverIsDeviceMapValid(deviceMap) != 0 as libc::c_int ||
           data.is_null() {
        /* set return error code */
        retVal = -(2 as libc::c_int)
    } else {
        /* Package the command with given data and send it to device */
        retVal =
            rlDriverExecuteSetApi(deviceMap,
                                  0x8 as libc::c_uint as rlUInt16_t,
                                  0xb as libc::c_uint as rlUInt16_t,
                                  data as *mut rlUInt8_t,
                                  ::std::mem::size_of::<rlInterRxGainPhConf_t>()
                                      as libc::c_ulong as rlUInt16_t)
    }
    return retVal;
}
/* * @fn rlReturnVal_t rlGetRfBootupStatus(rlUInt8_t deviceMap, rlRfBootStatusCfg_t *data)
*
*   @brief Get radarSS bootup status
*   @param[in] deviceMap - Bitmap of devices to send the message
*   @param[in] data - bootup status configuration
*
*   @return rlReturnVal_t Success - 0, Failure - Error Code
*
*   This API gets the radarSS bootup status
*
*   @note : This API is not supported in IWR6843 ES1.0
*/
/* DesignId :  MMWL_DesignId_096 */
/* Requirements : AUTORADAR_REQ-911 */
#[no_mangle]
pub unsafe extern "C" fn rlGetRfBootupStatus(mut deviceMap: rlUInt8_t,
                                             mut data:
                                                 *mut rlRfBootStatusCfg_t)
 -> rlReturnVal_t {
    let mut retVal: rlReturnVal_t = 0;
    /* check for NULL pointer */
    if data.is_null() {
        /* set error code if data pointer is passed NULL */
        retVal = -(14 as libc::c_int)
    } else {
        /* Package the command with given data and send it to device */
        retVal =
            rlDriverExecuteGetApi(deviceMap,
                                  0x11 as libc::c_uint as rlUInt16_t,
                                  0x4 as libc::c_uint as rlUInt16_t,
                                  data as *mut rlUInt8_t,
                                  0 as libc::c_uint as rlUInt16_t)
    }
    return retVal;
}
/* * @fn rlReturnVal_t rlSetInterChirpBlkCtrl(rlUInt8_t deviceMap, rlInterChirpBlkCtrlCfg_t *data)
*
*   @brief Sets Inter-chip turn on and turn off times or various RF blocks
*   @param[in] deviceMap - Bitmap of devices to send the message
*   @param[in] data - Inter chirp block control config
*
*   @return rlReturnVal_t Success - 0, Failure - Error Code
*
*   This API programs the Inter-chip turn on and turn off times or various RF blocks
*   @note The minimum inter-chirp time should be greater than maximum of the following
*   1. abs(rx02RfTurnOffTime) + max(abs(rx02RfPreEnTime), abs(rx02RfTurnOnTime))
*   2. abs(rx13RfTurnOffTime) + max(abs(rx13RfPreEnTime), abs(rx13RfTurnOnTime))
*   3. abs(rx02BbTurnOffTime) + max(abs(rx02BbPreEnTime), abs(rx02BbTurnOnTime))
*   4. abs(rx13BbTurnOffTime) + max(abs(rx13BbPreEnTime), abs(rx13BbTurnOnTime))
*   5. abs(rxLoChainTurnOffTime) + abs(rxLoChainTurnOnTime)
*   6. abs(txLoChainTurnOffTime) + abs(txLoChainTurnOnTime)
*
*   @note : The inter-chirp timing control configuration API is supported in this release. Please
*           refer latest DFP release note for more info.
*/
/* DesignId :  MMWL_DesignId_097 */
/* Requirements : AUTORADAR_REQ-912 */
#[no_mangle]
pub unsafe extern "C" fn rlSetInterChirpBlkCtrl(mut deviceMap: rlUInt8_t,
                                                mut data:
                                                    *mut rlInterChirpBlkCtrlCfg_t)
 -> rlReturnVal_t {
    let mut retVal: rlReturnVal_t = 0;
    /* check if deviceIndex is out of defined value */
    if rlDriverIsDeviceMapValid(deviceMap) != 0 as libc::c_int ||
           data.is_null() {
        /* set return error code */
        retVal = -(2 as libc::c_int)
    } else {
        /* Package the command with given data and send it to device */
        retVal =
            rlDriverExecuteSetApi(deviceMap,
                                  0x8 as libc::c_uint as rlUInt16_t,
                                  0x12 as libc::c_uint as rlUInt16_t,
                                  data as *mut rlUInt8_t,
                                  ::std::mem::size_of::<rlInterChirpBlkCtrlCfg_t>()
                                      as libc::c_ulong as rlUInt16_t)
    }
    return retVal;
}
/* * @fn rlReturnVal_t rlSetSubFrameStart(rlUInt8_t deviceMap, rlSubFrameStartCfg_t *data)
*
*   @brief Triggers the next sub-frame in software triggered sub-frame mode
*   @param[in] deviceMap - Bitmap of devices to send the message
*   @param[in] data - Sub-frame start config
*
*   @return rlReturnVal_t Success - 0, Failure - Error Code
*
*   This API triggers the next sub-frame in software triggered sub-frame mode
*
*   @note 1: If the user wishes to trigger each sub-frame independently, then after advanced frame
*            config, the rlSensorStart should be issued once using rlSensorStop. This does not
*            start any sub-frames but it will prepare the hardware for sub-frame trigger. Next any
*            subsequent sub-frame trigger will start the sub-frames.
*   @note 2: If the user wishes to use sub-frame trigger, he has to ensure that sub-frame trigger
*            command is issued k*N times where k is the number of sub-frames in each frame and N is
*            the number of frames. If the user wishes to stop frames in between, then he has to
*            issue the rlSensorStop only after k*M triggers of sub-frame trigger command (where M
*            is an integer). i.e. rlSensorStop can be issued only at frame boundaries.
*   @note 3: If software based sub-frame trigger mode is chosen by the user, watchdog feature will
*            not be available. User has to ensure that the watchdog is disabled before enabling the
*            software based sub-frame trigger mode.
*   @note 4: If sub-frame trigger or hardware trigger mode is used to trigger the frames/sub-
*            frames and if frames need to be stopped before the specified number of frames, then
*            the the frame stop command using rlSensorStop API should be issued
*            while the frame is on-going. If the frames are stopped while the device is idle,
*            it can lead to errors.
*/
/* DesignId :  MMWL_DesignId_107 */
/* Requirements : AUTORADAR_REQ-938 */
#[no_mangle]
pub unsafe extern "C" fn rlSetSubFrameStart(mut deviceMap: rlUInt8_t,
                                            mut data:
                                                *mut rlSubFrameStartCfg_t)
 -> rlReturnVal_t {
    let mut retVal: rlReturnVal_t = 0;
    /* check if deviceIndex is out of defined value */
    if rlDriverIsDeviceMapValid(deviceMap) != 0 as libc::c_int ||
           data.is_null() {
        /* set return error code */
        retVal = -(2 as libc::c_int)
    } else {
        /* Package the command with given data and send it to device */
        retVal =
            rlDriverExecuteSetApi(deviceMap,
                                  0x8 as libc::c_uint as rlUInt16_t,
                                  0x13 as libc::c_uint as rlUInt16_t,
                                  data as *mut rlUInt8_t,
                                  ::std::mem::size_of::<rlSubFrameStartCfg_t>()
                                      as libc::c_ulong as rlUInt16_t)
    }
    return retVal;
}
/* * @fn rlReturnVal_t rlRfPhShiftCalibDataRestore(rlUInt8_t deviceMap,
*                                                 rlPhShiftCalibrationData_t *data)
*
*   @brief Injects phase shifter calibration data to the device
*   @param[in] deviceMap - Bitmap of devices to send the message
*   @param[in] data - Calibration data of number of TX channels enabled chunks stored at
*                     application space
*
*   @return rlReturnVal_t Success - 0, Failure - Error Code
*
*   This API restores the phase shifter calibration data which was stored previously using the
*   rlRfPhShiftCalibDataStore command. Application needs to feed number of TX channels enabled
*   chunks of phase shifter calibration data. This is device specific feature, please refer data
*   sheet.
*
*   Note: This wrapper function assumes that the calibApply field in the data buffer for each TX
*   channel is set to zero. It will take care of setting the CalibApply to '1' only for the API
*   with the final TX channel's calibration data.
*/
/* DesignId : MMWL_DesignId_109 */
/* Requirements : AUTORADAR_REQ-1003 */
#[no_mangle]
pub unsafe extern "C" fn rlRfPhShiftCalibDataRestore(mut deviceMap: rlUInt8_t,
                                                     mut data:
                                                         *mut rlPhShiftCalibrationData_t)
 -> rlReturnVal_t {
    let mut retVal: rlReturnVal_t = 0;
    let mut idx: rlUInt8_t = 0;
    /* check if deviceIndex is out of defined value */
    if rlDriverIsDeviceMapValid(deviceMap) != 0 as libc::c_int ||
           data.is_null() {
        /* set return error code */
        retVal = -(2 as libc::c_int)
    } else {
        /* Initialize Command and Response Sub Blocks */
        let mut inMsg: rlDriverMsg_t =
            {
                let mut init =
                    rlDriverMsg{opcode:
                                    {
                                        let mut init =
                                            rlDriverOpcode{dir:
                                                               0 as
                                                                   libc::c_int
                                                                   as
                                                                   rlUInt8_t,
                                                           msgType: 0,
                                                           msgId: 0,
                                                           nsbc: 0,};
                                        init
                                    },
                                subblocks: 0 as *mut rlPayloadSb_t,
                                remChunks: 0,};
                init
            };
        let mut outMsg: rlDriverMsg_t =
            {
                let mut init =
                    rlDriverMsg{opcode:
                                    {
                                        let mut init =
                                            rlDriverOpcode{dir:
                                                               0 as
                                                                   libc::c_int
                                                                   as
                                                                   rlUInt8_t,
                                                           msgType: 0,
                                                           msgId: 0,
                                                           nsbc: 0,};
                                        init
                                    },
                                subblocks: 0 as *mut rlPayloadSb_t,
                                remChunks: 0,};
                init
            };
        let mut inPayloadSb: rlPayloadSb_t =
            {
                let mut init =
                    rlPayloadSb{sbid: 0 as libc::c_int as rlUInt16_t,
                                len: 0,
                                pSblkData: 0 as *mut rlUInt8_t,};
                init
            };
        let mut outPayloadSb: rlPayloadSb_t =
            {
                let mut init =
                    rlPayloadSb{sbid: 0 as libc::c_int as rlUInt16_t,
                                len: 0,
                                pSblkData: 0 as *mut rlUInt8_t,};
                init
            };
        /* set return value to zero by default */
        retVal = 0 as libc::c_int;
        /* Invoke Set Calibration data command for RL_MAX_CALIB_DATA_CHUNK chunks */
        idx = 0 as libc::c_uint as rlUInt8_t;
        while (idx as libc::c_uint) < 3 as libc::c_uint {
            /* Construct command packet */
            rlDriverConstructInMsg(0x4 as libc::c_uint as rlUInt16_t,
                                   &mut inMsg, &mut inPayloadSb);
            if idx as libc::c_uint ==
                   (3 as libc::c_uint).wrapping_sub(1 as libc::c_uint) {
                (*data).PhShiftcalibChunk[idx as usize].calibApply =
                    1 as libc::c_uint as rlUInt8_t
            }
            /* Fill in-message Payload */
            /*AR_CODE_REVIEW MR:R.18.1 <REVIEWED> "require pointer increment to jump to
            next calibData */
            /*AR_CODE_REVIEW MR:R.18.1 <REVIEWED> "require pointer increment to jump to
            next calibData */
            /*LDRA_INSPECTED 87 S */
            /*LDRA_INSPECTED 567 S */
            rlDriverFillPayload(0x4 as libc::c_uint as rlUInt16_t,
                                0xc as libc::c_uint as rlUInt16_t,
                                &mut inPayloadSb,
                                &mut *(*data).PhShiftcalibChunk.as_mut_ptr().offset(idx
                                                                                        as
                                                                                        isize)
                                    as *mut rlPhShiftCalibrationStore_t as
                                    *mut rlUInt8_t,
                                ::std::mem::size_of::<rlPhShiftCalibrationStore_t>()
                                    as libc::c_ulong as rlUInt16_t);
            /* Construct response packet */
            rlDriverConstructOutMsg(1 as libc::c_uint as rlUInt16_t,
                                    &mut outMsg, &mut outPayloadSb);
            /* Fill in-message Payload */
            rlDriverFillPayload(0 as libc::c_uint as rlUInt16_t,
                                0 as libc::c_uint as rlUInt16_t,
                                &mut outPayloadSb, 0 as *mut rlUInt8_t,
                                0 as libc::c_uint as rlUInt16_t);
            /* Send Command to mmWave Radar Device */
            retVal += rlDriverCmdInvoke(deviceMap, inMsg, &mut outMsg);
            /* check for return value */
            if 0 as libc::c_int != retVal { break ; }
            idx = idx.wrapping_add(1)
        }
    }
    return retVal;
}
/* * @fn rlReturnVal_t rlRfPhShiftCalibDataStore(rlUInt8_t deviceMap,
*                                                 rlPhShiftCalibrationData_t *data)
*
*   @brief Read calibration data from the device
*   @param[in] deviceMap - Bitmap of devices to send the message.
*   @param[in] data - Phase shift calibration data of number of TX channels enbled chunks which
*                     will filled by device.
*
*   @return rlReturnVal_t Success - 0, Failure - Error Code
*
*   This API reads the phase shifter calibration data from the device which can be injected later
*   using the rlRfPhShifterCalibDataRestore command. RadarSS will return number of TX chunks of
*   phase shifter calibration data.This is device specific feature, please refer data sheet.
*
*   Note: This function assumes that the 3 txIndex fields in the combined data buffer that is being
*   passed to this function is populated with values 0, 1 and 2 to indicate the buffer space for
*   each TX channel.
*/
/* DesignId : MMWL_DesignId_108 */
/* Requirements : AUTORADAR_REQ-1003 */
#[no_mangle]
pub unsafe extern "C" fn rlRfPhShiftCalibDataStore(mut deviceMap: rlUInt8_t,
                                                   mut data:
                                                       *mut rlPhShiftCalibrationData_t)
 -> rlReturnVal_t {
    let mut retVal: rlReturnVal_t = 0;
    /* check if deviceIndex is out of defined value */
    if rlDriverIsDeviceMapValid(deviceMap) != 0 as libc::c_int ||
           data.is_null() {
        /* set return error code */
        retVal = -(2 as libc::c_int)
    } else {
        /* Initialize Command and Response Sub Blocks */
        let mut inMsg: rlDriverMsg_t =
            {
                let mut init =
                    rlDriverMsg{opcode:
                                    {
                                        let mut init =
                                            rlDriverOpcode{dir:
                                                               0 as
                                                                   libc::c_int
                                                                   as
                                                                   rlUInt8_t,
                                                           msgType: 0,
                                                           msgId: 0,
                                                           nsbc: 0,};
                                        init
                                    },
                                subblocks: 0 as *mut rlPayloadSb_t,
                                remChunks: 0,};
                init
            };
        let mut outMsg: rlDriverMsg_t =
            {
                let mut init =
                    rlDriverMsg{opcode:
                                    {
                                        let mut init =
                                            rlDriverOpcode{dir:
                                                               0 as
                                                                   libc::c_int
                                                                   as
                                                                   rlUInt8_t,
                                                           msgType: 0,
                                                           msgId: 0,
                                                           nsbc: 0,};
                                        init
                                    },
                                subblocks: 0 as *mut rlPayloadSb_t,
                                remChunks: 0,};
                init
            };
        let mut inPayloadSb: rlPayloadSb_t =
            {
                let mut init =
                    rlPayloadSb{sbid: 0 as libc::c_int as rlUInt16_t,
                                len: 0,
                                pSblkData: 0 as *mut rlUInt8_t,};
                init
            };
        let mut outPayloadSb: rlPayloadSb_t =
            {
                let mut init =
                    rlPayloadSb{sbid: 0 as libc::c_int as rlUInt16_t,
                                len: 0,
                                pSblkData: 0 as *mut rlUInt8_t,};
                init
            };
        let mut phShifterCaldataGetCfg: rlPhShiftCalDataGetCfg_t =
            {
                let mut init =
                    rlPhShiftCalDataGetCfg{txIndex:
                                               0 as libc::c_uint as rlUInt8_t,
                                           reserved0: 0,
                                           reserved1: 0,};
                init
            };
        let mut idx: rlUInt8_t = 0;
        /* set return value to zero by default */
        /* AR_CODE_REVIEW MR:R.2.2 <REVIEWED> "retVal initialized to 0 if not it means error" */
        /*LDRA_INSPECTED 8 D */
        retVal = 0 as libc::c_int;
        /* Invoke Get Calibration data command for RL_MAX_CALIB_DATA_CHUNK chunks */
        idx = 0 as libc::c_uint as rlUInt8_t;
        while (idx as libc::c_uint) < 3 as libc::c_uint {
            if !(&mut *(*data).PhShiftcalibChunk.as_mut_ptr().offset(idx as
                                                                         isize)
                     as *mut rlPhShiftCalibrationStore_t).is_null() {
                phShifterCaldataGetCfg.txIndex =
                    (*data).PhShiftcalibChunk[idx as usize].txIndex;
                /* Construct command packet */
                rlDriverConstructInMsg(0x5 as libc::c_uint as rlUInt16_t,
                                       &mut inMsg, &mut inPayloadSb);
                /* Fill in-message Payload */
                rlDriverFillPayload(0x5 as libc::c_uint as rlUInt16_t,
                                    0xc as libc::c_uint as rlUInt16_t,
                                    &mut inPayloadSb,
                                    &mut phShifterCaldataGetCfg as
                                        *mut rlPhShiftCalDataGetCfg_t as
                                        *mut rlUInt8_t,
                                    ::std::mem::size_of::<rlPhShiftCalDataGetCfg_t>()
                                        as libc::c_ulong as rlUInt16_t);
                /* Construct response packet */
                rlDriverConstructOutMsg(1 as libc::c_uint as rlUInt16_t,
                                        &mut outMsg, &mut outPayloadSb);
                /* Fill in-message Payload */
                /*AR_CODE_REVIEW MR:R.18.1 <REVIEWED> "require pointer increment to jump to
                  next calibData */
                /*AR_CODE_REVIEW MR:R.18.4 <REVIEWED> "require pointer increment to jump to
                  next calibData */
                /*LDRA_INSPECTED 87 S */
                /*LDRA_INSPECTED 567 S */
                rlDriverFillPayload(0 as libc::c_uint as rlUInt16_t,
                                    0 as libc::c_uint as rlUInt16_t,
                                    &mut outPayloadSb,
                                    &mut *(*data).PhShiftcalibChunk.as_mut_ptr().offset(idx
                                                                                            as
                                                                                            isize)
                                        as *mut rlPhShiftCalibrationStore_t as
                                        *mut rlUInt8_t,
                                    0 as libc::c_uint as rlUInt16_t);
                /* Send Command to mmWave Radar Device */
                retVal += rlDriverCmdInvoke(deviceMap, inMsg, &mut outMsg);
                /* check for return value */
                if 0 as libc::c_int != retVal { break ; }
            }
            idx = idx.wrapping_add(1)
        }
    }
    return retVal;
}
/* * @fn rlReturnVal_t rlGetRfDieId(rlUInt8_t deviceMap, rlRfDieIdCfg_t *data)
*
*   @brief Get device die ID status
*   @param[in] deviceMap - Bitmap of devices to send the message
*   @param[out] data - Die ID status
*
*   @return rlReturnVal_t Success - 0, Failure - Error Code
*
*   This API gets the device Die ID status
*/
/* DesignId :  MMWL_DesignId_110 */
/* Requirements : AUTORADAR_REQ-1004 */
#[no_mangle]
pub unsafe extern "C" fn rlGetRfDieId(mut deviceMap: rlUInt8_t,
                                      mut data: *mut rlRfDieIdCfg_t)
 -> rlReturnVal_t {
    let mut retVal: rlReturnVal_t = 0;
    /* check for NULL pointer */
    if data.is_null() {
        /* set error code if data pointer is passed NULL */
        retVal = -(14 as libc::c_int)
    } else {
        /* Package the command with given data and send it to device */
        retVal =
            rlDriverExecuteGetApi(deviceMap,
                                  0x11 as libc::c_uint as rlUInt16_t,
                                  0x3 as libc::c_uint as rlUInt16_t,
                                  data as *mut rlUInt8_t,
                                  0 as libc::c_uint as rlUInt16_t)
    }
    return retVal;
}
/* * @fn rlReturnVal_t rlRfGetCpuFault(rlUInt8_t deviceMap, rlCpuFault_t *data)
*
*   @brief Get RadarSS CPU fault status
*   @param[in] deviceMap - Bitmap of devices to send the message
*   @param[in] data  - Container for RadarSS CPU fault status
*
*   @return rlReturnVal_t Success - 0, Failure - Error Code
*
*   This API gets the RadarSS CPU fault status.
*/
/* DesignId : MMWL_DesignId_123 */
/* Requirements : AUTORADAR_REQ-1039 */
#[no_mangle]
pub unsafe extern "C" fn rlRfGetCpuFault(mut deviceMap: rlUInt8_t,
                                         mut data: *mut rlCpuFault_t)
 -> rlReturnVal_t {
    let mut retVal: rlReturnVal_t = 0;
    /* check if deviceIndex is out of defined value */
    if rlDriverIsDeviceMapValid(deviceMap) != 0 as libc::c_int ||
           data.is_null() {
        /* set error code if DeviceMAP is invalid or data pointer is null */
        retVal = -(2 as libc::c_int)
    } else {
        /* Package the command with given data and send it to device */
        retVal =
            rlDriverExecuteGetApi(deviceMap,
                                  0x11 as libc::c_uint as rlUInt16_t,
                                  0x1 as libc::c_uint as rlUInt16_t,
                                  data as *mut rlUInt8_t,
                                  ::std::mem::size_of::<rlCpuFault_t>() as
                                      libc::c_ulong as rlUInt16_t)
    }
    return retVal;
}
/* * @fn rlReturnVal_t rlRfGetEsmFault(rlUInt8_t deviceMap, rlBssEsmFault_t *data)
*
*   @brief Get RadarSS ESM fault status.
*   @param[in] deviceMap - Bitmap of devices to send the message
*   @param[in] data  - Container for RadarSS ESM fault status
*
*   @return rlReturnVal_t Success - 0, Failure - Error Code
*
*   This API gets the RadarSS ESM fault status.
*/
/* DesignId : MMWL_DesignId_123 */
/* Requirements : AUTORADAR_REQ-1039 */
#[no_mangle]
pub unsafe extern "C" fn rlRfGetEsmFault(mut deviceMap: rlUInt8_t,
                                         mut data: *mut rlBssEsmFault_t)
 -> rlReturnVal_t {
    let mut retVal: rlReturnVal_t = 0;
    /* check if deviceIndex is out of defined value */
    if rlDriverIsDeviceMapValid(deviceMap) != 0 as libc::c_int ||
           data.is_null() {
        /* set error code if DeviceMAP is invalid or data pointer is null */
        retVal = -(2 as libc::c_int)
    } else {
        /* Package the command with given data and send it to device */
        retVal =
            rlDriverExecuteGetApi(deviceMap,
                                  0x11 as libc::c_uint as rlUInt16_t,
                                  0x2 as libc::c_uint as rlUInt16_t,
                                  data as *mut rlUInt8_t,
                                  ::std::mem::size_of::<rlBssEsmFault_t>() as
                                      libc::c_ulong as rlUInt16_t)
    }
    return retVal;
}
/* * @fn rlReturnVal_t rlRfApllSynthBwCtlConfig(rlUInt8_t deviceMap,
                                             rlRfApllSynthBwControl_t* data)
*
*   @brief Control bandwidth of the APLL and Synthesizer
*   @param[in] deviceMap - Bitmap of devices to send the message
*   @param[in] data - APLL and Synthesizer B/W control data
*
*   @return rlReturnVal_t Success - 0, Failure - Error Code
*
*   This API is used to control bandwidth of the APLL and Synthesizer
*
*   @note 1 : This API is supported only in xWR6843. \n
*   @note 2 : Recommended to issue this API before \ref rlRfInit API. The RF_INIT synthesizer
*             boot calibration shall run after changing the APLL BW. \n
*/
/* DesignId : MMWL_DesignId_126 */
/* Requirements : AUTORADAR_REQ-1053 */
#[no_mangle]
pub unsafe extern "C" fn rlRfApllSynthBwCtlConfig(mut deviceMap: rlUInt8_t,
                                                  mut data:
                                                      *mut rlRfApllSynthBwControl_t)
 -> rlReturnVal_t {
    let mut retVal: rlReturnVal_t = 0;
    /* check if deviceIndex is out of defined value */
    if rlDriverIsDeviceMapValid(deviceMap) != 0 as libc::c_int ||
           data.is_null() {
        /* set return error code */
        retVal = -(2 as libc::c_int)
    } else {
        /* Package the command with given data and send it to device */
        retVal =
            rlDriverExecuteSetApi(deviceMap,
                                  0x4 as libc::c_uint as rlUInt16_t,
                                  0xd as libc::c_uint as rlUInt16_t,
                                  data as *mut rlUInt8_t,
                                  ::std::mem::size_of::<rlRfApllSynthBwControl_t>()
                                      as libc::c_ulong as rlUInt16_t)
    }
    return retVal;
}
/* *
*  @defgroup Sensor Sensor
*  @brief mmwave radar RF/Sensor Configuration Module
*
*  @image html mmwave_frontend.png
*
*  The RF/Sensor Configuration module controls the different HW blocks inside mmWave Front end.
*  mmWave Front End has below key blocks
*  -# Chirp sequencer (Radar Timing Engine) - This block is responsible for constructing
*  the sequence of FMCW chirps or frames and programming the timing engine
*  -# Rx/Tx Channel - This defines how many Rx and Tx channel needs to be enabled. Also it
*  defines how to configure the mmWave front end in cascade mode for Imaging Radar
*  -# Rx Analog Chain - This defines how the received signal is mixed and how different filters
*  in the chain can be configured
*  -# ADC and Digital Front End Configuration - This defines how the IF data is digitized and how
*  it is sampled for further processing in the DSP or Hardware Accelerator. Same ADC data can be
*  sent over LVDS/CSI2 interface to an extenal processor

*  The Sensor Control APIs needs to be called by application in below sequence
*  ## Initial/Static Configuration #
*  Application should first configure the mmWave Front end or Radar SS with below Static
*  configurations
*  - Channel Configuration - \ref rlSetChannelConfig
*  - ADC output Configuration - \ref rlSetAdcOutConfig
*  - Low power mode Configuration  - \ref rlSetLowPowerModeConfig
*
*  ## Initialization and Calibration #
*  After initial static configurations, application should initialize RF
*  and shall wait for calibration complete Asynchornous event
*  RL_RF_AE_INITCALIBSTATUS_SB
*  - \ref rlRfInit
*
*  ## FMCW chirp Configuration #
*  After RF initilization, Application can configure chirps and frame using
*  below APIs
*  - Profile Configuration - \ref rlSetProfileConfig
*  - Chirp Configuraiton - \ref rlSetChirpConfig
*  - Frame Configuration - \ref rlSetFrameConfig or rlSetAdvFrameConfig
*    Note about HW SYNC_IN pulse in hardware triggered mode
*
*    a. The SYNC_IN pulse must not arrive before the frame end boundary
*    b. If frame trigger delay is used with hardware triggered mode, then external SYNC_IN pulse
*    periodicity should take care of the configured frame trigger delay and frame periodicity.
*    The external pulse should be issued only after the sum total of frame trigger delay and
*    frame periodicity. See figure below
*    @image html mmwave_hwsyncincareabout.png
*    c. The inter frame blank time should be at least 250 uS(100 uS for frame preparation and 150
*    uS for any calibration updates to hardware). Add 150 uS to inter-frame blank time for test
*    source configuration if test source is enabled.
*    d.With respect to the SYNC-IN pulse, the actual transmission has 160ns delay and 5ns
*    uncertainty in single-chip and only a 300 ps uncertainty (due to tight inter-chip
*    synchronization needed) in multi-chip sensor applications as defined in rlSetChannelConfig.
*
*  ## Frame Trigger #
*  After All the configuration, Application can use Sensor Start API
*  to start Frame and shall wait for Frame Ready Asynchronous event
*  RL_RF_AE_FRAME_TRIGGER_RDY_SB
*  - \ref rlSensorStart
*
*  ## Below is the list of advance features in mmWave Front end #
*  ## Advance Frame #
* Legacy frame config API \ref rlSetFrameConfig supports looping of the same FMCW frame.
* In order to configure multiple FMCW frames with different chirp profiles, user needs
* to use \ref rlSetAdvFrameConfig API. Advance Frame consists of one or upto 4 Sub-Frames
* Each Sub-Frame consists of multiple bursts. Each burst consists of multiple chirps as
* shown in diagram below.\n
* To enable Advance Frame, Application needs to follow below
* sequence
*  - Profile Configuration - \ref rlSetProfileConfig
*  - Chirp Configuraiton - \ref rlSetChirpConfig
*  - Advance Frame Configuration - \ref rlSetAdvFrameConfig
*
*  @image html adv_frame_seq.png
*
*  ## Dynamic Chirp Configuration #
*  Using Legacy chirp configuration API \ref rlSetChirpConfig, chirps can't be re-configure
*  without stopping the ongoing FMCW frame using \ref rlSensorStop API. \n
*  If user needs to re-configure chirp during the frame, it needs to use Dynamic chirp
*  config APIs. Once the API is received by mmWave Front end, it would re-configure the
*  chirps for next FMCW frame. Dynamic Chirps can be defined using below APIs
*  - Dynamic Chirp Configuration - \ref rlSetDynChirpCfg
*  - Enable Dynamic Chirps - \ref rlSetDynChirpEn
*
*  Diagram below shows the Dynamic Chirp behaviour. Note that since dynamic chirps are
*  configured at run time, there is not error checks done on the input data. If input data
*  is out of range or invalid, device might misbehave.
*
*  @image html dyn_chip_seq.png
*
* ## Calibration #
* TI mmWave Front end includes built-in processor that is programmed by TI to handle RF
   calibrations and functional safety monitoring.  The RF calibrations ensure that the
   performance of the device is maintained across temperature and process corners

  -# Some of the calibrations are just temperature and process based look-up-tables, which are
     used to update the RF/Analog components
  -# Built-in temperature sensors enable the device to monitor the temperature every few seconds
     and update the relevant components accordingly
*
* ## Power Save Modes State Diagram #
*  xWR6x43 devices support 3 power down states, RF POWER DOWN, APLL POWER DOWN and 
*  APLL/GPADC POWER DOWN. The state transitions allowed are shown in the below figure. 
*  Any invalid state transition command would result in the device returning an error. 
*
*  @image html PowerSaveModesStateDiagram.png
*

* Below is the list of calibrations and corresponding duration in microseconds \n
* Boot Time Calibration
<table>
<caption id="bootTimeCalibration">Calibration Duration</caption>
<tr><th>Calibration                      <th>Duration for xWR1xxx(us)<th>Duration for xWR6843(us)
<tr><td>APLL<td>330<td>330
<tr><td>Synth VCO<td>2500<td>2500
<tr><td>LO DIST<td>12<td>12
<tr><td>ADC DC <td>600<td>600
<tr><td>HPF cutoff <td>3500<td>3500
<tr><td>LPF cut off <td>9000<td>200
<tr><td>Peak detector<td>6000<td>8000
<tr><td>TX power (assumes 2 TX use-case)<td>6000<td>6000
<tr><td>RX gain <td>2300<td>1500
<tr><td>TX phase <td>36000<td>36000
<tr><td>RX IQMM (Not applcicable for Real ADC mode)<td>26000<td>42000
</table>

* Run Time Calibration
<table>
<caption id="runTimeCalibration">Calibration Duration</caption>
<tr><th>Calibration                      <th>Duration(us)<th>Duration for xWR6843 (us)
<tr><td>APLL<td>150<td>150
<tr><td>Synth VCO<td>350<td>350
<tr><td>LO DIST<td>30<td>30
<tr><td>Peak detector<td>500<td>600
<tr><td>TX power (assumes 1 TX, 1 profile CLPC)<td>800<td>800
<tr><td>TX power (assumes 1 TX, 1 profile OLPC)<td>30<td>30
<tr><td>RX gain <td>30<td>30
<tr><td>Application of calibration to hardware (This needs to be included always) <td>100<td>100
</table>
*
* ## Chirp, Burst and Frame timings #
* The minimum chirp cycle time, inter-burst time, inter sub-frame/frame
  time requirements for 1st gen xWR1xxx and xWR6843 devices are documented in this section.

* Chirp Cycle Time
<table>
<caption id="chirpCycleTime">Minimum chirp cycle time</caption>
<tr><th>Use case              <th>Min Chirp cycle time(us) for xWR1xxx<th>Min Chirp cycle time(us)
 for xWR6843<th>Description
<tr><td>Typical chirps in normal mode of operation<td>15<td>13<td>The normal chirps used in a 
burst or a frame using legacy chirp configuration API
<tr><td>WDT enable configuration time<td>10<td>NA<td>Add WDT configure time to minimum chirp cycle
time if it is enabled.By default WDT is disabled,it can be enabled using \ref rlRfSetDeviceCfg API
<tr><td>Per chirp phase shifter(PS) configuration time<td>10<td>NA<td>Add per chirp PS configure 
time to minimum chirp cycle time if it is enabled. By default per chirp PS is disabled, it can be 
enabled using AWR_RF_RADAR_MISC_CTL_SB API
<tr><td>Chirps in Continuous framing mode<td>NA<td>20<td>A single chirp used in a burst. 
Continuous framing mode is a mode in which a single chirp is programmed in a burst using advanced 
frame configuration API. In this mode it is recommended to set idle time of chirp minimum 10us to 
save Inter chirp power save override time (Refer below table)
</table>

* Minimum Inter Burst Time
<table>
<caption id="interBurstTime">Minimum Inter Burst Time</caption>
<tr><th>Min inter burst time     <th>Time(us) for xWR1xxx<th>Time(us) for xWR6843<th>Description
<tr><td>Typical inter burst time<td>100 (inter burst pwr save default enabled)<td>55<td>The 
minimum inter burst idle time required in normal bursts with legacy chirps configured in a 
advanced frame configuration API with inter burst power save disabled.
<tr><td>Inter burst power save time<td>NA<td>55<td>Add inter burst power save time to minimum 
inter burst time if it is enabled. By default inter-burst power save is enabled, it can be 
disabled using \ref rlRfSetDeviceCfg API
<tr><td>Inter chirp power save override time (power save disable)<td>15<td>15<td>Add inter chirp 
power save override time to minimum inter burst time if chirp idle time < 10us in a burst or can 
be controlled using \ref rlRfDynamicPowerSave API
<tr><td>Calibration or Monitoring chirp time<td>150<td>145<td>Add calibration or Monitoring chirp 
time to minimum inter burst time if calibration or monitors intended to be run in inter burst idle
time. The calibration and monitoring chirps can run only in inter sub-frame or inter-frame 
interval if this time is not allocated in inter-burst time. Add calibration or Monitoring 
duration to minimum inter burst or sub-frame/frame time based on \ref runTimeCalibration and 
\ref AnalogMonitoringDuration .
<tr><td>Normal chirps (Continuous framing mode)<td>NA<td>10<td>Add Continuous framing mode normal 
chirp configuration time to minimum inter burst time. Continuous framing mode is a mode in which 
a single chirp is programmed in a burst using advanced frame configuration API.
</table>

* Minimum Inter Sub-frame or Frame Time
<table>
<caption id="interSubFrameTime">Minimum Inter Sub-frame or Frame Time</caption>
<tr><th>Min inter subframe/frame time<th>Time(us) for xWR1xxx and xWR6843<th>Description
<tr><td>Typical inter subframe/frame time<td>300<td>The minimum inter sub-frame/frame idle time
required in normal sub-frames with legacy chirps configured in a advanced frame configuration API
or in a legacy frame config API. This time includes time required for minimum inter-burst idle 
time, inter burst power save, inter chirp power save override and single calibration/monitoring
chirp time.
<tr><td>Calibration or Monitoring duration<td>Table 12.2 and Table 12.3<td>Add 
calibration or Monitoring duration to minimum inter sub-frame/frame time
<tr><td>Loop-back burst configuration time<td>300<td>Add Loop-back burst configuration time to
minimum inter sub-frame time for loop back sub-frames if it is enabled in advance frame config API
<tr><td>Dynamic legacy chirp configuration time (for 16 chirps)<td>20 for 16 chirps + 500<td>Add 
dynamic legacy chirp configuration time to minimum inter frame time if dynamic chirp/phase-shifter 
APIs are issued in runtime
<tr><td>Dynamic profile configuration time (for 1 profile)<td>1200<td>Add dynamic profile 
configuration time to minimum inter frame time if dynamic profile API is issued in runtime.
<tr><td>Test source config time<td>150 for xWR1xxx devices and 170 for xWR6843 devices<td>Add 
test source configuration time to minimum inter sub-frame time if test source API is issued.
</table>

* Typical APLL and Synth BW settings
<table>
<caption id="ApllSynthBwSettings">Typical APLL and Synth BW settings</caption>
<tr><th>Synth ICP<th>Synth Rtrim<th>APLL ICP<th>APLL Rtrim LPF
    <th>APLL Rtrim VCO<th>VCO1_BW<th>VCO2_BW<th>APLL_BW<th>Emission Improv
    <th>1M PN Degrad at 60G<th>100K PN Improv at 60G<th>Max Slope (MHz/us)
<tr><td>1<td>8<td>0x26<td>0x9<td>8<td>1.5M<td>1.5M<td>150K<td>2 dB<td>2 dB<td>0 dB<td>250
<tr><td>3<td>8<td>0x26<td>0x9<td>8<td>0.75M<td>0.75M<td>150K<td>2 dB<td>2 dB<td>0 dB<td>125
<tr><td>1<td>8<td>0x3F<td>0x9<td>8<td>1.5M<td>1.5M<td>300K<td>2 dB<td>4 dB<td>5 dB<td>250
<tr><td>1<td>8<td>0x26<td>0x9<td>5<td>1.5M<td>1.5M<td>150K<td>8 dB<td>1.5 dB<td>0 dB<td>250
<tr><td>3<td>8<td>0x26<td>0x9<td>5<td>0.75M<td>0.75M<td>150K<td>8 dB<td>1.5 dB<td>0 dB<td>125
<tr><td>1<td>8<td>0x3F<td>0x9<td>5<td>1.5M<td>1.5M<td>300K<td>8 dB<td>3.5 dB<td>5 dB<td>250
<tr><td>1<td>8<td>0x26<td>0x9<td>6<td>1.5M<td>1.5M<td>150K<td>5 dB<td>1 dB<td>0 dB<td>250
<tr><td>3<td>8<td>0x26<td>0x9<td>6<td>0.75M<td>0.75M<td>150K<td>5 dB<td>1 dB<td>0 dB<td>125
<tr><td>1<td>8<td>0x3F<td>0x9<td>6<td>1.5M<td>1.5M<td>300K<td>5 dB<td>3 dB<td>5 dB<td>250
<tr><td>1<td>8<td>0x26<td>0x9<td>18<td>1.5M<td>1.5M<td>150K<td>0 dB<td>0 dB<td>0 dB<td>250
<tr><td>3<td>8<td>0x26<td>0x9<td>18<td>0.75M<td>0.75M<td>150K<td>0 dB<td>0 dB<td>0 dB<td>125
<tr><td>1<td>8<td>0x3F<td>0x9<td>18<td>1.5M<td>1.5M<td>300K<td>0 dB<td>2 dB<td>5 dB<td>250
</table>

*    Related Files
*   - rl_sensor.c
*  @addtogroup Sensor
*  @{
*/
/* *****************************************************************************
 * FUNCTION DECLARATIONS
 ******************************************************************************
 */
/* RF/Sensor Configuration Functions */
/*Rx and Tx Channel Configuration */
/*ADC Out Configuration */
/*Low Power Mode */
/*RF Init */
/*Profile Configuration */
/*Chirp Configuration */
/*Frame Configuration */
/*Sensor Trigger */
/*Advance Frame Configuration */
/*Continous mode Configuration */
/*BPM Configuration */
/*Test Source Configuration */
/*Get RF Characterization Time and Temperature information */
/*Get RF Digital Front End Statistics */
/*Dynamic Power save Configuration */
/*RadarSS Device configuration */
/*GPADC Read(From external Input) configuration */
/* LDO bypass Configuration */
/*Per Chirp Phase Shifter Configuration */
/* PA loopback Configuration */
/* Phase Shift Loopback Configuration */
/* IF loopback Configuration */
/* Programmable Filter RAM coefficients */
/* programmable Filter Configuration */
/*Radar Misc Configuration */
/*Calibration/monitoring Time Unit Configuration */
/*Calibration/monitoring Freq Limit Configuration */
/*Init time calibration Configuration */
/*Run time calibration Configuration */
/*Rx Gain Look up Table (LUT) Update APIs */
/*Tx Gain Look up Table (LUT) Update APIs */
/*TX freq and power limits monitoring configuration */
/*Looback chirp configuration API */
/*Dynamic chirp configuration API */
/*Dynamic per-chirp phase shifter configuration API(AWR1243P) */
/*Calibration data store/restore configuration */
/*Update Inter Rx Gain/Phase offsets for Inter-RX mismatch compensation */
/*Get RadarSS/BSS bootup(Boot time monitoring tests) status */
/*Update Inter Rx Gain/Phase offsets for Inter-RX mismatch compensation */
/*Sub frame trigger API */
/*Phase shifter calibration data store/restore configuration*/
/*Get device die ID status*/
/* Get RadarSS CPU/ESM fault status functions */
/* APLL and Synthesizer Bandwidth control API */
/*Power Saving Mode configuration */
/* * @fn rlReturnVal_t rlSetPowerSaveModeConfig(rlUInt8_t deviceMap, rlPowerSaveModeCfg_t* data)
*
*   @brief Sets the power save mode Configuration
*   @param[in] deviceMap - Bitmap of devices to send the message
*   @param[in] data - Container for power save mode configuration data
*
*   @return rlReturnVal_t Success - 0, Failure - Error Code
*
*   This API defines the power saving modes configuration.
*
*   This section briefly describes the order to issue the various API SBs to enter/exit the
*   various RF power saving modes. \n
*   - Follow the same sequence laid out for single chip till the frames have been triggered. \n
*   - Wait for all configured frames to complete or issue an AWR_FRAMESTARTSTOP_CONF_SB to stop
*     them. \n
*   - AWR_POWER_SAVE_MODE_CONF_SET_SB \n
*     1. Issue the AWR_POWER_SAVE_MODE_CONF_SET_SB API with the LOWPOWER_STATE_TRANSITION_CMD
*        field set to '1'. This will make the device enter the RF power down state.
*     2. Ensure that the MSS and DSS clock sources have been moved to XTAL as we will be shutting
*        down the APLL in the next step. \n
*     3. Issue the AWR_POWER_SAVE_MODE_CONF_SET_SB API with the LOWPOWER_STATE_TRANSITION_CMD
*        field set to '3'. This will make the device enter APLL power down state. The user can
*        also choose to enter the more restrictive APLL and GPADC power down state by setting the
*        LOWPOWER_STATE_TRANSITION_CMD field to '5'. \n
*     4. Issue the AWR_POWER_SAVE_MODE_CONF_SET_SB API with the LOWPOWER_STATE_TRANSITION_CMD
*        field set to '4'. This will make the device exit APLL power down state back to its
*        previous state which is the RF power down state. If the user had entered APLL and GPADC
*        power down state, he will have to issue the API with the LOWPOWER_STATE_TRANSITION_CMD
*        set to '6'. Since the APLL is active once this state transition is complete, the user can
*        switch the MSS and DSS clock source back to APLL. \n
*     5. Issue the AWR_POWER_SAVE_MODE_CONF_SET_SB API with the LOWPOWER_STATE_TRANSITION_CMD
*        field set to '2'. This will make the device exit RF power down state back to the active
*        state. \n
*   - Now that the device is back to active state, the user needs to completely re-configure the
*     device from AWR_RF_STATIC_CONF_SET_SB API.
*
*   @note 1 : Any invalid state transition command would result in the device returning an error.
*   @note 2 : This API is only applicable to xWR6x43 devices in this release.
*   @note 3 : These low power state transitions are executed at the lowest priority in RADARSS. 
*             It is recommended to provide at least 1ms delay after issuing a 
*             AWR_POWER_SAVE_MODE_CONF_SET_SB API and before any other command is issued to the 
*             RADARSS. This will ensure that RADARSS has enough time to complete the previous 
*             transition before receiving other commands.
*/
/* DesignId : MMWL_DesignId_127 */
/* Requirements : AUTORADAR_REQ-1072 */
#[no_mangle]
pub unsafe extern "C" fn rlSetPowerSaveModeConfig(mut deviceMap: rlUInt8_t,
                                                  mut data:
                                                      *mut rlPowerSaveModeCfg_t)
 -> rlReturnVal_t {
    let mut retVal: rlReturnVal_t = 0;
    /* check if deviceIndex is out of defined value */
    if rlDriverIsDeviceMapValid(deviceMap) != 0 as libc::c_int ||
           data.is_null() {
        /* set return error code */
        retVal = -(2 as libc::c_int)
    } else {
        /* Package the command with given data and send it to device */
        retVal =
            rlDriverExecuteSetApi(deviceMap,
                                  0xc as libc::c_uint as rlUInt16_t,
                                  0x2 as libc::c_uint as rlUInt16_t,
                                  data as *mut rlUInt8_t,
                                  ::std::mem::size_of::<rlPowerSaveModeCfg_t>()
                                      as libc::c_ulong as rlUInt16_t)
    }
    return retVal;
}
/*
 * END OF rl_sensor.c FILE
 */
