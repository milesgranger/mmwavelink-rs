#![allow(
    dead_code,
    mutable_transmutes,
    non_camel_case_types,
    non_snake_case,
    non_upper_case_globals,
    unused_assignments,
    unused_mut
)]

use c2rust_bitfields::BitfieldStruct;

pub unsafe extern "C" fn memcpy(
    dest: *mut core::ffi::c_void,
    src: *const core::ffi::c_void,
    n: usize,
) -> *mut core::ffi::c_void {
    core::ptr::copy_nonoverlapping(src, dest, n);
    dest
}

pub unsafe extern "C" fn memset(
    dest: *mut core::ffi::c_void,
    c: core::ffi::c_int,
    n: usize,
) -> *mut core::ffi::c_void {
    // TODO: Rust impl of memset (if needed?)
    dest
}

use crate::controller::{
    rlAppendDummy, rlAppendSubBlock, rlGetSubBlock, rlGetSubBlockId, rlGetSubBlockLen,
};

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
pub type rlUInt8_t = core::ffi::c_uchar;
pub type rlUInt16_t = core::ffi::c_ushort;
pub type rlUInt32_t = core::ffi::c_uint;
pub type rlInt8_t = core::ffi::c_char;
pub type rlInt32_t = core::ffi::c_int;
/* ! \brief
* Communication Interface Handle
*/
pub type rlComIfHdl_t = *mut core::ffi::c_void;
/* ! \brief
* OS Message Queue Object Handle
*/
pub type rlOsiMsgQHdl_t = *mut core::ffi::c_void;
/* ! \brief
* OS Semaphore Object Handle
*/
pub type rlOsiSemHdl_t = *mut core::ffi::c_void;
/* ! \brief
* OS Mutex Object Handle
*/
pub type rlOsiMutexHdl_t = *mut core::ffi::c_void;
/* ! \brief
* OS Time data type
*/
pub type rlOsiTime_t = rlUInt32_t;
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
* mmWaveLink Support CRC type
*/
pub type rlCrcType_t = rlUInt8_t;
/* Function pointers for spawn task function and event handlers*/
/* ! \brief
* mmWaveLink Spawn Task Function
*/
pub type RL_P_OSI_SPAWN_ENTRY = Option<unsafe extern "C" fn(_: *const core::ffi::c_void) -> ()>;
/* ! \brief
* mmWaveLink Event Handler callback
*/
pub type RL_P_EVENT_HANDLER =
    Option<unsafe extern "C" fn(_: rlUInt8_t, _: *mut core::ffi::c_void) -> ()>;
/* ! \brief
* Communication interface(SPI, MailBox, UART etc) callback functions
*/
#[derive(Copy, Clone)]
#[repr(C)]
pub struct rlComIfCbs {
    pub rlComIfOpen: Option<unsafe extern "C" fn(_: rlUInt8_t, _: rlUInt32_t) -> rlComIfHdl_t>,
    pub rlComIfRead: Option<
        unsafe extern "C" fn(_: rlComIfHdl_t, _: *mut rlUInt8_t, _: rlUInt16_t) -> rlInt32_t,
    >,
    pub rlComIfWrite: Option<
        unsafe extern "C" fn(_: rlComIfHdl_t, _: *mut rlUInt8_t, _: rlUInt16_t) -> rlInt32_t,
    >,
    pub rlComIfClose: Option<unsafe extern "C" fn(_: rlComIfHdl_t) -> rlInt32_t>,
}
pub type rlComIfCbs_t = rlComIfCbs;
/* ! \brief
* OS mutex callback functions
*/
#[derive(Copy, Clone)]
#[repr(C)]
pub struct rlOsiMutexCbs {
    pub rlOsiMutexCreate:
        Option<unsafe extern "C" fn(_: *mut rlOsiMutexHdl_t, _: *mut rlInt8_t) -> rlInt32_t>,
    pub rlOsiMutexLock:
        Option<unsafe extern "C" fn(_: *mut rlOsiMutexHdl_t, _: rlOsiTime_t) -> rlInt32_t>,
    pub rlOsiMutexUnLock: Option<unsafe extern "C" fn(_: *mut rlOsiMutexHdl_t) -> rlInt32_t>,
    pub rlOsiMutexDelete: Option<unsafe extern "C" fn(_: *mut rlOsiMutexHdl_t) -> rlInt32_t>,
}
pub type rlOsiMutexCbs_t = rlOsiMutexCbs;
/* ! \brief
* OS semaphore callback functions
*/
#[derive(Copy, Clone)]
#[repr(C)]
pub struct rlOsiSemCbs {
    pub rlOsiSemCreate:
        Option<unsafe extern "C" fn(_: *mut rlOsiSemHdl_t, _: *mut rlInt8_t) -> rlInt32_t>,
    pub rlOsiSemWait:
        Option<unsafe extern "C" fn(_: *mut rlOsiSemHdl_t, _: rlOsiTime_t) -> rlInt32_t>,
    pub rlOsiSemSignal: Option<unsafe extern "C" fn(_: *mut rlOsiSemHdl_t) -> rlInt32_t>,
    pub rlOsiSemDelete: Option<unsafe extern "C" fn(_: *mut rlOsiSemHdl_t) -> rlInt32_t>,
}
pub type rlOsiSemCbs_t = rlOsiSemCbs;
/* ! \brief
* OS message queue/Spawn callback functions
*/
#[derive(Copy, Clone)]
#[repr(C)]
pub struct rlOsiMsgQCbs {
    pub rlOsiSpawn: Option<
        unsafe extern "C" fn(
            _: RL_P_OSI_SPAWN_ENTRY,
            _: *const core::ffi::c_void,
            _: rlUInt32_t,
        ) -> rlInt32_t,
    >,
}
pub type rlOsiMsgQCbs_t = rlOsiMsgQCbs;
/* ! \brief
* OS services callback functions
*/
#[derive(Copy, Clone)]
#[repr(C)]
pub struct rlOsiCbs {
    pub mutex: rlOsiMutexCbs_t,
    pub sem: rlOsiSemCbs_t,
    pub queue: rlOsiMsgQCbs_t,
}
pub type rlOsiCbs_t = rlOsiCbs;
/* ! \brief
* mmWaveLink Asynchronous event callback function
*/
#[derive(Copy, Clone)]
#[repr(C)]
pub struct rlEventCbs {
    pub rlAsyncEvent: Option<
        unsafe extern "C" fn(_: rlUInt8_t, _: rlUInt16_t, _: rlUInt16_t, _: *mut rlUInt8_t) -> (),
    >,
}
pub type rlEventCbs_t = rlEventCbs;
/* ! \brief
* mmWaveLink Timer callback functions
*/
#[derive(Copy, Clone)]
#[repr(C)]
pub struct rlTimerCbs {
    pub rlDelay: Option<unsafe extern "C" fn(_: rlUInt32_t) -> rlInt32_t>,
}
pub type rlTimerCbs_t = rlTimerCbs;
/* ! \brief
* mmWaveLink callback functions for Command parser
*/
#[derive(Copy, Clone)]
#[repr(C)]
pub struct rlCmdParserCbs {
    pub rlCmdParser: Option<unsafe extern "C" fn(_: rlUInt8_t, _: rlInt32_t) -> rlInt32_t>,
    pub rlPostCnysStep: Option<unsafe extern "C" fn(_: rlUInt8_t) -> rlInt32_t>,
}
pub type rlCmdParserCbs_t = rlCmdParserCbs;
/* ! \brief
* mmWaveLink CRC callback function
* @note : Device SPI protocol Limitation for AWR1243: The CRC length of the message
* or Async-event shall be multiple of 4 bytes to enable reliable retry recovery
* mechanism in case of any checksum failure in a message.
*/
#[derive(Copy, Clone)]
#[repr(C)]
pub struct rlCrcCbs {
    pub rlComputeCRC: Option<
        unsafe extern "C" fn(
            _: *mut rlUInt8_t,
            _: rlUInt32_t,
            _: rlUInt8_t,
            _: *mut rlUInt8_t,
        ) -> rlInt32_t,
    >,
}
pub type rlCrcCbs_t = rlCrcCbs;
/* ! \brief
* mmWaveLink Device Control, Interrupt callback functions
*/
#[derive(Copy, Clone)]
#[repr(C)]
pub struct rlDeviceCtrlCbs {
    pub rlDeviceEnable: Option<unsafe extern "C" fn(_: rlUInt8_t) -> rlInt32_t>,
    pub rlDeviceDisable: Option<unsafe extern "C" fn(_: rlUInt8_t) -> rlInt32_t>,
    pub rlDeviceMaskHostIrq: Option<unsafe extern "C" fn(_: rlComIfHdl_t) -> ()>,
    pub rlDeviceUnMaskHostIrq: Option<unsafe extern "C" fn(_: rlComIfHdl_t) -> ()>,
    pub rlDeviceWaitIrqStatus:
        Option<unsafe extern "C" fn(_: rlComIfHdl_t, _: rlUInt8_t) -> rlInt32_t>,
    pub rlCommIfAssertIrq: Option<unsafe extern "C" fn(_: rlUInt8_t) -> rlUInt16_t>,
    pub rlRegisterInterruptHandler: Option<
        unsafe extern "C" fn(
            _: rlUInt8_t,
            _: RL_P_EVENT_HANDLER,
            _: *mut core::ffi::c_void,
        ) -> rlInt32_t,
    >,
}
pub type rlDeviceCtrlCbs_t = rlDeviceCtrlCbs;
/* ! \brief
* mmWaveLink print function prototype
*/
pub type rlPrintFptr = Option<unsafe extern "C" fn(_: *const rlInt8_t, _: ...) -> rlInt32_t>;
/* ! \brief
* mmWaveLink debug callback structure
*/
#[derive(Copy, Clone)]
#[repr(C)]
pub struct rlDbgCb {
    pub rlPrint: rlPrintFptr,
    pub dbgLevel: rlUInt8_t,
}
pub type rlDbgCb_t = rlDbgCb;
/* ! \brief
* mmWaveLink client callback structure
*/
#[derive(Copy, Clone)]
#[repr(C)]
pub struct rlClientCbs {
    pub comIfCb: rlComIfCbs_t,
    pub osiCb: rlOsiCbs_t,
    pub eventCb: rlEventCbs_t,
    pub devCtrlCb: rlDeviceCtrlCbs_t,
    pub timerCb: rlTimerCbs_t,
    pub cmdParserCb: rlCmdParserCbs_t,
    pub crcCb: rlCrcCbs_t,
    pub crcType: rlCrcType_t,
    pub ackTimeout: rlUInt32_t,
    pub platform: rlUInt8_t,
    pub arDevType: rlUInt8_t,
    pub dbgCb: rlDbgCb_t,
}
pub type rlClientCbs_t = rlClientCbs;
/* ***************************************************************************************
 * FileName     : rl_protocol.h
 *
 * Description  : This file defines the functions required for Communication Protocol
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
/* RHCP Length Constants*/
/* RHCP CRC max Length in bytes */
/* *< Max Payload Len (256 - 16(HDR) - 8(CRC)) */
/* ! \brief
* mmWaveLink API Error Type
*/
pub type rlSysNRespType_t = rlUInt16_t;
/* ! \brief
* mmWaveLink API Error Sub block structure
*/
#[derive(Copy, Clone)]
#[repr(C)]
pub struct rlErrorResp {
    pub errorType: rlSysNRespType_t,
    pub sbcID: rlUInt16_t,
}
pub type rlErrorResp_t = rlErrorResp;
/* ! \brief
* RHCP SYNC Pattern Structure
*/
#[derive(Copy, Clone)]
#[repr(C)]
pub struct rlSyncPattern {
    pub sync1: rlUInt16_t,
    pub sync2: rlUInt16_t,
}
pub type rlSyncPattern_t = rlSyncPattern;
/* ! \brief
* Command op-code ID contains 4 fields (16 bits)
* Bit 10-15: Reserved
* Bit 8-9: Operation - Set/Get.
* Bit 4-7: Operation Type
* Bit 0-3: Direction
*/
#[derive(Copy, Clone, BitfieldStruct)]
#[repr(C)]
pub struct rlOpcode {
    #[bitfield(name = "b4Direction", ty = "rlUInt16_t", bits = "0..=3")]
    #[bitfield(name = "b2MsgType", ty = "rlUInt16_t", bits = "4..=5")]
    #[bitfield(name = "b10MsgId", ty = "rlUInt16_t", bits = "6..=15")]
    pub b4Direction_b2MsgType_b10MsgId: [u8; 2],
}
pub type rlOpcode_t = rlOpcode;
/* ! \brief
* Command op-code ID contains 4 fields (16 bits)
* Bit 10-15: Reserved
* Bit 8-9: Operation - Set/Get.
* Bit 4-7: Operation Type
* Bit 0-3: Direction
*/
#[derive(Copy, Clone, BitfieldStruct)]
#[repr(C)]
pub struct rlHdrFlags {
    #[bitfield(name = "b2RetryFlag", ty = "rlUInt16_t", bits = "0..=1")]
    #[bitfield(name = "b2AckFlag", ty = "rlUInt16_t", bits = "2..=3")]
    #[bitfield(name = "b4Version", ty = "rlUInt16_t", bits = "4..=7")]
    #[bitfield(name = "b2Crc", ty = "rlUInt16_t", bits = "8..=9")]
    #[bitfield(name = "b2CrcLen", ty = "rlUInt16_t", bits = "10..=11")]
    #[bitfield(name = "b4SeqNum", ty = "rlUInt16_t", bits = "12..=15")]
    pub b2RetryFlag_b2AckFlag_b4Version_b2Crc_b2CrcLen_b4SeqNum: [u8; 2],
}
pub type rlHdrFlags_t = rlHdrFlags;
/* ! \brief
* RHCP protocol header structure
*/
#[derive(Copy, Clone)]
#[repr(C)]
pub struct rlProtHeader {
    pub opcode: rlOpcode_t,
    pub len: rlUInt16_t,
    pub flags: rlHdrFlags_t,
    pub remChunks: rlUInt16_t,
    pub nsbc: rlUInt16_t,
    pub chksum: rlUInt16_t,
}
pub type rlProtHeader_t = rlProtHeader;
/* ! \brief
* RHCP message structure
*/
#[derive(Copy, Clone)]
#[repr(C)]
pub struct rlRhcpMsg {
    pub syncPattern: rlSyncPattern_t,
    pub hdr: rlProtHeader_t,
    pub payload: [rlUInt8_t; 240],
}
pub type rlRhcpMsg_t = rlRhcpMsg;
#[derive(Copy, Clone)]
#[repr(C)]
pub struct rlPayloadSb {
    pub sbid: rlUInt16_t,
    pub len: rlUInt16_t,
    pub pSblkData: *mut rlUInt8_t,
}
/* MSG must be multiple of this value */
/* ! \brief
* RHCP Payload Structure
*/
pub type rlPayloadSb_t = rlPayloadSb;
#[derive(Copy, Clone)]
#[repr(C)]
pub struct rlAsyncEvt {
    pub evtMsg: rlRhcpMsg_t,
}
/* ! \brief
* RHCP Async Event structure
*/
pub type rlAsyncEvt_t = rlAsyncEvt;
#[derive(Copy, Clone)]
#[repr(C)]
pub struct rlFunctionParams {
    pub cmd: *mut rlRhcpMsg_t,
    pub rsp: *mut rlRhcpMsg_t,
    pub asyncEvt: rlAsyncEvt_t,
    pub rxMsgClass: rlUInt8_t,
    pub alignReserved1: rlUInt8_t,
    pub alignReserved2: rlUInt8_t,
    pub alignReserved3: rlUInt8_t,
    pub msgCRC: [rlUInt8_t; 8],
}
/* ! \brief
* mmwave radar Driver Function Params
*/
pub type rlFunctionParams_t = rlFunctionParams;
#[derive(Copy, Clone)]
#[repr(C)]
pub struct rlComDevInx {
    pub comIfHdl: [rlComIfHdl_t; 4],
    pub rlDevIndex: [rlUInt8_t; 4],
}
/* ! \brief
* Communication handle and device-index for deifferent devices connected to Host
*/
pub type rlComDevInx_t = rlComDevInx;
#[derive(Copy, Clone)]
#[repr(C)]
pub struct loggingFunctions {
    pub rlPrintAr: [rlPrintFptr; 5],
}
/* *
 * @brief  Communication Interface Handles
 */
/* *
 * @brief  stores device Index
 */
/* ! \brief
* mmwave Logging functions
*/
pub type rlLogCtx_t = loggingFunctions;
#[derive(Copy, Clone)]
#[repr(C)]
pub struct rlDriverData {
    pub funcParams: rlFunctionParams_t,
    pub isDriverInitialized: rlUInt8_t,
    pub deviceMap: rlUInt8_t,
    pub isCmdRespWaited: [rlUInt8_t; 4],
    pub isRespWriteWaited: [rlUInt8_t; 4],
    pub rxIrqCnt: [rlUInt8_t; 4],
    pub rxDoneCnt: [rlUInt8_t; 4],
    pub cmdSeqNum: [rlUInt16_t; 4],
    pub commDevIdx: rlComDevInx_t,
    pub globalMutex: rlOsiMutexHdl_t,
    pub cmdSem: rlOsiSemHdl_t,
    pub spawnQueue: rlOsiMsgQHdl_t,
    pub clientCtx: rlClientCbs_t,
    pub logObj: rlLogCtx_t,
    pub retryCount: rlUInt8_t,
    pub txMsgPtr: *mut rlRhcpMsg_t,
    pub rxMsgPtr: *mut rlRhcpMsg_t,
}
/* ! \brief
* mmwave radar Driver Global Structure
*/
pub type rlDriverData_t = rlDriverData;
#[derive(Copy, Clone)]
#[repr(C)]
pub struct rlSyncHeader {
    pub syncPattern: rlSyncPattern_t,
    pub protHdr: rlProtHeader_t,
}
pub type rlSyncHeader_t = rlSyncHeader;
#[derive(Copy, Clone)]
#[repr(C)]
pub union rlReadBuf {
    pub tempBuf: core::mem::ManuallyDrop<[rlUInt8_t; 16]>,
    pub syncHeader: rlSyncHeader_t,
}
/* *
 * @brief  Current API parameters
 */
/* *
 * @brief  Driver Status
 */
/* *
 * @brief  Bitmap of devices connected radarSS/DSS Mailbox in case of 16xx autonomous
 */
/* *
 * @brief  Driver Command Wait Flag
 */
/* *
 * @brief  if writing a data waits for Host IRQ
 */
/* *
 * @brief  Driver Host Interrupt count
 */
/* *
 * @brief  Driver serviced Host Interrupt count
 */
/* *
 * @brief  Driver command sequnce number
 */
/* *
 * @brief Communication handle and device-index for deifferent devices connected to Host
 */
/* *
 * @brief  Driver Global Lock Mutex
 */
/* *
 * @brief  Driver Command Wait Semaphore
 */
/* *
 * @brief  Driver Spawn queue
 */
/* *
 * @brief  Client context
 */
/* *
 * @brief  As per debug level callback functions will be assinged
 */
/* *
 * @brief  Retry count for re-sending command if device doesn't send
 * response within timeout or device sends NACK.
 */
/* *
 * @brief  Tx message buffer pointer
 */
/* *
 * @brief  Rx message buffer pointer
 */
/* ! \brief
* mmwave radar Driver Protocol header read buffer
*/
pub type rlReadBuf_t = rlReadBuf;
#[derive(Copy, Clone)]
#[repr(C)]
pub struct rlDriverOpcode {
    pub dir: rlUInt8_t,
    pub msgType: rlUInt8_t,
    pub msgId: rlUInt16_t,
    pub nsbc: rlUInt16_t,
}
/* ! \brief
* mmwave radar Driver Opcode
*/
pub type rlDriverOpcode_t = rlDriverOpcode;
#[derive(Copy, Clone)]
#[repr(C)]
pub struct rlDriverMsg {
    pub opcode: rlDriverOpcode_t,
    pub subblocks: *mut rlPayloadSb_t,
    pub remChunks: rlUInt16_t,
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
/* ! \brief
* mmwave radar Driver Payload
*/
pub type rlDriverMsg_t = rlDriverMsg;
/* *****************************************************************************
 * GLOBAL VARIABLES/DATA-TYPES DEFINITIONS
 ******************************************************************************
 */
/* mmwave radar Driver global structure */
static mut rl_driverData: rlDriverData_t = {
    let mut init = rlDriverData {
        funcParams: {
            let mut init = rlFunctionParams {
                cmd: 0 as *const rlRhcpMsg_t as *mut rlRhcpMsg_t,
                rsp: 0 as *const rlRhcpMsg_t as *mut rlRhcpMsg_t,
                asyncEvt: rlAsyncEvt_t {
                    evtMsg: rlRhcpMsg_t {
                        syncPattern: rlSyncPattern_t { sync1: 0, sync2: 0 },
                        hdr: rlProtHeader_t {
                            opcode: rlOpcode_t {
                                b4Direction_b2MsgType_b10MsgId: [0; 2],
                            },
                            len: 0,
                            flags: rlHdrFlags_t {
                                b2RetryFlag_b2AckFlag_b4Version_b2Crc_b2CrcLen_b4SeqNum: [0; 2],
                            },
                            remChunks: 0,
                            nsbc: 0,
                            chksum: 0,
                        },
                        payload: [0; 240],
                    },
                },
                rxMsgClass: 0,
                alignReserved1: 0,
                alignReserved2: 0,
                alignReserved3: 0,
                msgCRC: [0; 8],
            };
            init
        },
        isDriverInitialized: 0,
        deviceMap: 0,
        isCmdRespWaited: [0; 4],
        isRespWriteWaited: [0; 4],
        rxIrqCnt: [0; 4],
        rxDoneCnt: [0; 4],
        cmdSeqNum: [0; 4],
        commDevIdx: rlComDevInx_t {
            comIfHdl: [0 as *const core::ffi::c_void as *mut core::ffi::c_void; 4],
            rlDevIndex: [0; 4],
        },
        globalMutex: 0 as *const core::ffi::c_void as *mut core::ffi::c_void,
        cmdSem: 0 as *const core::ffi::c_void as *mut core::ffi::c_void,
        spawnQueue: 0 as *const core::ffi::c_void as *mut core::ffi::c_void,
        clientCtx: rlClientCbs_t {
            comIfCb: rlComIfCbs_t {
                rlComIfOpen: None,
                rlComIfRead: None,
                rlComIfWrite: None,
                rlComIfClose: None,
            },
            osiCb: rlOsiCbs_t {
                mutex: rlOsiMutexCbs_t {
                    rlOsiMutexCreate: None,
                    rlOsiMutexLock: None,
                    rlOsiMutexUnLock: None,
                    rlOsiMutexDelete: None,
                },
                sem: rlOsiSemCbs_t {
                    rlOsiSemCreate: None,
                    rlOsiSemWait: None,
                    rlOsiSemSignal: None,
                    rlOsiSemDelete: None,
                },
                queue: rlOsiMsgQCbs_t { rlOsiSpawn: None },
            },
            eventCb: rlEventCbs_t { rlAsyncEvent: None },
            devCtrlCb: rlDeviceCtrlCbs_t {
                rlDeviceEnable: None,
                rlDeviceDisable: None,
                rlDeviceMaskHostIrq: None,
                rlDeviceUnMaskHostIrq: None,
                rlDeviceWaitIrqStatus: None,
                rlCommIfAssertIrq: None,
                rlRegisterInterruptHandler: None,
            },
            timerCb: rlTimerCbs_t { rlDelay: None },
            cmdParserCb: rlCmdParserCbs_t {
                rlCmdParser: None,
                rlPostCnysStep: None,
            },
            crcCb: rlCrcCbs_t { rlComputeCRC: None },
            crcType: 0,
            ackTimeout: 0,
            platform: 0,
            arDevType: 0,
            dbgCb: rlDbgCb_t {
                rlPrint: None,
                dbgLevel: 0,
            },
        },
        logObj: rlLogCtx_t {
            rlPrintAr: [None; 5],
        },
        retryCount: 0,
        txMsgPtr: 0 as *const rlRhcpMsg_t as *mut rlRhcpMsg_t,
        rxMsgPtr: 0 as *const rlRhcpMsg_t as *mut rlRhcpMsg_t,
    };
    init
};
/* mmwave radar Driver Command/Response Buffer */
#[no_mangle]
pub static mut rl_txMsg: rlRhcpMsg_t = {
    let mut init = rlRhcpMsg {
        syncPattern: {
            let mut init = rlSyncPattern {
                sync1: 0 as core::ffi::c_int as rlUInt16_t,
                sync2: 0,
            };
            init
        },
        hdr: rlProtHeader_t {
            opcode: rlOpcode_t {
                b4Direction_b2MsgType_b10MsgId: [0; 2],
            },
            len: 0,
            flags: rlHdrFlags_t {
                b2RetryFlag_b2AckFlag_b4Version_b2Crc_b2CrcLen_b4SeqNum: [0; 2],
            },
            remChunks: 0,
            nsbc: 0,
            chksum: 0,
        },
        payload: [0; 240],
    };
    init
};
#[no_mangle]
pub static mut rl_rxMsg: rlRhcpMsg_t = {
    let mut init = rlRhcpMsg {
        syncPattern: {
            let mut init = rlSyncPattern {
                sync1: 0 as core::ffi::c_int as rlUInt16_t,
                sync2: 0,
            };
            init
        },
        hdr: rlProtHeader_t {
            opcode: rlOpcode_t {
                b4Direction_b2MsgType_b10MsgId: [0; 2],
            },
            len: 0,
            flags: rlHdrFlags_t {
                b2RetryFlag_b2AckFlag_b4Version_b2Crc_b2CrcLen_b4SeqNum: [0; 2],
            },
            remChunks: 0,
            nsbc: 0,
            chksum: 0,
        },
        payload: [0; 240],
    };
    init
};
/* *****************************************************************************
 * FUNCTION DEFINITIONS
 ******************************************************************************
 */
/* * @fn void rlDriverShiftDWord(rlUInt8_t buf[])
*
*   @brief Shifts one byte in the byte array
*   @param[in] buf - Byte array
*
*   Shifts one byte in the byte array
*/
/* SourceId :  */
/* DesignId :  */
/* Requirements :  */
#[no_mangle]
pub unsafe extern "C" fn rlDriverShiftDWord(mut buf: *mut rlUInt8_t) {
    let mut shiftIdx: rlUInt8_t = 0;
    /* shift each byte in recevied byte array */
    shiftIdx = 0 as core::ffi::c_uint as rlUInt8_t;
    while (shiftIdx as core::ffi::c_uint) < 7 as core::ffi::c_uint {
        /* overwritting each data byte with next data byte of array */
        *buf.offset(shiftIdx as isize) = *buf
            .offset((shiftIdx as core::ffi::c_uint).wrapping_add(1 as core::ffi::c_uint) as isize);
        shiftIdx = shiftIdx.wrapping_add(1)
    }
    /* set last byte to zero */
    *buf.offset(7 as core::ffi::c_uint as isize) = 0 as core::ffi::c_uint as rlUInt8_t;
}
/* * @fn rlReturnVal_t rlDriverCalCRC(rlUInt8_t* data, rlUInt16_t dataLen,
*                      rlUInt8_t crcType, rlUInt8_t crc[RL_CRC_LEN_MAX])
*
*   @brief Calculates 16bit/32bit/64bit CRC
*   @param[in] data - Byte array
*   @param[in] dataLen - Length of the byte array
*   @param[in] crcType - Type of CRC 16/32/64bit
*   @param[in] crc - Received CRC
*
*   @return rlReturnVal_t Success - 0, Failure - Error Code
*
*   Calculates 16bit/32bit/64bit CRC
*/
/* DesignId :  */
/* Requirements : AUTORADAR_REQ-774 */
#[no_mangle]
pub unsafe extern "C" fn rlDriverCalCRC(
    mut data: *mut rlUInt8_t,
    mut dataLen: rlUInt16_t,
    mut crcType: rlUInt8_t,
    mut crc: *mut rlUInt8_t,
) -> rlReturnVal_t {
    let mut retVal: rlReturnVal_t = 0;
    /* check if API is defined by the application during powerOn */
    if rl_driverData.clientCtx.crcCb.rlComputeCRC.is_some() {
        /* compute CRC on given data */
        retVal = rl_driverData
            .clientCtx
            .crcCb
            .rlComputeCRC
            .expect("non-null function pointer")(
            data,
            dataLen as rlUInt32_t,
            crcType,
            &mut *crc.offset(0 as core::ffi::c_uint as isize),
        )
    } else {
        /* set error code if CRC compute API is not set by Application */
        retVal = -(15 as core::ffi::c_int)
    }
    return retVal;
}
/* * @fn rlReturnVal_t rlDriverVerifyCRC(rlUInt8_t* data, rlUInt16_t dataLen,
                         rlUInt8_t crcType, rlUInt8_t crc[RL_CRC_LEN_MAX])
*
*   @brief Compares received CRC with Calculated CRC
*   @param[in] data - Byte array
*   @param[in] dataLen - Length of the byte array
*   @param[in] crcType - Type of CRC 16/32/64bit
*   @param[in] crc - Received CRC
*
*   @return rlReturnVal_t CRC Check Pass - 0,
*                         CRC Check Fail - RL_RET_CODE_CRC_FAILED
*
*   Compares received CRC with Calculated CRC
*/
/* DesignId :  */
/* Requirements : AUTORADAR_REQ-780 */
#[no_mangle]
pub unsafe extern "C" fn rlDriverVerifyCRC(
    mut data: *mut rlUInt8_t,
    mut dataLen: rlUInt16_t,
    mut crcType: rlUInt8_t,
    mut crc: *mut rlUInt8_t,
) -> rlReturnVal_t {
    let mut indx: rlUInt8_t = 0;
    let mut crcByte: [rlUInt8_t; 8] = [0; 8];
    let mut retVal: rlReturnVal_t = 0 as core::ffi::c_int;
    /* compute CRC on given data */
    if 0 as core::ffi::c_int
        != rl_driverData
            .clientCtx
            .crcCb
            .rlComputeCRC
            .expect("non-null function pointer")(
            data,
            dataLen as rlUInt32_t,
            crcType,
            &mut *crcByte.as_mut_ptr().offset(0 as core::ffi::c_uint as isize),
        )
    {
        /* set error code if CRC callback returns non-zero */
        retVal += -(4 as core::ffi::c_int)
    } else {
        /* compare computed and received CRC value */
        indx = 0 as core::ffi::c_uint as rlUInt8_t;
        while (indx as core::ffi::c_uint) < (2 as core::ffi::c_uint) << crcType as core::ffi::c_int
        {
            if crcByte[indx as usize] as core::ffi::c_int
                != *crc.offset(indx as isize) as core::ffi::c_int
            {
                /* set error code if computed and received CRC value mismatched */
                retVal += -(6 as core::ffi::c_int);
                break;
            } else {
                indx = indx.wrapping_add(1)
            }
        }
    }
    return retVal;
}
/* * @fn rlReturnVal_t rlDriverCalChkSum(rlUInt8_t* data, rlUInt8_t len,
*                             rlUInt16_t* checksum)
*
*   @brief Calculates the 16 bit Checksum on a byte array
*   @param[in] hdrData - Header data
*   @param[in] len -  Length of the byte array
*   @param[out] checksum - Calcualted checksum of the byte array
*
*   @return rlReturnVal_t Success - 0, Failure - Error Code
*
*   Compares Calculates the 16 bit checksum on a byte array
*/
/* DesignId :  */
/* Requirements : AUTORADAR_REQ-774, AUTORADAR_REQ-777 */
#[no_mangle]
pub unsafe extern "C" fn rlDriverCalChkSum(
    mut hdrData: *mut rlProtHeader_t,
    mut len: rlUInt8_t,
    mut checksum: *mut rlUInt16_t,
) -> rlReturnVal_t {
    /* TBD - MISRA compliant Checksum code */
    let mut checkSumVal: rlUInt32_t = 0 as core::ffi::c_uint;
    let mut localGenHdr: *mut rlUInt8_t = hdrData as *mut rlUInt8_t;
    let mut retVal: rlReturnVal_t = 0;
    /* if received data pointer is not NULL */
    if !localGenHdr.is_null() {
        /* if length is 2 or more bytes     */
        while len as core::ffi::c_uint > 1 as core::ffi::c_uint {
            /* AR_CODE_REVIEW MR:R.11.2 <APPROVED> "Pointer conversion
             * from data array
             * to calculate Checksum" */
            /*LDRA_INSPECTED 94 S */
            /*LDRA_INSPECTED 95 S */
            checkSumVal = (checkSumVal as core::ffi::c_uint)
                .wrapping_add(*(localGenHdr as *mut rlUInt16_t) as core::ffi::c_uint)
                as rlUInt32_t as rlUInt32_t;
            localGenHdr = localGenHdr.offset(2 as core::ffi::c_uint as isize);
            /* If high order bit set, fold */
            if checkSumVal & 0x80000000 as core::ffi::c_uint != 0 as core::ffi::c_uint {
                checkSumVal = (checkSumVal & 0xffff as core::ffi::c_uint)
                    .wrapping_add(checkSumVal >> 16 as core::ffi::c_uint)
            }
            /* decrement length by 2 as checkSum is calculated on each 2 bytes */
            len = (len as core::ffi::c_uint).wrapping_sub(2 as core::ffi::c_uint) as rlUInt8_t
                as rlUInt8_t
        }
        /* Take care of left over byte */
        if len as core::ffi::c_uint > 0 as core::ffi::c_uint {
            checkSumVal = (checkSumVal as core::ffi::c_uint)
                .wrapping_add(*localGenHdr as rlUInt32_t) as rlUInt32_t
                as rlUInt32_t
        }
        /* Add all half words to calcuated SUM */
        while checkSumVal >> 16 as core::ffi::c_uint != 0 as core::ffi::c_uint {
            checkSumVal = (checkSumVal & 0xffff as core::ffi::c_uint)
                .wrapping_add(checkSumVal >> 16 as core::ffi::c_uint)
        }
        /* Calculate Checksum as compliment of the SUM */
        *checksum = !checkSumVal as rlUInt16_t;
        retVal = 0 as core::ffi::c_int
    } else {
        retVal = -(9 as core::ffi::c_int)
    }
    return retVal;
}
/* * @fn rlReturnVal_t rlDriverValidateHdr(rlProtHeader_t protHdr)
*
*   @brief Validates the header by comparing Checksum
*   @param[in] protHdr - Protocol Header
*
*   @return rlReturnVal_t Success - 0, Failure - Error Code
*
*   Validates the header by comparing checksum
*/
/* DesignId :  */
/* Requirements : AUTORADAR_REQ-777 */
#[no_mangle]
pub unsafe extern "C" fn rlDriverValidateHdr(mut protHdr: rlProtHeader_t) -> rlReturnVal_t {
    let mut retVal: rlReturnVal_t = 0;
    let mut checkSum: rlUInt16_t = 0 as core::ffi::c_uint as rlUInt16_t;
    /* Calculate checksum*/
    /* AR_CODE_REVIEW MR:R.2.2 <APPROVED> "Checksum is getting updated in called function" */
    /*LDRA_INSPECTED 8 D */
    if 0 as core::ffi::c_int
        != rlDriverCalChkSum(
            &mut protHdr,
            (12 as core::ffi::c_uint).wrapping_sub(2 as core::ffi::c_uint) as rlUInt8_t,
            &mut checkSum,
        )
    {
        /* Set error code if checksum callback returns non-zero value */
        retVal = -(4 as core::ffi::c_int)
    } else if protHdr.chksum as core::ffi::c_int != checkSum as core::ffi::c_int {
        /* Compare calcualted and received checksum*/
        /* Checksum doesn't match, return error*/
        retVal = -(7 as core::ffi::c_int)
    } else {
        /* checksum successfully matched */
        retVal = 0 as core::ffi::c_int
    }
    return retVal;
}
/* * @fn rlUInt8_t rlDeviceIdentifyCmdDir(rlUInt16_t msgId, rlUInt8_t platform)
*
*   @brief Get the direction of command packet based on MsgID and platform
*   @param[in] msgId - Message ID of data packet
*   @param[in] platform - Platform Type where mmWaveLink instance is running
*
*   @return rlUInt8_t command packet direction
*
*   Returns direction of command packet based on MsgID and platform.
*/
/* DesignId :  */
/* Requirements :  */
#[no_mangle]
pub unsafe extern "C" fn rlDeviceIdentifyCmdDir(
    mut msgId: rlUInt16_t,
    mut platform: rlUInt8_t,
) -> rlUInt8_t {
    let mut cmdDir: rlUInt8_t = 0;
    /* if MsgId is for radarSS */
    if (0 as core::ffi::c_uint) < msgId as core::ffi::c_uint
        && 0x80 as core::ffi::c_uint > msgId as core::ffi::c_uint
    {
        /* if mmWaveLink is running on MSS */
        if 0x1 as core::ffi::c_uint == platform as core::ffi::c_uint {
            cmdDir = 0x8 as core::ffi::c_uint as rlUInt8_t
        } else if 0 as core::ffi::c_uint == platform as core::ffi::c_uint {
            cmdDir = 0x1 as core::ffi::c_uint as rlUInt8_t
        } else {
            /* if mmWaveLink is running on Host */
            /* if mmWaveLink is running on DSS */
            cmdDir = 0xa as core::ffi::c_uint as rlUInt8_t
        }
    } else if 0x200 as core::ffi::c_uint <= msgId as core::ffi::c_uint
        && 0x280 as core::ffi::c_uint > msgId as core::ffi::c_uint
    {
        /* if MsgId is for MSS */
        /* if mmWaveLink is running on Host */
        if 0 as core::ffi::c_uint == platform as core::ffi::c_uint {
            /* MsgId for MSS need not send to MSS itself */
            cmdDir = 0x5 as core::ffi::c_uint as rlUInt8_t
        } else {
            /* If MSS wants to configure these MsgID to DSS */
            cmdDir = 0xb as core::ffi::c_uint as rlUInt8_t
        }
    } else if 0x100 as core::ffi::c_uint <= msgId as core::ffi::c_uint
        && 0x180 as core::ffi::c_uint > msgId as core::ffi::c_uint
    {
        /* if msgId is for DSS */
        /* MsgId for DSS need not send to DSS itself */
        /* if mmWaveLink is running on MSS */
        if 0x1 as core::ffi::c_uint == platform as core::ffi::c_uint {
            /* set direction MSS_TO_DSS */
            cmdDir = 0xb as core::ffi::c_uint as rlUInt8_t
        } else {
            /* if mmWaveLink is running on Host */
            /* set direction HOST_TO_DSS */
            cmdDir = 0x3 as core::ffi::c_uint as rlUInt8_t
        }
    } else {
        /* set direction INVALID */
        cmdDir = 0 as core::ffi::c_uint as rlUInt8_t
    }
    return cmdDir;
}
/* * @fn rlReturnVal_t rlDriverAsyncEventHandler(rlUInt8_t devIndex,
*               rlUInt16_t nsbc, rlUInt8_t *payload, rlUInt16_t payloadLen)
*
*   @brief Handles asynchronous response/error from mmwave radar device
*   @param[in] devIndex - Device Index
*   @param[in] nsbc - No. of Sub-Block in MSG
*   @param[in] payload - Protocol payload message data
*   @param[in] payloadLen - Length of payload message data
*
*   @return rlReturnVal_t Success - 0, Failure - Error Code
*
*   Handles asynchronous response/error from mmwave radar device
*/
/* DesignId :  */
/* Requirements : AUTORADAR_REQ-783 */
#[no_mangle]
pub unsafe extern "C" fn rlDriverAsyncEventHandler(
    mut devIndex: rlUInt8_t,
    mut nsbc: rlUInt16_t,
    mut payload: *mut rlUInt8_t,
    mut payloadLen: rlUInt16_t,
) -> rlReturnVal_t {
    let mut indx: rlUInt16_t = 0;
    let mut sbLen: rlUInt16_t = 0 as core::ffi::c_uint as rlUInt16_t;
    let mut sbcId: rlUInt16_t = 0 as core::ffi::c_uint as rlUInt16_t;
    let mut recSbsLen: rlUInt16_t = 0 as core::ffi::c_uint as rlUInt16_t;
    let mut retVal: rlReturnVal_t = 0 as core::ffi::c_int;
    let mut payldAddr: *mut rlUInt8_t = payload;
    /* Lood for all the Events Sub Block and call event handler */
    indx = 0 as core::ffi::c_uint as rlUInt16_t;
    while (indx as core::ffi::c_int) < nsbc as core::ffi::c_int {
        /* check for payload pointer for NULL */
        if payldAddr.is_null() || recSbsLen as core::ffi::c_int > payloadLen as core::ffi::c_int {
            /* AR_CODE_REVIEW MR:R.2.2 <APPROVED> "Error value set on top of default value" */
            /*LDRA_INSPECTED 8 D */
            retVal = -(9 as core::ffi::c_int);
            break;
        } else {
            /* Read Sub Block Id */
            rlGetSubBlockId(payldAddr as *const rlUInt8_t, &mut sbcId);
            /* Read Sub Block Len */
            rlGetSubBlockLen(payldAddr as *const rlUInt8_t, &mut sbLen);
            /* Call Application registered callback to indicate event*/
            if rl_driverData.clientCtx.eventCb.rlAsyncEvent.is_some() {
                /* Call event handler and pass event payload */
                /* AR_CODE_REVIEW MR:R.18.1,R.18.4 <APPROVED> "pointer arithmetic required" */
                /*LDRA_INSPECTED 567 S */
                /*LDRA_INSPECTED 87 S */
                rl_driverData
                    .clientCtx
                    .eventCb
                    .rlAsyncEvent
                    .expect("non-null function pointer")(
                    devIndex,
                    sbcId,
                    (sbLen as core::ffi::c_uint)
                        .wrapping_sub((2 as core::ffi::c_uint).wrapping_add(2 as core::ffi::c_uint))
                        as rlUInt16_t,
                    payldAddr.offset(
                        (0 as core::ffi::c_uint)
                            .wrapping_add(2 as core::ffi::c_uint)
                            .wrapping_add(2 as core::ffi::c_uint) as isize,
                    ),
                );
                /* Increase received payload length*/
                recSbsLen =
                    (recSbsLen as core::ffi::c_int + sbLen as core::ffi::c_int) as rlUInt16_t;
                /* increment payload address */
                payldAddr = payldAddr.offset(sbLen as core::ffi::c_int as isize)
            }
            indx = indx.wrapping_add(1)
        }
    }
    return retVal;
}
/* * @fn void rlDriverHostIrqHandler(rlUInt8_t deviceIndex, void *pValue)
*
*   @brief Interrupt Service Routine to handle host interrupt from mmwave
*          radar device
*   @param[in] deviceIndex - source device Index
*   @param[in] pValue - Interrupt Routine Argument
*
*   @return none
*
*   Interrupt Service Routine to handle host interrupt from mmwave radar device
*/
/* DesignId :  */
/* Requirements : AUTORADAR_REQ-777 */
#[no_mangle]
pub unsafe extern "C" fn rlDriverHostIrqHandler(
    mut deviceIndex: rlUInt8_t,
    mut pValue: *mut core::ffi::c_void,
) {
    let mut tempVar: rlUInt8_t = 0;
    /* get rlDriver global structure pointer */
    let mut rlDrvData: *mut rlDriverData_t = rlDriverGetHandle();
    /* check for NULL pointer */
    pValue.is_null();
    /* Mask the Interrupt - Required if interrupt is Level triggered*/
    /* AR_CODE_REVIEW MR:D.4.1,D.4.14 <APPROVED> "rlDrvData is pointer to a global strcture,
    can't be NULL" */
    /*LDRA_INSPECTED 45 D */
    if (*rlDrvData)
        .clientCtx
        .devCtrlCb
        .rlDeviceMaskHostIrq
        .is_some()
        && 0 as *mut core::ffi::c_void != (*rlDrvData).commDevIdx.comIfHdl[deviceIndex as usize]
    {
        /* Mask Host IRQ */
        (*rlDrvData)
            .clientCtx
            .devCtrlCb
            .rlDeviceMaskHostIrq
            .expect("non-null function pointer")(
            (*rlDrvData).commDevIdx.comIfHdl[deviceIndex as usize],
        );
    }
    /* Increment the Irq count*/
    /* AR_CODE_REVIEW MR:R.2.2 <APPROVED> "storing incremented value to global structure" */
    /*LDRA_INSPECTED 105 D */
    ::core::ptr::write_volatile(
        &mut (*rlDrvData).rxIrqCnt[deviceIndex as usize] as *mut rlUInt8_t,
        ((*rlDrvData).rxIrqCnt[deviceIndex as usize] as core::ffi::c_uint)
            .wrapping_add(1 as core::ffi::c_uint) as rlUInt8_t,
    );
    /* store deviceIndex in Global */
    (*rlDrvData).commDevIdx.rlDevIndex[deviceIndex as usize] = deviceIndex;
    tempVar = ((*rlDrvData).isCmdRespWaited[deviceIndex as usize] as core::ffi::c_int
        | (*rlDrvData).isRespWriteWaited[deviceIndex as usize] as core::ffi::c_int)
        as rlUInt8_t;
    /* Check if command transaction is in progress*/
    if 1 as core::ffi::c_uint as rlUInt8_t as core::ffi::c_int == tempVar as core::ffi::c_int {
        /* Release command response wait semaphore to unblock command thread*/
        (*rlDrvData)
            .clientCtx
            .osiCb
            .sem
            .rlOsiSemSignal
            .expect("non-null function pointer")(&mut (*rlDrvData).cmdSem);
    } else {
        /* No response is expected, Add to the Spawn queue to be hanled
         * in different context*/
        (*rlDrvData)
            .clientCtx
            .osiCb
            .queue
            .rlOsiSpawn
            .expect("non-null function pointer")(
            ::core::mem::transmute::<
                Option<unsafe extern "C" fn(_: *const core::ffi::c_void) -> rlReturnVal_t>,
                RL_P_OSI_SPAWN_ENTRY,
            >(Some(
                rlDriverMsgReadSpawnCtx
                    as unsafe extern "C" fn(_: *const core::ffi::c_void) -> rlReturnVal_t,
            )),
            &mut *(*rlDrvData)
                .commDevIdx
                .rlDevIndex
                .as_mut_ptr()
                .offset(deviceIndex as isize) as *mut rlUInt8_t
                as *const core::ffi::c_void,
            0 as core::ffi::c_int as rlUInt32_t,
        );
    };
}
/* * @fn rlReturnVal_t rlDriverProcRdMsg(rlUInt8_t devIdx, rlReturnVal_t inVal)
*
*   @brief Process received message for Async event message
*   @param[in] devIdx - device Index
*   @param[in] inVal  - status value
*
*   @return error status
*
*   Process received message for Async event message
*/
#[no_mangle]
pub unsafe extern "C" fn rlDriverProcRdMsg(
    mut devIdx: rlUInt8_t,
    mut inVal: rlReturnVal_t,
) -> rlReturnVal_t {
    /* get rlDriver global structure pointer */
    let mut rlDrvData: *mut rlDriverData_t = rlDriverGetHandle();
    let mut retVal: rlReturnVal_t = 0;
    /* Increment IRQ done IR*/
    /* AR_CODE_REVIEW MR:D.4.1,D.4.14 <APPROVED> "rlDrvData is pointer to a global strcture,
    can't be NULL" */
    /* AR_CODE_REVIEW MR:R.2.2  <APPROVED> "storing incremented value to global structure" */
    /*LDRA_INSPECTED 105 D */
    /*LDRA_INSPECTED 45 D */
    ::core::ptr::write_volatile(
        &mut (*rlDrvData).rxDoneCnt[devIdx as usize] as *mut rlUInt8_t,
        ((*rlDrvData).rxDoneCnt[devIdx as usize] as core::ffi::c_uint)
            .wrapping_add(1 as core::ffi::c_uint) as rlUInt8_t,
    );
    /* Check received message class*/
    match (*rlDrvData).funcParams.rxMsgClass as core::ffi::c_int {
        3 => {
            /* If CRC check passed then only parse Async Event */
            if inVal == 0 as core::ffi::c_int {
                /* parse received Async Event Message */
                /*LDRA_INSPECTED 95 S */
                retVal = rlDriverAsyncEventHandler(
                    devIdx,
                    (*rlDrvData).funcParams.asyncEvt.evtMsg.hdr.nsbc,
                    &mut *(*rlDrvData)
                        .funcParams
                        .asyncEvt
                        .evtMsg
                        .payload
                        .as_mut_ptr()
                        .offset(0 as core::ffi::c_int as isize)
                        as *mut rlUInt8_t,
                    ((*rlDrvData).funcParams.asyncEvt.evtMsg.hdr.len as core::ffi::c_uint)
                        .wrapping_sub(12 as core::ffi::c_uint) as rlUInt16_t,
                )
            } else {
                let mut errPayload: [rlUInt16_t; 8] = [0; 8];
                /* Generate local payload containing [SBID+SBLEN+error value] */
                errPayload[0 as core::ffi::c_int as usize] = (0x380 as core::ffi::c_uint)
                    .wrapping_mul(32 as core::ffi::c_uint)
                    .wrapping_add(0 as core::ffi::c_uint)
                    as rlUInt16_t;
                errPayload[1 as core::ffi::c_int as usize] = (2 as core::ffi::c_uint)
                    .wrapping_add(2 as core::ffi::c_uint)
                    .wrapping_add(4 as core::ffi::c_uint)
                    as rlUInt16_t;
                /* Copy last return value[error] to payload of this async event msg */
                memcpy(
                    &mut *errPayload
                        .as_mut_ptr()
                        .offset(2 as core::ffi::c_int as isize)
                        as *mut rlUInt16_t as *mut core::ffi::c_void,
                    &mut inVal as *mut rlReturnVal_t as *mut rlUInt8_t as *const core::ffi::c_void,
                    ::core::mem::size_of::<rlReturnVal_t>() as _,
                );
                /* Send error Async Event message to application containing error value */
                /* AR_CODE_REVIEW MR:R.10.3 <INSPECTED> "All param types are matching to function
                argument type. LDRA tool issue." */
                /*LDRA_INSPECTED 458 S */
                retVal = rlDriverAsyncEventHandler(
                    devIdx,
                    1 as core::ffi::c_uint as rlUInt16_t,
                    &mut *errPayload
                        .as_mut_ptr()
                        .offset(0 as core::ffi::c_int as isize)
                        as *mut rlUInt16_t as *mut rlUInt8_t,
                    (2 as core::ffi::c_uint)
                        .wrapping_add(2 as core::ffi::c_uint)
                        .wrapping_add(4 as core::ffi::c_uint) as rlUInt16_t,
                )
            }
        }
        2 => {
            /* These types are legal in this context. Do nothing */
            retVal = 0 as core::ffi::c_int
        }
        1 => {
            /* Command response is illegal in this context. */
            retVal = 0 as core::ffi::c_int
        }
        0 | 4 => {
            /* if mmWaveLink is running on MSS/DSS and receives command from other Core/HOST */
            /* Check if command parser API callback is assigned by the application */
            if rl_driverData.clientCtx.cmdParserCb.rlCmdParser.is_some() {
                /* invoke callback to parse the received command packet */
                rl_driverData
                    .clientCtx
                    .cmdParserCb
                    .rlCmdParser
                    .expect("non-null function pointer")(
                    (*rlDrvData).funcParams.rxMsgClass, inVal
                );
            }
            retVal = 0 as core::ffi::c_int
        }
        5 => {
            /* do nothing */
            retVal = 0 as core::ffi::c_int
        }
        _ => {
            /* set error code */
            retVal = -(1 as core::ffi::c_int)
        }
    }
    return retVal;
}
/* * @fn rlReturnVal_t rlDriverMsgReadSpawnCtx(void *pValue)
*
*   @brief Handles Interrupt in application(user) context
*   @param[in] pValue - Interrupt Routine Argument
*
*   @return none
*
*   Handles Interrupt in application(user) context
*/
/* DesignId :  */
/* Requirements : AUTORADAR_REQ-782 */
#[no_mangle]
pub unsafe extern "C" fn rlDriverMsgReadSpawnCtx(
    mut pValue: *const core::ffi::c_void,
) -> rlReturnVal_t {
    let mut retVal: rlReturnVal_t = 0;
    let mut msgRdRetVal: rlReturnVal_t = 0;
    let mut deviceIndex: rlUInt8_t = 0;
    let mut lclRxIrqCnt: rlUInt8_t = 0;
    let mut lclRxDoneCnt: rlUInt8_t = 0;
    /* get rlDriver global structure pointer */
    let mut rlDrvData: *mut rlDriverData_t = rlDriverGetHandle();
    /* check for NULL pointer */
    if pValue == 0 as *mut core::ffi::c_void {
        /* No Argument Passed */
        deviceIndex = 0 as core::ffi::c_uint as rlUInt8_t
    } else {
        /* argument passed is Device Index */
        deviceIndex = *(pValue as *const rlUInt8_t)
    }
    /* This function pointer is always being checked on powerOn and mmwavelink
    fails poweron with error return value, in this failure case no API call is allowed */
    /* AR_CODE_REVIEW MR:D.4.1,D.4.14 <APPROVED> "rlDrvData is pointer to a global strcture,
    can't be NULL" */
    /*LDRA_INSPECTED 45 D */
    if 0 as core::ffi::c_int
        != (*rlDrvData)
            .clientCtx
            .osiCb
            .mutex
            .rlOsiMutexLock
            .expect("non-null function pointer")(
            &mut (*rlDrvData).globalMutex,
            0xffff as core::ffi::c_uint,
        )
    {
        let mut errPayload: [rlUInt16_t; 8] = [0; 8];
        let mut inVal: rlReturnVal_t = 0;
        let mut ptrData: *mut rlReturnVal_t = 0 as *mut rlReturnVal_t;
        /* Generate local payload containing [SBID+SBLEN+error value] */
        errPayload[0 as core::ffi::c_int as usize] = (0x380 as core::ffi::c_uint)
            .wrapping_mul(32 as core::ffi::c_uint)
            .wrapping_add(0x1 as core::ffi::c_uint)
            as rlUInt16_t;
        errPayload[1 as core::ffi::c_int as usize] = (2 as core::ffi::c_uint)
            .wrapping_add(2 as core::ffi::c_uint)
            .wrapping_add(4 as core::ffi::c_uint)
            as rlUInt16_t;
        /* If MutexLock returns non-zero then treat this as error and set error code to inVal */
        inVal = -(10 as core::ffi::c_int);
        ptrData = &mut inVal;
        if !ptrData.is_null() {
            /* Copy last return value[error] to payload of this async event msg */
            /* AR_CODE_REVIEW MR:R.21.17 <INSPECTED> "Local variable and array can't be null.
            LDRA tool issue." */
            /*LDRA_INSPECTED 140 D */
            memcpy(
                &mut *errPayload
                    .as_mut_ptr()
                    .offset(2 as core::ffi::c_int as isize) as *mut rlUInt16_t
                    as *mut core::ffi::c_void,
                ptrData as *mut core::ffi::c_void,
                ::core::mem::size_of::<rlReturnVal_t>() as _,
            );
        }
        /* Send error Async Event message to application containing error value */
        /* AR_CODE_REVIEW MR:R.10.3 <INSPECTED> "All param types are matching to function
        argument type. LDRA tool issue." */
        /*LDRA_INSPECTED 458 S */
        retVal = rlDriverAsyncEventHandler(
            deviceIndex,
            1 as core::ffi::c_uint as rlUInt16_t,
            &mut *errPayload
                .as_mut_ptr()
                .offset(0 as core::ffi::c_int as isize) as *mut rlUInt16_t
                as *mut rlUInt8_t,
            (2 as core::ffi::c_uint)
                .wrapping_add(2 as core::ffi::c_uint)
                .wrapping_add(4 as core::ffi::c_uint) as rlUInt16_t,
        )
    } else {
        /* Messages might have been read by CmdResp context. Therefore after getting LockObj,
        check again where the Pending Rx Msg is still present. */
        lclRxIrqCnt = (*rlDrvData).rxIrqCnt[deviceIndex as usize];
        lclRxDoneCnt = (*rlDrvData).rxDoneCnt[deviceIndex as usize];
        /* unlock Mutex if all Received IRQ are handled i.e. DONE */
        if lclRxIrqCnt as core::ffi::c_int == lclRxDoneCnt as core::ffi::c_int {
            (*rlDrvData)
                .clientCtx
                .osiCb
                .mutex
                .rlOsiMutexUnLock
                .expect("non-null function pointer")(&mut (*rlDrvData).globalMutex);
            retVal = 0 as core::ffi::c_int
        } else {
            /* Receive data over communication channel*/
            /* AR_CODE_REVIEW MR:D.4.7,R.17.7 <APPROVED> "variable is used in next function
            call" */
            /*LDRA_INSPECTED 91 D */
            msgRdRetVal = rlDriverMsgRead(rlDrvData, deviceIndex);
            /* process received message */
            retVal = rlDriverProcRdMsg(deviceIndex, msgRdRetVal);
            /* Unlock Global Mutex */
            (*rlDrvData)
                .clientCtx
                .osiCb
                .mutex
                .rlOsiMutexUnLock
                .expect("non-null function pointer")(&mut (*rlDrvData).globalMutex);
        }
    }
    return retVal;
}
/* * @fn rlReturnVal_t rlDriverMsgCmdReply(rlDriverData_t* rlDrvData,
*                                           rlUInt8_t devIndex)
*
*   @brief Wait and handle command response
*   @param[in] rlDrvData - Pointer to mmWaveLink global structure
*   @param[in] devIndex - Device index of slave device from where response
*                         is expected
*
*   @return rlReturnVal_t Success - 0, Failure - Error Code
*
*   Wait and handle command response
*/
#[no_mangle]
pub unsafe extern "C" fn rlDriverMsgCmdReply(
    mut rlDrvData: *mut rlDriverData_t,
    mut devIndex: rlUInt8_t,
) -> rlReturnVal_t {
    let mut retVal: rlReturnVal_t = 0;
    let mut retStatus: rlReturnVal_t = 0;
    let mut tempVar: rlUInt8_t = 0;
    /* Receive data over communication channel */
    retStatus = rlDriverMsgRead(rlDrvData, devIndex);
    /* Data received successfully */
    ::core::ptr::write_volatile(
        &mut (*rlDrvData).rxDoneCnt[devIndex as usize] as *mut rlUInt8_t,
        ::core::ptr::read_volatile::<rlUInt8_t>(
            &(*rlDrvData).rxDoneCnt[devIndex as usize] as *const rlUInt8_t,
        )
        .wrapping_add(1),
    );
    if 0 as core::ffi::c_int != retStatus {
        /* If Async event message has some issue then notify to the
         * application with the error value */
        if 0x3 as core::ffi::c_uint == (*rlDrvData).funcParams.rxMsgClass as core::ffi::c_uint {
            let mut errPayload: [rlUInt16_t; 8] = [0; 8];
            /* Generate local payload with [SBID+SBLEN+Error_value] */
            errPayload[0 as core::ffi::c_int as usize] = (0x380 as core::ffi::c_uint)
                .wrapping_mul(32 as core::ffi::c_uint)
                .wrapping_add(0 as core::ffi::c_uint)
                as rlUInt16_t;
            errPayload[1 as core::ffi::c_int as usize] = (2 as core::ffi::c_uint)
                .wrapping_add(2 as core::ffi::c_uint)
                .wrapping_add(4 as core::ffi::c_uint)
                as rlUInt16_t;
            memcpy(
                &mut *errPayload
                    .as_mut_ptr()
                    .offset(2 as core::ffi::c_int as isize) as *mut rlUInt16_t
                    as *mut core::ffi::c_void,
                &mut retStatus as *mut rlReturnVal_t as *mut rlUInt8_t as *const core::ffi::c_void,
                ::core::mem::size_of::<rlReturnVal_t>() as _,
            );
            /* Send error Async Event message to application containing error value */
            /* AR_CODE_REVIEW MR:R.10.3 <INSPECTED> "All param types are matching to function
            argument type. LDRA tool issue." */
            /*LDRA_INSPECTED 458 S */
            rlDriverAsyncEventHandler(
                devIndex,
                1 as core::ffi::c_uint as rlUInt16_t,
                &mut *errPayload
                    .as_mut_ptr()
                    .offset(0 as core::ffi::c_int as isize) as *mut rlUInt16_t
                    as *mut rlUInt8_t,
                (2 as core::ffi::c_uint)
                    .wrapping_add(2 as core::ffi::c_uint)
                    .wrapping_add(4 as core::ffi::c_uint) as rlUInt16_t,
            );
            if retStatus == -(6 as core::ffi::c_int) {
                /* In case CRC failed for the actual async event
                then in the CMD context wait for response msg */
                retStatus = 0 as core::ffi::c_int
            }
        }
        /* Error in received data, return error */
        retVal = retStatus
    } else if 0x1 as core::ffi::c_uint == (*rlDrvData).funcParams.rxMsgClass as core::ffi::c_uint {
        /* Check received message class */
        /* Command response received, clear the Wait flag to exit loop */
        tempVar = 0 as core::ffi::c_uint as rlUInt8_t;
        ::core::ptr::write_volatile(
            &mut (*rlDrvData).isCmdRespWaited[devIndex as usize] as *mut rlUInt8_t,
            tempVar,
        );
        /* check if received msg-ID doesn't match with CMD msg-ID */
        if rl_rxMsg.hdr.opcode.b10MsgId() as core::ffi::c_int
            != rl_txMsg.hdr.opcode.b10MsgId() as core::ffi::c_int
        {
            /* set error code if MsgId of response doesn't match with CMD msg ID */
            retVal = -(13 as core::ffi::c_int)
        } else if rl_rxMsg.hdr.flags.b4SeqNum() as core::ffi::c_int
            != rl_txMsg.hdr.flags.b4SeqNum() as core::ffi::c_int
        {
            retVal = -(18 as core::ffi::c_int)
        } else {
            /* response sequence number matches with CMD sequence number */
            retVal = 0 as core::ffi::c_int
        }
        /* In case CmdResp has been read without  waiting on cmdSem that */
        /* Sem object. That to prevent old signal to be processed. */
        /* Clear the Semaphore */
        (*rlDrvData)
            .clientCtx
            .osiCb
            .sem
            .rlOsiSemWait
            .expect("non-null function pointer")(
            &mut (*rlDrvData).cmdSem, 0 as core::ffi::c_uint
        );
    } else if 0x3 as core::ffi::c_uint == (*rlDrvData).funcParams.rxMsgClass as core::ffi::c_uint {
        /* Async event received when command response is awaited,
        Handle the event and keep waiting for the response*/
        /* AR_CODE_REVIEW MR:R.10.3  <APPROVED> "All parameter types are matching to function
        argument type. LDRA tool issue." */
        /*LDRA_INSPECTED 458 S */
        /*LDRA_INSPECTED 95 S */
        rlDriverAsyncEventHandler(
            devIndex,
            (*rlDrvData).funcParams.asyncEvt.evtMsg.hdr.nsbc,
            &mut *(*rlDrvData)
                .funcParams
                .asyncEvt
                .evtMsg
                .payload
                .as_mut_ptr()
                .offset(0 as core::ffi::c_int as isize) as *mut rlUInt8_t,
            ((*rlDrvData).funcParams.asyncEvt.evtMsg.hdr.len as core::ffi::c_uint)
                .wrapping_sub(12 as core::ffi::c_uint) as rlUInt16_t,
        );
        retVal = 0 as core::ffi::c_int
        /* If CRC check fails, the Async Event is Ignored */
    } else if 0x2 as core::ffi::c_uint == (*rlDrvData).funcParams.rxMsgClass as core::ffi::c_uint {
        /* If NACK received for the CMD sent, set CMD Response Wait TAG to FALSE */
        tempVar = 0 as core::ffi::c_uint as rlUInt8_t;
        ::core::ptr::write_volatile(
            &mut (*rlDrvData).isCmdRespWaited[devIndex as usize] as *mut rlUInt8_t,
            tempVar,
        );
        /* set error code to CRC Failed */
        retVal = -(16 as core::ffi::c_int)
    } else {
        /* Invalid Class*/
        retVal = -(11 as core::ffi::c_int)
    }
    return retVal;
}
/* * @fn rlReturnVal_t rlDriverMsgReadCmdCtx(rlUInt8_t devIndex)
*
*   @brief Wait and handle command response
*   @param[in] devIndex - Device index of slave device from where response
*                         is expected
*
*   @return rlReturnVal_t Success - 0, Failure - Error Code
*
*   Wait and handle command response
*/
/* DesignId : MMWL_DesignId_102 */
/* Requirements : AUTORADAR_REQ-782 */
#[no_mangle]
pub unsafe extern "C" fn rlDriverMsgReadCmdCtx(mut devIndex: rlUInt8_t) -> rlReturnVal_t {
    let mut retVal: rlReturnVal_t = 0;
    let mut rlDrvData: *mut rlDriverData_t = rlDriverGetHandle();
    /* check for NULL pointer */
    if rlDrvData.is_null() {
        retVal = -(9 as core::ffi::c_int)
    } else {
        /* AR_CODE_REVIEW MR:R.2.2 <APPROVED> "Return value is being set under different
        conditions, where it's set with default value first" */
        /*LDRA_INSPECTED 8 D */
        retVal = 0 as core::ffi::c_int;
        /*    after command response is received and isCmdRespWaited flag is set FALSE, it is
        necessary to read out all Async messages in Commands context, because slave device
        could have dispatched some Async messages before receiving the command */
        /* AR_CODE_REVIEW MR:D.2.1 <APPROVED> "This is function must loop till
         * command response is received successfully or retry timer expires ,
         * if any Hw hang then WDT reset recovers from this error" */
        /*LDRA_INSPECTED 28 D */
        while 1 as core::ffi::c_uint as rlUInt8_t as core::ffi::c_int
            == (*rlDrvData).isCmdRespWaited[devIndex as usize] as core::ffi::c_int
            && 0 as core::ffi::c_int == retVal
        {
            if (*rlDrvData).rxIrqCnt[devIndex as usize] as core::ffi::c_int
                != (*rlDrvData).rxDoneCnt[devIndex as usize] as core::ffi::c_int
            {
                /* init to illegal value and verify it's overwritten with the
                 * valid one */
                (*rlDrvData).funcParams.rxMsgClass = 0x5 as core::ffi::c_uint as rlUInt8_t;
                /* Receive data over communication channel */
                retVal += rlDriverMsgCmdReply(rlDrvData, devIndex)
            } else {
                /* AR_CODE_REVIEW MR:D.2.1 <APPROVED> "This is function must loop till
                 * command response is received successfully or retry timer expires,
                 * if any Hw hang then WDT reset recovers from this error" */
                /*LDRA_INSPECTED 28 D */
                /* cmdSem will be signaled by IRQ */
                if !(0 as core::ffi::c_int
                    != (*rlDrvData)
                        .clientCtx
                        .osiCb
                        .sem
                        .rlOsiSemWait
                        .expect("non-null function pointer")(
                        &mut (*rlDrvData).cmdSem,
                        (*rlDrvData).clientCtx.ackTimeout,
                    ))
                {
                    continue;
                }
                /* setCmd Response Wait Tag to False when timer expires */
                ::core::ptr::write_volatile(
                    &mut (*rlDrvData).isCmdRespWaited[devIndex as usize] as *mut rlUInt8_t,
                    0 as core::ffi::c_uint as rlUInt8_t,
                );
                retVal += -(8 as core::ffi::c_int);
                break;
            }
        }
        /* if any Rx Msg is pending to process and ACK Timout error hasn't happend */
        if (*rlDrvData).rxIrqCnt[devIndex as usize] as core::ffi::c_int
            != (*rlDrvData).rxDoneCnt[devIndex as usize] as core::ffi::c_int
            && retVal != -(9 as core::ffi::c_int)
            && (*rlDrvData).isCmdRespWaited[devIndex as usize] as core::ffi::c_int
                == 0 as core::ffi::c_uint as rlUInt8_t as core::ffi::c_int
        {
            /* Spawn a thread/task to read pending Rx Msg */
            if (*rlDrvData)
                .clientCtx
                .osiCb
                .queue
                .rlOsiSpawn
                .expect("non-null function pointer")(
                ::core::mem::transmute::<
                    Option<unsafe extern "C" fn(_: *const core::ffi::c_void) -> rlReturnVal_t>,
                    RL_P_OSI_SPAWN_ENTRY,
                >(Some(
                    rlDriverMsgReadSpawnCtx
                        as unsafe extern "C" fn(_: *const core::ffi::c_void) -> rlReturnVal_t,
                )),
                &mut *(*rlDrvData)
                    .commDevIdx
                    .rlDevIndex
                    .as_mut_ptr()
                    .offset(devIndex as isize) as *mut rlUInt8_t
                    as *const core::ffi::c_void,
                0 as core::ffi::c_uint,
            ) != 0 as core::ffi::c_int
            {
                retVal = -(10 as core::ffi::c_int)
            }
        }
    }
    return retVal;
}
/* *****************************************************************************
 * FUNCTION PROTOTYPES
 ******************************************************************************
 */
/* * @fn rlReturnVal_t rlDriverOriginDirCheck(rlUInt8_t deviceRunId,
*                            rlUInt8_t dataDir)
*
*   @brief Checks for Direction of data received if that matches with
*          where mmWaveLink is running
*   @param[in] deviceRunId - ID where mmWaveLink is running
*   @param[in] dataDir - direction in rcvd data packet
*
*   @return rlReturnVal_t Success - 0, Failure - Error Code
*
*   Checks for Direction of data received if that matches with where
*   mmWaveLink is running
*/
/* DesignId :  */
/* Requirements :  */
unsafe extern "C" fn rlDriverOriginDirCheck(
    mut deviceRunId: rlUInt8_t,
    mut dataDir: rlUInt8_t,
) -> rlReturnVal_t {
    let mut retVal: rlReturnVal_t = 0;
    match deviceRunId as core::ffi::c_int {
        0 => {
            /* if mmWaveLink instance is running on Host */
            if 0x2 as core::ffi::c_uint == dataDir as core::ffi::c_uint
                || 0x6 as core::ffi::c_uint == dataDir as core::ffi::c_uint
                || 0x4 as core::ffi::c_uint == dataDir as core::ffi::c_uint
            {
                /* set OK to return value, i.e. requested DataDir is correct
                as per running instance of mmWaveLink */
                retVal = 0 as core::ffi::c_int
            } else {
                /* request DataDir is invalid as per running instance of mmWaveLink */
                retVal = -(2 as core::ffi::c_int)
            }
        }
        1 => {
            /* if mmWaveLink instance is running on MSS */
            if 0x7 as core::ffi::c_uint == dataDir as core::ffi::c_uint
                || 0xc as core::ffi::c_uint == dataDir as core::ffi::c_uint
                || 0x5 as core::ffi::c_uint == dataDir as core::ffi::c_uint
            {
                /* set OK to return value, i.e. requested DataDir is correct
                as per running instance of mmWaveLink */
                retVal = 0 as core::ffi::c_int
            } else {
                /* request DataDir is invalid as per running instance of mmWaveLink */
                retVal = -(2 as core::ffi::c_int)
            }
        }
        2 => {
            /* if mmWaveLink instance is running on DSS */
            /* if Data direction is towards DSS */
            if 0x9 as core::ffi::c_uint == dataDir as core::ffi::c_uint
                || 0xb as core::ffi::c_uint == dataDir as core::ffi::c_uint
                || 0x3 as core::ffi::c_uint == dataDir as core::ffi::c_uint
            {
                /* set OK to return value, i.e. requested DataDir is correct
                as per running instance of mmWaveLink */
                retVal = 0 as core::ffi::c_int
            } else {
                /* request DataDir is invalid as per running instance of mmWaveLink */
                retVal = -(2 as core::ffi::c_int)
            }
        }
        _ => {
            /* Invalid: set error code */
            retVal = -(2 as core::ffi::c_int)
        }
    }
    return retVal;
}
#[no_mangle]
pub unsafe extern "C" fn rlDriverRdVerifyMsg(
    mut readBuf: rlReadBuf_t,
    mut devIndex: rlUInt8_t,
) -> rlReturnVal_t {
    let mut rxLengthRecv: rlUInt16_t = 0;
    let mut retVal: rlReturnVal_t = 0;
    let mut readRetVal: rlReturnVal_t = 0;
    let mut payloadLen: rlUInt16_t = 0;
    let mut msgCrcLen: rlUInt16_t = 0;
    let mut msgCrcType: rlUInt16_t = 0;
    let mut isCrcPresent: rlUInt8_t = 0;
    /* get Received Message length excluding SYNC */
    rxLengthRecv = (*(&mut readBuf.syncHeader.protHdr as *mut rlProtHeader_t)).len;
    /* check if received msg length is under valid Msg size */
    if rxLengthRecv as core::ffi::c_uint
        >= (256 as core::ffi::c_uint).wrapping_sub(4 as core::ffi::c_uint)
    {
        retVal = -(1 as core::ffi::c_int)
    } else {
        let mut rhcpMsg: *mut rlRhcpMsg_t = 0 as *mut rlRhcpMsg_t;
        /* Get Rx Message Class from received buffer header */
        /* Process Header Here - Verify each field */
        rl_driverData.funcParams.rxMsgClass = (*(&mut readBuf.syncHeader.protHdr
            as *mut rlProtHeader_t))
            .opcode
            .b2MsgType() as rlUInt8_t;
        /* get the Payload Value removing Header length from Rx Msg Length */
        payloadLen = (rxLengthRecv as core::ffi::c_int
            - 12 as core::ffi::c_uint as rlUInt16_t as core::ffi::c_int)
            as rlUInt16_t;
        if 0x3 as core::ffi::c_uint == rl_driverData.funcParams.rxMsgClass as core::ffi::c_uint {
            /* Get RHCP message structure pointer */
            /* AR_CODE_REVIEW MR:D.4.1,D.4.14 <APPROVED> "rlDrvData is pointer to a global
            structure, can't be NULL" */
            /*LDRA_INSPECTED 45 D */
            rhcpMsg = &mut rl_driverData.funcParams.asyncEvt.evtMsg
        } else {
            /* Get RHCP message structure pointer */
            rhcpMsg = &mut rl_rxMsg
        }
        /* copy header to global structure */
        (*rhcpMsg).hdr = readBuf.syncHeader.protHdr;
        /* Copy SYNC from Communication Channel*/
        (*rhcpMsg).syncPattern = readBuf.syncHeader.syncPattern;
        /* Check whether CRC is present*/
        if (*rhcpMsg).hdr.flags.b2Crc() as core::ffi::c_uint != 0 as core::ffi::c_uint {
            /* if CRC is not present in Msg the reset crc variables */
            isCrcPresent = 0 as core::ffi::c_uint as rlUInt8_t;
            msgCrcLen = 0 as core::ffi::c_uint as rlUInt16_t;
            msgCrcType = 0 as core::ffi::c_uint as rlUInt16_t
        } else {
            isCrcPresent = 1 as core::ffi::c_uint as rlUInt8_t;
            /* It may be size 2/4/8 based on 16/32/64 bit */
            msgCrcType = (*rhcpMsg).hdr.flags.b2CrcLen();
            /* set CRC length in bytes based on CRC Type */
            msgCrcLen = ((2 as core::ffi::c_uint)
                << (msgCrcType as core::ffi::c_uint & 0x3 as core::ffi::c_uint))
                as rlUInt16_t
        }
        /* Calculate payload length from header legnth*/
        payloadLen = if isCrcPresent as core::ffi::c_int
            == 1 as core::ffi::c_uint as rlUInt8_t as core::ffi::c_int
        {
            (payloadLen as core::ffi::c_int) - msgCrcLen as core::ffi::c_int
        } else {
            payloadLen as core::ffi::c_int
        } as rlUInt16_t;
        /* This is an Response/AsyncEvent message. Read the rest of it. */
        if payloadLen as core::ffi::c_uint > 0 as core::ffi::c_uint {
            if rl_driverData
                .clientCtx
                .comIfCb
                .rlComIfRead
                .expect("non-null function pointer")(
                rl_driverData.commDevIdx.comIfHdl[devIndex as usize],
                &mut *(*rhcpMsg)
                    .payload
                    .as_mut_ptr()
                    .offset(0 as core::ffi::c_uint as isize),
                payloadLen,
            ) != payloadLen as rlInt32_t
            {
                /* If Read from Communication channel failed then set Error code */
                readRetVal = -(4 as core::ffi::c_int)
            } else {
                readRetVal = 0 as core::ffi::c_int
            }
        } else {
            readRetVal = 0 as core::ffi::c_int
        }
        /* If CRC is present - Read and verify*/
        if isCrcPresent as core::ffi::c_int
            == 1 as core::ffi::c_uint as rlUInt8_t as core::ffi::c_int
            && readRetVal == 0 as core::ffi::c_int
        {
            /* Read the CRC Bytes */
            if rl_driverData
                .clientCtx
                .comIfCb
                .rlComIfRead
                .expect("non-null function pointer")(
                rl_driverData.commDevIdx.comIfHdl[devIndex as usize],
                rl_driverData.funcParams.msgCRC.as_mut_ptr(),
                msgCrcLen,
            ) != msgCrcLen as rlInt32_t
            {
                /* Set the error code if read data fails */
                retVal = -(4 as core::ffi::c_int)
            } else {
                memcpy(
                    &mut *(*rhcpMsg).payload.as_mut_ptr().offset(payloadLen as isize)
                        as *mut rlUInt8_t as *mut core::ffi::c_void,
                    &mut *rl_driverData
                        .funcParams
                        .msgCRC
                        .as_mut_ptr()
                        .offset(0 as core::ffi::c_uint as isize)
                        as *mut rlUInt8_t as *const core::ffi::c_void,
                    msgCrcLen as _,
                );
                /* Check if CRC is enabled from the application and it's type
                matched with received MSG CRC type */
                if rl_driverData.clientCtx.crcType as core::ffi::c_int
                    != 3 as core::ffi::c_uint as rlUInt8_t as core::ffi::c_int
                    && rl_driverData.clientCtx.crcType as core::ffi::c_int
                        == msgCrcType as rlUInt8_t as core::ffi::c_int
                {
                    /* Validate CRC first as Opcode might be corrupt as well */
                    /* AR_CODE_REVIEW MR:R.10.3  <APPROVED> "All parameter types are matching to
                    function argument type. LDRA tool issue." */
                    /*LDRA_INSPECTED 458 S */
                    retVal = rlDriverVerifyCRC(
                        &mut (*rhcpMsg).hdr as *mut rlProtHeader_t as *mut rlUInt8_t,
                        (rxLengthRecv as core::ffi::c_int - msgCrcLen as core::ffi::c_int)
                            as rlUInt16_t,
                        msgCrcType as rlUInt8_t,
                        rl_driverData.funcParams.msgCRC.as_mut_ptr(),
                    )
                } else {
                    retVal = 0 as core::ffi::c_int
                }
            }
        } else {
            retVal = 0 as core::ffi::c_int
        }
    }
    return retVal;
}
/* * @fn rlReturnVal_t rlDriverMsgRead(rlDriverData_t* rlDrvData, rlUInt8_t devIndex)
*
*   @brief Receive and validate protocol header and payload
*   @param[in] rlDrvData - Pointer to mmwavelink global structure
*   @param[in] devIndex - Device index of slave device from where data is
*                         received
*
*   @return rlReturnVal_t Success - 0, Failure - Error Code
*
*   Receive and validate protocol header and payload
*/
/* DesignId : MMWL_DesignId_026 */
/* Requirements : AUTORADAR_REQ-777, AUTORADAR_REQ-779 */
#[no_mangle]
pub unsafe extern "C" fn rlDriverMsgRead(
    mut rlDrvData: *mut rlDriverData_t,
    mut devIndex: rlUInt8_t,
) -> rlReturnVal_t {
    let mut retVal: rlReturnVal_t = 0;
    let mut hdrRetVal: rlReturnVal_t = 0;
    let mut readBuf: rlReadBuf_t = rlReadBuf {
        tempBuf: core::mem::ManuallyDrop::new([
            0 as core::ffi::c_int as rlUInt8_t,
            0,
            0,
            0,
            0,
            0,
            0,
            0,
            0,
            0,
            0,
            0,
            0,
            0,
            0,
            0,
        ]),
    };
    let mut payloadLen: rlUInt16_t = 0;
    /* check for NULL pointer */
    if rlDrvData.is_null() {
        /* set error code */
        retVal = -(9 as core::ffi::c_int)
    } else {
        let mut hdrType: rlReturnVal_t = 0;
        /* Read message Header from given device index */
        hdrType = rlDriverRxHdrRead(
            readBuf.tempBuf.as_ptr() as *mut _,
            (*rlDrvData).commDevIdx.comIfHdl[devIndex as usize],
        );
        /* if it's not CNYS pattern then calculate checksum before consuming or
         * forwarding to other core */
        if 0x1 as core::ffi::c_int == hdrType {
            /* Verify Checksum, return value 0/-7 (RL_RET_CODE_CHKSUM_FAILED) */
            hdrRetVal = rlDriverValidateHdr(readBuf.syncHeader.protHdr);
            /* Received msg can be command/response/async_event
             * and it is destined to the same device where this mmWaveLink ir running
             */
            if 0 as core::ffi::c_int == hdrRetVal
                && 0 as core::ffi::c_int
                    == rlDriverOriginDirCheck(
                        (*rlDrvData).clientCtx.platform,
                        readBuf.syncHeader.protHdr.opcode.b4Direction() as rlUInt8_t,
                    )
            {
                retVal = rlDriverRdVerifyMsg(readBuf, devIndex)
            } else if 0 as core::ffi::c_int == hdrRetVal {
                (*rlDrvData).funcParams.rxMsgClass = 0x4 as core::ffi::c_uint as rlUInt8_t;
                /* if rcvd data is not meant for the device where mmWaveLink is running,
                 * Then pass data to intended destination; Assumption: byPass of
                 * Data will happening in MSS only
                 */
                rl_rxMsg.syncPattern = readBuf.syncHeader.syncPattern;
                rl_rxMsg.hdr = readBuf.syncHeader.protHdr;
                /* this length includes the CRC field also */
                payloadLen = (readBuf.syncHeader.protHdr.len as core::ffi::c_int
                    - 12 as core::ffi::c_uint as rlUInt16_t as core::ffi::c_int)
                    as rlUInt16_t;
                /* read full data before writing to destination COMM handle */
                if rl_driverData
                    .clientCtx
                    .comIfCb
                    .rlComIfRead
                    .expect("non-null function pointer")(
                    (*rlDrvData).commDevIdx.comIfHdl[devIndex as usize],
                    &mut *rl_rxMsg
                        .payload
                        .as_mut_ptr()
                        .offset(0 as core::ffi::c_int as isize),
                    payloadLen,
                ) != payloadLen as rlInt32_t
                {
                    /* Set error code if read is failed */
                    retVal = -(4 as core::ffi::c_int)
                } else {
                    /* Set as passed if data read returns zero */
                    retVal = 0 as core::ffi::c_int
                }
            } else if -(7 as core::ffi::c_int) == hdrRetVal
                && rl_driverData.clientCtx.platform as core::ffi::c_uint == 0 as core::ffi::c_uint
            {
                let mut dummyToFlushSpi: [rlUInt16_t; 8] = [
                    0xffff as core::ffi::c_int as rlUInt16_t,
                    0xffff as core::ffi::c_int as rlUInt16_t,
                    0xffff as core::ffi::c_int as rlUInt16_t,
                    0xffff as core::ffi::c_int as rlUInt16_t,
                    0xffff as core::ffi::c_int as rlUInt16_t,
                    0xffff as core::ffi::c_int as rlUInt16_t,
                    0xffff as core::ffi::c_int as rlUInt16_t,
                    0xffff as core::ffi::c_int as rlUInt16_t,
                ];
                let mut readCnt: rlUInt16_t = 0;
                /* If checksum mismatched of the received message and link is running on
                Host then it needs to flush the AWR device's MibSPI RAM so that MSS
                can re-sync its buffer pointer */
                /* Get the CRC Length what is configured */
                let mut msgCrcLen: rlUInt16_t = ((2 as core::ffi::c_uint)
                    << (rl_driverData.clientCtx.crcType as core::ffi::c_uint
                        & 0x3 as core::ffi::c_uint))
                    as rlUInt16_t;
                /* Read 240 bytes from SPI buff to allign MSS SPI_RX DMA with HOST TX buff */
                readCnt = 1 as core::ffi::c_uint as rlUInt16_t;
                while (readCnt as core::ffi::c_uint)
                    < (4 as core::ffi::c_uint).wrapping_add(12 as core::ffi::c_uint)
                {
                    rl_driverData
                        .clientCtx
                        .comIfCb
                        .rlComIfRead
                        .expect("non-null function pointer")(
                        (*rlDrvData).commDevIdx.comIfHdl[devIndex as usize],
                        &mut *dummyToFlushSpi
                            .as_mut_ptr()
                            .offset(0 as core::ffi::c_uint as isize)
                            as *mut rlUInt16_t as *mut rlUInt8_t,
                        (4 as core::ffi::c_uint).wrapping_add(12 as core::ffi::c_uint)
                            as rlUInt16_t,
                    );
                    readCnt = readCnt.wrapping_add(1)
                }
                /* Check whether CRC is present read it from MibSPI buff */
                if rl_txMsg.hdr.flags.b2Crc() as core::ffi::c_uint == 0 as core::ffi::c_uint {
                    /* Read remaining data to clear SPI buffer */
                    rl_driverData
                        .clientCtx
                        .comIfCb
                        .rlComIfRead
                        .expect("non-null function pointer")(
                        (*rlDrvData).commDevIdx.comIfHdl[devIndex as usize],
                        &mut *dummyToFlushSpi
                            .as_mut_ptr()
                            .offset(0 as core::ffi::c_uint as isize)
                            as *mut rlUInt16_t as *mut rlUInt8_t,
                        msgCrcLen,
                    );
                }
                /* In case Checksum of header is corrupted MMWL will create internal Async event
                message to Notify the application about checksum failure */
                rl_driverData.funcParams.rxMsgClass = 0x3 as core::ffi::c_uint as rlUInt8_t;
                /* Set header Validate function return value to retVal */
                retVal = hdrRetVal
            } else {
                /* Set header Validate function return value to retVal */
                retVal = hdrRetVal;
                /* In case CRC/Checksum is corrupted then store RxMsg Class with Async event */
                rl_driverData.funcParams.rxMsgClass = (*(&mut readBuf.syncHeader.protHdr
                    as *mut rlProtHeader_t))
                    .opcode
                    .b2MsgType() as rlUInt8_t
            }
        } else if 0x2 as core::ffi::c_int == hdrType {
            /* In case mmWaveLink instance is running on MSS, it may receive CNYS from Host Over SPI */
            /* Check callback is assigned by the application */
            if rl_driverData.clientCtx.cmdParserCb.rlPostCnysStep.is_some() {
                /* invoke function to complete the task required after receiving CNYS */
                retVal = rl_driverData
                    .clientCtx
                    .cmdParserCb
                    .rlPostCnysStep
                    .expect("non-null function pointer")(devIndex)
            } else {
                /* Set error code if callback is NULL */
                retVal = -(15 as core::ffi::c_int)
            }
        } else {
            retVal = hdrType;
            /* If timeout for HostIRQ down then notify this to Host via AsyncEvent */
            if hdrType == -(17 as core::ffi::c_int) {
                rl_driverData.funcParams.rxMsgClass = 0x3 as core::ffi::c_uint as rlUInt8_t
            } else {
                /* In case CRC/Checksum is corrupted then store RxMsg Class with Async event */
                rl_driverData.funcParams.rxMsgClass = (*(&mut readBuf.syncHeader.protHdr
                    as *mut rlProtHeader_t))
                    .opcode
                    .b2MsgType() as rlUInt8_t
            }
        }
        /*  Unmask Interrupt call */
        if (*rlDrvData)
            .clientCtx
            .devCtrlCb
            .rlDeviceUnMaskHostIrq
            .is_some()
            && retVal != -(9 as core::ffi::c_int)
        {
            /* Un Mask Interrupt */
            (*rlDrvData)
                .clientCtx
                .devCtrlCb
                .rlDeviceUnMaskHostIrq
                .expect("non-null function pointer")(
                (*rlDrvData).commDevIdx.comIfHdl[devIndex as usize],
            );
        }
    }
    return retVal;
}
/* * @fn rlReturnVal_t rlDriverMsgWrite(rlDriverData_t* rlDrvData, rlComIfHdl_t comIfHdl)
*
*   @brief Write command header and payload data over communication channel
*   @param[in] rlDrvData - Pointer to mmwavelink global structure
*   @param[in] comIfHdl - Communication interface handle
*
*   @return rlReturnVal_t Success - 0, Failure - Error Code
*
*   Write command header and payload data over communication channel
*/
/* DesignId : MMWL_DesignId_025 */
/* Requirements : AUTORADAR_REQ_772, AUTORADAR_REQ-774 */
#[no_mangle]
pub unsafe extern "C" fn rlDriverMsgWrite(
    mut rlDrvData: *mut rlDriverData_t,
    mut comIfHdl: rlComIfHdl_t,
) -> rlReturnVal_t {
    let mut retVal: rlReturnVal_t = 0;
    /* check for NULL pointer */
    if !comIfHdl.is_null() && !rlDrvData.is_null() {
        let mut checkSum: rlUInt16_t = 0 as core::ffi::c_uint as rlUInt16_t;
        let mut msgCrcLen: rlUInt16_t = 0;
        let mut payloadLen: rlUInt16_t = 0;
        let mut tempLen: rlUInt16_t = 0;
        /* Calculate Checksum */
        /* AR_CODE_REVIEW MR:R.2.2 <APPROVED> "Checksum is getting updated in called function" */
        /*LDRA_INSPECTED 8 D */
        rlDriverCalChkSum(
            &mut rl_txMsg.hdr,
            (12 as core::ffi::c_uint).wrapping_sub(2 as core::ffi::c_uint) as rlUInt8_t,
            &mut checkSum,
        );
        rl_txMsg.hdr.chksum = checkSum;
        /* get Payload length removing Header length from Msg Length */
        payloadLen = (rl_txMsg.hdr.len as core::ffi::c_int
            - 12 as core::ffi::c_uint as rlUInt16_t as core::ffi::c_int)
            as rlUInt16_t;
        /* check for Data Direction to choose Sync Pattern */
        if 0x4 as core::ffi::c_uint == rl_txMsg.hdr.opcode.b4Direction() as core::ffi::c_uint
            || 0x6 as core::ffi::c_uint == rl_txMsg.hdr.opcode.b4Direction() as core::ffi::c_uint
            || 0xc as core::ffi::c_uint == rl_txMsg.hdr.opcode.b4Direction() as core::ffi::c_uint
        {
            /* set device to Host Sync Pattern */
            rl_txMsg.syncPattern.sync1 = 0xdcba as core::ffi::c_uint as rlUInt16_t;
            rl_txMsg.syncPattern.sync2 = 0xabcd as core::ffi::c_uint as rlUInt16_t
        } else {
            /* set Host to device Sync Pattern */
            rl_txMsg.syncPattern.sync1 = 0x1234 as core::ffi::c_uint as rlUInt16_t;
            rl_txMsg.syncPattern.sync2 = 0x4321 as core::ffi::c_uint as rlUInt16_t
        }
        /* Check if CRC is enabled, Calculate and update payload length*/
        if rl_txMsg.hdr.flags.b2Crc() as core::ffi::c_uint == 0 as core::ffi::c_uint {
            /* It may be size 2/4/8 based on 16/32/64 bit */
            msgCrcLen = ((2 as core::ffi::c_uint)
                << ((*rlDrvData).clientCtx.crcType as core::ffi::c_uint & 0x3 as core::ffi::c_uint))
                as rlUInt16_t;
            /* compute CRC */
            rlDriverCalCRC(
                &mut rl_txMsg.hdr as *mut rlProtHeader_t as *mut rlUInt8_t,
                (rl_txMsg.hdr.len as core::ffi::c_int - msgCrcLen as core::ffi::c_int)
                    as rlUInt16_t,
                (*rlDrvData).clientCtx.crcType,
                (*rlDrvData).funcParams.msgCRC.as_mut_ptr(),
            );
            /* copy computed CRC to Tx Msg buffer */
            memcpy(
                &mut *rl_txMsg.payload.as_mut_ptr().offset(
                    (payloadLen as core::ffi::c_int - msgCrcLen as core::ffi::c_int) as isize,
                ) as *mut rlUInt8_t as *mut core::ffi::c_void,
                &mut *(*rlDrvData)
                    .funcParams
                    .msgCRC
                    .as_mut_ptr()
                    .offset(0 as core::ffi::c_int as isize) as *mut rlUInt8_t
                    as *const core::ffi::c_void,
                msgCrcLen as _,
            );
        }
        tempLen = ((4 as core::ffi::c_uint).wrapping_add(12 as core::ffi::c_uint) as rlUInt16_t
            as core::ffi::c_int
            + payloadLen as core::ffi::c_int) as rlUInt16_t;
        /* write Tx Msg to destination either over Mailbox internal to
        mmWave device or to External Host Over SPI */
        if rl_driverData
            .clientCtx
            .comIfCb
            .rlComIfWrite
            .expect("non-null function pointer")(
            comIfHdl,
            &mut rl_txMsg as *mut rlRhcpMsg_t as *mut rlUInt8_t,
            tempLen,
        ) != tempLen as rlInt32_t
        {
            /* set error code */
            retVal = -(4 as core::ffi::c_int)
        } else {
            /* set Error code as OK */
            retVal = 0 as core::ffi::c_int
        }
    } else {
        /* set error code if pointers are NULL */
        retVal = -(9 as core::ffi::c_int)
    }
    return retVal;
}
/* * @fn rlUInt8_t rlDriverReceiveSync(rlComIfHdl_t comIfHdl, rlUInt8_t syncBuf[],
*                           rlInt32_t *syncType)
*
*   @brief: Parse received msg' header
*
*   @param[in] comIfHdl - Communication interface handle
*   @param[in] syncBuf  - Sync buffer
*   @param[in] syncType - sync type
*
*   @return rlReturnVal_t Success - 0, Failure - Error Code
*
*    Parse received msg'header
*/
/* DesignId :  */
/* Requirements : AUTORADAR_REQ-777 */
unsafe extern "C" fn rlDriverReceiveSync(
    mut comIfHdl: rlComIfHdl_t,
    mut syncBuf: *mut rlUInt8_t,
    mut syncType: *mut rlInt32_t,
) -> rlUInt8_t {
    let mut count: rlUInt8_t = 0 as core::ffi::c_int as rlUInt8_t;
    let mut retVal: rlInt32_t = 0;
    let mut recSyncPattern: rlSyncPattern_t = {
        let mut init = rlSyncPattern {
            sync1: 0 as core::ffi::c_uint as rlUInt16_t,
            sync2: 0 as core::ffi::c_uint as rlUInt16_t,
        };
        init
    };
    let mut errVal: rlReturnVal_t = 0;
    /* check for NULL pointer */
    if !syncType.is_null() {
        /* Read 4 bytes SYNC Pattern) */
        if rl_driverData
            .clientCtx
            .comIfCb
            .rlComIfRead
            .expect("non-null function pointer")(
            comIfHdl,
            &mut *syncBuf.offset(0 as core::ffi::c_uint as isize),
            4 as core::ffi::c_uint as rlUInt16_t,
        ) != 4 as core::ffi::c_uint as rlInt32_t
        {
            /* set error code */
            errVal = -(4 as core::ffi::c_int)
        } else {
            /* if SYNC pattern has been read properly then copy it */
            memcpy(
                &mut recSyncPattern as *mut rlSyncPattern_t as *mut core::ffi::c_void,
                &mut *syncBuf.offset(0 as core::ffi::c_uint as isize) as *mut rlUInt8_t
                    as *const core::ffi::c_void,
                4 as core::ffi::c_uint as _,
            );
            errVal = 0 as core::ffi::c_int
        }
        retVal = 0 as core::ffi::c_int;
        /* Wait for SYNC_PATTERN from the device (when mmWaveLink is running on Ext Host*/
        while retVal == 0 as core::ffi::c_int && errVal == 0 as core::ffi::c_int {
            /* check if matched with SYNC pattern Host-to-device or device-to-Host */
            if recSyncPattern.sync1 as core::ffi::c_uint == 0x1234 as core::ffi::c_uint
                && recSyncPattern.sync2 as core::ffi::c_uint == 0x4321 as core::ffi::c_uint
                || recSyncPattern.sync1 as core::ffi::c_uint == 0xdcba as core::ffi::c_uint
                    && recSyncPattern.sync2 as core::ffi::c_uint == 0xabcd as core::ffi::c_uint
            {
                /* set to SYNC Matched flag if H2D or D2H SYNC pattern is matching
                for big/little endian data */
                retVal = 0x1 as core::ffi::c_int
            } else if recSyncPattern.sync1 as core::ffi::c_uint == 0x5678 as core::ffi::c_uint
                && recSyncPattern.sync2 as core::ffi::c_uint == 0x8765 as core::ffi::c_uint
            {
                /* if mmwavelink running on device and connect to Host over SPI then
                it may recieve CNYS to send data */
                /* set to CNYS Matched flag if H2D CNYS pattern is matching
                for big/little endian data */
                retVal = 0x2 as core::ffi::c_int
            } else if count as core::ffi::c_int
                >= 252 as core::ffi::c_uint as rlUInt16_t as core::ffi::c_int
            {
                let mut crcLen: rlUInt16_t = ((2 as core::ffi::c_uint)
                    << (rl_driverData.clientCtx.crcType as core::ffi::c_uint
                        & 0x3 as core::ffi::c_uint))
                    as rlUInt16_t;
                /* check if count is beyond SYNC Scan threshold */
                /* If pattern not found then read few extra bytes (CRC Len) to make
                whole read count [4n+CRCLen] aligned */
                if rl_driverData
                    .clientCtx
                    .comIfCb
                    .rlComIfRead
                    .expect("non-null function pointer")(
                    comIfHdl,
                    &mut *syncBuf.offset(4 as core::ffi::c_uint as isize),
                    crcLen,
                ) != crcLen as rlInt32_t
                {
                    /* Set error code to terminate this loop */
                    errVal += -(4 as core::ffi::c_int)
                } else {
                    /* Set error code to terminate this loop */
                    errVal += -(1 as core::ffi::c_int)
                }
            } else {
                /*  Read next 4 bytes to Low 4 bytes of buffer */
                if 0 as core::ffi::c_int as core::ffi::c_uint
                    == (count as core::ffi::c_uint).wrapping_rem(4 as core::ffi::c_uint)
                {
                    /* Read 4 bytes SYNC Pattern) */
                    if rl_driverData
                        .clientCtx
                        .comIfCb
                        .rlComIfRead
                        .expect("non-null function pointer")(
                        comIfHdl,
                        &mut *syncBuf.offset(4 as core::ffi::c_uint as isize),
                        4 as core::ffi::c_uint as rlUInt16_t,
                    ) != 4 as core::ffi::c_uint as rlInt32_t
                    {
                        /* Set error code to terminate this loop */
                        errVal += -(4 as core::ffi::c_int);
                        break;
                    }
                }
                /*  Shift Buffer Up for checking if the sync is shifted */
                rlDriverShiftDWord(syncBuf);
                /* copy data to recv sync pattern to compare further */
                memcpy(
                    &mut recSyncPattern as *mut rlSyncPattern_t as *mut core::ffi::c_void,
                    &mut *syncBuf.offset(0 as core::ffi::c_uint as isize) as *mut rlUInt8_t
                        as *const core::ffi::c_void,
                    4 as core::ffi::c_uint as _,
                );
                /* increment read counter */
                count = count.wrapping_add(1)
            }
        }
        if errVal == 0 as core::ffi::c_int {
            *syncType = retVal
        } else {
            *syncType = errVal
        }
    }
    count =
        (count as core::ffi::c_uint).wrapping_rem(4 as core::ffi::c_uint) as rlUInt8_t as rlUInt8_t;
    return count;
}
/* * @fn rlReturnVal_t rlDriverRxHdrRead(rlUInt8_t hdrBuf[RHCP_HEADER_LEN], rlComIfHdl_t comIfHdl)
*
*   @brief Read SYNC and Header from communication channel
*   @param[in] comIfHdl - Communication interface handle
*   @param[out] hdrBuf - Buffer to hold SyncPattern+header information
*
*   @return rlReturnVal_t pattern matched type, Failure - Error Code
*
*   Read SYNC and Header from communication channel
*/
/* DesignId :  */
/* Requirements : AUTORADAR_REQ-777 */
/*AR_CODE_REVIEW MR:R.2.2 <APPROVED> "errVal is re initialized under different
if else conditions based on what the error is" */
/*LDRA_INSPECTED 8 D */
#[no_mangle]
pub unsafe extern "C" fn rlDriverRxHdrRead(
    mut hdrBuf: *mut rlUInt8_t,
    mut comIfHdl: rlComIfHdl_t,
) -> rlReturnVal_t {
    /* syncBuf: it should be 2/4 byte aligned, as in the application where
     * it uses DMA to Rd/Wr DMA might have limitation of src/dest address
     * alignement
     */
    let mut syncBuf: [rlUInt8_t; 8] = [0 as core::ffi::c_uint as rlUInt8_t, 0, 0, 0, 0, 0, 0, 0];
    /* This buffer contains CNYS pattern (4Bytes) and 12Bytes of dummy sequence.
      Host writes this buffer in response to Host-IRQ raised by AWR device to indicate
      that device can now write response/async event data to its SPI buffer which will
      be read by Host.
    */
    let mut cnysBuf: [rlUInt16_t; 8] = [
        0x5678 as core::ffi::c_uint as rlUInt16_t,
        0x8765 as core::ffi::c_uint as rlUInt16_t,
        0xffff as core::ffi::c_uint as rlUInt16_t,
        0xffff as core::ffi::c_uint as rlUInt16_t,
        0xffff as core::ffi::c_uint as rlUInt16_t,
        0xffff as core::ffi::c_uint as rlUInt16_t,
        0xffff as core::ffi::c_uint as rlUInt16_t,
        0xffff as core::ffi::c_uint as rlUInt16_t,
    ];
    let mut syncType: rlInt32_t = 0 as core::ffi::c_int;
    let mut syncCnt: rlUInt8_t = 0;
    let mut errVal: rlInt32_t = 0;
    /* check for NULL pointer */
    if comIfHdl.is_null() {
        errVal = -(9 as core::ffi::c_int);
        syncType += errVal
    } else {
        /* If mmWaveLink is running on Ext Host */
        if rl_driverData.clientCtx.platform as core::ffi::c_uint == 0 as core::ffi::c_uint {
            /* Write CNYS pattern to mmWave Radar  */
            if rl_driverData
                .clientCtx
                .comIfCb
                .rlComIfWrite
                .expect("non-null function pointer")(
                comIfHdl,
                &mut *cnysBuf.as_mut_ptr().offset(0 as core::ffi::c_uint as isize)
                    as *mut rlUInt16_t as *mut rlUInt8_t,
                (4 as core::ffi::c_uint).wrapping_add(12 as core::ffi::c_uint) as rlUInt16_t,
            ) != (4 as core::ffi::c_uint).wrapping_add(12 as core::ffi::c_uint) as rlInt32_t
            {
                /* Set error code if data write function fails to write SYNC pattern */
                errVal = -(4 as core::ffi::c_int)
            } else {
                errVal = 0 as core::ffi::c_int
            }
            /* Need to wait till host Irq is down */
            /* Check if Host Irq Status can be polled, else use fixed delay */
            if rl_driverData
                .clientCtx
                .devCtrlCb
                .rlDeviceWaitIrqStatus
                .is_some()
                && rl_driverData
                    .clientCtx
                    .devCtrlCb
                    .rlDeviceWaitIrqStatus
                    .expect("non-null function pointer")(
                    comIfHdl,
                    0 as core::ffi::c_uint as rlUInt8_t,
                ) != 0 as core::ffi::c_int
                && errVal == 0 as core::ffi::c_int
            {
                /* If IRQ polling timed out then re-write the CNYS pattern to mmwave Radar */
                if rl_driverData
                    .clientCtx
                    .comIfCb
                    .rlComIfWrite
                    .expect("non-null function pointer")(
                    comIfHdl,
                    &mut *cnysBuf.as_mut_ptr().offset(0 as core::ffi::c_uint as isize)
                        as *mut rlUInt16_t as *mut rlUInt8_t,
                    (4 as core::ffi::c_uint).wrapping_add(12 as core::ffi::c_uint) as rlUInt16_t,
                ) != (4 as core::ffi::c_uint).wrapping_add(12 as core::ffi::c_uint) as rlInt32_t
                {
                    /* Set error code if data write function fails to write SYNC pattern */
                    errVal += -(4 as core::ffi::c_int)
                } else if rl_driverData
                    .clientCtx
                    .devCtrlCb
                    .rlDeviceWaitIrqStatus
                    .expect("non-null function pointer")(
                    comIfHdl,
                    0 as core::ffi::c_uint as rlUInt8_t,
                ) != 0 as core::ffi::c_int
                {
                    errVal += -(17 as core::ffi::c_int)
                }
            } else if rl_driverData.clientCtx.timerCb.rlDelay.is_some() {
                /* Check if Delay callback is present and invoke */
                /* add 1 mSec delay */
                rl_driverData
                    .clientCtx
                    .timerCb
                    .rlDelay
                    .expect("non-null function pointer")(1 as core::ffi::c_uint);
            }
        } else {
            errVal = 0 as core::ffi::c_int
        }
        if errVal == 0 as core::ffi::c_int {
            /* AR_CODE_REVIEW MR:D.4.7  <APPROVED> "syncCnt is used when pattern is matched to
            copy and read data" */
            /*LDRA_INSPECTED 91 D */
            syncCnt = rlDriverReceiveSync(
                comIfHdl,
                &mut *syncBuf.as_mut_ptr().offset(0 as core::ffi::c_uint as isize),
                &mut syncType,
            );
            if 0x2 as core::ffi::c_int == syncType || 0x1 as core::ffi::c_int == syncType {
                let mut tempLen: rlUInt16_t = 0;
                let mut payloadBuf: *mut rlUInt8_t = 0 as *mut rlUInt8_t;
                /* copying shifted data to hdrBuf */
                payloadBuf = &mut *syncBuf.as_mut_ptr().offset(0 as core::ffi::c_uint as isize)
                    as *mut rlUInt8_t;
                if !payloadBuf.is_null() {
                    /* AR_CODE_REVIEW MR:R.21.17 <INSPECTED> "Local array can't be null.
                    LDRA tool issue." */
                    /*LDRA_INSPECTED 140 D */
                    memcpy(
                        &mut *hdrBuf.offset(0 as core::ffi::c_int as isize) as *mut rlUInt8_t
                            as *mut core::ffi::c_void,
                        payloadBuf as *const core::ffi::c_void,
                        (4 as core::ffi::c_uint).wrapping_add(syncCnt as core::ffi::c_uint) as _,
                    );
                }
                tempLen = (12 as core::ffi::c_uint)
                    .wrapping_sub(0xff as core::ffi::c_uint & syncCnt as core::ffi::c_uint)
                    as rlUInt16_t;
                /*  Here we've read Sync Pattern. Read the remaining header */
                if rl_driverData
                    .clientCtx
                    .comIfCb
                    .rlComIfRead
                    .expect("non-null function pointer")(
                    comIfHdl,
                    &mut *hdrBuf.offset(
                        (4 as core::ffi::c_uint).wrapping_add(syncCnt as core::ffi::c_uint)
                            as isize,
                    ),
                    tempLen,
                ) != tempLen as rlInt32_t
                {
                    syncType += -(4 as core::ffi::c_int)
                }
            }
        } else {
            syncType += errVal
        }
    }
    return syncType;
}
/* * @fn rlReturnVal_t rlDriverOsiInit(void)
*
*   @brief Initializes the OSI layer abstraction for mmwavelink
*
*   @return rlReturnVal_t Success - 0, Failure - Error Code
*
*   Initializes the OS mutex, semaphore and queue interface for mmwavelink
*/
/* DesignId : MMWL_DesignId_002 */
/* Requirements : AUTORADAR_REQ-784 */
#[no_mangle]
pub unsafe extern "C" fn rlDriverOsiInit() -> rlReturnVal_t {
    let mut retVal: rlReturnVal_t = 0;
    let mut funcRetVal: rlInt32_t = 0;
    /* Create Global Mutex Object */
    funcRetVal = rl_driverData
        .clientCtx
        .osiCb
        .mutex
        .rlOsiMutexCreate
        .expect("non-null function pointer")(
        &mut rl_driverData.globalMutex,
        b"GlobalLockObj\x00" as *const u8 as *const core::ffi::c_char as *mut rlInt8_t,
    );
    /* Create Command Semaphore */
    funcRetVal += rl_driverData
        .clientCtx
        .osiCb
        .sem
        .rlOsiSemCreate
        .expect("non-null function pointer")(
        &mut rl_driverData.cmdSem,
        b"CmdSem\x00" as *const u8 as *const core::ffi::c_char as *mut rlInt8_t,
    );
    /* check for above function call return value */
    if funcRetVal != 0 as core::ffi::c_int {
        /* set error code */
        retVal = -(10 as core::ffi::c_int)
    } else {
        retVal = 0 as core::ffi::c_int
    }
    return retVal;
}
unsafe extern "C" fn rlDriverOsiCbCheck(mut clientCb: rlClientCbs_t) -> rlReturnVal_t {
    let mut retVal: rlReturnVal_t = 0;
    /* Check if application has passed mutex interace functions */
    if clientCb.osiCb.mutex.rlOsiMutexCreate.is_none()
        || clientCb.osiCb.mutex.rlOsiMutexLock.is_none()
        || clientCb.osiCb.mutex.rlOsiMutexUnLock.is_none()
        || clientCb.osiCb.mutex.rlOsiMutexDelete.is_none()
    {
        retVal = -(15 as core::ffi::c_int)
    } else if clientCb.osiCb.sem.rlOsiSemCreate.is_none()
        || clientCb.osiCb.sem.rlOsiSemWait.is_none()
        || clientCb.osiCb.sem.rlOsiSemSignal.is_none()
        || clientCb.osiCb.sem.rlOsiSemDelete.is_none()
    {
        retVal = -(15 as core::ffi::c_int)
    } else if ::core::mem::transmute::<
        Option<unsafe extern "C" fn(_: rlUInt8_t, _: rlUInt32_t) -> rlComIfHdl_t>,
        *mut core::ffi::c_void,
    >(clientCb.comIfCb.rlComIfOpen)
    .is_null()
        || clientCb.comIfCb.rlComIfClose.is_none()
        || clientCb.comIfCb.rlComIfRead.is_none()
        || clientCb.comIfCb.rlComIfWrite.is_none()
    {
        retVal = -(15 as core::ffi::c_int)
    } else {
        /* Check if application has passed semaphore interface functions */
        /* Check if application has passed communication interface functions */
        /* if no error then set return value as OK */
        retVal = 0 as core::ffi::c_int
    }
    return retVal;
}
/* * @fn rlReturnVal_t rlDriverClientCbCheck(rlClientCbs_t clientCb)
*
*   @brief Check for interace callbacks passed by application during rlDevicePowerOn.
*   @param[in] clientCb - Client callbacks
*
*   @return rlReturnVal_t Success - 0, Failure - Error Code
*
*   Check for interace functions passed by application during rlDevicePowerOn.
*/
unsafe extern "C" fn rlDriverClientCbCheck(mut clientCb: rlClientCbs_t) -> rlReturnVal_t {
    let mut retVal: rlReturnVal_t = 0;
    if 0 as core::ffi::c_int != rlDriverOsiCbCheck(clientCb) {
        retVal = -(15 as core::ffi::c_int)
    } else if clientCb.eventCb.rlAsyncEvent.is_none()
        || clientCb.devCtrlCb.rlDeviceDisable.is_none()
        || clientCb.devCtrlCb.rlDeviceEnable.is_none()
        || clientCb.devCtrlCb.rlDeviceMaskHostIrq.is_none()
    {
        retVal = -(15 as core::ffi::c_int)
    } else if clientCb.platform as core::ffi::c_uint == 0 as core::ffi::c_uint
        && (clientCb.devCtrlCb.rlDeviceWaitIrqStatus.is_none()
            && clientCb.timerCb.rlDelay.is_none())
        || clientCb.osiCb.queue.rlOsiSpawn.is_none()
    {
        retVal = -(15 as core::ffi::c_int)
    } else if clientCb.devCtrlCb.rlDeviceUnMaskHostIrq.is_none()
        || clientCb.devCtrlCb.rlRegisterInterruptHandler.is_none()
        || 3 as core::ffi::c_uint != clientCb.crcType as core::ffi::c_uint
            && clientCb.crcCb.rlComputeCRC.is_none()
    {
        retVal = -(15 as core::ffi::c_int)
    } else {
        retVal = 0 as core::ffi::c_int
    }
    return retVal;
}
/* Check if application has passed device interace functions */
/* When mmWaveLink instance is running on HOST, check for interface for IRQ and delay */
/* If CRC is enabled from application then check for CRC compute interface function */
/* *
*  @defgroup Communication_Protocol Communication Protocol
*  @brief mmwave Communication Driver Module
*
*  The mmwave radar Host Communication Driver Module implements the mmwave radar
*  communication protocol:
    -# It is a simple stop and wait protocol. Each message needs to be acknowledged
        by receiver before next message can be sent.
    -# Messages are classifieds as "Command", "Response" and "Asynchronous Event"
    -# If Command can not be processed immediately, ACK response is sent immediately
        (If requested). "Asynchronous Event"  is sent upon completion of the command
        execution.
*
*  @addtogroup Communication_Protocol
*  @{
*/
/* *****************************************************************************
 * FUNCTION DECLARATIONS
 ******************************************************************************
 */
/* * @fn rlReturnVal_t rlDriverInit(rlClientCbs_t clientCb, rlUInt8_t deviceMap)
*
*   @brief Initializes the mmwave radar Driver
*   @param[in] clientCb - Client callbacks
*   @param[in] deviceMap - Bitmap of devices to be connected
*
*   @return rlReturnVal_t Success - 0, Failure - Error Code
*
*   Initializes the mmwave radar Driver. Initialize memory, create OS objects,
*   Open communication channel, Register Interrupt Handler
*/
/* DesignId : MMWL_DesignId_003 */
/* Requirements : AUTORADAR_REQ-707 */
#[no_mangle]
pub unsafe extern "C" fn rlDriverInit(
    mut deviceMap: rlUInt8_t,
    mut clientCb: rlClientCbs,
) -> rlReturnVal_t {
    let mut retVal: rlReturnVal_t = 0;
    let mut cbCheck: rlReturnVal_t = 0;
    let mut index: rlUInt8_t = 0 as core::ffi::c_uint as rlUInt8_t;
    /* check for all interface APIs passed by the Application */
    cbCheck = rlDriverClientCbCheck(clientCb);
    if 0 as core::ffi::c_int != cbCheck {
        retVal = cbCheck
    } else if deviceMap as core::ffi::c_uint
        <= ((1 as core::ffi::c_uint) << 4 as core::ffi::c_uint)
            .wrapping_sub(0x1 as core::ffi::c_uint)
    {
        /* Initialize Driver Global Data */
        /* Initialize Driver Global Data */
        while (index as core::ffi::c_uint) < 4 as core::ffi::c_uint {
            ::core::ptr::write_volatile(
                &mut rl_driverData.isCmdRespWaited[index as usize] as *mut rlUInt8_t,
                0 as core::ffi::c_uint as rlUInt8_t,
            );
            rl_driverData.isRespWriteWaited[index as usize] = 0 as core::ffi::c_uint as rlUInt8_t;
            ::core::ptr::write_volatile(
                &mut rl_driverData.rxDoneCnt[index as usize] as *mut rlUInt8_t,
                0 as core::ffi::c_uint as rlUInt8_t,
            );
            ::core::ptr::write_volatile(
                &mut rl_driverData.rxIrqCnt[index as usize] as *mut rlUInt8_t,
                0 as core::ffi::c_uint as rlUInt8_t,
            );
            index = index.wrapping_add(1)
        }
        /*Copy Client Context Info*/
        rl_driverData.clientCtx = clientCb;
        /* Store deviceMap */
        rl_driverData.deviceMap = deviceMap;
        /* Set command retry count */
        rl_driverData.retryCount = 3 as core::ffi::c_uint as rlUInt8_t;
        /* intialize and stitch all OS interfaces */
        retVal = rlDriverOsiInit();
        rl_driverData.txMsgPtr = &mut rl_txMsg;
        rl_driverData.rxMsgPtr = &mut rl_rxMsg;
        index = 0 as core::ffi::c_uint as rlUInt8_t;
        loop
        /* If deviceIndex is set in devceMap requested by application */
        {
            if deviceMap as core::ffi::c_uint
                & (1 as core::ffi::c_uint) << index as core::ffi::c_int
                != 0 as core::ffi::c_uint
                && retVal == 0 as core::ffi::c_int
            {
                /* reset to zero sequence number for that deviceIndex */
                rl_driverData.cmdSeqNum[index as usize] = 0 as core::ffi::c_uint as rlUInt16_t;
                /* store the deviceIndex in a global structure */
                rl_driverData.commDevIdx.rlDevIndex[index as usize] = index;
                /* Open communication interface handle */
                rl_driverData.commDevIdx.comIfHdl[index as usize] = rl_driverData
                    .clientCtx
                    .comIfCb
                    .rlComIfOpen
                    .expect("non-null function pointer")(
                    index, 0 as core::ffi::c_uint
                );
                /* check for NULL pointer */
                if 0 as *mut core::ffi::c_void == rl_driverData.commDevIdx.comIfHdl[index as usize]
                {
                    retVal += -(4 as core::ffi::c_int)
                } else if rl_driverData
                    .clientCtx
                    .devCtrlCb
                    .rlRegisterInterruptHandler
                    .expect("non-null function pointer")(
                    index,
                    ::core::mem::transmute::<
                        Option<unsafe extern "C" fn(_: rlUInt8_t, _: *mut core::ffi::c_void) -> ()>,
                        RL_P_EVENT_HANDLER,
                    >(Some(
                        rlDriverHostIrqHandler
                            as unsafe extern "C" fn(_: rlUInt8_t, _: *mut core::ffi::c_void) -> (),
                    )),
                    0 as *mut core::ffi::c_void,
                ) != 0 as core::ffi::c_int
                {
                    retVal += -(4 as core::ffi::c_int)
                }
                /* Register Host Interrupt Handler */
                /* Get next Device Map based on index */
                deviceMap = (deviceMap as core::ffi::c_uint
                    & !((1 as core::ffi::c_uint) << index as core::ffi::c_int))
                    as rlUInt8_t;
                /* check for any error */
                if retVal != 0 as core::ffi::c_int {
                    break;
                }
            }
            /* increment Device Index */
            index = index.wrapping_add(1);
            if !(deviceMap as core::ffi::c_uint != 0 as core::ffi::c_uint
                && (index as core::ffi::c_uint) < 4 as core::ffi::c_uint)
            {
                break;
            }
        }
    } else {
        retVal = -(2 as core::ffi::c_int)
    }
    /* If no error during deviceInit then set the flag to 1 */
    if retVal == 0 as core::ffi::c_int {
        rl_driverData.isDriverInitialized = 1 as core::ffi::c_uint as rlUInt8_t
    }
    return retVal;
}
/* * @fn rlReturnVal_t rlDriverAddDevice(rlUInt8_t deviceMap)
*
*   @brief Adds mmwave radar device
*   @param[in] deviceMap - Bitmap of device to be connected
*
*   @return rlReturnVal_t Success - 0, Failure - Error Code
*
*   Adds mmwave radar device
*   Open communication channel, Register Interrupt Handler with device
*/
/* DesignId :  */
/* Requirements :  */
#[no_mangle]
pub unsafe extern "C" fn rlDriverAddDevice(mut deviceMap: rlUInt8_t) -> rlReturnVal_t {
    let mut retVal: rlReturnVal_t = 0;
    let mut index: rlUInt8_t = 0 as core::ffi::c_uint as rlUInt8_t;
    if rl_driverData.isDriverInitialized as core::ffi::c_int
        != 0 as core::ffi::c_uint as rlUInt8_t as core::ffi::c_int
        && deviceMap as core::ffi::c_uint
            <= ((1 as core::ffi::c_uint) << 4 as core::ffi::c_uint)
                .wrapping_sub(0x1 as core::ffi::c_uint)
    {
        /* Add to the global device map */
        rl_driverData.deviceMap = (rl_driverData.deviceMap as core::ffi::c_int
            | deviceMap as core::ffi::c_int) as rlUInt8_t;
        loop {
            /* If deviceIndex is set in devceMap requested by application */
            if deviceMap as core::ffi::c_uint
                & (1 as core::ffi::c_uint) << index as core::ffi::c_int
                != 0 as core::ffi::c_uint
            {
                /* reset to zero sequence number for that deviceIndex */
                rl_driverData.cmdSeqNum[index as usize] = 0 as core::ffi::c_uint as rlUInt16_t;
                /* store the deviceIndex in a global structure */
                rl_driverData.commDevIdx.rlDevIndex[index as usize] = index;
                /* Open communication interface handle */
                rl_driverData.commDevIdx.comIfHdl[index as usize] = rl_driverData
                    .clientCtx
                    .comIfCb
                    .rlComIfOpen
                    .expect("non-null function pointer")(
                    index, 0 as core::ffi::c_uint
                );
                if 0 as *mut core::ffi::c_void != rl_driverData.commDevIdx.comIfHdl[index as usize]
                {
                    /* register Interrupt handler */
                    if rl_driverData
                        .clientCtx
                        .devCtrlCb
                        .rlRegisterInterruptHandler
                        .expect("non-null function pointer")(
                        index,
                        ::core::mem::transmute::<
                            Option<
                                unsafe extern "C" fn(_: rlUInt8_t, _: *mut core::ffi::c_void) -> (),
                            >,
                            RL_P_EVENT_HANDLER,
                        >(Some(
                            rlDriverHostIrqHandler
                                as unsafe extern "C" fn(
                                    _: rlUInt8_t,
                                    _: *mut core::ffi::c_void,
                                ) -> (),
                        )),
                        0 as *mut core::ffi::c_void,
                    ) != 0 as core::ffi::c_int
                        || rl_driverData
                            .clientCtx
                            .devCtrlCb
                            .rlDeviceEnable
                            .expect("non-null function pointer")(index)
                            < 0 as core::ffi::c_int
                    {
                        /* set the error code */
                        retVal = -(4 as core::ffi::c_int)
                    } else {
                        retVal = 0 as core::ffi::c_int
                    }
                } else {
                    retVal = -(4 as core::ffi::c_int)
                }
                deviceMap = (deviceMap as core::ffi::c_uint
                    & !((1 as core::ffi::c_uint) << index as core::ffi::c_int))
                    as rlUInt8_t
            } else {
                retVal = 0 as core::ffi::c_int
            }
            /* break the loop if any error occured during above callbacks */
            if 0 as core::ffi::c_int != retVal {
                break;
            }
            /* increment device index */
            index = index.wrapping_add(1);
            if !(deviceMap as core::ffi::c_uint != 0 as core::ffi::c_uint
                && (index as core::ffi::c_uint) < 4 as core::ffi::c_uint)
            {
                break;
            }
        }
    } else if deviceMap as core::ffi::c_uint
        > ((1 as core::ffi::c_uint) << 4 as core::ffi::c_uint)
            .wrapping_sub(0x1 as core::ffi::c_uint)
    {
        retVal = -(2 as core::ffi::c_int)
    } else {
        retVal = -(11 as core::ffi::c_int)
    }
    return retVal;
}
/* * @fn rlReturnVal_t rlDriverRemoveDevices(rlUInt8_t deviceMap)
*
*   @brief Disconnects the mmwave radar devices
*   @param[in] deviceMap - Bitmap of mmwave devices to be disconnected
*
*   @return rlReturnVal_t Success - 0, Failure - Error Code
*
*   disconnects the mmwave radar devices
*   Close communication channel, Unregister Interrupt Handler
*/
/* DesignId :  */
/* Requirements :  */
#[no_mangle]
pub unsafe extern "C" fn rlDriverRemoveDevices(mut deviceMap: rlUInt8_t) -> rlReturnVal_t {
    let mut index: rlUInt8_t = 0;
    let mut retVal: rlReturnVal_t = 0 as core::ffi::c_int;
    /* Clear the device map from driver data */
    rl_driverData.deviceMap = (rl_driverData.deviceMap as core::ffi::c_int
        & !(deviceMap as core::ffi::c_int)) as rlUInt8_t;
    index = 0 as core::ffi::c_uint as rlUInt8_t;
    while deviceMap as core::ffi::c_uint != 0 as core::ffi::c_uint
        && (index as core::ffi::c_uint) < 4 as core::ffi::c_uint
    {
        if deviceMap as core::ffi::c_uint & (1 as core::ffi::c_uint) << index as core::ffi::c_int
            != 0 as core::ffi::c_uint
        {
            /* Close mmwave radar device communication Channel */
            if !rl_driverData.commDevIdx.comIfHdl[index as usize].is_null() {
                retVal += rl_driverData
                    .clientCtx
                    .comIfCb
                    .rlComIfClose
                    .expect("non-null function pointer")(
                    rl_driverData.commDevIdx.comIfHdl[index as usize],
                );
                rl_driverData.commDevIdx.comIfHdl[index as usize] = 0 as *mut core::ffi::c_void
            }
            /* Un Register Interrupt Handler */
            rl_driverData
                .clientCtx
                .devCtrlCb
                .rlRegisterInterruptHandler
                .expect("non-null function pointer")(
                index, None, 0 as *mut core::ffi::c_void
            );
            deviceMap = (deviceMap as core::ffi::c_uint
                & !((1 as core::ffi::c_uint) << index as core::ffi::c_int))
                as rlUInt8_t
        }
        index = index.wrapping_add(1)
    }
    return retVal;
}
/* * @fn rlReturnVal_t rlDriverDeInit(void)
*
*   @brief De Initializes the mmwave radar Driver
*
*   @return rlReturnVal_t Success - 0, Failure - Error Code
*
*   DeInitializes the mmwave radar Driver. Clear memory, destroy OS objects,
*   Close communication channel, Unregister Interrupt Handler
*/
/* DesignId :  */
/* Requirements : AUTORADAR_REQ-711 */
#[no_mangle]
pub unsafe extern "C" fn rlDriverDeInit() -> rlReturnVal_t {
    let mut index: rlUInt8_t = 0;
    let mut deviceMap: rlUInt8_t = rl_driverData.deviceMap;
    let mut retVal: rlReturnVal_t = 0 as core::ffi::c_int;
    index = 0 as core::ffi::c_uint as rlUInt8_t;
    while deviceMap as core::ffi::c_uint != 0 as core::ffi::c_uint
        && (index as core::ffi::c_uint) < 4 as core::ffi::c_uint
    {
        if deviceMap as core::ffi::c_uint & (1 as core::ffi::c_uint) << index as core::ffi::c_int
            != 0 as core::ffi::c_uint
        {
            /* Close mmwave radar device communication Channel */
            if !rl_driverData.commDevIdx.comIfHdl[index as usize].is_null() {
                retVal += rl_driverData
                    .clientCtx
                    .comIfCb
                    .rlComIfClose
                    .expect("non-null function pointer")(
                    rl_driverData.commDevIdx.comIfHdl[index as usize],
                );
                rl_driverData.commDevIdx.comIfHdl[index as usize] = 0 as *mut core::ffi::c_void
            }
            /* Un Register Interrupt Handler */
            rl_driverData
                .clientCtx
                .devCtrlCb
                .rlRegisterInterruptHandler
                .expect("non-null function pointer")(
                index, None, 0 as *mut core::ffi::c_void
            );
            deviceMap = (deviceMap as core::ffi::c_uint
                & !((1 as core::ffi::c_uint) << index as core::ffi::c_int))
                as rlUInt8_t
        }
        index = index.wrapping_add(1)
    }
    /* Destroy Global Mutex */
    if !rl_driverData.globalMutex.is_null() {
        retVal += rl_driverData
            .clientCtx
            .osiCb
            .mutex
            .rlOsiMutexDelete
            .expect("non-null function pointer")(&mut rl_driverData.globalMutex);
        rl_driverData.globalMutex = 0 as *mut core::ffi::c_void
    }
    /* Destroy Command Semaphore */
    if !rl_driverData.cmdSem.is_null() {
        retVal += rl_driverData
            .clientCtx
            .osiCb
            .sem
            .rlOsiSemDelete
            .expect("non-null function pointer")(&mut rl_driverData.cmdSem);
        rl_driverData.cmdSem = 0 as *mut core::ffi::c_void
    }
    rl_driverData.deviceMap = 0 as core::ffi::c_uint as rlUInt8_t;
    rl_driverData.isDriverInitialized = 0 as core::ffi::c_uint as rlUInt8_t;
    return retVal;
}
/* * @fn rlDriverData_t* rlDriverGetHandle(void)
*
*   @brief Returns mmwave radar Driver Global Structure
*
*   @return rlDriverData_t Pointer to mmwave radar Driver Global Structure
*
*   Returns mmwave radar Driver Global Structure
*/
/* DesignId :  */
/* Requirements :  */
#[no_mangle]
pub unsafe extern "C" fn rlDriverGetHandle() -> *mut rlDriverData_t {
    /* return driverData pointer/handle */
    return &mut rl_driverData;
}
/* * @fn rlDriverGetPlatformId(void)
*
*   @brief Returns RL Platform ID (i.e. where mmWaveLink is executing)
*
*   @return rlUInt8_t platform
*
*   Returns Device run ID (i.e. where mmWaveLink is executing)
*/
/* DesignId :  */
/* Requirements :  */
#[no_mangle]
pub unsafe extern "C" fn rlDriverGetPlatformId() -> rlUInt8_t {
    /* return platform ID */
    return rl_driverData.clientCtx.platform;
}
/* * @fn rlDriverGetArDeviceType(void)
*
*   @brief Returns AR device type which mmWavelink is communicating
*
*   @return rlUInt8_t platform
*
*   Returns AR device type which mmWavelink is communicating
*/
/* DesignId :  */
/* Requirements :  */
#[no_mangle]
pub unsafe extern "C" fn rlDriverGetArDeviceType() -> rlUInt8_t {
    /* return AWR device Type */
    return rl_driverData.clientCtx.arDevType;
}
/* * @fn rlUInt8_t rlDriverIsDeviceMapValid(rlUInt8_t deviceMap)
*
*   @brief Checks if given deviecMap is valid wrt to global DeviceMap
 *         set to mmWaveLink
*   @param[in] deviceMap - Bitmap of devices to send the message
*
*   @return rlReturnVal_t Success - 0, Failure - Error Code
*
*   @brief It enables RF/Analog Subsystem.
*/
/* DesignId :  */
/* Requirements :  */
#[no_mangle]
pub unsafe extern "C" fn rlDriverIsDeviceMapValid(mut deviceMap: rlUInt8_t) -> rlReturnVal_t {
    let mut storedDevMap: rlUInt8_t = 0;
    let mut retVal: rlReturnVal_t = 0;
    /* get the no. of connected device */
    storedDevMap = rl_driverData.deviceMap;
    if (storedDevMap as core::ffi::c_int & deviceMap as core::ffi::c_int) as core::ffi::c_uint
        != 0 as core::ffi::c_uint
    {
        /* set return value to success */
        retVal = 0 as core::ffi::c_int
    } else {
        /* set return value to failure */
        retVal = -(1 as core::ffi::c_int)
    }
    return retVal;
}
/* * @fn rlReturnVal_t rlDriverWaitForResponse(rlUInt8_t devIndex,
*                                             rlDriverMsg_t* outMsg)
*
*   @brief: Wait for Device's response
*
*   @param[in ] devIndex - devIndex is index of device not the deviceMap
*   @param[out] outMsg   - Driver msg
*
*   @return Success : 0, Fail : Error code
*
*   Wait for driver response
*/
/* DesignId : MMWL_DesignId_024 */
/* Requirements : AUTORADAR_REQ-774 */
/*AR_CODE_REVIEW MR:R.2.2 <APPROVED> "payloadLen is re initialized after each pass to the
rlGetSubBlock fucntion as it is passed again to the function" */
/*LDRA_INSPECTED 8 D */
#[no_mangle]
pub unsafe extern "C" fn rlDriverWaitForResponse(
    mut devIndex: rlUInt8_t,
    mut outMsg: *mut rlDriverMsg_t,
) -> rlReturnVal_t {
    let mut rspChunks: rlUInt16_t = 0;
    let mut errorSB: rlPayloadSb_t = rlPayloadSb_t {
        sbid: 0,
        len: 0,
        pSblkData: 0 as *mut rlUInt8_t,
    };
    let mut retVal: rlReturnVal_t = 0;
    let mut retVal1: rlReturnVal_t = 0;
    let mut indx: rlUInt16_t = 0;
    let mut payloadLen: rlUInt8_t = 0 as core::ffi::c_uint as rlUInt8_t;
    loop {
        /* Wait for Host Interrupt and Read the Response */
        retVal1 = rlDriverMsgReadCmdCtx(devIndex);
        if 0 as core::ffi::c_int == retVal1 {
            /* Get number of chunks in the response */
            rspChunks = rl_rxMsg.hdr.remChunks;
            /* Check if Number of Sub Block doesn't exceed expected number */
            if (*outMsg).opcode.nsbc as core::ffi::c_int >= rl_rxMsg.hdr.nsbc as core::ffi::c_int {
                (*outMsg).opcode.nsbc = rl_rxMsg.hdr.nsbc;
                /* Loop for number of chunks and copy payload */
                indx = 0 as core::ffi::c_uint as rlUInt16_t;
                while (indx as core::ffi::c_int) < (*outMsg).opcode.nsbc as core::ffi::c_int {
                    /* Copy Payload to output variable*/
                    /* AR_CODE_REVIEW MR:R.18.4 <APPROVED> "pointer arithmetic required" */
                    /*LDRA_INSPECTED 87 S */
                    rlGetSubBlock(
                        rl_rxMsg
                            .payload
                            .as_mut_ptr()
                            .offset(payloadLen as core::ffi::c_int as isize),
                        &mut (*(*outMsg).subblocks.offset(indx as isize)).sbid,
                        &mut (*(*outMsg).subblocks.offset(indx as isize)).len,
                        (*(*outMsg).subblocks.offset(indx as isize)).pSblkData,
                    );
                    payloadLen = (payloadLen as core::ffi::c_int
                        + (*(*outMsg).subblocks.offset(indx as isize)).len as rlUInt8_t
                            as core::ffi::c_int) as rlUInt8_t;
                    indx = indx.wrapping_add(1)
                }
            }
            retVal = retVal1
        } else if -(13 as core::ffi::c_int) == retVal1 {
            /* Number of Sub Block is unexpected. Error Case */
            /* AR_CODE_REVIEW MR:R.2.2  <APPROVED> "Values are used by called function.
            LDRA Tool Issue" */
            /*LDRA_INSPECTED 105 D */
            let mut errMsgSbData: rlErrorResp_t = {
                let mut init = rlErrorResp {
                    errorType: 0 as core::ffi::c_uint as rlSysNRespType_t,
                    sbcID: 0 as core::ffi::c_uint as rlUInt16_t,
                };
                init
            };
            /* Initialize with zero which will set with valid value in next function call */
            errorSB.sbid = 0 as core::ffi::c_uint as rlUInt16_t;
            errorSB.len = 0 as core::ffi::c_uint as rlUInt16_t;
            errorSB.pSblkData = &mut errMsgSbData as *mut rlErrorResp_t as *mut rlUInt8_t;
            /* Copy Payload to local variable*/
            /* AR_CODE_REVIEW MR:R.18.4 <APPROVED> "pointer arithmetic required" */
            /*LDRA_INSPECTED 87 S */
            rlGetSubBlock(
                rl_rxMsg
                    .payload
                    .as_mut_ptr()
                    .offset(payloadLen as core::ffi::c_int as isize),
                &mut errorSB.sbid,
                &mut errorSB.len,
                errorSB.pSblkData,
            );
            /* Return Error to indicate command failure */
            retVal = errMsgSbData.errorType as rlReturnVal_t;
            rspChunks = 0 as core::ffi::c_uint as rlUInt16_t
        } else {
            /* Timeout in receiving response*/
            rspChunks = 0 as core::ffi::c_uint as rlUInt16_t;
            retVal = retVal1
        }
        if !(rspChunks as core::ffi::c_uint > 0 as core::ffi::c_uint) {
            break;
        }
    }
    return retVal;
}
unsafe extern "C" fn rlDriverCmdWriter(
    mut devIndex: rlUInt8_t,
    mut outMsg: *mut rlDriverMsg_t,
) -> rlReturnVal_t {
    let mut retVal: rlReturnVal_t = 0;
    let mut retryCount: rlUInt8_t = 0 as core::ffi::c_uint as rlUInt8_t;
    let mut rlDrvData: *mut rlDriverData_t = rlDriverGetHandle();
    let mut isPayloadValid: rlUInt8_t = 0 as core::ffi::c_uint as rlUInt8_t;
    let mut isNackRetry: rlUInt8_t = 0 as core::ffi::c_uint as rlUInt8_t;
    loop {
        if retryCount as core::ffi::c_uint != 0 as core::ffi::c_uint
            && isNackRetry as core::ffi::c_int
                == 0 as core::ffi::c_uint as rlUInt8_t as core::ffi::c_int
        {
            /* AR_CODE_REVIEW MR:R.2.2 <APPROVED> "Global variable is used by
            other function" */
            /*LDRA_INSPECTED 105 D */
            /* AR_CODE_REVIEW MR:D.4.1,D.4.14 <APPROVED> "rlDrvData Can't be NULL" */
            /*LDRA_INSPECTED 45 D */
            (*(*rlDrvData).funcParams.cmd)
                .hdr
                .flags
                .set_b2RetryFlag(0x3 as core::ffi::c_uint as rlUInt16_t)
        }
        retryCount = retryCount.wrapping_add(1);
        if (*rlDrvData).clientCtx.ackTimeout != 0 as core::ffi::c_uint {
            /* Set flag to true and check for it in Host IRQ  handler routine*/
            /* AR_CODE_REVIEW MR:R.2.2 <APPROVED> "Global variable is used by
            other function" */
            /*LDRA_INSPECTED 105 D */
            ::core::ptr::write_volatile(
                &mut (*rlDrvData).isCmdRespWaited[devIndex as usize] as *mut rlUInt8_t,
                1 as core::ffi::c_uint as rlUInt8_t,
            )
        }
        /* send the command to mmWave Radar device */
        retVal = rlDriverMsgWrite(
            rlDrvData,
            (*rlDrvData).commDevIdx.comIfHdl[devIndex as usize],
        );
        if 0 as core::ffi::c_int == retVal {
            /* Check if It needs to wait for ACK*/
            if (*rlDrvData).clientCtx.ackTimeout != 0 as core::ffi::c_uint {
                /* wait for respond */
                retVal += rlDriverWaitForResponse(devIndex, outMsg)
            }
            /* Response received Successfully */
            if 0 as core::ffi::c_int == retVal
                || -(6 as core::ffi::c_int) != retVal && -(8 as core::ffi::c_int) != retVal
            {
                /* AR_CODE_REVIEW MR:R.2.2 <APPROVED> "Global variable is used by
                other function" */
                /*LDRA_INSPECTED 105 D */
                ::core::ptr::write_volatile(
                    &mut (*rlDrvData).isCmdRespWaited[devIndex as usize] as *mut rlUInt8_t,
                    0 as core::ffi::c_uint as rlUInt8_t,
                );
                /* Increment the Sequence Number */
                (*rlDrvData).cmdSeqNum[devIndex as usize] =
                    (*rlDrvData).cmdSeqNum[devIndex as usize].wrapping_add(1);
                /* If device sends NACK then mmWaveLink needs to resend same
                command with incremented seqNum but no RETRY-Flag */
                if -(16 as core::ffi::c_int) == retVal {
                    /* Set the flag */
                    isNackRetry = 1 as core::ffi::c_uint as rlUInt8_t;
                    /* Modulo on cmdSeqNum (4Bits field) and fill Command Sequence Number */
                    /* AR_CODE_REVIEW MR:R.2.2 <APPROVED> "Global variable is used
                    by other function" */
                    /*LDRA_INSPECTED 105 D */
                    /*LDRA_INSPECTED 8 D */
                    (*(*rlDrvData).funcParams.cmd).hdr.flags.set_b4SeqNum(
                        ((*rlDrvData).cmdSeqNum[devIndex as usize] as core::ffi::c_uint)
                            .wrapping_rem(16 as core::ffi::c_uint)
                            as rlUInt16_t,
                    )
                } else {
                    /* AR_CODE_REVIEW MR:R.2.2 <APPROVED> "Set to TRUE for valid payload" */
                    /*LDRA_INSPECTED 8 D */
                    isPayloadValid = 1 as core::ffi::c_uint as rlUInt8_t;
                    /* Reset the flag */
                    isNackRetry = 0 as core::ffi::c_uint as rlUInt8_t
                }
            } else {
                /* if return value is not NACK_ERROR then reset this flag */
                isNackRetry = 0 as core::ffi::c_uint as rlUInt8_t
            }
            if !((retryCount as core::ffi::c_uint)
                < (rl_driverData.retryCount as core::ffi::c_uint)
                    .wrapping_add(1 as core::ffi::c_uint)
                && isPayloadValid as core::ffi::c_int
                    == 0 as core::ffi::c_uint as rlUInt8_t as core::ffi::c_int)
            {
                break;
            }
        } else {
            /* Error in command message write */
            if (*rlDrvData).clientCtx.ackTimeout != 0 as core::ffi::c_uint {
                /* AR_CODE_REVIEW MR:R.2.2 <APPROVED> "Global variable is used by
                other function" */
                /*LDRA_INSPECTED 105 D */
                ::core::ptr::write_volatile(
                    &mut (*rlDrvData).isCmdRespWaited[devIndex as usize] as *mut rlUInt8_t,
                    0 as core::ffi::c_uint as rlUInt8_t,
                )
            }
            break;
        }
    }
    return retVal;
}
/* * @fn rlReturnVal_t rlDriverCmdSendRetry(rlUInt8_t deviceMap, rlDriverMsg_t* outMsg)
*
*   @brief: Send command and wait for response
*
*   @param[in ] deviceMap - device map contains bitmap of all device connected
*   @param[out] outMsg   - output msg
*
*   @return Success : 0, Fail : Error code
*
*   Send command and wait for response
*/
/* DesignId : MMWL_DesignId_023 */
/* Requirements : AUTORADAR_REQ-772, AUTORADAR_REQ-774, AUTORADAR_REQ-775, AUTORADAR_REQ-778,\
AUTORADAR_REQ-781 */
#[no_mangle]
pub unsafe extern "C" fn rlDriverCmdSendRetry(
    mut deviceMap: rlUInt8_t,
    mut outMsg: *mut rlDriverMsg_t,
) -> rlReturnVal_t {
    let mut retVal: rlReturnVal_t = 0;
    let mut devIndex: rlUInt8_t = 0 as core::ffi::c_uint as rlUInt8_t;
    let mut rlDrvData: *mut rlDriverData_t = rlDriverGetHandle();
    if rlDrvData.is_null() || (*rlDrvData).funcParams.cmd.is_null() {
        retVal = -(9 as core::ffi::c_int)
    } else {
        retVal = 0 as core::ffi::c_int
    }
    /* Send Command to Device connected and wait for response one by one */
    /* AR_CODE_REVIEW MR:D.2.1 <APPROVED> "This loop terminates when it sends commands to all
     * connected devices or when any of devices returns -ve response" */
    /*LDRA_INSPECTED 28 D */
    while deviceMap as core::ffi::c_uint != 0 as core::ffi::c_uint
        && 0 as core::ffi::c_int == retVal
    {
        if deviceMap as core::ffi::c_uint & (1 as core::ffi::c_uint) << devIndex as core::ffi::c_int
            != 0 as core::ffi::c_uint
        {
            /* End of If */
            /* Fill Command Sequence Number */
            /* AR_CODE_REVIEW MR:R.2.2 <APPROVED> "Global variable is used by other function" */
            /*LDRA_INSPECTED 105 D */
            /*LDRA_INSPECTED 8 D */
            (*(*rlDrvData).funcParams.cmd).hdr.flags.set_b4SeqNum(
                ((*rlDrvData).cmdSeqNum[devIndex as usize] as core::ffi::c_uint)
                    .wrapping_rem(16 as core::ffi::c_uint) as rlUInt16_t,
            );
            /* Write command to slave device */
            retVal += rlDriverCmdWriter(devIndex, outMsg);
            deviceMap = (deviceMap as core::ffi::c_uint
                & !((1 as core::ffi::c_uint) << devIndex as core::ffi::c_int))
                as rlUInt8_t
        }
        /* AR_CODE_REVIEW MR:R.2.2 <APPROVED> "Global variable is used by other function" */
        /*LDRA_INSPECTED 105 D */
        ::core::ptr::write_volatile(
            &mut (*rlDrvData).isCmdRespWaited[devIndex as usize] as *mut rlUInt8_t,
            0 as core::ffi::c_uint as rlUInt8_t,
        );
        devIndex = devIndex.wrapping_add(1)
    }
    return retVal;
}
/* * @fn void rlDriverAppendDummyByte(void)
*
*   @brief Append the dummy bytes to payload part of message
*
*   @param[in] None
*
*   @return None.
*
*   Append the dummy bytes to payload part of message if required based on CRC type used.
*/
/* DesignId : MMWL_DesignId_023 */
/* Requirements : AUTORADAR_REQ-772, AUTORADAR_REQ-774, AUTORADAR_REQ-776 */
unsafe extern "C" fn rlDriverAppendDummyByte() {
    let mut protAlignSize: rlUInt8_t = 0;
    let mut msgCrcLen: rlUInt16_t = 0;
    rl_txMsg
        .hdr
        .flags
        .set_b2CrcLen(rl_driverData.clientCtx.crcType as rlUInt16_t);
    /* It may be size 2/4/8 based on 16/32/64 bit */
    msgCrcLen = ((2 as core::ffi::c_uint)
        << (rl_driverData.clientCtx.crcType as core::ffi::c_uint & 0x3 as core::ffi::c_uint))
        as rlUInt16_t;
    protAlignSize =
        if (rl_driverData.clientCtx.crcType as core::ffi::c_uint) < 2 as core::ffi::c_uint {
            0x4 as core::ffi::c_uint
        } else {
            0x8 as core::ffi::c_uint
        } as rlUInt8_t;
    /* cmd pointer is assigned to rl_rxMsg, which can't be NULL as it's global structure. */
    if !rl_driverData.funcParams.cmd.is_null() {
        /* Add Padding Byte to payload - This is required before CRC calculation*/
        if ((*rl_driverData.funcParams.cmd).hdr.len as core::ffi::c_int
            % protAlignSize as core::ffi::c_int) as core::ffi::c_uint
            != 0 as core::ffi::c_uint
        {
            /* AR_CODE_REVIEW MR:R.18.1,R.18.4 <APPROVED> "pointer arithmetic required" */
            /*LDRA_INSPECTED 567 S */
            /* AR_CODE_REVIEW MR:R.10.3  <APPROVED> "All parameter types are matching to
            function argument type. LDRA tool issue." */
            /*LDRA_INSPECTED 458 S */
            /*LDRA_INSPECTED 87 S */
            rlAppendDummy(
                (rl_driverData.funcParams.cmd as *mut rlUInt8_t)
                    .offset((*rl_driverData.funcParams.cmd).hdr.len as core::ffi::c_int as isize)
                    .offset(4 as core::ffi::c_uint as isize),
                (protAlignSize as core::ffi::c_int
                    - ((*rl_driverData.funcParams.cmd).hdr.len as core::ffi::c_int
                        % protAlignSize as core::ffi::c_int) as rlUInt8_t
                        as core::ffi::c_int) as rlUInt8_t,
            );
            (*rl_driverData.funcParams.cmd).hdr.len = ((*rl_driverData.funcParams.cmd).hdr.len
                as core::ffi::c_int
                + (protAlignSize as core::ffi::c_int
                    - ((*rl_driverData.funcParams.cmd).hdr.len as core::ffi::c_int
                        % protAlignSize as core::ffi::c_int) as rlUInt16_t
                        as core::ffi::c_int) as rlUInt16_t as core::ffi::c_int)
                as rlUInt16_t
        }
        /* add on crc length to header length field */
        (*rl_driverData.funcParams.cmd).hdr.len =
            ((*rl_driverData.funcParams.cmd).hdr.len as core::ffi::c_int
                + msgCrcLen as core::ffi::c_int) as rlUInt16_t
    };
}
/* * @fn rlReturnVal_t rlDriverCmdInvoke(rlUInt8_t deviceMap,
*                        rlDriverMsg_t inMsg, rlDriverMsg_t* outMsg)
*
*   @brief Invokes a command to mmwave radar Device. Implements mmwave radar
*          Host Communication Protocol(RHCP)
*
*   @param[in] deviceMap - Bitmap of devices to send the message
*   @param[in] inMsg - Command Opcode(Direction, Class, MsgId) and Subblocks
*   @param[out] outMsg - Response Opcode(Direction, Class, MsgId) and Subblocks
*
*   @return rlReturnVal_t Success - 0, Failure - Error Code
*
*   Invokes a command to mmwave radar Device. Waits for Response if ACK is
*   requested. Computes
*   CRC/Checksum according to mmwave radar Host Communication Protocol(RHCP)
*/
/* DesignId : MMWL_DesignId_023 */
/* Requirements : AUTORADAR_REQ-772, AUTORADAR_REQ-774, AUTORADAR_REQ-776 */
#[no_mangle]
pub unsafe extern "C" fn rlDriverCmdInvoke(
    mut deviceMap: rlUInt8_t,
    mut inMsg: rlDriverMsg_t,
    mut outMsg: *mut rlDriverMsg_t,
) -> rlReturnVal_t {
    let mut retVal: rlReturnVal_t = 0;
    let mut indx: rlUInt16_t = 0;
    let mut payloadLen: rlUInt16_t = 0 as core::ffi::c_uint as rlUInt16_t;
    if rl_driverData.isDriverInitialized as core::ffi::c_int
        != 0 as core::ffi::c_uint as rlUInt8_t as core::ffi::c_int
    {
        /* If Mutex lock is failed then return with error code */
        if 0 as core::ffi::c_int
            != rl_driverData
                .clientCtx
                .osiCb
                .mutex
                .rlOsiMutexLock
                .expect("non-null function pointer")(
                &mut rl_driverData.globalMutex,
                0xffff as core::ffi::c_uint,
            )
        {
            /* If MutexLock returns non-zero then treat this as error and
            set error code to retVal */
            retVal = -(10 as core::ffi::c_int)
        } else {
            let mut retVal1: rlReturnVal_t = 0 as core::ffi::c_int;
            /* Fill Command Header */
            /* AR_CODE_REVIEW MR:R.2.2 <APPROVED> "Some of variables are re-assigned based
            on different conditions."*/
            /*LDRA_INSPECTED 8 D */
            rl_txMsg
                .hdr
                .opcode
                .set_b4Direction(inMsg.opcode.dir as rlUInt16_t);
            rl_txMsg
                .hdr
                .opcode
                .set_b2MsgType(inMsg.opcode.msgType as rlUInt16_t);
            rl_txMsg.hdr.opcode.set_b10MsgId(
                (rl_txMsg.hdr.opcode.b10MsgId() as core::ffi::c_uint & 0 as core::ffi::c_uint
                    | inMsg.opcode.msgId as core::ffi::c_uint & 0x3ff as core::ffi::c_uint)
                    as rlUInt16_t,
            );
            rl_txMsg.hdr.remChunks = inMsg.remChunks;
            rl_txMsg.hdr.nsbc = inMsg.opcode.nsbc;
            rl_txMsg
                .hdr
                .flags
                .set_b2AckFlag(0 as core::ffi::c_uint as rlUInt16_t);
            rl_txMsg
                .hdr
                .flags
                .set_b2Crc(0 as core::ffi::c_uint as rlUInt16_t);
            rl_txMsg
                .hdr
                .flags
                .set_b2CrcLen(0 as core::ffi::c_uint as rlUInt16_t);
            rl_txMsg
                .hdr
                .flags
                .set_b2RetryFlag(0 as core::ffi::c_uint as rlUInt16_t);
            rl_txMsg
                .hdr
                .flags
                .set_b4SeqNum(0 as core::ffi::c_uint as rlUInt16_t);
            rl_txMsg
                .hdr
                .flags
                .set_b4Version(0 as core::ffi::c_uint as rlUInt16_t);
            /* Fill Payload */
            indx = 0 as core::ffi::c_uint as rlUInt16_t;
            while (indx as core::ffi::c_int) < inMsg.opcode.nsbc as core::ffi::c_int {
                /* append all subblock len, id and data to one global structure */
                retVal1 += rlAppendSubBlock(
                    &mut *rl_txMsg.payload.as_mut_ptr().offset(payloadLen as isize),
                    (*inMsg.subblocks.offset(indx as isize)).sbid,
                    (*inMsg.subblocks.offset(indx as isize)).len,
                    (*inMsg.subblocks.offset(indx as isize)).pSblkData,
                );
                /* increment payload length as appending each sub-block data in a message */
                payloadLen = (payloadLen as core::ffi::c_int
                    + ((*inMsg.subblocks.offset(indx as isize)).len as core::ffi::c_int
                        + (2 as core::ffi::c_uint).wrapping_add(2 as core::ffi::c_uint)
                            as rlUInt16_t as core::ffi::c_int))
                    as rlUInt16_t;
                if 0 as core::ffi::c_int != retVal1 {
                    break;
                }
                indx = indx.wrapping_add(1)
            }
            /* if above for loop is not terminated due to any error */
            if indx as core::ffi::c_int == inMsg.opcode.nsbc as core::ffi::c_int {
                rl_txMsg.hdr.len = (12 as core::ffi::c_uint)
                    .wrapping_add(payloadLen as core::ffi::c_uint)
                    as rlUInt16_t;
                /* Update command and response buffers */
                rl_driverData.funcParams.cmd = &mut rl_txMsg;
                rl_driverData.funcParams.rsp = &mut rl_rxMsg;
                /* Check if ACK is Requested*/
                if rl_driverData.clientCtx.ackTimeout == 0 as core::ffi::c_uint {
                    /* No ACK Requested */
                    rl_txMsg
                        .hdr
                        .flags
                        .set_b2AckFlag(0x3 as core::ffi::c_uint as rlUInt16_t)
                }
                /* Check if CRC is required to be sent*/
                if rl_driverData.clientCtx.crcType as core::ffi::c_uint == 3 as core::ffi::c_uint {
                    /* CRC Not Included */
                    rl_txMsg
                        .hdr
                        .flags
                        .set_b2Crc(0x3 as core::ffi::c_uint as rlUInt16_t)
                }
                /* Append Dummy Bytes if CRC is present */
                if (*rl_driverData.funcParams.cmd).hdr.flags.b2Crc() as core::ffi::c_uint
                    == 0 as core::ffi::c_uint
                {
                    rlDriverAppendDummyByte();
                }
                retVal = rlDriverCmdSendRetry(deviceMap, outMsg);
                /* Release the Global Mutex */
                rl_driverData
                    .clientCtx
                    .osiCb
                    .mutex
                    .rlOsiMutexUnLock
                    .expect("non-null function pointer")(
                    &mut rl_driverData.globalMutex
                );
            } else {
                retVal = retVal1
            }
        }
    } else {
        retVal = -(11 as core::ffi::c_int)
    }
    return retVal;
}
/* * @fn rlReturnVal_t rlDriverConfigureCrc(rlCrcType_t crcType)
*
*   @brief  Configures the CRC Type in mmwavelink Driver
*   @param[in] crcType - CRC Types
*
*   @return rlReturnVal_t Success - 0, Failure - Error Code
*
*   Configures the CRC Type in mmwavelink Driver
*/
#[no_mangle]
pub unsafe extern "C" fn rlDriverConfigureCrc(mut crcType: rlCrcType_t) -> rlReturnVal_t {
    let mut retVal: rlReturnVal_t = 0;
    /* Check if driver is initialized */
    if rl_driverData.isDriverInitialized as core::ffi::c_uint == 1 as core::ffi::c_uint {
        /* Set CRC Type to global structure */
        rl_driverData.clientCtx.crcType = crcType;
        retVal = 0 as core::ffi::c_int
    } else {
        /* if driver is not initialized then return and error value */
        retVal = -(11 as core::ffi::c_int)
    }
    return retVal;
}
/* * @fn rlReturnVal_t rlDriverConfigureAckTimeout(rlUInt32_t ackTimeout)
*
*   @brief  Configures the Acknowledgement timeout in mmwavelink Driver
*   @param[in] ackTimeout - ACK timeout, 0 - No ACK
*
*   @return rlReturnVal_t Success - 0, Failure - Error Code
*
*   Configures the Acknowledgement timeout in mmwavelink Driver
*/
/* DesignId :  */
/* Requirements :  */
#[no_mangle]
pub unsafe extern "C" fn rlDriverConfigureAckTimeout(mut ackTimeout: rlUInt32_t) -> rlReturnVal_t {
    let mut retVal: rlReturnVal_t = 0;
    /* Check if driver is initialized */
    if rl_driverData.isDriverInitialized as core::ffi::c_uint == 1 as core::ffi::c_uint {
        /* set ACK timeout to global structure */
        rl_driverData.clientCtx.ackTimeout = ackTimeout;
        retVal = 0 as core::ffi::c_int
    } else {
        /* if driver is not initialized then return and error value */
        retVal = -(11 as core::ffi::c_int)
    }
    return retVal;
}
/* ***************************************************************************************
 * FileName     : rl_trace.h
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
 */
/* ***************************************************************************************
 * FILE INCLUSION PROTECTION
 ****************************************************************************************
 */
/* ***************************************************************************************
* INCLUDE FILES
****************************************************************************************
*/
/* ***************************************************************************************
 * MACRO DEFINITIONS
 ****************************************************************************************
 */
/* Debug fuunction */
/* * @fn rlPrintFptr rlGetLogFptr(rlUInt8_t dbgLevel)
*
*   @brief: Configure loggin function to print error message
*   @param[in] dbgLevel - Debug level
*
*   @return rlPrintFptr Success - valid fucntion pointer,
*                       Failure - NULL
*
*   Configures the logging function for error in mmwavelink Driver
*/
/* DesignId :  */
/* Requirements :  */
#[no_mangle]
pub unsafe extern "C" fn rlGetLogFptr(mut dbgLevel: rlUInt8_t) -> rlPrintFptr {
    let mut retFuncPtr: rlPrintFptr = None;
    if dbgLevel as core::ffi::c_uint > 0 as core::ffi::c_uint {
        /* If debug level is valid the set debug function pointer */
        retFuncPtr = rl_driverData.logObj.rlPrintAr[(dbgLevel as core::ffi::c_int
            - 1 as core::ffi::c_uint as rlUInt8_t as core::ffi::c_int)
            as usize]
    } else {
        /* If debug level is valid the set debug function pointer to NULL */
        retFuncPtr = None
    }
    return retFuncPtr;
}
/* * @fn rlReturnVal_t rlLogInit(void)
*
*   @brief: Initialise logging function pointers.
*
*   @return rlReturnVal_t Success - ( 0),
*                         Failure - (-1)
*
*   Initialise logging functions in mmwavelink Driver
*/
/* DesignId : MMWL_DesignId_031 */
/* Requirements : AUTORADAR_REQ-712 */
#[no_mangle]
pub unsafe extern "C" fn rlLogInit() -> rlReturnVal_t {
    let mut retVal: rlReturnVal_t = 0;
    let mut rlDrvData: *mut rlDriverData_t = &mut rl_driverData;
    let mut fPtr: rlPrintFptr = None;
    let mut level: rlUInt8_t = 0;
    let mut idx: rlUInt8_t = 0;
    /* store debug level to local variable */
    level = (*rlDrvData).clientCtx.dbgCb.dbgLevel;
    /* store Function pointer to local variable */
    fPtr = (*rlDrvData).clientCtx.dbgCb.rlPrint;
    /* check for Function pointer for NON-NULL */
    if fPtr.is_some() {
        match level as core::ffi::c_int {
            5 | 4 | 3 | 2 | 1 => {
                idx = level;
                while idx as core::ffi::c_int
                    > 0 as core::ffi::c_uint as rlUInt8_t as core::ffi::c_int
                {
                    (*rlDrvData).logObj.rlPrintAr[(idx as core::ffi::c_uint)
                        .wrapping_sub(1 as core::ffi::c_uint)
                        as usize] = fPtr;
                    idx = idx.wrapping_sub(1)
                }
            }
            0 => {
                fPtr.expect("non-null function pointer")(
                    b"INFO: MMWAVELINK Logging is disabled\n\x00" as *const u8
                        as *const core::ffi::c_char,
                );
                /* reset all function pointers to NULL */
                memset(
                    &mut *(*rlDrvData)
                        .logObj
                        .rlPrintAr
                        .as_mut_ptr()
                        .offset(0 as core::ffi::c_int as isize)
                        as *mut rlPrintFptr as *mut core::ffi::c_void,
                    0 as core::ffi::c_uint as core::ffi::c_int,
                    ::core::mem::size_of::<rlLogCtx_t>() as _,
                );
            }
            _ => {
                fPtr.expect("non-null function pointer")(
                    b"INFO: Invalid MMWAVELINK Logging, hence disbled\n\x00" as *const u8
                        as *const core::ffi::c_char,
                );
                /* if dbgLevel is set beyond expected value then assign NONE */
                (*rlDrvData).clientCtx.dbgCb.dbgLevel = 0 as core::ffi::c_uint as rlUInt8_t;
                /* reset all function pointers to NULL */
                memset(
                    &mut *(*rlDrvData)
                        .logObj
                        .rlPrintAr
                        .as_mut_ptr()
                        .offset(0 as core::ffi::c_int as isize)
                        as *mut rlPrintFptr as *mut core::ffi::c_void,
                    0 as core::ffi::c_uint as core::ffi::c_int,
                    ::core::mem::size_of::<rlLogCtx_t>() as _,
                );
            }
        }
        retVal = 0 as core::ffi::c_int
    } else {
        /* reset all function pointers to NULL */
        memset(
            &mut *(*rlDrvData)
                .logObj
                .rlPrintAr
                .as_mut_ptr()
                .offset(0 as core::ffi::c_int as isize) as *mut rlPrintFptr
                as *mut core::ffi::c_void,
            0 as core::ffi::c_uint as core::ffi::c_int,
            ::core::mem::size_of::<rlLogCtx_t>() as _,
        );
        retVal = -(15 as core::ffi::c_int)
    }
    return retVal;
}
/* * @fn void rlDriverConstructInMsg(rlUInt16_t msgId, rlDriverMsg_t* inMsg,
*    rlPayloadSb_t* payloadPtr)
*
*   @brief: Construct command packet (inMsg) based on given message-ID and payload
*
*   @param[in] msgId - message ID of command packet
*   @param[out] inMsg - Message structure input pointer
*   @param[in] payloadPtr - payload data pointer
*   @return - none
*
*   Construct Set command packet based on given message-ID and payload
*/
#[no_mangle]
pub unsafe extern "C" fn rlDriverConstructInMsg(
    mut msgId: rlUInt16_t,
    mut inMsg: *mut rlDriverMsg,
    mut payloadPtr: *mut rlPayloadSb,
) {
    /* check for NULL pointer */
    if !inMsg.is_null() {
        let mut cmdDir: rlUInt8_t = 0;
        /* Set Command Header Opcode */
        (*inMsg).opcode.nsbc = 1 as core::ffi::c_uint as rlUInt16_t;
        (*inMsg).opcode.msgType = 0 as core::ffi::c_uint as rlUInt8_t;
        (*inMsg).opcode.msgId = msgId;
        (*inMsg).remChunks = 0 as core::ffi::c_uint as rlUInt16_t;
        /* get command direction based on requested MsgId */
        cmdDir = rlDeviceIdentifyCmdDir(msgId, rlDriverGetPlatformId());
        (*inMsg).opcode.dir = cmdDir;
        (*inMsg).subblocks = payloadPtr
    };
}
/* * @fn void rlDriverConstructOutMsg(rlUInt16_t numSblk, rlDriverMsg_t* outMsg,
*                                       rlPayloadSb_t* payloadPtr)
*
*   @brief: Construct command packet based on given message-ID and payload
*
*   @param[in] numSblk - number of sub-blocks in message
*   @param[out] outMsg - Message structure input pointer
*   @param[in] payloadPtr - payload data pointer
*   @return - none
*
*   Construct Get command packet based on given message-ID and payload
*/
#[no_mangle]
pub unsafe extern "C" fn rlDriverConstructOutMsg(
    mut numSblk: rlUInt16_t,
    mut outMsg: *mut rlDriverMsg_t,
    mut payloadPtr: *mut rlPayloadSb_t,
) {
    /* check for NULL pointer */
    if !outMsg.is_null() {
        /* Set num of sub-block to outMsg Opcode field */
        (*outMsg).opcode.nsbc = numSblk;
        /* assign payload pointer to subblock ot outMsg */
        (*outMsg).subblocks = payloadPtr
    };
}
/* * @fn void rlDriverFillPayload(rlUInt16_t msgId, rlUInt16_t sbcID, rlPayloadSb_t* payloadPtr,
*    rlUInt8_t* data, rlUInt16_t inLen)
*
*   @brief: Fill payload based on given message-ID, sub-block ID and data.
*   @param[in] msgId - message ID of command packet
*   @param[in] sbcID  - sub block ID of command packet
*   @param[in] payloadPtr - payload data pointer
*   @param[out] data - data pointer
*   @param[in] inLen - lenght of data
*   @return - none
*
*   Fill payload based on given message-ID, sub-block ID and data.
*/
#[no_mangle]
pub unsafe extern "C" fn rlDriverFillPayload(
    mut msgId: rlUInt16_t,
    mut sbcID: rlUInt16_t,
    mut payloadPtr: *mut rlPayloadSb_t,
    mut data: *mut rlUInt8_t,
    mut inLen: rlUInt16_t,
) {
    /* check for NULL pointer */
    if !payloadPtr.is_null() {
        /* get Unique Sub-Block ID and asign it to Command Sub Block */
        (*payloadPtr).sbid = (msgId as core::ffi::c_uint)
            .wrapping_mul(32 as core::ffi::c_uint)
            .wrapping_add(sbcID as core::ffi::c_uint) as rlUInt16_t;
        /* set Command Sub-block length to sizeof command strcuture type */
        (*payloadPtr).len = inLen;
        /* set sub-block data pointer to payload structure */
        (*payloadPtr).pSblkData = data
    };
}
/* * @fn rlReturnVal_t rlDriverExecuteGetApi(rlUInt8_t deviceMap, rlUInt16_t msgId,
*                          rlUInt16_t sbcID, rlUInt8_t *msgData, rlUInt16_t inLen)
*
*   @brief: Construct get message and invoke command.
*   @param[in] deviceMap - Bitmap of devices to send the message
*   @param[in] msgId - message ID of command packet
*   @param[in] sbcID  - sub block ID of command packet
*   @param[out] msgData - message data pointer
*   @param[in] inLen - message payload length
*   @return rlReturnVal_t Success - 0, Failure - Error Code
*
*   Construct get message and invoke command where it waits for response from device.
*/
#[no_mangle]
pub unsafe extern "C" fn rlDriverExecuteGetApi(
    mut deviceMap: rlUInt8_t,
    mut msgId: rlUInt16_t,
    mut sbcID: rlUInt16_t,
    mut msgData: *mut rlUInt8_t,
    mut inLen: rlUInt16_t,
) -> rlReturnVal_t {
    /*LDRA waiver   8 D - DD data flow anomalies found- */
    let mut retVal: rlReturnVal_t = 0;
    /* Initialize Command and Response Sub Blocks */
    /* Initialize in-message structure to zero */
    let mut inMsg: rlDriverMsg_t = {
        let mut init = rlDriverMsg {
            opcode: {
                let mut init = rlDriverOpcode {
                    dir: 0 as core::ffi::c_int as rlUInt8_t,
                    msgType: 0,
                    msgId: 0,
                    nsbc: 0,
                };
                init
            },
            subblocks: 0 as *mut rlPayloadSb_t,
            remChunks: 0,
        };
        init
    };
    /* Initialize out-message structure to zero */
    /* AR_CODE_REVIEW MR:R.2.2 <APPROVED> "Values are used by called function. LDRA Tool Issue" */
    /*LDRA_INSPECTED 105 D */
    let mut outMsg: rlDriverMsg_t = {
        let mut init = rlDriverMsg {
            opcode: {
                let mut init = rlDriverOpcode {
                    dir: 0 as core::ffi::c_int as rlUInt8_t,
                    msgType: 0,
                    msgId: 0,
                    nsbc: 0,
                };
                init
            },
            subblocks: 0 as *mut rlPayloadSb_t,
            remChunks: 0,
        };
        init
    };
    /* Initialize in-payload sub-block structure to zero */
    let mut inPayloadSb: rlPayloadSb_t = {
        let mut init = rlPayloadSb {
            sbid: 0 as core::ffi::c_int as rlUInt16_t,
            len: 0,
            pSblkData: 0 as *mut rlUInt8_t,
        };
        init
    };
    /* Initialize out-payload sub-block structure to zero */
    let mut outPayloadSb: rlPayloadSb_t = {
        let mut init = rlPayloadSb {
            sbid: 0 as core::ffi::c_int as rlUInt16_t,
            len: 0,
            pSblkData: 0 as *mut rlUInt8_t,
        };
        init
    };
    /* Construct command packet */
    /* AR_CODE_REVIEW MR:R.2.2 <APPROVED> "Values are used by called function. LDRA Tool Issue" */
    /*LDRA_INSPECTED 8 D */
    rlDriverConstructInMsg(msgId, &mut inMsg, &mut inPayloadSb);
    /* Fill in-message Payload */
    /* AR_CODE_REVIEW MR:R.2.2 <APPROVED> "Values are used by called function. LDRA Tool Issue" */
    /*LDRA_INSPECTED 105 D */
    rlDriverFillPayload(
        msgId,
        sbcID,
        &mut inPayloadSb,
        0 as *mut rlUInt8_t,
        0 as core::ffi::c_uint as rlUInt16_t,
    );
    /* Construct response packet */
    rlDriverConstructOutMsg(
        1 as core::ffi::c_uint as rlUInt16_t,
        &mut outMsg,
        &mut outPayloadSb,
    );
    /* Fill out-message Payload */
    /* AR_CODE_REVIEW MR:R.2.2 <APPROVED> "Values are used by called function. LDRA Tool Issue" */
    /*LDRA_INSPECTED 105 D */
    rlDriverFillPayload(
        0 as core::ffi::c_uint as rlUInt16_t,
        0 as core::ffi::c_uint as rlUInt16_t,
        &mut outPayloadSb,
        msgData,
        inLen,
    );
    /* Send Command to mmWave Radar Device */
    /* AR_CODE_REVIEW MR:D.4.1,D.4.14 <APPROVED> "Can't be NULL" */
    /*LDRA_INSPECTED 45 D */
    retVal = rlDriverCmdInvoke(deviceMap, inMsg, &mut outMsg);
    return retVal;
}
/* * @fn rlReturnVal_t rlDriverExecuteSetApi(rlUInt8_t deviceMap, rlUInt16_t msgId,
*                                         rlUInt16_t sbcID, rlUInt8_t *msgData, rlUInt16_t inLen)
*
*   @brief: Construct set message and invoke command.
*   @param[in] deviceMap - Bitmap of devices to send the message
*   @param[in] msgId - message ID of command packet
*   @param[in] sbcID  - sub block ID of command packet
*   @param[in] msgData - message data pointer
*   @param[in] inLen - message payload length
*   @return rlReturnVal_t Success - 0, Failure - Error Code
*
*   Construct set message and invoke command where it waits for response from device.
*/
#[no_mangle]
pub unsafe extern "C" fn rlDriverExecuteSetApi(
    mut deviceMap: rlUInt8_t,
    mut msgId: rlUInt16_t,
    mut sbcID: rlUInt16_t,
    mut msgData: *mut rlUInt8_t,
    mut inLen: rlUInt16_t,
) -> rlReturnVal_t {
    /*LDRA waiver   8 D - DD data flow anomalies found- */
    let mut retVal: rlReturnVal_t = 0;
    /* Initialize in-message structure to zero */
    let mut inMsg: rlDriverMsg_t = {
        let mut init = rlDriverMsg {
            opcode: {
                let mut init = rlDriverOpcode {
                    dir: 0 as core::ffi::c_int as rlUInt8_t,
                    msgType: 0,
                    msgId: 0,
                    nsbc: 0,
                };
                init
            },
            subblocks: 0 as *mut rlPayloadSb_t,
            remChunks: 0,
        };
        init
    };
    /* Initialize out-message structure to zero */
    let mut outMsg: rlDriverMsg_t = {
        let mut init = rlDriverMsg {
            opcode: {
                let mut init = rlDriverOpcode {
                    dir: 0 as core::ffi::c_int as rlUInt8_t,
                    msgType: 0,
                    msgId: 0,
                    nsbc: 0,
                };
                init
            },
            subblocks: 0 as *mut rlPayloadSb_t,
            remChunks: 0,
        };
        init
    };
    /* Initialize in-payload sub-block structure to zero */
    let mut inPayloadSb: rlPayloadSb_t = {
        let mut init = rlPayloadSb {
            sbid: 0 as core::ffi::c_int as rlUInt16_t,
            len: 0,
            pSblkData: 0 as *mut rlUInt8_t,
        };
        init
    };
    /* Construct command packet */
    /* AR_CODE_REVIEW MR:R.2.2 <APPROVED> "Values are updated in other function.
    LDRA Tool Issue" */
    /*LDRA_INSPECTED 8 D */
    rlDriverConstructInMsg(msgId, &mut inMsg, &mut inPayloadSb);
    /* Fill in-message Payload */
    /* AR_CODE_REVIEW MR:R.2.2 <APPROVED> "Values are used by called function.
    LDRA Tool Issue" */
    /*LDRA_INSPECTED 105 D */
    rlDriverFillPayload(msgId, sbcID, &mut inPayloadSb, msgData, inLen);
    /* Send Command to mmWave Radar Device */
    /* AR_CODE_REVIEW MR:D.4.1,D.4.14 <APPROVED> "Can't be NULL" */
    /*LDRA_INSPECTED 45 D */
    retVal = rlDriverCmdInvoke(deviceMap, inMsg, &mut outMsg);
    return retVal;
}
/* * @fn rlReturnVal_t rlDriverSetRetryCount(rlUInt8_t retryCnt)
*
*   @brief: Set the retry count for re-sending command
*   @param[in] retryCnt - Retry count [0: no retry, n: no. of retries]
*                         value limit to RL_API_CMD_RETRY_COUNT(3)
*
*   @return rlReturnVal_t Success - 0, Failure - Error Code(-2)
*
*   Set the retry count to global rlDriverData_t structure.
*/
#[no_mangle]
pub unsafe extern "C" fn rlDriverSetRetryCount(mut retryCnt: rlUInt8_t) -> rlReturnVal_t {
    let mut retVal: rlReturnVal_t = 0;
    /* Check if driver is initialized */
    if rl_driverData.isDriverInitialized as core::ffi::c_uint == 1 as core::ffi::c_uint {
        if retryCnt as core::ffi::c_uint <= 3 as core::ffi::c_uint {
            /* set command retry count to global structure */
            rl_driverData.retryCount = retryCnt;
            retVal = 0 as core::ffi::c_int
        } else {
            /* Retry count can be set to max pre-defined value,
             * else return an error
             */
            retVal = -(2 as core::ffi::c_int)
        }
    } else {
        /* if driver is not initialized then return and error value */
        retVal = -(11 as core::ffi::c_int)
    }
    return retVal;
}
/*
 * END OF rl_driver.c FILE
 */
