#![allow(dead_code, mutable_transmutes, non_camel_case_types, non_snake_case,
         non_upper_case_globals, unused_assignments, unused_mut)]
#![register_tool(c2rust)]
#![feature(register_tool)]
extern "C" {
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
#[derive(Copy, Clone)]
#[repr(C)]
pub struct rlMonDigEnables {
    pub enMask: rlUInt32_t,
    pub testMode: rlUInt8_t,
    pub reserved0: rlUInt8_t,
    pub reserved1: rlUInt16_t,
    pub reserved2: rlUInt32_t,
}
pub type rlMonDigEnables_t = rlMonDigEnables;
#[derive(Copy, Clone)]
#[repr(C)]
pub struct rlDigMonPeriodicConf {
    pub reportMode: rlUInt8_t,
    pub reserved0: rlUInt8_t,
    pub reserved1: rlUInt16_t,
    pub periodicEnableMask: rlUInt32_t,
    pub reserved2: rlUInt32_t,
}
pub type rlDigMonPeriodicConf_t = rlDigMonPeriodicConf;
#[derive(Copy, Clone)]
#[repr(C)]
pub struct rlMonAnaEnables {
    pub enMask: rlUInt32_t,
    pub ldoScEn: rlUInt32_t,
}
pub type rlMonAnaEnables_t = rlMonAnaEnables;
#[derive(Copy, Clone)]
#[repr(C)]
pub struct rlTempMonConf {
    pub reportMode: rlUInt8_t,
    pub reserved0: rlUInt8_t,
    pub anaTempThreshMin: rlInt16_t,
    pub anaTempThreshMax: rlInt16_t,
    pub digTempThreshMin: rlInt16_t,
    pub digTempThreshMax: rlInt16_t,
    pub tempDiffThresh: rlUInt16_t,
    pub reserved1: rlUInt32_t,
    pub reserved2: rlUInt32_t,
}
pub type rlTempMonConf_t = rlTempMonConf;
#[derive(Copy, Clone)]
#[repr(C)]
pub struct rlRxGainPhaseMonConf {
    pub profileIndx: rlUInt8_t,
    pub rfFreqBitMask: rlUInt8_t,
    pub reportMode: rlUInt8_t,
    pub txSel: rlUInt8_t,
    pub rxGainAbsThresh: rlUInt16_t,
    pub rxGainMismatchErrThresh: rlUInt16_t,
    pub rxGainFlatnessErrThresh: rlUInt16_t,
    pub rxGainPhaseMismatchErrThresh: rlUInt16_t,
    pub rxGainMismatchOffsetVal: [[rlInt16_t; 3]; 4],
    pub rxGainPhaseMismatchOffsetVal: [[rlUInt16_t; 3]; 4],
    pub reserved0: rlUInt32_t,
    pub reserved1: rlUInt32_t,
}
pub type rlRxGainPhaseMonConf_t = rlRxGainPhaseMonConf;
#[derive(Copy, Clone)]
#[repr(C)]
pub struct rlRxNoiseMonConf {
    pub profileIndx: rlUInt8_t,
    pub rfFreqBitMask: rlUInt8_t,
    pub reserved0: rlUInt16_t,
    pub reportMode: rlUInt8_t,
    pub reserved1: rlUInt8_t,
    pub noiseThresh: rlUInt16_t,
    pub reserved2: rlUInt32_t,
}
pub type rlRxNoiseMonConf_t = rlRxNoiseMonConf;
#[derive(Copy, Clone)]
#[repr(C)]
pub struct rlRxIfStageMonConf {
    pub profileIndx: rlUInt8_t,
    pub reportMode: rlUInt8_t,
    pub reserved0: rlUInt16_t,
    pub reserved1: rlUInt16_t,
    pub hpfCutoffErrThresh: rlUInt16_t,
    pub lpfCutoffErrThresh: rlUInt16_t,
    pub ifaGainErrThresh: rlUInt16_t,
    pub reserved2: rlUInt32_t,
}
pub type rlRxIfStageMonConf_t = rlRxIfStageMonConf;
#[derive(Copy, Clone)]
#[repr(C)]
pub struct rlTxPowMonConf {
    pub profileIndx: rlUInt8_t,
    pub rfFreqBitMask: rlUInt8_t,
    pub reserved0: rlUInt16_t,
    pub reportMode: rlUInt8_t,
    pub reserved1: rlUInt8_t,
    pub txPowAbsErrThresh: rlUInt16_t,
    pub txPowFlatnessErrThresh: rlUInt16_t,
    pub reserved2: rlUInt16_t,
    pub reserved3: rlUInt32_t,
}
pub type rlTxPowMonConf_t = rlTxPowMonConf;
#[derive(Copy, Clone)]
#[repr(C)]
pub struct rlAllTxPowMonConf {
    pub tx0PowrMonCfg: *mut rlTxPowMonConf_t,
    pub tx1PowrMonCfg: *mut rlTxPowMonConf_t,
    pub tx2PowrMonCfg: *mut rlTxPowMonConf_t,
}
pub type rlAllTxPowMonConf_t = rlAllTxPowMonConf;
#[derive(Copy, Clone)]
#[repr(C)]
pub struct rlTxBallbreakMonConf {
    pub reportMode: rlUInt8_t,
    pub reserved0: rlUInt8_t,
    pub txReflCoeffMagThresh: rlInt16_t,
    pub monStartFreqConst: rlUInt32_t,
    pub reserved1: rlUInt32_t,
}
pub type rlTxBallbreakMonConf_t = rlTxBallbreakMonConf;
#[derive(Copy, Clone)]
#[repr(C)]
pub struct rlAllTxBallBreakMonCfg {
    pub tx0BallBrkMonCfg: *mut rlTxBallbreakMonConf_t,
    pub tx1BallBrkMonCfg: *mut rlTxBallbreakMonConf_t,
    pub tx2BallBrkMonCfg: *mut rlTxBallbreakMonConf_t,
}
pub type rlAllTxBallBreakMonCfg_t = rlAllTxBallBreakMonCfg;
#[derive(Copy, Clone)]
#[repr(C)]
pub struct rlTxGainPhaseMismatchMonConf {
    pub profileIndx: rlUInt8_t,
    pub rfFreqBitMask: rlUInt8_t,
    pub txEn: rlUInt8_t,
    pub rxEn: rlUInt8_t,
    pub reportMode: rlUInt8_t,
    pub reserved0: rlUInt8_t,
    pub txGainMismatchThresh: rlInt16_t,
    pub txPhaseMismatchThresh: rlUInt16_t,
    pub txGainMismatchOffsetVal: [[rlUInt16_t; 3]; 3],
    pub txPhaseMismatchOffsetVal: [[rlUInt16_t; 3]; 3],
    pub reserved1: rlUInt16_t,
    pub reserved2: rlUInt32_t,
}
pub type rlTxGainPhaseMismatchMonConf_t = rlTxGainPhaseMismatchMonConf;
#[derive(Copy, Clone)]
#[repr(C)]
pub struct rlTxBpmMonConf {
    pub profileIndx: rlUInt8_t,
    pub phaseShifterMonCnfg: rlUInt8_t,
    pub phaseShifterMon1: rlUInt8_t,
    pub phaseShifterMon2: rlUInt8_t,
    pub reportMode: rlUInt8_t,
    pub rxEn: rlUInt8_t,
    pub txBpmPhaseErrThresh: rlUInt16_t,
    pub txBpmAmplErrThresh: rlUInt16_t,
    pub phaseShifterThreshMax: rlUInt16_t,
    pub phaseShifterThreshMin: rlUInt16_t,
    pub reserved: rlUInt16_t,
}
pub type rlTxBpmMonConf_t = rlTxBpmMonConf;
#[derive(Copy, Clone)]
#[repr(C)]
pub struct rlAllTxBpmMonConf {
    pub tx0BpmMonCfg: *mut rlTxBpmMonConf_t,
    pub tx1BpmMonCfg: *mut rlTxBpmMonConf_t,
    pub tx2BpmMonCfg: *mut rlTxBpmMonConf_t,
}
pub type rlAllTxBpmMonConf_t = rlAllTxBpmMonConf;
#[derive(Copy, Clone)]
#[repr(C)]
pub struct rlSynthFreqMonConf {
    pub profileIndx: rlUInt8_t,
    pub reportMode: rlUInt8_t,
    pub freqErrThresh: rlUInt16_t,
    pub monStartTime: rlInt8_t,
    pub monitorMode: rlUInt8_t,
    pub vcoMonEn: rlUInt8_t,
    pub reserved1: rlUInt8_t,
    pub reserved2: rlUInt32_t,
}
pub type rlSynthFreqMonConf_t = rlSynthFreqMonConf;
#[derive(Copy, Clone)]
#[repr(C)]
pub struct rlExtAnaSignalsMonConf {
    pub reportMode: rlUInt8_t,
    pub reserved0: rlUInt8_t,
    pub signalInpEnables: rlUInt8_t,
    pub signalBuffEnables: rlUInt8_t,
    pub signalSettlingTime: [rlUInt8_t; 6],
    pub signalThresh: [rlUInt8_t; 12],
    pub reserved1: rlUInt16_t,
    pub reserved2: rlUInt32_t,
    pub reserved3: rlUInt32_t,
}
pub type rlExtAnaSignalsMonConf_t = rlExtAnaSignalsMonConf;
#[derive(Copy, Clone)]
#[repr(C)]
pub struct rlTxIntAnaSignalsMonConf {
    pub profileIndx: rlUInt8_t,
    pub reportMode: rlUInt8_t,
    pub txPhShiftDacMonThresh: rlUInt16_t,
    pub reserved1: rlUInt32_t,
}
pub type rlTxIntAnaSignalsMonConf_t = rlTxIntAnaSignalsMonConf;
#[derive(Copy, Clone)]
#[repr(C)]
pub struct rlAllTxIntAnaSignalsMonConf {
    pub tx0IntAnaSgnlMonCfg: *mut rlTxIntAnaSignalsMonConf_t,
    pub tx1IntAnaSgnlMonCfg: *mut rlTxIntAnaSignalsMonConf_t,
    pub tx2IntAnaSgnlMonCfg: *mut rlTxIntAnaSignalsMonConf_t,
}
pub type rlAllTxIntAnaSignalsMonConf_t = rlAllTxIntAnaSignalsMonConf;
#[derive(Copy, Clone)]
#[repr(C)]
pub struct rlRxIntAnaSignalsMonConf {
    pub profileIndx: rlUInt8_t,
    pub reportMode: rlUInt8_t,
    pub reserved0: rlUInt16_t,
    pub reserved1: rlUInt32_t,
}
pub type rlRxIntAnaSignalsMonConf_t = rlRxIntAnaSignalsMonConf;
#[derive(Copy, Clone)]
#[repr(C)]
pub struct rlPmClkLoIntAnaSignalsMonConf {
    pub profileIndx: rlUInt8_t,
    pub reportMode: rlUInt8_t,
    pub sync20GSigSel: rlUInt8_t,
    pub sync20GMinThresh: rlInt8_t,
    pub sync20GMaxThresh: rlInt8_t,
    pub reserved0: rlUInt8_t,
    pub reserved1: rlUInt16_t,
}
pub type rlPmClkLoIntAnaSignalsMonConf_t = rlPmClkLoIntAnaSignalsMonConf;
#[derive(Copy, Clone)]
#[repr(C)]
pub struct rlGpadcIntAnaSignalsMonConf {
    pub reportMode: rlUInt8_t,
    pub reserved0: rlUInt8_t,
    pub reserved1: rlUInt16_t,
    pub reserved2: rlUInt32_t,
}
pub type rlGpadcIntAnaSignalsMonConf_t = rlGpadcIntAnaSignalsMonConf;
#[derive(Copy, Clone)]
#[repr(C)]
pub struct rlPllContrlVoltMonConf {
    pub reportMode: rlUInt8_t,
    pub reserved0: rlUInt8_t,
    pub signalEnables: rlUInt16_t,
    pub reserved1: rlUInt32_t,
}
pub type rlPllContrVoltMonConf_t = rlPllContrlVoltMonConf;
#[derive(Copy, Clone)]
#[repr(C)]
pub struct rlDualClkCompMonConf {
    pub reportMode: rlUInt8_t,
    pub reserved0: rlUInt8_t,
    pub dccPairEnables: rlUInt16_t,
    pub reserved1: rlUInt32_t,
}
pub type rlDualClkCompMonConf_t = rlDualClkCompMonConf;
#[derive(Copy, Clone)]
#[repr(C)]
pub struct rlRxSatMonConf {
    pub profileIndx: rlUInt8_t,
    pub satMonSel: rlUInt8_t,
    pub reserved0: rlUInt16_t,
    pub primarySliceDuration: rlUInt16_t,
    pub numSlices: rlUInt16_t,
    pub rxChannelMask: rlUInt8_t,
    pub reserved1: rlUInt8_t,
    pub reserved2: rlUInt16_t,
    pub reserved3: rlUInt32_t,
    pub reserved4: rlUInt32_t,
}
pub type rlRxSatMonConf_t = rlRxSatMonConf;
#[derive(Copy, Clone)]
#[repr(C)]
pub struct rlSigImgMonConf {
    pub profileIndx: rlUInt8_t,
    pub numSlices: rlUInt8_t,
    pub timeSliceNumSamples: rlUInt16_t,
    pub reserved0: rlUInt32_t,
    pub reserved1: rlUInt32_t,
}
pub type rlSigImgMonConf_t = rlSigImgMonConf;
#[derive(Copy, Clone)]
#[repr(C)]
pub struct rlRxMixInPwrMonConf {
    pub profileIndx: rlUInt8_t,
    pub reportMode: rlUInt8_t,
    pub txEnable: rlUInt8_t,
    pub reserved0: rlUInt8_t,
    pub thresholds: rlUInt16_t,
    pub reserved1: rlUInt16_t,
    pub reserved2: rlUInt32_t,
}
pub type rlRxMixInPwrMonConf_t = rlRxMixInPwrMonConf;
#[derive(Copy, Clone)]
#[repr(C)]
pub struct rlAnaFaultInj {
    pub reserved0: rlUInt8_t,
    pub rxGainDrop: rlUInt8_t,
    pub rxPhInv: rlUInt8_t,
    pub rxHighNoise: rlUInt8_t,
    pub rxIfStagesFault: rlUInt8_t,
    pub rxLoAmpFault: rlUInt8_t,
    pub txLoAmpFault: rlUInt8_t,
    pub txGainDrop: rlUInt8_t,
    pub txPhInv: rlUInt8_t,
    pub synthFault: rlUInt8_t,
    pub supplyLdoFault: rlUInt8_t,
    pub miscFault: rlUInt8_t,
    pub miscThreshFault: rlUInt8_t,
    pub reserved1: rlUInt8_t,
    pub reserved2: rlUInt16_t,
    pub reserved3: rlUInt16_t,
    pub reserved4: rlUInt16_t,
}
pub type rlAnaFaultInj_t = rlAnaFaultInj;
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
 * FileName     : rl_monitoring.c
 *
 * Description  : This file defines the functions required to configure monitoring reports from
 * mmWave Device.
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
 *  SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, data, OR PROFITS; OR BUSINESS
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
* 0.8.6    24Jul2017   Jitendra Gupta    MMWL-30         RF/Analog Monitoring APIs
*                      Kaushal Kukkar    MMWL-23         Big Endian Support
*                      Kaushal Kukkar    MMWL-13         Fault Injection API
*
* 1.2.2.4  20Feb2019   Pavan Penikalapati MMWL162        Fixed comment for rlRfTxPowrMonConfig
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
/* * @fn rlReturnVal_t rlRfDigMonEnableConfig(rlUInt8_t deviceMap, rlMonDigEnables_t* data)
*
*   @brief Sets the consolidated configuration of all digital monitoring
*   @param[in] deviceMap - Bitmap of devices to send the message
*   @param[in] data - Monitor digital enable configuration
*
*   @return rlReturnVal_t Success - 0, Failure - Error Code
*
*   This API SB sets the consolidated configuration of all digital monitoring. This API should
*   be issued only when frames are not running, these are destructive tests. The scheduling of
*   these monitoring should be handled in the external application. Report of these monitoring
*   will be available in the async event RL_RF_AE_DIG_LATENTFAULT_REPORT_AE_SB.
*
*   @note 1: This API is not supported in IWR6843 ES1.0 and xWR1443 ES3.0
*/
/* DesignId : MMWL_DesignId_073 */
/* Requirements : AUTORADAR_REQ-768 */
#[no_mangle]
pub unsafe extern "C" fn rlRfDigMonEnableConfig(mut deviceMap: rlUInt8_t,
                                                mut data:
                                                    *mut rlMonDigEnables_t)
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
                                  0xe as libc::c_uint as rlUInt16_t,
                                  0 as libc::c_uint as rlUInt16_t,
                                  data as *mut rlUInt8_t,
                                  ::std::mem::size_of::<rlMonDigEnables_t>()
                                      as libc::c_ulong as rlUInt16_t)
    }
    return retVal;
}
/* * @fn rlReturnVal_t rlRfDigMonPeriodicConfig(rlUInt8_t deviceMap, rlDigMonPeriodicConf_t* data)
*
*   @brief Sets the consolidated configuration
*   @param[in] deviceMap - Bitmap of devices to send the message
*   @param[in] data - Digital monitor periodic configuration
*
*   @return rlReturnVal_t Success - 0, Failure - Error Code
*
*   Sets the consolidated configuration of all periodic digital monitoring within
*   radar sub-system.
*
*   @note 1: This API is not supported in IWR6843 ES1.0 and xWR1443 ES3.0
*/
/* DesignId : MMWL_DesignId_074 */
/* Requirements : AUTORADAR_REQ-768 */
#[no_mangle]
pub unsafe extern "C" fn rlRfDigMonPeriodicConfig(mut deviceMap: rlUInt8_t,
                                                  mut data:
                                                      *mut rlDigMonPeriodicConf_t)
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
                                  0xe as libc::c_uint as rlUInt16_t,
                                  0x1 as libc::c_uint as rlUInt16_t,
                                  data as *mut rlUInt8_t,
                                  ::std::mem::size_of::<rlDigMonPeriodicConf_t>()
                                      as libc::c_ulong as rlUInt16_t)
    }
    return retVal;
}
/* * @fn rlReturnVal_t rlRfAnaMonConfig(rlUInt8_t deviceMap, rlMonAnaEnables_t* data)
*
*   @brief This function contains the consolidated configuration of all analog
*   monitoring. The enabled monitoring functions are executed with a periodicity
*   of CAL_MON_TIME_UNITS (rlRfCalMonTimeUntConf_t.calibMonTimeUnit) number
*   of logical frames. The host should ensure that all the enabled monitors
*   can be completed in the available inter-frame times, based on the monitoring
*   durations. The time taken for each monitoring is not defined in this document.
*
*   @param[in] deviceMap - Bitmap of devices to send the message
*   @param[in] data - Monitor analog enable configuration
*
*   @return rlReturnVal_t Success - 0, Failure - Error Code
*
*   Sets the consolidated configuration of all analog monitoring.
*
*   @note 1: This API is not supported in IWR6843 ES1.0 \n
*   @note 2: None of the Safety Monitoring supported in QM devices except Rx saturation and 
*            signal image monitor, The monitoring configurations defined below from sub-block ID 
*            0x01C0  to 0x01DF are not valid in QM devices. \n
*   @note 3: All Monitoring configurations and enable control APIs shall be issues before
*            triggering the frames. The run time programming or configuration update for monitors
*            are not supported while frames are running.
*/
/* DesignId : MMWL_DesignId_075 */
/* Requirements : AUTORADAR_REQ-769, AUTORADAR_REQ-602 */
#[no_mangle]
pub unsafe extern "C" fn rlRfAnaMonConfig(mut deviceMap: rlUInt8_t,
                                          mut data: *mut rlMonAnaEnables_t)
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
                                  0xe as libc::c_uint as rlUInt16_t,
                                  0x2 as libc::c_uint as rlUInt16_t,
                                  data as *mut rlUInt8_t,
                                  ::std::mem::size_of::<rlMonAnaEnables_t>()
                                      as libc::c_ulong as rlUInt16_t)
    }
    return retVal;
}
/* * @fn rlReturnVal_t rlRfTempMonConfig(rlUInt8_t deviceMap, rlTempMonConf_t* data)
*
*   @brief  This API configure the on chip temperature monitors and report the
*   soft results from the monitor. The corresponding monitors are collectively
*   named TEMPERATURE_MONITOR. These monitors observe the temperature near
*   various RF analog and digital modules using temperature sensors and GPADC
*   and compare them against configurable thresholds. The report is sent as an
*   async event RL_RF_AE_MON_TEMPERATURE_REPORT_SB. Sets information related to
*   temperature monitoring
*   @param[in] deviceMap - Bitmap of devices to send the message
*   @param[in] data - Temperature monitor configuration
*
*   @return rlReturnVal_t Success - 0, Failure - Error Code
*
*   Sets information related to temperature monitoring.
*   Once configured, radarSS will send Async event (RL_RF_AE_MON_TEMPERATURE_REPORT_SB) to
*   report monitoring data
*
*   @note : In xWR6x43 devices, if digTempThreshMin and digTempThreshMax are both are set to zero, 
*           the device would not perform minimum or maximum threshold checks for the digital 
*           sensors. It would also ignore the digital sensors while computing the temperature
*           difference across the enabled temperature sensors to compare against the programmed 
*           threshold value in tempDiffThresh.
*/
/* DesignId : MMWL_DesignId_076 */
/* Requirements : AUTORADAR_REQ-857 */
#[no_mangle]
pub unsafe extern "C" fn rlRfTempMonConfig(mut deviceMap: rlUInt8_t,
                                           mut data: *mut rlTempMonConf_t)
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
                                  0xe as libc::c_uint as rlUInt16_t,
                                  0x3 as libc::c_uint as rlUInt16_t,
                                  data as *mut rlUInt8_t,
                                  ::std::mem::size_of::<rlTempMonConf_t>() as
                                      libc::c_ulong as rlUInt16_t)
    }
    return retVal;
}
/* * @fn rlReturnVal_t rlRfRxGainPhMonConfig(rlUInt8_t deviceMap, rlRxGainPhaseMonConf_t* data)
*
*   @brief  This API is to set RX gain and phase monitoring config to device.
*   @param[in] deviceMap - Bitmap of devices to send the message
*   @param[in] data -  Rx gain phase monitoring configuration
*
*   @return rlReturnVal_t Success - 0, Failure - Error Code
*
*   This is a monitoring configuration API, containing information related to RX gain and phase
*   monitoring. Once configured, radarSS will send Async event (RL_RF_AE_MON_RX_GAIN_PHASE_REPORT)
*   to report monitoring data.
*
*   @note 1: This API is not supported in IWR6843 ES1.0
*   @note 2: It is recommended for the user to configure this monitor in verbose mode (Mode 0),
*            so that Host can compute actual RX gain through temperature compensation and detect
*            presence of interference using Noise Power.
*   @note 3: In quiet mode, the user may consider programming broad thresholds for Absolute Gain
*            Error, taking into account the temperature variation of reported RX gain value.

*/
/* DesignId : MMWL_DesignId_077 */
/* Requirements : AUTORADAR_REQ-858, AUTORADAR_REQ-1041 */
#[no_mangle]
pub unsafe extern "C" fn rlRfRxGainPhMonConfig(mut deviceMap: rlUInt8_t,
                                               mut data:
                                                   *mut rlRxGainPhaseMonConf_t)
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
                                  0xe as libc::c_uint as rlUInt16_t,
                                  0x4 as libc::c_uint as rlUInt16_t,
                                  data as *mut rlUInt8_t,
                                  ::std::mem::size_of::<rlRxGainPhaseMonConf_t>()
                                      as libc::c_ulong as rlUInt16_t)
    }
    return retVal;
}
/* * @fn rlReturnVal_t rlRfRxNoiseMonConfig(rlUInt8_t deviceMap, rlRxNoiseMonConf_t* data)
*
*   @brief Sets information related to RX noise monitoring.
*   @param[in] deviceMap - Bitmap of devices to send the message
*   @param[in] data - Rx noise monitor configuration
*
*   @return rlReturnVal_t Success - 0, Failure - Error Code
*
*   This is a monitoring configuration API, containing
*   information related to RX noise monitoring of a profile.
*   Once configured, radarSS will send Async event (RL_RF_AE_MON_RX_NOISE_FIG_REPORT) to
*   report monitoring data.
*
*   @note 1: The noise monitor reports the real baseband receivers noise figure with LNA disabled.
*            In complex receiver modes (i.e., complex 1x, complex 2x and pseudo real), the system
*            noise figure is 3dB lower (better) than the reported number.
*   @note 2: This API is not supported in IWR6843 ES1.0
*   @note 3: The Rx gain and phase monitoring shall be enabled when enabling Rx noise figure
*            Monitoring. This monitors only baseband noise figure.
*   @note 4: The RX Noise figure monitor API is not supported in this release. Please refer latest
*            DFP release note for more info.
*/
/* DesignId : MMWL_DesignId_078 */
/* Requirements : AUTORADAR_REQ-859*/
#[no_mangle]
pub unsafe extern "C" fn rlRfRxNoiseMonConfig(mut deviceMap: rlUInt8_t,
                                              mut data:
                                                  *mut rlRxNoiseMonConf_t)
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
                                  0xe as libc::c_uint as rlUInt16_t,
                                  0x5 as libc::c_uint as rlUInt16_t,
                                  data as *mut rlUInt8_t,
                                  ::std::mem::size_of::<rlRxNoiseMonConf_t>()
                                      as libc::c_ulong as rlUInt16_t)
    }
    return retVal;
}
/* * @fn rlReturnVal_t rlRfRxIfStageMonConfig(rlUInt8_t deviceMap, rlRxIfStageMonConf_t* data)
*
*   @brief Sets information related to RX IF filter attenuation monitoring
*   @param[in] deviceMap - Bitmap of devices to send the message
*   @param[in] data - Rx IF stage monitoring configuration
*
*   @return rlReturnVal_t Success - 0, Failure - Error Code
*
*   This API is a Monitoring Configuration API, containing information
*   related to RX IF filter attenuation monitoring.
*   Once configured, radarSS will send Async event (RL_RF_AE_MON_RX_IF_STAGE_REPORT) to
*   report monitoring data
*
*   @note : This API is not supported in IWR6843 ES1.0
*/
/* DesignId : MMWL_DesignId_079 */
/* Requirements : AUTORADAR_REQ-860 */
#[no_mangle]
pub unsafe extern "C" fn rlRfRxIfStageMonConfig(mut deviceMap: rlUInt8_t,
                                                mut data:
                                                    *mut rlRxIfStageMonConf_t)
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
                                  0xe as libc::c_uint as rlUInt16_t,
                                  0x6 as libc::c_uint as rlUInt16_t,
                                  data as *mut rlUInt8_t,
                                  ::std::mem::size_of::<rlRxIfStageMonConf_t>()
                                      as libc::c_ulong as rlUInt16_t)
    }
    return retVal;
}
/* * @fn rlReturnVal_t rlRfTxPowrMonConfig(rlUInt8_t deviceMap, rlAllTxPowMonConf_t* data)
*
*   @brief Sets information related to TX power monitoring.
*   @param[in] deviceMap - Bitmap of devices to send the message
*   @param[in] data - Tx power monitor configuration
*
*   @return rlReturnVal_t Success - 0, Failure - Error Code
*
*   This API is a Monitoring Configuration API, containing information
*   related to TX0/1/2 power monitoring.
*   Once configured, radarSS will send Async event (RL_RF_AE_MON_TX0_POWER_REPORT,
*   RL_RF_AE_MON_TX1_POWER_REPORT, RL_RF_AE_MON_TX2_POWER_REPORT) to report monitoring data
*
*   @note 1: The TX[0:2] power monitoring accuracy degrades at high TX backoffs and is unreliable
*            for backoffs higher than 20dB. \n
*   @note 2: This API is not supported in IWR6843 ES1.0
*/
/* DesignId : MMWL_DesignId_080 */
/* Requirements : AUTORADAR_REQ-861 */
#[no_mangle]
pub unsafe extern "C" fn rlRfTxPowrMonConfig(mut deviceMap: rlUInt8_t,
                                             mut data:
                                                 *mut rlAllTxPowMonConf_t)
 -> rlReturnVal_t {
    let mut retVal: rlReturnVal_t = 0;
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
                                                           0 as libc::c_int as
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
    let mut numSbc: rlUInt16_t = 0 as libc::c_uint as rlUInt16_t;
    /* Initialize Command and Response Sub Blocks */
    let mut inPayloadSb: [rlPayloadSb_t; 32] =
        [{
             let mut init =
                 rlPayloadSb{sbid: 0 as libc::c_int as rlUInt16_t,
                             len: 0,
                             pSblkData: 0 as *mut rlUInt8_t,};
             init
         }, rlPayloadSb_t{sbid: 0, len: 0, pSblkData: 0 as *mut rlUInt8_t,},
         rlPayloadSb_t{sbid: 0, len: 0, pSblkData: 0 as *mut rlUInt8_t,},
         rlPayloadSb_t{sbid: 0, len: 0, pSblkData: 0 as *mut rlUInt8_t,},
         rlPayloadSb_t{sbid: 0, len: 0, pSblkData: 0 as *mut rlUInt8_t,},
         rlPayloadSb_t{sbid: 0, len: 0, pSblkData: 0 as *mut rlUInt8_t,},
         rlPayloadSb_t{sbid: 0, len: 0, pSblkData: 0 as *mut rlUInt8_t,},
         rlPayloadSb_t{sbid: 0, len: 0, pSblkData: 0 as *mut rlUInt8_t,},
         rlPayloadSb_t{sbid: 0, len: 0, pSblkData: 0 as *mut rlUInt8_t,},
         rlPayloadSb_t{sbid: 0, len: 0, pSblkData: 0 as *mut rlUInt8_t,},
         rlPayloadSb_t{sbid: 0, len: 0, pSblkData: 0 as *mut rlUInt8_t,},
         rlPayloadSb_t{sbid: 0, len: 0, pSblkData: 0 as *mut rlUInt8_t,},
         rlPayloadSb_t{sbid: 0, len: 0, pSblkData: 0 as *mut rlUInt8_t,},
         rlPayloadSb_t{sbid: 0, len: 0, pSblkData: 0 as *mut rlUInt8_t,},
         rlPayloadSb_t{sbid: 0, len: 0, pSblkData: 0 as *mut rlUInt8_t,},
         rlPayloadSb_t{sbid: 0, len: 0, pSblkData: 0 as *mut rlUInt8_t,},
         rlPayloadSb_t{sbid: 0, len: 0, pSblkData: 0 as *mut rlUInt8_t,},
         rlPayloadSb_t{sbid: 0, len: 0, pSblkData: 0 as *mut rlUInt8_t,},
         rlPayloadSb_t{sbid: 0, len: 0, pSblkData: 0 as *mut rlUInt8_t,},
         rlPayloadSb_t{sbid: 0, len: 0, pSblkData: 0 as *mut rlUInt8_t,},
         rlPayloadSb_t{sbid: 0, len: 0, pSblkData: 0 as *mut rlUInt8_t,},
         rlPayloadSb_t{sbid: 0, len: 0, pSblkData: 0 as *mut rlUInt8_t,},
         rlPayloadSb_t{sbid: 0, len: 0, pSblkData: 0 as *mut rlUInt8_t,},
         rlPayloadSb_t{sbid: 0, len: 0, pSblkData: 0 as *mut rlUInt8_t,},
         rlPayloadSb_t{sbid: 0, len: 0, pSblkData: 0 as *mut rlUInt8_t,},
         rlPayloadSb_t{sbid: 0, len: 0, pSblkData: 0 as *mut rlUInt8_t,},
         rlPayloadSb_t{sbid: 0, len: 0, pSblkData: 0 as *mut rlUInt8_t,},
         rlPayloadSb_t{sbid: 0, len: 0, pSblkData: 0 as *mut rlUInt8_t,},
         rlPayloadSb_t{sbid: 0, len: 0, pSblkData: 0 as *mut rlUInt8_t,},
         rlPayloadSb_t{sbid: 0, len: 0, pSblkData: 0 as *mut rlUInt8_t,},
         rlPayloadSb_t{sbid: 0, len: 0, pSblkData: 0 as *mut rlUInt8_t,},
         rlPayloadSb_t{sbid: 0, len: 0, pSblkData: 0 as *mut rlUInt8_t,}];
    /* Construct command packet */
    rlDriverConstructInMsg(0xe as libc::c_uint as rlUInt16_t, &mut inMsg,
                           &mut *inPayloadSb.as_mut_ptr().offset(0 as
                                                                     libc::c_uint
                                                                     as
                                                                     isize));
    /* check if deviceIndex is out of defined value */
    if rlDriverIsDeviceMapValid(deviceMap) != 0 as libc::c_int ||
           data.is_null() {
        retVal = -(2 as libc::c_int)
    } else {
        /* check for NULL pointer */
        if !(*data).tx0PowrMonCfg.is_null() {
            /* Fill in-message Payload */
            rlDriverFillPayload(0xe as libc::c_uint as rlUInt16_t,
                                0x7 as libc::c_uint as rlUInt16_t,
                                &mut *inPayloadSb.as_mut_ptr().offset(numSbc
                                                                          as
                                                                          isize),
                                (*data).tx0PowrMonCfg as *mut rlUInt8_t,
                                ::std::mem::size_of::<rlTxPowMonConf_t>() as
                                    libc::c_ulong as rlUInt16_t);
            numSbc = numSbc.wrapping_add(1)
        }
        /* check for NULL pointer */
        if !(*data).tx1PowrMonCfg.is_null() {
            /* Fill in-message Payload */
            rlDriverFillPayload(0xe as libc::c_uint as rlUInt16_t,
                                0x8 as libc::c_uint as rlUInt16_t,
                                &mut *inPayloadSb.as_mut_ptr().offset(numSbc
                                                                          as
                                                                          isize),
                                (*data).tx1PowrMonCfg as *mut rlUInt8_t,
                                ::std::mem::size_of::<rlTxPowMonConf_t>() as
                                    libc::c_ulong as rlUInt16_t);
            numSbc = numSbc.wrapping_add(1)
        }
        /* check for NULL pointer */
        if !(*data).tx2PowrMonCfg.is_null() {
            /* Fill in-message Payload */
            rlDriverFillPayload(0xe as libc::c_uint as rlUInt16_t,
                                0x9 as libc::c_uint as rlUInt16_t,
                                &mut *inPayloadSb.as_mut_ptr().offset(numSbc
                                                                          as
                                                                          isize),
                                (*data).tx2PowrMonCfg as *mut rlUInt8_t,
                                ::std::mem::size_of::<rlTxPowMonConf_t>() as
                                    libc::c_ulong as rlUInt16_t);
            numSbc = numSbc.wrapping_add(1)
        }
        if numSbc as libc::c_uint > 0 as libc::c_uint {
            /* setting num of sub-block to inMsg */
            inMsg.opcode.nsbc = numSbc;
            /* Send Command to mmWave Radar Device */
            retVal = rlDriverCmdInvoke(deviceMap, inMsg, &mut outMsg)
        } else { retVal = -(2 as libc::c_int) }
    }
    return retVal;
}
/* * @fn rlReturnVal_t rlRfTxBallbreakMonConfig(rlUInt8_t deviceMap, rlAllTxBallBreakMonCfg_t* data)
*
*   @brief Sets information related to TX ball break detection.
*   @param[in] deviceMap - Bitmap of devices to send the message
*   @param[in] data - Tx ballbreak monitor config
*
*   @return rlReturnVal_t Success - 0, Failure - Error Code
*
*   This API is a Monitoring Configuration API, containing information
*   related to TX ball break detection.
*   Once configured, radarSS will send Async event (RL_RF_AE_MON_TX0_BALLBREAK_REPORT,
*   RL_RF_AE_MON_TX1_BALLBREAK_REPORT, RL_RF_AE_MON_TX2_BALLBREAK_REPORT) to report monitoring data
*
*   @note : This API is not supported in IWR6843 ES1.0
*/
/* DesignId : MMWL_DesignId_081 */
/* Requirements : AUTORADAR_REQ-862 */
#[no_mangle]
pub unsafe extern "C" fn rlRfTxBallbreakMonConfig(mut deviceMap: rlUInt8_t,
                                                  mut data:
                                                      *mut rlAllTxBallBreakMonCfg_t)
 -> rlReturnVal_t {
    let mut retVal: rlReturnVal_t = 0;
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
                                                           0 as libc::c_int as
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
    let mut numSbc: rlUInt16_t = 0 as libc::c_uint as rlUInt16_t;
    /* Initialize Command and Response Sub Blocks */
    let mut inPayloadSb: [rlPayloadSb_t; 32] =
        [{
             let mut init =
                 rlPayloadSb{sbid: 0 as libc::c_int as rlUInt16_t,
                             len: 0,
                             pSblkData: 0 as *mut rlUInt8_t,};
             init
         }, rlPayloadSb_t{sbid: 0, len: 0, pSblkData: 0 as *mut rlUInt8_t,},
         rlPayloadSb_t{sbid: 0, len: 0, pSblkData: 0 as *mut rlUInt8_t,},
         rlPayloadSb_t{sbid: 0, len: 0, pSblkData: 0 as *mut rlUInt8_t,},
         rlPayloadSb_t{sbid: 0, len: 0, pSblkData: 0 as *mut rlUInt8_t,},
         rlPayloadSb_t{sbid: 0, len: 0, pSblkData: 0 as *mut rlUInt8_t,},
         rlPayloadSb_t{sbid: 0, len: 0, pSblkData: 0 as *mut rlUInt8_t,},
         rlPayloadSb_t{sbid: 0, len: 0, pSblkData: 0 as *mut rlUInt8_t,},
         rlPayloadSb_t{sbid: 0, len: 0, pSblkData: 0 as *mut rlUInt8_t,},
         rlPayloadSb_t{sbid: 0, len: 0, pSblkData: 0 as *mut rlUInt8_t,},
         rlPayloadSb_t{sbid: 0, len: 0, pSblkData: 0 as *mut rlUInt8_t,},
         rlPayloadSb_t{sbid: 0, len: 0, pSblkData: 0 as *mut rlUInt8_t,},
         rlPayloadSb_t{sbid: 0, len: 0, pSblkData: 0 as *mut rlUInt8_t,},
         rlPayloadSb_t{sbid: 0, len: 0, pSblkData: 0 as *mut rlUInt8_t,},
         rlPayloadSb_t{sbid: 0, len: 0, pSblkData: 0 as *mut rlUInt8_t,},
         rlPayloadSb_t{sbid: 0, len: 0, pSblkData: 0 as *mut rlUInt8_t,},
         rlPayloadSb_t{sbid: 0, len: 0, pSblkData: 0 as *mut rlUInt8_t,},
         rlPayloadSb_t{sbid: 0, len: 0, pSblkData: 0 as *mut rlUInt8_t,},
         rlPayloadSb_t{sbid: 0, len: 0, pSblkData: 0 as *mut rlUInt8_t,},
         rlPayloadSb_t{sbid: 0, len: 0, pSblkData: 0 as *mut rlUInt8_t,},
         rlPayloadSb_t{sbid: 0, len: 0, pSblkData: 0 as *mut rlUInt8_t,},
         rlPayloadSb_t{sbid: 0, len: 0, pSblkData: 0 as *mut rlUInt8_t,},
         rlPayloadSb_t{sbid: 0, len: 0, pSblkData: 0 as *mut rlUInt8_t,},
         rlPayloadSb_t{sbid: 0, len: 0, pSblkData: 0 as *mut rlUInt8_t,},
         rlPayloadSb_t{sbid: 0, len: 0, pSblkData: 0 as *mut rlUInt8_t,},
         rlPayloadSb_t{sbid: 0, len: 0, pSblkData: 0 as *mut rlUInt8_t,},
         rlPayloadSb_t{sbid: 0, len: 0, pSblkData: 0 as *mut rlUInt8_t,},
         rlPayloadSb_t{sbid: 0, len: 0, pSblkData: 0 as *mut rlUInt8_t,},
         rlPayloadSb_t{sbid: 0, len: 0, pSblkData: 0 as *mut rlUInt8_t,},
         rlPayloadSb_t{sbid: 0, len: 0, pSblkData: 0 as *mut rlUInt8_t,},
         rlPayloadSb_t{sbid: 0, len: 0, pSblkData: 0 as *mut rlUInt8_t,},
         rlPayloadSb_t{sbid: 0, len: 0, pSblkData: 0 as *mut rlUInt8_t,}];
    /* Construct command packet */
    rlDriverConstructInMsg(0xe as libc::c_uint as rlUInt16_t, &mut inMsg,
                           &mut *inPayloadSb.as_mut_ptr().offset(0 as
                                                                     libc::c_uint
                                                                     as
                                                                     isize));
    /* check if deviceIndex is out of defined value */
    if rlDriverIsDeviceMapValid(deviceMap) != 0 as libc::c_int ||
           data.is_null() {
        retVal = -(2 as libc::c_int)
    } else {
        /* check for NULL pointer */
        if !(*data).tx0BallBrkMonCfg.is_null() {
            /* Fill in-message Payload */
            rlDriverFillPayload(0xe as libc::c_uint as rlUInt16_t,
                                0xa as libc::c_uint as rlUInt16_t,
                                &mut *inPayloadSb.as_mut_ptr().offset(numSbc
                                                                          as
                                                                          isize),
                                (*data).tx0BallBrkMonCfg as *mut rlUInt8_t,
                                ::std::mem::size_of::<rlTxBallbreakMonConf_t>()
                                    as libc::c_ulong as rlUInt16_t);
            numSbc = numSbc.wrapping_add(1)
        }
        /* check for NULL pointer */
        if !(*data).tx1BallBrkMonCfg.is_null() {
            /* Fill in-message Payload */
            rlDriverFillPayload(0xe as libc::c_uint as rlUInt16_t,
                                0xb as libc::c_uint as rlUInt16_t,
                                &mut *inPayloadSb.as_mut_ptr().offset(numSbc
                                                                          as
                                                                          isize),
                                (*data).tx1BallBrkMonCfg as *mut rlUInt8_t,
                                ::std::mem::size_of::<rlTxBallbreakMonConf_t>()
                                    as libc::c_ulong as rlUInt16_t);
            numSbc = numSbc.wrapping_add(1)
        }
        /* check for NULL pointer */
        if !(*data).tx2BallBrkMonCfg.is_null() {
            /* Fill in-message Payload */
            rlDriverFillPayload(0xe as libc::c_uint as rlUInt16_t,
                                0xc as libc::c_uint as rlUInt16_t,
                                &mut *inPayloadSb.as_mut_ptr().offset(numSbc
                                                                          as
                                                                          isize),
                                (*data).tx2BallBrkMonCfg as *mut rlUInt8_t,
                                ::std::mem::size_of::<rlTxBallbreakMonConf_t>()
                                    as libc::c_ulong as rlUInt16_t);
            numSbc = numSbc.wrapping_add(1)
        }
        if numSbc as libc::c_uint > 0 as libc::c_uint {
            /* setting num of sub-block to inMsg */
            inMsg.opcode.nsbc = numSbc;
            /* Send Command to mmWave Radar Device */
            retVal = rlDriverCmdInvoke(deviceMap, inMsg, &mut outMsg)
        } else { retVal = -(2 as libc::c_int) }
    }
    return retVal;
}
/* * @fn rlReturnVal_t rlRfTxGainPhaseMismatchMonConfig(rlUInt8_t deviceMap,
                                                        rlTxGainPhaseMismatchMonConf_t* data)
*
*   @brief Sets information related to TX gain and phase mismatch monitoring
*   @param[in] deviceMap - Bitmap of devices to send the message
*   @param[in] data - Tx gain phase mismatch monitor configuration
*
*   @return rlReturnVal_t Success - 0, Failure - Error Code
*
*   This API is a Monitoring Configuration API, containing information
*   related to TX gain and phase mismatch monitoring.
*   Once configured, radarSS will send Async event (RL_RF_AE_MON_TX_GAIN_MISMATCH_REPORT) to
*   report monitoring data
*
*   @note 1: Even when the TXs are matched, TX3 loopback path has gain and phase offsets wrt TX1
*            (and TX2), which get reported as mismatches in this API. These deterministic offsets
*            can be compensated either through the OFFSET_VALUE fields (quiet mode) or through
*            post processing by the host (verbose mode). \n Nominally, when the TXs are matched,
*            TX3 - TX1 gain (i.e. loopback amplitude) is reported as -8dB. Nominally, when the TXs
*            are matched, the reported TX3 - TX1 phase difference varies linearly with RF and it
*            is reported as -5degree (76GHz) and 15degree (81GHz).
*   @note 2: This API is not supported in IWR6843 ES1.0
*   @note 3: The TX gain and phase mismatch monitor API is not supported in this release. Please
*            refer latest DFP release note for more info.
*/
/* DesignId : MMWL_DesignId_082 */
/* Requirements : AUTORADAR_REQ-863 */
#[no_mangle]
pub unsafe extern "C" fn rlRfTxGainPhaseMismatchMonConfig(mut deviceMap:
                                                              rlUInt8_t,
                                                          mut data:
                                                              *mut rlTxGainPhaseMismatchMonConf_t)
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
                                  0xe as libc::c_uint as rlUInt16_t,
                                  0xd as libc::c_uint as rlUInt16_t,
                                  data as *mut rlUInt8_t,
                                  ::std::mem::size_of::<rlTxGainPhaseMismatchMonConf_t>()
                                      as libc::c_ulong as rlUInt16_t)
    }
    return retVal;
}
/* * @fn rlReturnVal_t rlRfTxBpmMonConfig(rlUInt8_t deviceMap, rlAllTxBpmMonConf_t* data)
*
*   @brief Sets information related to TX BPM monitoring.
*   @param[in] deviceMap - Bitmap of devices to send the message
*   @param[in] data - Tx BPM monitor configuration
*
*   @return rlReturnVal_t Success - 0, Failure - Error Code
*
*   This API is a Monitoring Configuration API, containing information
*   related to TX BPM monitoring.
*   Once configured, radarSS will send Async event (RL_RF_AE_MON_TX0_BPM_REPORT,
*   RL_RF_AE_MON_TX1_BPM_REPORT, RL_RF_AE_MON_TX2_BPM_REPORT) to report monitoring data
*
*   @note 1: This API is not supported in IWR6843 ES1.0
*   @note 2: The TX BPM monitor APIs are not supported in this release. Please refer latest DFP
*            release note for more info.
*/
/* DesignId : MMWL_DesignId_083 */
/* Requirements : AUTORADAR_REQ-864 */
#[no_mangle]
pub unsafe extern "C" fn rlRfTxBpmMonConfig(mut deviceMap: rlUInt8_t,
                                            mut data:
                                                *mut rlAllTxBpmMonConf_t)
 -> rlReturnVal_t {
    let mut retVal: rlReturnVal_t = 0;
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
                                                           0 as libc::c_int as
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
    let mut numSbc: rlUInt16_t = 0 as libc::c_uint as rlUInt16_t;
    /* Initialize Command and Response Sub Blocks */
    let mut inPayloadSb: [rlPayloadSb_t; 32] =
        [{
             let mut init =
                 rlPayloadSb{sbid: 0 as libc::c_int as rlUInt16_t,
                             len: 0,
                             pSblkData: 0 as *mut rlUInt8_t,};
             init
         }, rlPayloadSb_t{sbid: 0, len: 0, pSblkData: 0 as *mut rlUInt8_t,},
         rlPayloadSb_t{sbid: 0, len: 0, pSblkData: 0 as *mut rlUInt8_t,},
         rlPayloadSb_t{sbid: 0, len: 0, pSblkData: 0 as *mut rlUInt8_t,},
         rlPayloadSb_t{sbid: 0, len: 0, pSblkData: 0 as *mut rlUInt8_t,},
         rlPayloadSb_t{sbid: 0, len: 0, pSblkData: 0 as *mut rlUInt8_t,},
         rlPayloadSb_t{sbid: 0, len: 0, pSblkData: 0 as *mut rlUInt8_t,},
         rlPayloadSb_t{sbid: 0, len: 0, pSblkData: 0 as *mut rlUInt8_t,},
         rlPayloadSb_t{sbid: 0, len: 0, pSblkData: 0 as *mut rlUInt8_t,},
         rlPayloadSb_t{sbid: 0, len: 0, pSblkData: 0 as *mut rlUInt8_t,},
         rlPayloadSb_t{sbid: 0, len: 0, pSblkData: 0 as *mut rlUInt8_t,},
         rlPayloadSb_t{sbid: 0, len: 0, pSblkData: 0 as *mut rlUInt8_t,},
         rlPayloadSb_t{sbid: 0, len: 0, pSblkData: 0 as *mut rlUInt8_t,},
         rlPayloadSb_t{sbid: 0, len: 0, pSblkData: 0 as *mut rlUInt8_t,},
         rlPayloadSb_t{sbid: 0, len: 0, pSblkData: 0 as *mut rlUInt8_t,},
         rlPayloadSb_t{sbid: 0, len: 0, pSblkData: 0 as *mut rlUInt8_t,},
         rlPayloadSb_t{sbid: 0, len: 0, pSblkData: 0 as *mut rlUInt8_t,},
         rlPayloadSb_t{sbid: 0, len: 0, pSblkData: 0 as *mut rlUInt8_t,},
         rlPayloadSb_t{sbid: 0, len: 0, pSblkData: 0 as *mut rlUInt8_t,},
         rlPayloadSb_t{sbid: 0, len: 0, pSblkData: 0 as *mut rlUInt8_t,},
         rlPayloadSb_t{sbid: 0, len: 0, pSblkData: 0 as *mut rlUInt8_t,},
         rlPayloadSb_t{sbid: 0, len: 0, pSblkData: 0 as *mut rlUInt8_t,},
         rlPayloadSb_t{sbid: 0, len: 0, pSblkData: 0 as *mut rlUInt8_t,},
         rlPayloadSb_t{sbid: 0, len: 0, pSblkData: 0 as *mut rlUInt8_t,},
         rlPayloadSb_t{sbid: 0, len: 0, pSblkData: 0 as *mut rlUInt8_t,},
         rlPayloadSb_t{sbid: 0, len: 0, pSblkData: 0 as *mut rlUInt8_t,},
         rlPayloadSb_t{sbid: 0, len: 0, pSblkData: 0 as *mut rlUInt8_t,},
         rlPayloadSb_t{sbid: 0, len: 0, pSblkData: 0 as *mut rlUInt8_t,},
         rlPayloadSb_t{sbid: 0, len: 0, pSblkData: 0 as *mut rlUInt8_t,},
         rlPayloadSb_t{sbid: 0, len: 0, pSblkData: 0 as *mut rlUInt8_t,},
         rlPayloadSb_t{sbid: 0, len: 0, pSblkData: 0 as *mut rlUInt8_t,},
         rlPayloadSb_t{sbid: 0, len: 0, pSblkData: 0 as *mut rlUInt8_t,}];
    /* Construct command packet */
    rlDriverConstructInMsg(0xe as libc::c_uint as rlUInt16_t, &mut inMsg,
                           &mut *inPayloadSb.as_mut_ptr().offset(0 as
                                                                     libc::c_uint
                                                                     as
                                                                     isize));
    /* check if deviceIndex is out of defined value */
    if rlDriverIsDeviceMapValid(deviceMap) != 0 as libc::c_int ||
           data.is_null() {
        retVal = -(2 as libc::c_int)
    } else {
        /* check for NULL pointer */
        if !(*data).tx0BpmMonCfg.is_null() {
            /* Fill in-message Payload */
            rlDriverFillPayload(0xe as libc::c_uint as rlUInt16_t,
                                0xe as libc::c_uint as rlUInt16_t,
                                &mut *inPayloadSb.as_mut_ptr().offset(numSbc
                                                                          as
                                                                          isize),
                                (*data).tx0BpmMonCfg as *mut rlUInt8_t,
                                ::std::mem::size_of::<rlTxBpmMonConf_t>() as
                                    libc::c_ulong as rlUInt16_t);
            numSbc = numSbc.wrapping_add(1)
        }
        /* check for NULL pointer */
        if !(*data).tx1BpmMonCfg.is_null() {
            /* Fill in-message Payload */
            rlDriverFillPayload(0xe as libc::c_uint as rlUInt16_t,
                                0xf as libc::c_uint as rlUInt16_t,
                                &mut *inPayloadSb.as_mut_ptr().offset(numSbc
                                                                          as
                                                                          isize),
                                (*data).tx1BpmMonCfg as *mut rlUInt8_t,
                                ::std::mem::size_of::<rlTxBpmMonConf_t>() as
                                    libc::c_ulong as rlUInt16_t);
            numSbc = numSbc.wrapping_add(1)
        }
        /* check for NULL pointer */
        if !(*data).tx2BpmMonCfg.is_null() {
            /* Fill in-message Payload */
            rlDriverFillPayload(0xe as libc::c_uint as rlUInt16_t,
                                0x10 as libc::c_uint as rlUInt16_t,
                                &mut *inPayloadSb.as_mut_ptr().offset(numSbc
                                                                          as
                                                                          isize),
                                (*data).tx2BpmMonCfg as *mut rlUInt8_t,
                                ::std::mem::size_of::<rlTxBpmMonConf_t>() as
                                    libc::c_ulong as rlUInt16_t);
            numSbc = numSbc.wrapping_add(1)
        }
        if numSbc as libc::c_uint > 0 as libc::c_uint {
            /* setting num of sub-block to inMsg */
            inMsg.opcode.nsbc = numSbc;
            /* Send Command to mmWave Radar Device */
            retVal = rlDriverCmdInvoke(deviceMap, inMsg, &mut outMsg)
        } else { retVal = -(2 as libc::c_int) }
    }
    return retVal;
}
/* * @fn rlReturnVal_t rlRfSynthFreqMonConfig(rlUInt8_t deviceMap, rlSynthFreqMonConf_t* data)
*
*   @brief Sets information related to synthesizer frequency
*   @param[in] deviceMap - Bitmap of devices to send the message
*   @param[in] data -Synthesizer frequency monitor configuration
*
*   @return rlReturnVal_t Success - 0, Failure - Error Code
*
*   This API is a monitoring configuration API, containing information related to synthesizer 
*   frequency monitoring during chirping. The report is sent as an async event 
*   AWR_MONITOR_SYNTH_FREQUENCY_REPORT_AE_SB for live monitor and 
*   AWR_MONITOR_SYNTHESIZER_FREQUENCY_NONLIVE_REPORT_AE_SB for nonlive monitor (This is a new 
*   feature addition in xWR6843 device only). \n
*   @note 1: (Live mode) It is recommended to re-issue this configuration API each time before 
*   enabling this monitor and frame trigger. The right sequence is as below: \n
*   1. Issue Synth frequency monitor configuration API. \n
*   2. Enable Synth frequency monitor. \n
*   3. Frame start. \n
*   4. Frame stop. \n
*   5. Frame start (Optional in case of multiple frames). \n
*   6. Frame stop (Optional in case of multiple frames). \n
*   7. Disable Synth frequency monitor (in case disabled for some reason). \n
*   8. Issue Synth frequency monitor configuration API. \n
*   9. Enable Synth frequency monitor. \n
*   10. Frame start. \n
*   @note 2: In non live mode, this API can be issued twice with MONITOR_CONFIG_MODE value set to
*   1 and 2 for two different VCOs configured in different profiles respectively. The consolidated 
*   report for two VCOs in non-live mode is sent in a separate Async event -
*   AWR_MONITOR_SYNTHESIZER_FREQUENCY_NONLIVE_REPORT \n
*   @note 3: In non live mode, the reporting mode and VCO_MON_EN for two VCO configurations should 
*   be same. \n
*   @note 4: The synth non-live mode monitor (Applicable to xWR6843 device only) internally 
*   generates a test chirp based on the profile associated with it. In order to limit its 
*   execution time, if the profile's ramp time exceeds 60us, the test chirp's ramp time is 
*   limited to 60us and the chirp slope is scaled to cover the profile's intended RF bandwidth. \n
*   @note 5: This API is not supported in IWR6843 ES1.0. \n
*   @note 6: Both Live and Non-live synth frequency monitors cannot be enabled together. \n
*/
/* DesignId : MMWL_DesignId_084 */
/* Requirements : AUTORADAR_REQ-865, AUTORADAR_REQ-1052 */
#[no_mangle]
pub unsafe extern "C" fn rlRfSynthFreqMonConfig(mut deviceMap: rlUInt8_t,
                                                mut data:
                                                    *mut rlSynthFreqMonConf_t)
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
                                  0xe as libc::c_uint as rlUInt16_t,
                                  0x11 as libc::c_uint as rlUInt16_t,
                                  data as *mut rlUInt8_t,
                                  ::std::mem::size_of::<rlSynthFreqMonConf_t>()
                                      as libc::c_ulong as rlUInt16_t)
    }
    return retVal;
}
/* * @fn rlReturnVal_t rlRfExtAnaSignalsMonConfig(rlUInt8_t deviceMap,
                                                  rlExtAnaSignalsMonConf_t* data)
*
*   @brief Sets information related to external DC signals monitoring
*   @param[in] deviceMap - Bitmap of devices to send the message
*   @param[in] data - External anlog signal monitor config
*
*   @return rlReturnVal_t Success - 0, Failure - Error Code
*
*   This API is a Monitoring Configuration API, containing information
*   related to external DC signals monitoring (available only in xWR1642 & xWR1843).
*   Once configured, radarSS will send Async event (RL_RF_AE_MON_EXT_ANALOG_SIG_REPORT) to
*   report monitoring data
*
*   @note : This API is not supported in IWR6843 ES1.0
*/
/* DesignId : MMWL_DesignId_085 */
/* Requirements : AUTORADAR_REQ-866 */
#[no_mangle]
pub unsafe extern "C" fn rlRfExtAnaSignalsMonConfig(mut deviceMap: rlUInt8_t,
                                                    mut data:
                                                        *mut rlExtAnaSignalsMonConf_t)
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
                                  0xe as libc::c_uint as rlUInt16_t,
                                  0x12 as libc::c_uint as rlUInt16_t,
                                  data as *mut rlUInt8_t,
                                  ::std::mem::size_of::<rlExtAnaSignalsMonConf_t>()
                                      as libc::c_ulong as rlUInt16_t)
    }
    return retVal;
}
/* * @fn rlReturnVal_t rlRfTxIntAnaSignalsMonConfig(rlUInt8_t deviceMap,
                                                    rlAllTxIntAnaSignalsMonConf_t* data)
*
*   @brief Sets information related to TX Internal Analog Signals monitoring
*   @param[in] deviceMap - Bitmap of devices to send the message
*   @param[in] data - Tx internal analog signal monitoring configuration
*
*   @return rlReturnVal_t Success - 0, Failure - Error Code
*
*   This API is a Monitoring Configuration API, containing information
*   related to TX Internal Analog Signals monitoring including Tx Phase shifter DAC monitor.
*   Once configured, radarSS will send Async event (RL_RF_AE_MON_TX0_INT_ANA_SIG_REPORT
*   RL_RF_AE_MON_TX1_INT_ANA_SIG_REPORT, RL_RF_AE_MON_TX2_INT_ANA_SIG_REPORT) to
*   report monitoring data
*
*   @note : This API is not supported in IWR6843 ES1.0
*/
/* DesignId : MMWL_DesignId_086 */
/* Requirements : AUTORADAR_REQ-867, AUTORADAR_REQ-1062 */
#[no_mangle]
pub unsafe extern "C" fn rlRfTxIntAnaSignalsMonConfig(mut deviceMap:
                                                          rlUInt8_t,
                                                      mut data:
                                                          *mut rlAllTxIntAnaSignalsMonConf_t)
 -> rlReturnVal_t {
    let mut retVal: rlReturnVal_t = 0;
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
                                                           0 as libc::c_int as
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
    let mut numSbc: rlUInt16_t = 0 as libc::c_uint as rlUInt16_t;
    /* Initialize Command and Response Sub Blocks */
    let mut inPayloadSb: [rlPayloadSb_t; 32] =
        [{
             let mut init =
                 rlPayloadSb{sbid: 0 as libc::c_int as rlUInt16_t,
                             len: 0,
                             pSblkData: 0 as *mut rlUInt8_t,};
             init
         }, rlPayloadSb_t{sbid: 0, len: 0, pSblkData: 0 as *mut rlUInt8_t,},
         rlPayloadSb_t{sbid: 0, len: 0, pSblkData: 0 as *mut rlUInt8_t,},
         rlPayloadSb_t{sbid: 0, len: 0, pSblkData: 0 as *mut rlUInt8_t,},
         rlPayloadSb_t{sbid: 0, len: 0, pSblkData: 0 as *mut rlUInt8_t,},
         rlPayloadSb_t{sbid: 0, len: 0, pSblkData: 0 as *mut rlUInt8_t,},
         rlPayloadSb_t{sbid: 0, len: 0, pSblkData: 0 as *mut rlUInt8_t,},
         rlPayloadSb_t{sbid: 0, len: 0, pSblkData: 0 as *mut rlUInt8_t,},
         rlPayloadSb_t{sbid: 0, len: 0, pSblkData: 0 as *mut rlUInt8_t,},
         rlPayloadSb_t{sbid: 0, len: 0, pSblkData: 0 as *mut rlUInt8_t,},
         rlPayloadSb_t{sbid: 0, len: 0, pSblkData: 0 as *mut rlUInt8_t,},
         rlPayloadSb_t{sbid: 0, len: 0, pSblkData: 0 as *mut rlUInt8_t,},
         rlPayloadSb_t{sbid: 0, len: 0, pSblkData: 0 as *mut rlUInt8_t,},
         rlPayloadSb_t{sbid: 0, len: 0, pSblkData: 0 as *mut rlUInt8_t,},
         rlPayloadSb_t{sbid: 0, len: 0, pSblkData: 0 as *mut rlUInt8_t,},
         rlPayloadSb_t{sbid: 0, len: 0, pSblkData: 0 as *mut rlUInt8_t,},
         rlPayloadSb_t{sbid: 0, len: 0, pSblkData: 0 as *mut rlUInt8_t,},
         rlPayloadSb_t{sbid: 0, len: 0, pSblkData: 0 as *mut rlUInt8_t,},
         rlPayloadSb_t{sbid: 0, len: 0, pSblkData: 0 as *mut rlUInt8_t,},
         rlPayloadSb_t{sbid: 0, len: 0, pSblkData: 0 as *mut rlUInt8_t,},
         rlPayloadSb_t{sbid: 0, len: 0, pSblkData: 0 as *mut rlUInt8_t,},
         rlPayloadSb_t{sbid: 0, len: 0, pSblkData: 0 as *mut rlUInt8_t,},
         rlPayloadSb_t{sbid: 0, len: 0, pSblkData: 0 as *mut rlUInt8_t,},
         rlPayloadSb_t{sbid: 0, len: 0, pSblkData: 0 as *mut rlUInt8_t,},
         rlPayloadSb_t{sbid: 0, len: 0, pSblkData: 0 as *mut rlUInt8_t,},
         rlPayloadSb_t{sbid: 0, len: 0, pSblkData: 0 as *mut rlUInt8_t,},
         rlPayloadSb_t{sbid: 0, len: 0, pSblkData: 0 as *mut rlUInt8_t,},
         rlPayloadSb_t{sbid: 0, len: 0, pSblkData: 0 as *mut rlUInt8_t,},
         rlPayloadSb_t{sbid: 0, len: 0, pSblkData: 0 as *mut rlUInt8_t,},
         rlPayloadSb_t{sbid: 0, len: 0, pSblkData: 0 as *mut rlUInt8_t,},
         rlPayloadSb_t{sbid: 0, len: 0, pSblkData: 0 as *mut rlUInt8_t,},
         rlPayloadSb_t{sbid: 0, len: 0, pSblkData: 0 as *mut rlUInt8_t,}];
    /* Construct command packet */
    rlDriverConstructInMsg(0xe as libc::c_uint as rlUInt16_t, &mut inMsg,
                           &mut *inPayloadSb.as_mut_ptr().offset(0 as
                                                                     libc::c_uint
                                                                     as
                                                                     isize));
    /* check if deviceIndex is out of defined value */
    if rlDriverIsDeviceMapValid(deviceMap) != 0 as libc::c_int ||
           data.is_null() {
        retVal = -(2 as libc::c_int)
    } else {
        /* check for NULL pointer */
        if !(*data).tx0IntAnaSgnlMonCfg.is_null() {
            /* Fill in-message Payload */
            rlDriverFillPayload(0xe as libc::c_uint as rlUInt16_t,
                                0x13 as libc::c_uint as rlUInt16_t,
                                &mut *inPayloadSb.as_mut_ptr().offset(numSbc
                                                                          as
                                                                          isize),
                                (*data).tx0IntAnaSgnlMonCfg as *mut rlUInt8_t,
                                ::std::mem::size_of::<rlTxIntAnaSignalsMonConf_t>()
                                    as libc::c_ulong as rlUInt16_t);
            numSbc = numSbc.wrapping_add(1)
        }
        /* check for NULL pointer */
        if !(*data).tx1IntAnaSgnlMonCfg.is_null() {
            /* Fill in-message Payload */
            rlDriverFillPayload(0xe as libc::c_uint as rlUInt16_t,
                                0x14 as libc::c_uint as rlUInt16_t,
                                &mut *inPayloadSb.as_mut_ptr().offset(numSbc
                                                                          as
                                                                          isize),
                                (*data).tx1IntAnaSgnlMonCfg as *mut rlUInt8_t,
                                ::std::mem::size_of::<rlTxIntAnaSignalsMonConf_t>()
                                    as libc::c_ulong as rlUInt16_t);
            numSbc = numSbc.wrapping_add(1)
        }
        /* check for NULL pointer */
        if !(*data).tx2IntAnaSgnlMonCfg.is_null() {
            /* Fill in-message Payload */
            rlDriverFillPayload(0xe as libc::c_uint as rlUInt16_t,
                                0x15 as libc::c_uint as rlUInt16_t,
                                &mut *inPayloadSb.as_mut_ptr().offset(numSbc
                                                                          as
                                                                          isize),
                                (*data).tx2IntAnaSgnlMonCfg as *mut rlUInt8_t,
                                ::std::mem::size_of::<rlTxIntAnaSignalsMonConf_t>()
                                    as libc::c_ulong as rlUInt16_t);
            numSbc = numSbc.wrapping_add(1)
        }
        if numSbc as libc::c_uint > 0 as libc::c_uint {
            /* setting num of sub-block to inMsg */
            inMsg.opcode.nsbc = numSbc;
            /* Send Command to mmWave Radar Device */
            retVal = rlDriverCmdInvoke(deviceMap, inMsg, &mut outMsg)
        } else { retVal = -(2 as libc::c_int) }
    }
    return retVal;
}
/* * @fn rlReturnVal_t rlRfRxIntAnaSignalsMonConfig(rlUInt8_t deviceMap,
                                                    rlRxIntAnaSignalsMonConf_t* data)
*
*   @brief Sets information related to RX Internal Analog Signals monitoring
*   @param[in] deviceMap - Bitmap of devices to send the message
*   @param[in] data - Rx internal analog signal monitoring config
*
*   @return rlReturnVal_t Success - 0, Failure - Error Code
*
*   This API is a Monitoring Configuration API, containing information
*   related to RX Internal Analog Signals monitoring.
*   Once configured, radarSS will send Async event (RL_RF_AE_MON_RX_INT_ANALOG_SIG_REPORT) to
*   report monitoring data
*
*   @note : This API is not supported in IWR6843 ES1.0
*/
/* DesignId : MMWL_DesignId_087 */
/* Requirements : AUTORADAR_REQ-868 */
#[no_mangle]
pub unsafe extern "C" fn rlRfRxIntAnaSignalsMonConfig(mut deviceMap:
                                                          rlUInt8_t,
                                                      mut data:
                                                          *mut rlRxIntAnaSignalsMonConf_t)
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
                                  0xe as libc::c_uint as rlUInt16_t,
                                  0x16 as libc::c_uint as rlUInt16_t,
                                  data as *mut rlUInt8_t,
                                  ::std::mem::size_of::<rlRxIntAnaSignalsMonConf_t>()
                                      as libc::c_ulong as rlUInt16_t)
    }
    return retVal;
}
/* * @fn rlReturnVal_t rlRfPmClkLoIntAnaSignalsMonConfig(rlUInt8_t deviceMap,
                                                         rlPmClkLoIntAnaSignalsMonConf_t* data)
*
*   @brief Sets information related to Power Management, Clock generation and LO distribution
*   @param[in] deviceMap - Bitmap of devices to send the message
*   @param[in] data - PMCLK internal analog signal monitoring config
*
*   @return rlReturnVal_t Success - 0, Failure - Error Code
*
*   This API is a Monitoring Configuration API, containing information
*   related to Power Management,Clock generation and LO distribution
*   circuits' Internal Analog Signals monitoring.
*   Once configured, radarSS will send Async event (RL_RF_AE_MON_PMCLKLO_INT_ANA_SIG_REPORT) to
*   report monitoring data
*
*   @note 1 : This API is not supported in IWR6843 ES1.0. \n
*   @note 2 : The 20G monitor is not supported in single chip configuration. \n
*/
/* DesignId : MMWL_DesignId_088 */
/* Requirements : AUTORADAR_REQ-869 */
#[no_mangle]
pub unsafe extern "C" fn rlRfPmClkLoIntAnaSignalsMonConfig(mut deviceMap:
                                                               rlUInt8_t,
                                                           mut data:
                                                               *mut rlPmClkLoIntAnaSignalsMonConf_t)
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
                                  0xe as libc::c_uint as rlUInt16_t,
                                  0x17 as libc::c_uint as rlUInt16_t,
                                  data as *mut rlUInt8_t,
                                  ::std::mem::size_of::<rlPmClkLoIntAnaSignalsMonConf_t>()
                                      as libc::c_ulong as rlUInt16_t)
    }
    return retVal;
}
/* * @fn rlReturnVal_t rlRfGpadcIntAnaSignalsMonConfig(rlUInt8_t deviceMap,
                                                       rlGpadcIntAnaSignalsMonConf_t* data)
*
*   @brief Sets information related to GPADC Internal Analog Signals monitoring
*   @param[in] deviceMap - Bitmap of devices to send the message
*   @param[in] data - GPADC internal analog signal monitoring config
*
*   @return rlReturnVal_t Success - 0, Failure - Error Code
*
*   This API is a Monitoring Configuration API, containing information
*   related to GPADC Internal Analog Signals monitoring.
*   Once configured, radarSS will send Async event (RL_RF_AE_MON_GPADC_INT_ANA_SIG_REPORT) to
*   report monitoring data.
*
*   @note : This API is not supported in IWR6843 ES1.0
*/
/* DesignId : MMWL_DesignId_089 */
/* Requirements : AUTORADAR_REQ-870 */
#[no_mangle]
pub unsafe extern "C" fn rlRfGpadcIntAnaSignalsMonConfig(mut deviceMap:
                                                             rlUInt8_t,
                                                         mut data:
                                                             *mut rlGpadcIntAnaSignalsMonConf_t)
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
                                  0xe as libc::c_uint as rlUInt16_t,
                                  0x18 as libc::c_uint as rlUInt16_t,
                                  data as *mut rlUInt8_t,
                                  ::std::mem::size_of::<rlGpadcIntAnaSignalsMonConf_t>()
                                      as libc::c_ulong as rlUInt16_t)
    }
    return retVal;
}
/* * @fn rlReturnVal_t rlRfPllContrlVoltMonConfig(rlUInt8_t deviceMap,
                                                  rlPllContrVoltMonConf_t* data)
*
*   @brief Sets information related to APLL and Synthesizer's control voltage signals monitoring.
*   @param[in] deviceMap - Bitmap of devices to send the message
*   @param[in] data - PLL control voltage monitoring config
*
*   @return rlReturnVal_t Success - 0, Failure - Error Code
*
*   This API is a Monitoring Configuration API, containing information
*   related to APLL and Synthesizer's control voltage signals monitoring
*   Once configured, radarSS will send Async event (RL_RF_AE_MON_PLL_CONTROL_VOLT_REPORT) to
*   report monitoring data
*
*   @note : This API is not supported in IWR6843 ES1.0
*/
/* DesignId : MMWL_DesignId_090 */
/* Requirements : AUTORADAR_REQ-871 */
#[no_mangle]
pub unsafe extern "C" fn rlRfPllContrlVoltMonConfig(mut deviceMap: rlUInt8_t,
                                                    mut data:
                                                        *mut rlPllContrVoltMonConf_t)
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
                                  0xe as libc::c_uint as rlUInt16_t,
                                  0x19 as libc::c_uint as rlUInt16_t,
                                  data as *mut rlUInt8_t,
                                  ::std::mem::size_of::<rlPllContrVoltMonConf_t>()
                                      as libc::c_ulong as rlUInt16_t)
    }
    return retVal;
}
/* * @fn rlReturnVal_t rlRfDualClkCompMonConfig(rlUInt8_t deviceMap, rlDualClkCompMonConf_t* data)
*
*   @brief Sets information related to the DCC based clock frequency monitoring.
*   @param[in] deviceMap - Bitmap of devices to send the message
*   @param[in] data - Dual clock comp monitoring config
*
*   @return rlReturnVal_t Success - 0, Failure - Error Code
*
*   This API is a Monitoring Configuration API, containing information
*   related to the DCC based clock frequency monitoring.
*   Once configured, radarSS will send Async event (RL_RF_AE_MON_DCC_CLK_FREQ_REPORT) to report
*   monitoring data
*
*   @note : This API is not supported in IWR6843 ES1.0
*/
/* DesignId : MMWL_DesignId_091 */
/* Requirements : AUTORADAR_REQ-872 */
#[no_mangle]
pub unsafe extern "C" fn rlRfDualClkCompMonConfig(mut deviceMap: rlUInt8_t,
                                                  mut data:
                                                      *mut rlDualClkCompMonConf_t)
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
                                  0xe as libc::c_uint as rlUInt16_t,
                                  0x1a as libc::c_uint as rlUInt16_t,
                                  data as *mut rlUInt8_t,
                                  ::std::mem::size_of::<rlDualClkCompMonConf_t>()
                                      as libc::c_ulong as rlUInt16_t)
    }
    return retVal;
}
/* * @fn rlReturnVal_t rlRfRxIfSatMonConfig(rlUInt8_t deviceMap, rlRxSatMonConf_t* data)
*
*   @brief Sets information related to RX saturation detector monitoring.
*   @param[in] deviceMap - Bitmap of devices to send the message
*   @param[in] data - Rx saturation monitoring config
*
*   @return rlReturnVal_t Success - 0, Failure - Error Code
*
*   This API is a Monitoring Configuration API, containing information
*   related to RX saturation detector monitoring.
*   It is recommended to re-issue these RX saturation monitor configuration APIs each time before
*   enabling these monitor and frame trigger. The right sequence is as below:  \n
*   1. Issue RX saturation monitor configuration API. \n
*   2. Enable RX saturation monitor. \n
*   3. Frame start. \n
*   4. Frame stop. \n
*   5. Frame start (Optional in case of multiple frames). \n
*   6. Frame stop (Optional in case of multiple frames). \n
*   7. Disable RX saturation monitor (in case disabled for some reason). \n
*   8. Issue RX saturation monitor configuration API. \n
*   9. Enable RX saturation monitor. \n
*   10. Frame start. \n
*
*   @note 1: This API is not supported in IWR6843 ES1.0
*/
/* DesignId : MMWL_DesignId_092 */
/* Requirements : AUTORADAR_REQ-873 */
#[no_mangle]
pub unsafe extern "C" fn rlRfRxIfSatMonConfig(mut deviceMap: rlUInt8_t,
                                              mut data: *mut rlRxSatMonConf_t)
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
                                  0xe as libc::c_uint as rlUInt16_t,
                                  0x1b as libc::c_uint as rlUInt16_t,
                                  data as *mut rlUInt8_t,
                                  ::std::mem::size_of::<rlRxSatMonConf_t>() as
                                      libc::c_ulong as rlUInt16_t)
    }
    return retVal;
}
/* * @fn rlReturnVal_t rlRfRxSigImgMonConfig(rlUInt8_t deviceMap, rlSigImgMonConf_t* data)
*
*   @brief Sets information related to signal and image band energy.
*   @param[in] deviceMap - Bitmap of devices to send the message
*   @param[in] data - Signal img monitoring config
*
*   @return rlReturnVal_t Success - 0, Failure - Error Code
*
*   This API is a Monitoring Configuration API, containing information
*   related to signal and image band energy. The Monitoring report is available as CQ1
*   (part of CQ) in CQ RAM. The application should transfer the report every chirp. \n
*   It is recommended to re-issue these RX signal image monitor configuration APIs each time
*   before enabling these monitor and frame trigger. The right sequence is as below:  \n
*   1. Issue RX signal image monitor configuration API. \n
*   2. Enable RX signal image monitor. \n
*   3. Frame start. \n
*   4. Frame stop. \n
*   5. Frame start (Optional in case of multiple frames). \n
*   6. Frame stop (Optional in case of multiple frames). \n
*   7. Disable RX signal image monitor (in case disabled for some reason). \n
*   8. Issue RX signal image monitor configuration API. \n
*   9. Enable RX signal image monitor. \n
*   10. Frame start. \n
*
*   @note : This API is not supported in IWR6843 ES1.0
*/
/* DesignId : MMWL_DesignId_093 */
/* Requirements : AUTORADAR_REQ-874 */
#[no_mangle]
pub unsafe extern "C" fn rlRfRxSigImgMonConfig(mut deviceMap: rlUInt8_t,
                                               mut data:
                                                   *mut rlSigImgMonConf_t)
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
                                  0xe as libc::c_uint as rlUInt16_t,
                                  0x1c as libc::c_uint as rlUInt16_t,
                                  data as *mut rlUInt8_t,
                                  ::std::mem::size_of::<rlSigImgMonConf_t>()
                                      as libc::c_ulong as rlUInt16_t)
    }
    return retVal;
}
/* * @fn rlReturnVal_t rlRfRxMixerInPwrConfig(rlUInt8_t deviceMap, rlSigImgMonConf_t* data)
*
*   @brief Sets information related to RX mixer input power monitoring.
*   @param[in] deviceMap - Bitmap of devices to send the message
*   @param[in] data - RX mixer input power config
*
*   @return rlReturnVal_t Success - 0, Failure - Error Code
*
*   This API is a Monitoring Configuration API, containing information
*   related to RX mixer input power monitoring.
*
*   @note 1: This API is not supported in IWR6843 ES1.0 \n
*   @note 2: The RX input power monitor API is debug only API. Please refer latest DFP release
*            note for more info.
*/
/* DesignId : MMWL_DesignId_094 */
/* Requirements : AUTORADAR_REQ-875 */
#[no_mangle]
pub unsafe extern "C" fn rlRfRxMixerInPwrConfig(mut deviceMap: rlUInt8_t,
                                                mut data:
                                                    *mut rlRxMixInPwrMonConf_t)
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
                                  0xe as libc::c_uint as rlUInt16_t,
                                  0x1d as libc::c_uint as rlUInt16_t,
                                  data as *mut rlUInt8_t,
                                  ::std::mem::size_of::<rlRxMixInPwrMonConf_t>()
                                      as libc::c_ulong as rlUInt16_t)
    }
    return retVal;
}
/* ***************************************************************************************
 * FileName     : rl_monitoring.h
 *
 * Description  : This file defines the functions required for Monitoring.
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
/* ! \brief
* Supported maximum number of RF frequencies for monitoring
*/
/* ! \brief
* Max Number of (primary + secondary) slices to monitor
*/
/* *****************************************************************************
 * GLOBAL VARIABLES/DATA-TYPES DEFINITIONS
 ******************************************************************************
 */
/* ! \brief
* Digital monitoring configuration
*/
/* *
     * @brief  Bit: Dig Monitoring \n
                 0 Reserved \n
                 1 CR4 and VIM lockstep test of diagnostic \n
                 2 Reserved \n
                 3 VIM test (Not supported in 1st Gen devices, refer latest release note) \n
                 4 Reserved \n
                 5 Reserved \n
                 6 CRC test of diagnostic \n
                 7 RAMPGEN memory ECC test of diagnostic (Not supported in 1st Gen devices,
                   refer latest release note) \n
                 8 DFE Parity test of diagnostic (RESERVED in xWR6x43 devices) \n
                 9 DFE memory ECC test of diagnostic \n
                 10 RAMPGEN lockstep test of diagnostic \n
                 11 FRC lockstep test of diagnostic \n
                 12 Reserved \n
                 13 Reserved \n
                 14 Reserved \n
                 15 Reserved \n
                 16 ESM test of diagnostic \n
                 17 DFE STC \n
                 18 Reserved \n
                 19 ATCM, BTCM ECC test of diagnostic \n
                 20 ATCM, BTCM parity test of diagnostic \n
                 21 Reserved \n
                 22 Reserved \n
                 23 Reserved \n
                 24 FFT test of diagnostic \n
                 25 RTI test of diagnostic \n
                 31:26 RESERVED \n
     */
/* *
     * @brief  Value    Definition \n
                 0     Production mode. Latent faults are tested and  any failures are reported \n
                 1     Characterization mode. Faults are injected and failures are reported
                       which allows testing of the failure reporting path  \n
     */
/* *
     * @brief  Reserved for Future use
     */
/* *
     * @brief  Reserved for Future use
     */
/* *
     * @brief  Reserved for Future use
     */
/* ! \brief
* Digital monitoring latent fault reporting configuration
*/
/* *
     * @brief  Value    Definition \n
                 0     Report is sent every monitoring period \n
                 1     Report is sent only on a failure \n
                 2     RESERVED \n
     */
/* *
     * @brief  Reserved for Future use
     */
/* *
     * @brief  Reserved for Future use
     */
/* *
     * @brief  Bit     Monitoring \n
               0      PERIODIC_CONFG_REGISTER_READ_EN \n
               1      RESERVED \n
               2      DFE_STC_EN \n
               3      FRAME_TIMING_MONITORING_EN \n
               31:4   RESERVED \n
     */
/* *
     * @brief  Reserved for Future use
     */
/* ! \brief
* Analog monitoring configuration
*/
/* *
     * @brief  Bit   Analog monitoring control \n
                0   TEMPERATURE_MONITOR_EN \n
                1   RX_GAIN_PHASE_MONITOR_EN \n
                2   RX_NOISE_MONITOR_EN \n
                3   RX_IFSTAGE_MONITOR_EN \n
                4   TX0_POWER_MONITOR_EN \n
                5   TX1_POWER_MONITOR_EN \n
                6   TX2_POWER_MONITOR_EN \n
                7   TX0_BALLBREAK_MONITOR_EN \n
                8   TX1_BALLBREAK_MONITOR_EN \n
                9   TX2_BALLBREAK_MONITOR_EN \n
                10  TX_GAIN_PHASE_MONITOR_EN \n
                11  TX0_BPM_MONITOR_EN \n
                12  TX1_BPM_MONITOR_EN \n
                13  TX2_BPM_MONITOR_EN \n
                14  SYNTH_FREQ_MONITOR_LIVE_EN \n
                15  EXTERNAL_ANALOG_SIGNALS_MONITOR_EN \n
                16  INTERNAL_TX0_SIGNALS_MONITOR_EN \n
                17  INTERNAL_TX1_SIGNALS_MONITOR_EN \n
                18  INTERNAL_TX2_SIGNALS_MONITOR_EN \n
                19  INTERNAL_RX_SIGNALS_MONITOR_EN \n
                20  INTERNAL_PMCLKLO_SIGNALS_MONITOR_EN \n
                21  INTERNAL_GPADC_SIGNALS_MONITOR_EN \n
                22  PLL_CONTROL_VOLTAGE_MONITOR_EN \n
                23  DCC_CLOCK_FREQ_MONITOR_EN \n
                24  RX_IF_SATURATION_MONITOR_EN \n
                25  RX_SIG_IMG_BAND_MONITORING_EN \n
                26  RX_MIXER_INPUT_POWER_MONITOR \n
                27  RESERVED \n
                28  SYNTH_FREQ_MONITOR_NON_LIVE_EN \n
                31:29   RESERVED \n
     */
/* *
     * @brief  LDO short circuit monitoring enable. There are no reports for these monitors. \n
               If there is any fault, the asyncevent RL_RF_AE_ANALOG_FAULT_SB will be sent. \n
                 Bit    Description \n
                 b0     APLL LDO short circuit monitoring \n
                        0 - disable, 1 - enable \n
                 b1     SYNTH VCO LDO short circuit monitoring \n
                        0 - disable, 1 - enable \n
                 b2     PA LDO short circuit monitoring \n
                        0 - disable, 1 - enable \n
                 b31:3 RESERVED \n
               @note : This feature is not supported in DFP 1.x (1st generation devices). Please
                       refer latest DFP release note for more details. \n
     */
/* ! \brief
* Temperature sensor monitoring configuration
*/
/* *
     * @brief  Value    Definition \n
                0      Report is sent every monitoring period without threshold check \n
                1      Report is send only upon a failure (after checking for thresholds) \n
                2      Report is sent every monitoring period with threshold check \n
     */
/* *
     * @brief  Reserved for Future use
     */
/* *
     * @brief  The temperatures read from near the sensors near the RF analog modules
                 are compared against a minimum threshold. The comparison result is part
                 of the monitoring report message (Error bit is set if any measurement
                 is outside this (minimum, maximum) range).\n
                 1 LSB = 1 degree Celsius, signed number \n
                 Valid range: -99 degree Celsius to 199 degree Celsius \n
     */
/* *
     * @brief  The temperatures read from near the sensors near the RF analog modules
                 are compared against a maximum threshold. The comparison result is part
                 of the monitoring report message (Error bit is set if any measurement
                 is outside this (minimum, maximum) range). \n
                 1 LSB = 1 degree Celsius, signed number \n
                 Valid range: -99 degree Celsius to 199 degree Celsius \n
     */
/* *
     * @brief  The temperatures read from near the sensor near the digital module are
                  compared against a minimum threshold. The comparison result is part of
                  the monitoring report message (Error bit is set if any measurement is
                  outside this (minimum, maximum) range). \n
                  1 LSB = 1 degree Celsius, signed number \n
                  Valid range: -99 degree Celsius to 199 degree Celsius \n
                  In xWR6x43, value 0 disables the monitor threshold check 
                  (together with DIG_TEMP_THRESH_MAX=0) \n
     */
/* *
     * @brief  The temperatures read from near the sensor near the digital module are
                 compared against a maximum threshold. The comparison result is part of
                 the monitoring report message (Error bit is set if any measurement is
                 outside this (minimum, maximum) range). \n
                 1 LSB = 1 degree Celsius, signed number \n
                 Valid range: -99 degree Celsius to 199 degree Celsius \n
                 In xWR6x43, value 0 disables the monitor threshold check 
                 (together with DIG_TEMP_THRESH_MIN=0) \n
     */
/* *
     * @brief  The maximum difference across temperatures read from all the enabled
                 sensors is compared against this threshold.The comparison result is part
                 of the monitoring report message(Error bit is set if the measured difference
                 exceeds this field). \n
                 1 LSB = 1o Celsius, unsigned number \n
                 Valid range: 0 degree Celsius to 100 degree Celsius \n
                 In xWR6x43, digital temperature sensors can be excluded from this check by 
                 setting DIG_TEMP_THRESH_MIN and DIG_TEMP_THRESH_MAX to value 0. \n
     */
/* *
     * @brief  Reserved for Future use
     */
/* *
     * @brief  Reserved for Future use
     */
/* ! \brief
* RX gain and phase monitoring configuration
*/
/* *
     * @brief  This field indicates the profile Index for which this configuration applies.
     */
/* *
     * @brief  This field indicates the RF frequencies inside the profile's RF band at which to
                 measure the required parameters. When each bit in this field is set, the
                 measurement at the corresponding RF frequency is enabled w.r.t. the profile's
                 RF band. \n
                 Bit number  RF frequency                        RF name \n
                     0       Lowest RF frequency                 RF1 \n
                             in profile's sweep bandwidth \n
                     1       Center RF frequency in profile's    RF2 \n
                             sweep bandwidth \n
                     2       Highest RF frequency in             RF3 \n
                             profile's sweep bandwidth \n
                 The RF name column is mentioned here to set the convention for the
                 purpose of reporting and describing many monitoring packets. \n
     */
/* *
     * @brief  Value       Definition \n
                0          Report is sent every monitoring period without threshold check \n
                1          Report is send only upon a failure (after checking for thresholds) \n
                           @note : It is recommended not to use quiet mode, as Host has to compute
                                   actual RX gain and need to monitor Noise power to detect
                                   presence of interference. \n
                2          Report is sent every monitoring period with threshold check  \n
     */
/* *
     * @brief  Value     Definition \n
                0       TX0 is used for generating loopback signal for RX gain measurement \n
                1       TX1 is used for generating loopback signal for RX gain measurement. \n
     */
/* *
     * @brief  The magnitude of difference between the programmed and measured RX gain for each \n
                 enabled channel at each enabled RF frequency, is compared against this \n
                 threshold. The comparison result is part of the monitoring report message \n
                 (Error bit is set if any measurement is above this threshold). Before the \n
                 comparison, the measured gains for each RF and RX are adjusted by subtracting \n
                 the offset given in the RX_GAIN_MISMATCH_OFFSET_VALUE field \n
                 1 LSB = 0.1 dB \n
                 Valid range: 0 to 65535 (0 to 6553dB) \n
     */
/* *
     * @brief  The magnitude of difference between measured RX gains across the enabled channels \n
                 at each enabled RF frequency is compared against this threshold. The comparison \n
                 result is part of the monitoring report message (Error bit is set if the \n
                 measurement is above this threshold). Before the comparison, the measured gains \n
                 for each RF and RX are adjusted by subtracting the offset given in the \n
                 RX_GAIN_MISMATCH_OFFSET_VALUE field. \n
                 1 LSB = 0.1 dB \n
                 Valid range: 0 to 65535 (0 to 6553dB) \n
     */
/* *
     * @brief  The magnitude of measured RX gain flatness error, for each enabled channel, is \n
                 compared against this threshold. The flatness error for a channel is defined as \n
                 the peak to peak variation across RF frequencies. The comparison result is part \n
                 of the monitoring report message (Error bit is set if any measurement is above \n
                 this threshold). Before the comparison, the measured gains for each RF and RX \n
                 are adjusted by subtracting the offset given in the \n
                 RX_GAIN_MISMATCH_OFFSET_VALUE field. \n
                 1 LSB = 0.1 dB \n
                 Valid range: 0 to 65535 (0 to 6553dB) \n
                 This flatness check is applicable only if multiple RF Frequencies are enabled,  \n
                 i.e., RF_FREQ_BITMASK has bit numbers 0,1,2 set \n
     */
/* *
     * @brief  The magnitude of measured RX phase mismatch across the enabled channels at each \n
                 enabled RF frequency is compared against this threshold. The comparison result \n
                 is part of the monitoring report message (Error bit is set if any measurement \n
                 is above this threshold). Before the comparison, the measured phases for each \n
                 RF and RX are adjusted by subtracting the offset given in the \n
                 RX_PHASE_MISMATCH_OFFSET_VALUE field. \n
                 1 LSB = 360(degree) / 2^16 . \n
                 Valid range: corresponding to 0 degree to 359.9 degree. \n
     */
/* *
     * @brief  The offsets to be subtracted from the measured RX gain for each RX and RF before \n
                 the relevant threshold comparisons are given here. \n
                 Byte numbers corresponding to different RX and RF, in this field are \n
                 here: \n
                         RF1     RF2     RF3 \n
                 RX0    [1:0]   [9:8]    [17:16] \n
                 RX1    [3:2]   [11:10]  [19:18] \n
                 RX2    [5:4]   [13:12]  [21:20] \n
                 RX3    [7:6]   [15:14]  [23:22] \n
                 1 LSB = 0.1 dB, signed number \n
                 Only the entries of enabled RF Frequencies and enabled RX channels are \n
                 considered. \n
     */
/* *
     ** @brief  The offsets to be subtracted from the measured RX phase for each RX and RF \n
                 before the relevant threshold comparisons are given here. Byte numbers \n
                 corresponding to different RX and RF, in this field are here: \n
                         RF1       RF2       RF3 \n
                 RX0     [1:0]     [9:8]     [17:16] \n
                 RX1     [3:2]     [11:10]   [19:18] \n
                 RX2     [5:4]     [13:12]   [21:20] \n
                 RX3     [7:6]     [15:14]   [23:22] \n
                 1 LSB = 360(degree) / 2^16 , unsigned number \n
                 Only the entries of enabled RF Frequencies and enabled RX channels are \n
                 considered. \n
     */
/* *
     * @brief  Reserved for Future use
     */
/* *
     * @brief  Reserved for Future use
     */
/* ! \brief
* RX noise monitoring configuration
*/
/* *
     * @brief  This field indicates the profile Index for which this configuration applies.
     */
/* *
     * @brief  This field indicates the exact RF frequencies inside the profile's RF band at \n
                 which to measure the required parameters. When each bit in this field is set, \n
                 the measurement at the corresponding RF frequency is enabled w.r.t. the \n
                 profile's RF band. \n
                 Bit number   RF frequency                    RF name \n
                     0        Lowest RF frequency in          RF1 \n
                             profile's sweep bandwidth \n
                     1        Center RF frequency in          RF2 \n
                             profile's sweep bandwidth \n
                     2        Highest RF frequency in         RF3 \n
                             profile's sweep bandwidth \n
                 The RF name column is mentioned here to set the convention for the purpose of \n
                 reporting and describing many monitoring packets. \n
     */
/* *
     * @brief  Reserved for Future use
     */
/* *
     * @brief  Value        Definition \n
                0          Report is sent every monitoring period without threshold check \n
                1          Report is send only upon a failure (after checking for thresholds) \n
                2          Report is sent every monitoring period with threshold check  \n
     */
/* *
     * @brief  Reserved for Future use
     */
/* *
     * @brief  The measured RX input referred noise figure at the enabled RF frequencies, for \n
                 all channels, is compared against this threshold. The comparison result is part \n
                 of the monitoring report message (Error bit is set if any measurement is above \n
                 this threshold). \n
                 1 LSB = 0.1 dB \n
                 Valid range: 0 to 65535 (0 to 6553dB) \n
     */
/* *
     * @brief  Reserved for Future use
     */
/* ! \brief
* RX IF stage monitoring configuration
*/
/* *
     * @brief  This field indicates the profile Index for which this configuration applies.
     */
/* *
     * @brief  Value       Definition \n
                 0        Report is sent every monitoring period without threshold check \n
                 1        Report is send only upon a failure (after checking for thresholds) \n
                 2        Report is sent every monitoring period with threshold check. \n
     */
/* *
     * @brief  Reserved for Future use
     */
/* *
     * @brief  Reserved for Future use
     */
/* *
     * @brief  The absolute values of RX IF HPF cutoff percentage frequency errors are \n
                 compared against the corresponding thresholds given in this field. The \n
                 comparison results are part of the monitoring report message (Error bit is set \n
                 if the absolute value of the errors exceeds respective thresholds). \n
                 1 LSB = 1%, unsigned number \n
                 Valid range: 1% to 128% \n
     */
/* *
     * @brief  The absolute values of RX IF LPF cutoff percentage frequency errors are compared \n
                 against the corresponding thresholds given in this field. The comparison \n
                 results are part of the monitoring report message (Error bit is set if the \n
                 absolute value of the errors exceeds respective thresholds). \n
                 1 LSB = 1%, unsigned number \n
                 Valid range: 1% to 128% \n
                 @note 1: This feature is not supported in AWR1243, xWR1443, xWR1642 and \n
                          xWR1843 devices. \n
                 @note 2: This feature is not supported in this release. Please refer latest \n
                          DFP release note for more details. \n
     */
/* *
     * @brief  The absolute deviation of RX IFA Gain from the expected gain for each enabled RX \n
                 channel is compared against the thresholds given in this field. The comparison \n
                 result is part of the monitoring report message (Error bit is set if the \n
                 absolute \n
                 value of the errors exceeds respective thresholds). \n
                 1 LSB = 0.1dB, unsigned number \n
                 Valid range: 0 to 65535 (0 to 6553dB) \n
     */
/* *
     * @brief  Reserved for Future use
     */
/* ! \brief
* TX power monitoring configuration
*/
/* *
     * @brief  This field indicates the profile Index for which this configuration applies.
     */
/* *
     * @brief  This field indicates the exact RF frequencies inside the profile's RF band at \n
                 which to measure the required parameters. When each bit in this field is set, \n
                 the measurement at the corresponding RF frequency is enabled w.r.t. the \n
                 profile's RF band. \n

                 Bit number      RF frequency                            RF \n
                     0           Lowest RF frequency in profile's        RF1 \n
                                 sweep bandwidth \n
                     1           Center RF frequency in profile's        RF2 \n
                                 sweep bandwidth \n
                     2           Highest RF frequency in profile's       RF3 \n
                                 sweep bandwidth \n
                 The RF Name column is mentioned here to set the convention for the purpose \n
                 of reporting and describing many monitoring packets. \n
     */
/* *
     * @brief  Reserved for Future use
     */
/* !< Value    Definition \n
            0     Report is sent every monitoring period without threshold check \n
            1     Report is send only upon a failure (after checking for thresholds) \n
            2     Report is sent everymonitoring period with threshold check. \n
     */
/* *
     * @brief  Reserved for Future use
     */
/* *
     * @brief  The magnitude of difference between the programmed and measured TX power for \n
                 each enabled channel at each enabled RF frequency, is compared against this \n
                 threshold. The comparison result is part of the monitoring report message(Error \n
                 bit is set if any measurement is above this threshold). \n
                 1 LSB = 0.1 dB \n
                 Valid range: 0 to 65535 (0 to 6553dB) \n
     */
/* *
     * @brief  The magnitude of measured TX power flatness error, for each enabled channel, is \n
                 compared against this threshold. The flatness error for a channel is defined as \n
                 the peak to peak variation across RF frequencies. The comparison result is part \n
                 of the monitoring report message(Error bit is set if any measurement is above \n
                 this threshold). \n
                 1 LSB = 0.1 dB \n
                 Valid range: 0 to 65535 (0 to 6553dB) \n
                 This flatness check is applicable only if multiple RF Frequencies are enabled. \n
     */
/* *
     * @brief  Reserved for Future use
     */
/* *
     * @brief  Reserved for Future use
     */
/* ! \brief
* TX power monitoring configuration
*/
/* *
     * @brief  Power Monitoring Configuration for Tx0
     */
/* *
     * @brief  Power Monitoring Configuration for Tx1
     */
/* *
     * @brief  Power Monitoring Configuration for Tx2
     */
/* ! \brief
* TX ballbreak monitoring configuration
*/
/* *
     * @brief  Value    Definition \n
                0      Report is sent every monitoring period without threshold check \n
                1      Report is send only upon a failure (after checking for thresholds) \n
                2      Report is sent every monitoring period with threshold check. \n
     */
/* *
     * @brief  Reserved for Future use
     */
/* !< The TX reflection coefficient's magnitude for each enabled channel is compared against \n
         the threshold given here. The comparison result is part of the monitoring report \n
         message (Error bit is set if the measurement is higher than or equal to this \n
         threshold, with the units of both quantities being the same). \n
         1 LSB = 0.1 dB, signed number \n
         Valid range: -32767 to +32767 (-3276dB to +3276dB) \n
     */
/* *
     * @brief  For xWR1xxx devices: \n
                    This field is reserved. Set to 0x0. \n
               For xWR6x43 devices: \n
                    Start frequency of the monitoring chirp. \n
                    For 60GHz Devices (57GHz to 63.8Ghz): 
                    1 LSB = 2.7e9/2^26 = 40.233 Hz \n
                    Valid range: Only even numbers from 0x5471C71C to 0x5E84BDA1 \n
     */
/* *
     * @brief  Reserved for Future use
     */
/* ! \brief
* TX ballbreak monitoring configuration
*/
/* *
     * @brief  Tx ballbreak monitoring config for Tx0
     */
/* *
     * @brief  Tx ballbreak monitoring config for Tx1
     */
/* *
     * @brief  Tx ballbreak monitoring config for Tx2.
     */
/* ! \brief
* TX gain and phase mismatch monitoring configuration
*/
/* *
     * @brief  This field indicates the Profile Index for which this monitoring configuration \n
                 applies. The TX settings corresponding to this profile index are used during \n
                 the monitoring. The RX gain used in this measurement may differ from the given \n
                 profile's RX gain. \n
     */
/* *
     * @brief  This field indicates the exact RF frequencies inside the profile's RF band at \n
                 which to measure the required parameters. When each bit in this field is set, \n
                 the measurement at the corresponding RF frequency is enabled wrt the profile's \n
                 RF band. \n
                 Bit         RF frequency                            RF \n
                 number                                              name \n
                 0           Lowest RF frequency in profile's        RF1 \n
                             sweep bandwidth \n
                 1           Center RF frequency in profile's        RF2 \n
                             sweep bandwidth \n
                 2           Highest RF frequency in profile's       RF3 \n
                             sweep bandwidth \n
                 The RF Name column is mentioned here to set the convention for the purpose of \n
                 reporting and describing many monitoring packets. \n
     */
/* *
     * @brief  This field indicates the TX channels that should be compared for gain and phase \n
                 balance. Setting the corresponding bit to 1 enables that channel for imbalance \n
                 measurement \n
                 Bit number     TX Channel \n
                     0          TX0 \n
                     1          TX1 \n
                     2          TX2 \n
     */
/* *
     * @brief  This field indicates the RX channels that should be enabled for TX to RX loopback
               measurement. Setting the corresponding bit to 1 enables that channel for imbalance
               measurement.
               Bit   RX Channel
                0    RX0
                1    RX1
                2    RX2
                3    RX3
     */
/* *
     * @brief  Value     Definition \n
                 0       Report is sent every monitoring period without threshold check \n
                 1       Report is send only upon a failure (after checking for thresholds) \n
                 2       Report is sent every monitoring period with threshold check. \n
     */
/* *
     * @brief  Reserved for Future use
     */
/* *
     * @brief  The magnitude of difference between measured TX powers across the enabled \n
                 channels at each enabled RF frequency is compared against this threshold. The \n
                 comparison result is part of the monitoring report message(Error bit is set if \n
                 the measurement is above this threshold). Before the comparison, the measured \n
                 gains for each RF and RX are adjusted by subtracting the offset given in the \n
                 TX_GAIN_MISMATCH_OFFSET_VALUE field. \n
                 1 LSB = 0.1dB, signed number \n
                 Valid range: 0 to 65535 (0 to 6553dB) \n
     */
/* *
     * @brief  The magnitude of measured TX phase mismatch across the enabled channels at each \n
                 enabled RF frequency is compared against this threshold. The comparison result \n
                 is part of the monitoring report message (Error bit is set if any measurement \n
                 is above this threshold). Before the comparison, the measured gains for each RF \n
                 and RX are adjusted by subtracting the offset given in the \n
                 TX_PHASE_MISMATCH_OFFSET_VALUE field. \n
                 1 LSB = 360(degree)/ 2^16 , unsigned number \n
                 Valid range: corresponding to 0 degree to 359.9 degree. \n
     */
/* *
     * @brief  The offsets to be subtracted from the measured TX gain for each TX and RF before \n
                 the relevant threshold comparisons are given here. Byte numbers corresponding \n
                 to different RX and RF, in this field are here: \n
                         RF1      RF2        RF3 \n
                TX0     [1:0]    [7:6]      [13:12] \n
                TX1     [3:2]    [9:8]      [15:14] \n
                TX2     [5:4]    [11:10]    [17:16] \n
                 1 LSB = 0.1 dB \n
                 Only the entries of enabled RF Frequencies and enabled TX \n
                 channels are considered. \n
     */
/* *
     * @brief  The offsets to be subtracted from the measured TX phase for each TX and RF before \n
                 the  \n
                 relevant threshold comparisons are given here. Byte numbers corresponding to  \n
                 different RX and RF, in this field are here: \n
                         RF1      RF2      RF3 \n
                 TX0     [1:0]    [7:6]    [13:12] \n
                 TX1     [3:2]    [9:8]    [15:14] \n
                 TX2     [5:4]    [11:10]  [17:16] \n
                 1 LSB = 360(degree)/216. \n
                 Only the entries of enabled RF Frequencies and enabled TX channels \n
                 are considered. \n
     */
/* *
     * @brief  Reserved for Future use
     */
/* *
     * @brief  Reserved for Future use
     */
/* ! \brief
* TX BPM monitoring configuration
*/
/* *
     * @brief  This field indicates the Profile Index for which this configuration applies.
     */
/* *
     * @brief  This field indicates the phase shifter monitoring configuration. \n
               Bit     Definition \n
               5:0     Phase shifter monitoring increment value \n
                       1 LSB = 5.625 degree \n
               6       Phase shifter monitoring auto increment enabled. On each FTTI phase shift \n
                       value increment by mentioned increment value at bit 0:5 \n
               7       Phase shifter monitoring enabled \n
               @note : Phase shifter monitoring control is not supported in this release for all 
                       devices. This is a RESERVED field and should be set to 0. \n
     */
/* *
     * @brief  Phase1 of the phase shifter of TX which needs to be monitored
               1 LSB = 5.625 degree
     */
/* *
     * @brief  Phase2 of the phase shifter of TX which needs to be monitored
               1 LSB = 5.625 degree
     */
/* *
     * @brief  Value      Definition \n
                 0       Report is sent every monitoring period without threshold check \n
                 1       Report is send only upon a failure (after checking for thresholds) \n
                 2       Report is sent every monitoring period with threshold check. \n
     */
/* *
     * @brief  This field indicates the RX channels that should be enabled for TX to RX loopback
               measurement. Setting the corresponding bit to 1 enables that channel for imbalance
               measurement.
               Bit   RX Channel
                0    RX0
                1    RX1
                2    RX2
                3    RX3
     */
/* *
     * @brief  The deviation of the TX output phase difference between the two BPM settings from \n
                 the ideal 180o is compared against the threshold given here. The comparison \n
                 result is part of the monitoring report message (Error bit is set if the \n
                 measurement is lower than this threshold, with the units of both quantities \n
                 being the same). \n
                 1 LSB = 360(degree) /2^16. \n
                 Valid range: corresponding 0 degree to 359.9 degree \n
     */
/* *
     * @brief  The deviation of the TX output amplitude difference between the two BPM settings \n
                 is compared against the threshold given here. The comparison result is part of \n
                 the monitoring report message (Error bit is set if the measurement is lower \n
                 than this threshold, with the units of both quantities being the same). \n
                 1 LSB = 0.1 dB \n
                 Valid range: 0 to 65535 (0 to 6553dB) \n
     */
/* *
     * @brief  Maximum threshold for the difference in the 2 configured phase shift values
               1 LSB = 5.625 degree
     */
/* *
    * @brief   Minimum threshold for the difference in the 2 configured phase shift values
               1 LSB = 5.625 degree
    */
/* *
     * @brief  Reserved for Future use
     */
/* ! \brief
* TX BPM monitoring configuration
*/
/* *
     * @brief  Tx-0 BPM monitoring config
     */
/* *
     * @brief  Tx-1 BPM monitoring config
     */
/* *
     * @brief  Tx-2 BPM monitoring config
     */
/* ! \brief
* Synthesizer frequency monitoring configuration
*/
/* *
     * @brief  This field indicates the Profile Index for which this configuration applies.
     */
/* *
     * @brief  Value    Definition \n
                 0     Report is sent every monitoring period without threshold check \n
                 1     Report is send only upon a failure (after checking for thresholds) \n
                 2     Report is sent every monitoring period with threshold check \n
     */
/* *
     * @brief  During the chirp, the error of the measured instantaneous chirp frequency w.r.t. \n
                 the desired value is continuously compared against this threshold. \n
                 The comparison result is part of the monitoring report message (Error bit is \n
                 set if the measurement is above this threshold, ever during the previous \n
                 monitoring period). \n
                 1 LSB = 10 kHz \n
                 Valid range: 0 to 65535 (0 to 655 MHz) \n
     */
/* *
     * @brief  This field determines when the monitoring starts in each \n
                 chirp relative to the start of the ramp. \n
                 1 LSB = 0.2us, signed number \n
                 Valid range: -25us to 25us \n
                 Recommended value: 6us or above \n
     */
/* *
     * @brief  This field configures whether this monitor should be done \n
               for functional active chirps (mode 0) or non live monitor chirps. In case of non \n
               live monitor, the configuration needs to be sent twice for two VCOs \n
               (use mode 1 and 2). \n
               Value    Definition \n
                 0     LIVE_CONFIG, The profile configuration for live mode is picked
                       from this API, supported only in master/single-chip mode. \n
                 1     VCO1_CONFIG, The profile configuration for Non-live mode is picked from 
                       this API for VCO1 monitor profile, supported in all modes (master, slave 
                       and single-chip). \n
                 2     VCO2_CONFIG, The profile configuration for Non-live mode is picked from 
                       this API for VCO2 monitor profile, supported in all modes (master, slave 
                       and single-chip). \n
               @note : This feature is supported only on xWR6843 device. \n
     */
/* *
     * @brief  This bit mask can be used to enable/disable the monitoring of non-live VCO profiles,
               this helps to control monitoring of only single VCO if needed. This setting should 
               be same in both VCO settings. \n
               Bits    Definition \n
                b0     Enable VCO1 non-live monitor \n
                b1     Enable VCO2 non-live monitor \n
                b31:2  RESERVED \n
               @note : This field is applicable only on xWR6843 device. \n
     */
/* *
     * @brief  Reserved for Future use
     */
/* *
     * @brief  Reserved for Future use
     */
/* ! \brief
* External analog signals monitoring configuration
*/
/* *
     * @brief  Value    Definition \n
               0       Report is sent every monitoring period without threshold check \n
               1       Report is send only upon a failure (after checking for thresholds) \n
               2       Report is sent every monitoring period with threshold check \n
     */
/* *
     * @brief  Reserved for Future use
     */
/* *
     * @brief  This field indicates the sets of externally fed DC signals which are to be \n
                 monitored using GPADC. When each bit in this field is set, the corresponding \n
                 signal is monitored. The monitored signals are compared against programmed \n
                 limits. The comparison result is part of the monitoring report message. \n
                 Bit Location  SIGNAL \n
                     0         ANALOGTEST1 \n
                     1         ANALOGTEST2 \n
                     2         ANALOGTEST3 \n
                     3         ANALOGTEST4 \n
                     4         ANAMUX \n
                     5         VSENSE \n
                     Others    RESERVED \n
     */
/* *
     * @brief  This field indicates the sets of externally fed DC signals which are to be \n
                 buffered before being fed to the GPADC. When each bit in this field is set, the \n
                 corresponding signal is buffered before the GPADC. The monitored signals are \n
                 compared against programmed limits. The comparison result is part of the \n
                 monitoring report message. \n
                 Bit      SIGNAL \n
                 0        ANALOGTEST1 \n
                 1        ANALOGTEST2 \n
                 2        ANALOGTEST3 \n
                 3        ANALOGTEST4 \n
                 4        ANAMUX \n
                 Others   RESERVED \n
     */
/* *
     * @brief  After connecting an external signal to the GPADC, the amount of time to wait for \n
                 it to settle before taking GPADC samples is programmed in this field. For each \n
                 signal, after that settling time, GPADC measurements take place for 6.4us \n
                 (averaging 4 samples of the GPADC output).The byte locations of the settling \n
                 times for each signal are tabulated here: \n
                 Byte Location   SIGNAL \n
                     0           ANALOGTEST1 \n
                     1           ANALOGTEST2 \n
                     2           ANALOGTEST3 \n
                     3           ANALOGTEST4 \n
                     4           ANAMUX \n
                     5           VSENSE \n
                     1 LSB = 0.8us \n
                 Valid range: 0 to 12us \n
                 Valid programming condition: all the signals that are enabled \n
                 should take a total of <100us, including the programmed settling \n
                 times and a fixed 6.4us of measurement time per enabled signal. \n
     */
/* *
     * @brief  The external DC signals measured on GPADC are compared against these minimum and \n
                 maximum thresholds. The comparison result is part of the monitoring report \n
                 message (Error bit is set if any measurement is outside this (minimum, maximum) \n
                 range). \n
                 Byte Location  Threshold     SIGNAL \n
                     0           Minimum      ANALOGTEST1 \n
                     1           Minimum      ANALOGTEST2 \n
                     2           Minimum      ANALOGTEST3 \n
                     3           Minimum      ANALOGTEST4 \n
                     4           Minimum      ANAMUX \n
                     5           Minimum      VSENSE \n
                     6           Maximum      ANALOGTEST1 \n
                     7           Maximum      ANALOGTEST2 \n
                     8           Maximum      ANALOGTEST3 \n
                     9           Maximum      ANALOGTEST4 \n
                     10          Maximum      ANAMUX \n
                     11          Maximum      VSENSE \n
                 1 LSB = 1.8V / 256 \n
                 Valid range: 0 to 255(0 to 1.79V) \n
     */
/* *
     * @brief  Reserved for Future use
     */
/* *
     * @brief  Reserved for Future use
     */
/* *
     * @brief  Reserved for Future use
     */
/* ! \brief
* Internal signals in the TX path monitoring configuration
*/
/* *
     * @brief  The RF analog settings corresponding to this profile are used for monitoring the \n
                 enabled signals, using test chirps (static frequency, at the center of the \n
                 profile's RF frequency band). \n
     */
/* *
     * @brief  Value   Definition \n
                 0    RESERVED \n
                 1    Report is send only upon a failure(after checking for thresholds) \n
                 2    Report is sent every monitoring period with threshold check. \n
     */
/* *
     * @brief  The TX phase shifter DAC monitor delta threshold \n
               1 LSB = 1.8V/1024 \n
               This field is applicable only for xWR6843 and xWR1843 devices. \n
               Value 0: TX_PS_DAC_MON is disabled \n
               Valid range: 1 to 1023 \n
     */
/* *
     * @brief  Reserved for Future use
     */
/* ! \brief
* Internal signals in the TX path monitoring configuration
*/
/* *
     * @brief  Internal signals in the Tx-0 path monitoring config
     */
/* *
     * @brief  Internal signals in the Tx-1 path monitoring config
     */
/* *
     * @brief  Internal signals in the Tx-2 path monitoring config
     */
/* *
      * @brief  The RF analog settings corresponding to this profile are used for monitoring the \n
                 enabled signals, using test chirps(static frequency,at the center of the \n
                 profile's RF frequency band). \n
     */
/* *
     * @brief  Value   Definition \n
                 0    RESERVED \n
                 1    Report is send only upon a failure(after checking for thresholds) \n
                 2    Report is sent every monitoring period with threshold check. \n
     */
/* *
     * @brief  Reserved for Future use
     */
/* *
     * @brief  Reserved for Future use
     */
/* ! \brief
* Internal signals for PM, CLK and LO monitoring configuration
*/
/* *
     * @brief  The RF analog settings corresponding to this profile are used for monitoring the \n
                 enabled signals, using test chirps(static frequency, at the center of the  \n
                 profile's RF frequency band). \n
     */
/* *
     * @brief  Value   Definition \n
                 0    RESERVED \n
                 1    Report is send only upon a failure(after checking for thresholds) \n
                 2    Report is sent every monitoring period with threshold check. \n
     */
/* *
     * @brief  Value   Definition \n
                 0    20GHz SYNC monitoring disabled \n
                 1    FMCW_SYNC_IN monitoring enabled \n
                 2    FMCW_SYNC_OUT monitoring enabled \n
                 3    FMCW_SYNC_CLKOUT monitoring enabled \n
        @note : 20G signal monitoring control is not supported in this release for all devices. 
                This is a RESERVED field and should be set to 0. \n
     */
/* *
     * @brief  Minimum threshold for 20GHz monitoring\n
               1 LSB = 1 dBm
     */
/* *
     * @brief  Maximum threshold for 20GHz monitoring\n
               1 LSB = 1 dBm
     */
/* *
     * @brief  Reserved for Future use
     */
/* *
     * @brief  Reserved for Future use
     */
/* ! \brief
* Internal signals for GPADC monitoring configuration
*/
/* *
     * @brief  Value    Definition \n
                0      RESERVED \n
                1      Report is send only upon a failure (after checking for thresholds) \n
                2      Report is sent every monitoring period with threshold check. \n
     */
/* *
     * @brief  Reserved for Future use
     */
/* *
     * @brief  Reserved for Future use
     */
/* *
     * @brief  Reserved for Future use
     */
/* ! \brief
* Internal signals for PLL control voltage monitoring configuration
*/
/* *
     * @brief  Value    Definition \n
                0      RESERVED \n
                1      Report is send only upon a failure (after checking for thresholds) \n
                2      Report is sent every monitoring period with threshold check. \n
     */
/* *
     * @brief  Reserved for Future use
     */
/* *
     * @brief  This field indicates the sets of signals which are to be monitored. When each bit \n
                in this field is set, the corresponding signal set is monitored using test \n
                chirps. Rest of the RF analog may not be ON during these test chirps. The APLL \n
                VCO control voltage can be monitored. The Synthesizer VCO control voltage for \n
                both VCO1 and VCO2 can be monitored, while operating at their respective \n
                minimum and maximum frequencies, and their respective VCO slope (Hz/V) can be \n
                monitored if both frequencies are enabled for that VCO. The monitored signals \n
                are compared against internally chosen valid limits. The comparison results are \n
                part of the monitoring \n
                report message. \n
                        Bit Location   SIGNAL \n
                            0          APLL_VCTRL \n
                            1          SYNTH_VCO1_VCTRL \n
                            2          SYNTH_VCO2_VCTRL \n
                          15:3         RESERVED \n
                The synthesizer VCO extreme frequencies are: \n
                Synthesizer VCO      Frequency Limits (Min, Max) \n
                    VCO1             (76GHz, 78GHz) \n
                    VCO2             (77GHz, 81GHz) \n
                Synthesizer measurements are done with TX switched off to avoid emissions. \n
     */
/* *
     * @brief  Reserved for Future use
     */
/* ! \brief
* Internal signals for DCC based clock monitoring configuration
*/
/* *
     * @brief  Value    Definition \n
                0      RESERVED \n
                1      Report is send only upon a failure (after checking for thresholds) \n
                2      Report is sent every monitoring period with threshold check. \n
     */
/* *
     * @brief  Reserved for Future use
     */
/* *
     * @brief  This field indicates which pairs of clocks to monitor. When a bit in the field is \n
                 set to 1, the firmware monitors the corresponding clock pair by deploying the \n
                 hardware's Dual Clock Comparator in the corresponding DCC mode. \n
                 Bit  CLOCK PAIR \n
                 0    BSS_600M \n
                 1    BSS_200M \n
                 2    BSS_100M \n
                 3    GPADC_10M \n
                 4    RCOSC_10M \n
                 15:5 RESERVED \n
                 The comparison results are part of the monitoring report message. The \n
                 definition of the clock pairs and their error thresholds for failure reporting \n
                 are given in the table below the message definition. \n
     */
/* *
     * @brief  Reserved for Future use
     */
/* ! \brief
* RX saturation monitoring configuration
*/
/* *
     * @brief  This field indicates the profile Index for which this configuration applies.
     */
/* *
     * @brief  01 => Enable only the ADC saturation monitor \n
               11 => Enable both the ADC and IFA1 saturation monitors \n
     */
/* *
     * @brief  Reserved for Future use
     */
/* *
     * @brief  It specifies the duration of each (primary) time slice. \n
                 1 LSB = 0.16us. \n
                 Valid range: 4 to floor(ADC sampling time us/0.16 us) \n
                 @note : The minimum allowed duration of each (primary) time slice is  \n
                 4 LSBs = 0.64us. Also, the maximum number of (primary) time slices that will \n
                 be monitored in a chirp is 64 so the recommendation is to set this value to \n
                 correspond to (ADC sampling time / 64). If the slice is smaller, such that the \n
                 ADC sampling time is longer than 64 primary slices,some regions of the valid \n
                 duration of a chirp may go un-monitored. \n
     */
/* *
     * @brief  Number of (primary + secondary) time slices to monitor. \n
                 Valid range: 1 to 127 \n
                 @note 1: Together with primarySliceDuration, this determines the full \n
                          duration of the ADC valid time that gets covered by the monitor \n
                          Primary slices = (N+1) / 2 \n
                          Secondary slices = Primary slices - 1 \n
                 @note 2: The total monitoring duration is recommended to be programmed slightly \n
                          smaller than ADC sampling time to avoid last primary slice miss in \n
                          the CQ data.  If this recommendation is not followed and if ADC \n
                          sampling time is less than total requested monitoring duration then \n
                          no error is generated but the total number of slices reported back in \n
                          CQ buffer would be a different value M, which is less than user \n
                          requested value of N. In such cases, there will be (M+1)/2 primary \n
                          slices and (M-1)/2 secondary slices. However, if ADC sampling time is \n
                          such that Secondary (M-1)/2 can be measured and not Primary (M+1)/2, \n
                          then primary slice (M+1)/2 will not be present in the CQ buffer. In \n
                          such scenario, CQ buffer will have the total number of slices reported \n
                          back as M-1 instead of M. \n
     */
/* *
     * @brief  This field is applicable only for SAT_MON_MODE = 0 Masks RX channels used for \n
                 monitoring. In every slice, saturation counts for all unmasked channels are \n
                 added together, and the total is capped to 127. The 8 bits are mapped \n
                 (MSB->LSB) to: \n
                 [RX3Q, RX2Q, RX1Q, RX0Q, RX3I, RX2I, RX1I, RX0I] \n
                 00000000 => All channels unmasked \n
                 11111111 => All channels masked. \n
     */
/* *
     * @brief  Reserved for Future use
     */
/* *
     * @brief  Reserved for Future use
     */
/* *
     * @brief  Reserved for Future use
     */
/* *
     * @brief  Reserved for Future use
     */
/* ! \brief
* Signal and image band energy monitoring configuration
*/
/* *
     * @brief  This field indicates the profile index for which this configuration applies.
     */
/* *
     * @brief  Number of (primary + secondary) slices to monitor Valid range: 1 to 127.
     */
/* *
     * @brief  This field specifies the number of samples constituting each time slice. The \n
                 minimum allowed value for this parameter is 4. \n
                 Valid range: 4 to NUM_ADC_SAMPLES \n
                 @note 1: The maximum number of (primary) time slices that will be monitored in \n
                          a chirp is 64, so our recommendation is that this value should at \n
                          least equal (NUM_ADC_SAMPLES / 64). If the slice is smaller, such \n
                          that the number of ADC samples per chirp is larger than 64 primary \n
                          slices, some regions of the valid duration of a chirp may go \n
                          un-monitored. \n
                 @note 2: In Complex1x mode, the minimum number of samples per slice is 4 and \n
                          for other modes it is 8. Also note that number of samples should be \n
                          an even number. \n
                 @note 3: The total monitoring duration is recommended to program slightly \n
                          smaller than ADC sampling time \n
     */
/* *
     * @brief  Reserved for Future use
     */
/* *
     * @brief  Reserved for Future use
     */
/* ! \brief
* RX mixer input power monitoring configuration
*/
/* *
     * @brief  The RF analog settings corresponding to this profile are used for monitoring RX \n
                 mixer input power using test chirps (static frequency, at the center of the \n
                 profile's RF frequency band). \n
     */
/* *
     * @brief    Indicates the desired reporting verbosity and threshold usage. \n
                 Value = 0    Report is sent every monitoring period without threshold check \n
                 Value = 1    Report is send only upon a failure (after checking for thresholds) \n
                 Value = 2    Report is sent every monitoring period with threshold check. \n
                 Other values: RESERVED. \n
     */
/* *
     * @brief  This field indicates if and which TX channels should be enabled while measuring \n
                 RX mixer input power. Setting a bit to 1 enables the corresponding TX channel. \n
                 Enabling a TX channel may help find reflection power while disabling may help \n
                 find interference power. \n
                   Bit number   TX Channel \n
                   0            TX0 \n
                   1            TX1 \n
                   2            TX2 \n
     */
/* *
     * @brief  Reserved for Future use
     */
/* *
     * @brief  The measured RX mixer input voltage swings during this monitoring is compared \n
                 against the minimum and maximum thresholds configured in this field. The
                 comparison result is part of the monitoring report message (Error bit is set if \n
                 any measurement is outside this (minimum, maximum) range). \n
                   Byte number  Threshold \n
                   0            Minimum Threshold \n
                   1            Maximum Threshold \n
                 Only the RX channels enabled in the static configuration APIs are monitored. \n
                 1 LSB = 1800mV/256, unsigned number \n
                 Valid range: 0 to 255 (0 to 1792.96 mV), maximum threshold >= minimum threshold \n
     */
/* *
     * @brief  Reserved for Future use
     */
/* *
     * @brief  Reserved for Future use
     */
/* ! \brief
* RX signal and image band energy statistics
*/
/* *
     * @brief  Number of (primary + secondary) slices to monitor Valid range: 1 to 127.
     */
/* *
    * @brief The signal band and image band are separated using a two-channel filter
             bank and the ADC sampling time duration is monitored in terms of primary and
             secondary time slices as configured using rlRfRxSigImgMonConfig.
             If Number of Slices configured in rlRfRxSigImgMonConfig is N, then number
             of Primary slices = N+1/2 and number of secondary slices = N-1/2 \n
             For each of the two bands (signal and image), for each time slice,
             the input-referred average power in the slice in negative dBm is
             recorded as an 8-bit unsigned number, with 1 LSB = -0.5 dBm.
             CQ data is stored in 16bit format as follows:
             Pi[1] Ps[1], Si[1] Ss[1], Pi[2] Ps[2], Si[2] Ss[2]......
             Pi[63] Ps[63], Si[63] Ss[63], Pi[64] Ps[64]
             Where,
             Pi = Primary Slice Image Band Power, 1LSB = -0.5dBm
             Ps = Primary Slice Signal Band Power, 1LSB = -0.5dBm
             Si = Secondary Slice Image Band Power, 1LSB = -0.5dBm
             Ss = Secondary Slice Signal Band Power, 1LSB = -0.5dBm

             This data is stored in CQ1 section of CQ RAM. If multiple chirps
             are defined, then this data is concatenated and stored in CQ RAM
             in ping pong manner

             @note : CQ0 section in CQ RAM will contain invalid data and the user should
             to ignore this.
    */
/* ! \brief
* RX ADC and IF saturation information
*/
/* *
     * @brief  Number of (primary + secondary) slices to monitor Valid range: 1 to 127.
     */
/* *
    * @brief The analog to digital interface includes a 100 MHz bit stream indicating
             saturation events in the ADC/IF sections, for each channel. This one-bit
             indicator for each channel is monitored during the ADC sampling time
             duration in a time-sliced manner as defined in rlRfRxIfSatMonConfig.\n
             If Number of Slices configured in rlRfRxSigImgMonConfig is N, then number
             of Primary slices = (N+1)/2 and number of secondary slices = (N-1)/2 \n
             For each time slice, a saturation event count is recorded. This count
             is the sum of saturation event counts across all RX channels selected
             for monitoring, capped to a maximum count of 255 (8 bits).\n
             CQ data is stored in 16bit format as follows:
             P[1], S[1], P[2] S[2]......P[63], S[63], P[64]
             Where,
             P[n] = indicates the accumulated saturation count for all enabled RX
                    channels in primary slice n
             S[n] = indicates the accumulated saturation count for all enabled RX
                    channels in secondary slice n

             This data is stored in CQ2 section of CQ RAM. If multiple chirps
             are defined, then this data is concatenated and stored in CQ RAM
             in ping pong manner

             @note 1: CQ0 section in CQ RAM will contain invalid data.
             @note 2: This satCqVal data transfer happen through DMA not through SPI. So there
                     is no problem of endianess.

    */
/* *
     * @brief  Reserved for Future use
     */
/* *
     * @brief Primary Fault: RX Gain. This field indicates which RX RF sections should have \n
               fault injected. If the fault is enabled, the RX RF gain drops significantly. \n
               The fault can be used to cause significant gain change, inter-RX gain imbalance \n
               and an uncontrolled amount of inter-RX phase imbalance. \n
               This fault can be seen in RX_GAIN_PHASE_MONITOR. \n
                Bit RX Channel  \n
                 0  RX0         \n
                 1  RX1         \n
                 2  RX2         \n
                 3  RX3         \n
                Others RESERVED \n
                For each bit, 1 = inject fault, 0 = remove injected fault
     */
/* *
     * @brief Primary Fault: RX Phase. This field indicates which RX channels should have \n
               fault injected. If the fault is enabled, the RX phase gets inverted. The fault \n
               can be used to cause a controlled amount (180 deg) of inter-RX phase imbalance. \n
               This fault can be seen in RX_GAIN_PHASE_MONITOR. \n
                Bit RX Channel   \n
                 0  RX0          \n
                 1  RX1          \n
                 2  RX2          \n
                 3  RX3          \n
                 Others RESERVED \n
                 For each bit, 1 = inject fault, 0 = remove injected fault
     */
/* *
     * @brief Primary Fault: RX Noise. This field indicates which RX channels should have fault \n
               injected. If the fault is enabled, the RX IFA square wave loopback paths are \n
               engaged to inject high noise at RX IFA input. The fault can be used to cause \n
               significant RX noise floor elevation. \n
               This fault can be seen in RX_GAIN_PHASE_MONITOR and RX_NOISE_FIGURE_MONITOR. \n
                Bit RX Channel   \n
                 0  RX0          \n
                 1  RX1          \n
                 2  RX2          \n
                 3  RX3          \n
                 Others RESERVED \n
                 For each bit, 1 = inject fault, 0 = remove injected fault
     */
/* *
     * @brief Primary Fault: Cutoff frequencies of RX IFA HPF & LPF, IFA Gain. This field \n
               indicates which RX channels should have fault injected. If the fault is enabled, \n
               the RX IFA HPF cutoff frequency becomes very high (about 15MHz). The fault can be \n
               used to cause the measured inband IFA gain, HPF and LPF attenuations to vary from \n
               ideal expectations. \n
               This fault can be seen in RX_IFSTAGE_MONITOR. \n
                Bit RX Channel   \n
                 0  RX0          \n
                 1  RX1          \n
                 2  RX2          \n
                 3  RX3          \n
                 Others RESERVED \n
                 For each bit, 1 = inject fault, 0 = remove injected fault \n
               @note : during the execution of RX_IFSTAGE_MONITOR, the RX_HIGH_NOISE faults are \n
                     temporarily removed.
     */
/* *
     * @brief Primary Fault: RX Mixer LO input swing reduction. This field indicates which RX \n
               channels should have fault injected. If the fault is enabled, the RX mixer LO \n
               input swing is significantly reduced. The fault is primarily expected to be \n
               detected by RX_INTERNAL_ANALOG_SIGNALS_MONITOR (under PWRDET_RX category). \n
                Bit Channel       \n
                 0   RX0 and RX1  \n
                 1   RX2 and RX3  \n
                 Others RESERVED  \n
                 For each bit, 1 = inject fault, 0 = remove injected fault
                 @note : This option is de-featured, please refer latest release note. \n
     */
/* *
     * @brief Primary Fault: TX PA input signal generator turning off. This field indicates \n
               which TX channels should have fault injected. If the fault is enabled, the \n
               amplifier generating TX power amplifier's LO input signal is turned off. The \n
               fault is primarily expected to be detected by \n
               TX<n>_INTERNAL_ANALOG_SIGNALS_MONITOR (under DCBIAS category). \n
                Bit Channel     \n
                 0  TX0 and TX1 \n
                 1  TX2 (applicable only if available in the xWR device) \n
                 Others RESERVED \n
                 For each bit, 1 = inject fault, 0 = remove injected fault
                 @note : This option is de-featured, please refer latest release note. \n
     */
/* *
     * @brief Primary Fault: TX Gain (power). This field indicates which TX RF sections should \n
               have fault injected. If the fault is enabled, the TX RF gain drops significantly. \n
               The fault can be used to cause significant TX output power change, inter-TX gain \n
               imbalance and an uncontrolled amount of inter-TX phase imbalance. \n
               This fault can be seen in TXn_POWER_MONITOR \n
                Bit TX Channel   \n
                 0  TX0          \n
                 1  TX1          \n
                 2  TX2          \n
                 Others RESERVED \n
                 For each bit, 1 = inject fault, 0 = remove injected fault
     */
/* *
     * @brief Primary Fault: TX Phase. This field indicates if TX channels should have fault \n
               injected, along with some further programmability. If the fault is enabled, the \n
               TX BPM polarity (phase) is forced to a constant value as programmed. The fault \n
               can be used to cause a controlled amount (180 degree) of inter-TX phase imbalance \n
               as well as BPM functionality failure. \n
               This fault can be seen in TX_GAIN_PHASE_MISMATCH_MONITOR and 
               TXn_PHASE_SHIFTER_MONITOR. \n
                Bit TX Channel     \n
                 0  TX fault (Common for all TX channels) \n
                 1  RESERVED       \n
                 2  RESERVED       \n
                 3  TX0 BPM VALUE  \n
                 4  TX1 BPM VALUE  \n
                 5  TX2 BPM VALUE  \n
                Others RESERVED    \n
                For each TX<n> BPM VALUE: Applicable only if TX FAULT is enabled. \n
                Value = 0: force TX<n> BPM polarity to 0 \n
                Value = 1: force TX<n> BPM polarity to 1.
     */
/* *
     * @brief Primary Fault: Synthesizer Frequency. This field indicates which Synthesizer \n
               faults should be injected. SYNTH_VCO_OPENLOOP: If the fault is enabled, the \n
               synthesizer is forced in open loop mode with the VCO control voltage forced to \n
               a constant. In order to avoid out of band emissions in this faulty state, this \n
               fault is injected just before the PLL_CONTROL_VOLTAGE_MONITOR is executed and \n
               released just after its completion. \n
               This fault can be seen in PLL_CONTROL_VOLTAGE_MONITOR. \n
               SYNTH_FREQ_MON_OFFSET: If the fault is enabled, the synthesizer frequency \n
               monitor's ideal frequency ramp waveform is forced to be offset from the actual \n
               ramp waveform by a constant, causing monitoring to detect failures. \n
               This fault can be seen in SYNTH_FREQ_MONITOR. \n
               Bit Enable Fault
                0  SYNTH_VCO_OPENLOOP
                1  SYNTH_FREQ_MON_OFFSET
                Others RESERVED
               For each bit, 1 = inject fault, 0 = remove injected fault
     */
/* *
     * @brief This field indicates whether some LDO output voltage faults should be injected or
              not.\n
               Bit Enable Fault \n
               0   SUPPLY_LDO_RX_LODIST_FAULT \n
               Others RESERVED\ n
               SUPPLY_LDO_RX_LODIST_FAULT: if enabled, the RX LO distribution sub system's LDO \n
               output voltage is slightly changed compared to normal levels to cause \n
               INTERNAL_PMCLKLO_SIGNALS_MONITOR to detect failure (under SUPPLY category). \n
               This fault can be seen in INTERNAL_PMCLKLO_SIGNALS_MONITOR. \n
               For each bit, 1 = inject fault, 0 = remove injected fault \n
               @note : This fault injection is ineffective under LDO bypass condition.
     */
/* *
     * @brief This field indicates whether a few miscellaneous faults should be injected or not. \n
               Bit Enable Fault         \n
               0  GPADC_CLK_FREQ_FAULT  \n
               Others RESERVED          \n
               GPADC_CLK_FREQ_FAULT: if enabled, the GPADC clock frequency is slightly increased \n
               compared to normal usage to cause BSS DCC_CLOCK_FREQ_MONITOR to detect failure. \n
               This fault can be seen in DCC_CLOCK_FREQ_MONITOR. \n
               For each bit, 1 = inject fault, 0 = remove injected fault
     */
/* *
     * @brief This field indicates whether faults should be forced in the threshold comparisons \n
               in the software layer of some monitors. If a fault is enabled, the logic in the \n
               min-max threshold comparisons used for failure detection is inverted, causing a \n
               fault to be reported. During these faults, no hardware fault condition is  \n
               injected in the device.
               This fault can be seen in GPADC_INTERNAL_SIGNALS_MONITOR. \n
                Bit Enable Fault
                0  EXTERNAL_ANALOG_SIGNALS_MONITOR
                1  GPADC_INTERNAL_SIGNALS_MONITOR
                Others RESERVED
                For each bit, 1 = inject fault, 0 = remove injected fault
     */
/* *
     * @brief  Reserved for Future use
     */
/* *
     * @brief  Reserved for Future use
     */
/* *
     * @brief  Reserved for Future use
     */
/* *
     * @brief  Reserved for Future use
     */
/* *
*  @defgroup Monitoring Monitoring
*  @brief mmwave radar RF/Sensor Monitoring module
*
* mmWave Device monitoring can be configured through a set of APIs defined in
 this section.  Note that these APIs cover the RF/Analog related monitoring mechanisms.
 There are separate monitoring mechanisms for the digital logic (including the
 processor, memory, etc.) which are internal to the device and not explicitly
 enabled through these APIs.\n

 The monitoring APIs are structured as follows:\n
  -# There are common configuration APIs that control the overall periodicity of
     monitoring, as well as, enable/disable control for each monitoring mechanism.\n
  -# Then, for each monitoring mechanism there is an individual API to allow the
     application to set an appropriate threshold for declaring failure from that monitoring.\n
  -# Also, for each monitoring mechanism, there is an individual API to report
     soft (raw) values from that monitoring.\n
  -# The Raw, Failure or Periodic Monitoring report   are sent to application using
     asynchronous events.

* Below is the list of Monitors and corresponding duration in microseconds \n

<table>
<caption id="AnalogMonitoringDuration">Analog Monitoring Duration</caption>
<tr><th>Monitors                      <th>xWR1xxx(us)<th>xWR6x43(us)
<tr><td>Temperature<td>200<td>200
<tr><td>RX gain phase (assumes 1 RF frequency)<td>1250<td>1250
<tr><td>RX noise figure (assumes 1 RF frequency)<td>250<td>250
<tr><td>RX IF stage (assumes 1 RF frequency)<td>1000<td>1400
<tr><td>TX power (assumes 1 TX, 1 RF frequency)<td>200<td>250
<tr><td>TX ballbreak (assumes 1 TX)<td>250<td>300
<tr><td>TX gain phase mismatch (assumes 1 TX, 1 RF frequency)<td>400<td>400
<tr><td>TX BPM<td>575<td>575
<tr><td>Synthesizer frequency <td>0<td>100
<tr><td>External analog signals (all 6 GPADC channels enabled)<td>150<td>150
<tr><td>TX Internal analog signals (assumes 1 TX)<td>200<td>300
<tr><td>TX Phase shifter DAC monitor (assumes 1 TX),(applicable only for xWR1843/6843 devices)
<td>2200<td>2300
<tr><td>RX internal analog signals<td>1700<td>1900
<tr><td>PMCLKLO internal analog signals<td>400<td>550
<tr><td>GPADC internal signals<td>50<td>50
<tr><td>PLL control voltage <td>250<td>300
<tr><td>Dual clock comparator (assumes 6 clock comparators)<td>110<td>300
<tr><td>RX saturation detector <td>0<td>100
<tr><td>RX signal and image band monitor<td>0<td>100
<tr><td>RX mixer input power<td>600<td>700
<tr><td>Synthesizer frequency non-live monitor<td>NA<td>500
</table>

<table>
<caption id="DigitalMonitoringDuration">Digital Monitoring Duration</caption>
<tr><th>Monitors                      <th>Duration(us)
<tr><td>Periodic configuration register readback<td>70
<tr><td>DFE LBIST monitoring<td>1000
<tr><td>Frame timing monitoring<td>10
</table>

<table>
<caption id="SoftwareOverheads">Software Overheads</caption>
<tr><th>Software Overhead                <th>Duration(us)
<tr><td>Periodic monitoring of stack usage<td>20
<tr><td>Minimum monitoring duration (report formation, digital energy monitor at the end of FTTI, 
temperature read every FTTI)<td>1000
<tr><td>Minimum calibration duration (report formation, temperature read every CAL_MON_TIME_UNIT)
<td>500
<tr><td>Idle time needed per FTTI for windowed watchdog (WDT)
<td>Frame period * CALIB_MON_TIME_UNIT/8 i.e.~12.5% of Frame period * CALIB_MON_TIME_UNIT is 
reserved for watchdog clearing time
</table>

*
*    Related Files
*   - rl_monitoring.c
*  @addtogroup Monitoring
*  @{
*/
/* *****************************************************************************
 * FUNCTION DECLARATIONS
 ******************************************************************************
 */
/* Digital Monitoring Configuration */
/* Digital Monitoring Periodic Configuration */
/* Analog Monitoring Configuration */
/* TemperatureSsensor  Monitoring Configuration */
/* RX Gain and Phase Monitoring Configuration */
/* RX Noise Monitoring Configuration */
/* RX IF Stage Monitoring Configuration */
/* TX Power Monitoring Configuration */
/* TX Ballbreak Monitoring Configuration */
/* TX Gain Phase Mismatch Monitoring Configuration */
/* TX BPM Monitoring Configuration */
/* Synth Freq Monitoring Configuration */
/* External Analog Signals Monitoring Configuration */
/* TX Internal Analog Signals Monitoring Configuration */
/* RX Internal Analog Signals Monitoring Configuration */
/* PM, CLK, LO Internal Analog Signals Monitoring Configuration */
/* GPADC Internal Analog Signals Monitoring Configuration */
/* PLL Control Voltage Monitoring Configuration */
/* Dual Clock Comparator Monitoring Configuration */
/* RX Saturation Monitoring Configuration */
/* RX Signal Image band Monitoring Configuration */
/* RX mixer input power monitoring.Configuration */
/* Analog fault injection Configuration */
/* * @fn rlReturnVal_t rlRfAnaFaultInjConfig(rlUInt8_t deviceMap, rlAnaFaultInj_t* data)
*
*   @brief Sets information related to RF fault injection
*   @param[in] deviceMap - Bitmap of devices to send the message
*   @param[in] data - Fault injection config
*
*   @return rlReturnVal_t Success - 0, Failure - Error Code
*
*   This API is a fault injection API which the host sends to the AWR device. It can be used to
*   inject faults in the analog circuits to test the corresponding monitors. After the faults are
*   injected, the regular enabled monitors will indicate the faults in their associated reports.
*
*   @note 1: This API is not supported in IWR6843 ES1.0 and xWR1443 ES3.0 \n
*   @note 2: This API should be issued when no frames are on-going. \n
*   @note 3: The fault injection should be tested by injecting one fault at a time. \n
*   @note 4: It is recommended to perform device reset after enabling fault injection before
*            moving to functional mode \n
*   @note 5: Some of the fault injection options are de-featured, please refer latest DFP release
*            note for more details. \n
*   @note 6: Disable all runtime calibrations while Fault is injected. \n
*/
/* DesignId : MMWL_DesignId_095 */
/* Requirements : AUTORADAR_REQ-876 */
#[no_mangle]
pub unsafe extern "C" fn rlRfAnaFaultInjConfig(mut deviceMap: rlUInt8_t,
                                               mut data: *mut rlAnaFaultInj_t)
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
                                  0xe as libc::c_uint as rlUInt16_t,
                                  0x1f as libc::c_uint as rlUInt16_t,
                                  data as *mut rlUInt8_t,
                                  ::std::mem::size_of::<rlAnaFaultInj_t>() as
                                      libc::c_ulong as rlUInt16_t)
    }
    return retVal;
}
/*
 * END OF rl_monitoring.c FILE
 */
