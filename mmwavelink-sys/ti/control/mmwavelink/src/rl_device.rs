#![allow(dead_code, mutable_transmutes, non_camel_case_types, non_snake_case,
         non_upper_case_globals, unused_assignments, unused_mut)]
#![register_tool(c2rust)]
#![feature(register_tool)]
extern "C" {
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
    #[no_mangle]
    fn rlDriverInit(deviceMap: rlUInt8_t, clientCb: rlClientCbs_t)
     -> rlReturnVal_t;
    #[no_mangle]
    fn rlDriverAddDevice(deviceMap: rlUInt8_t) -> rlReturnVal_t;
    #[no_mangle]
    fn rlDriverRemoveDevices(deviceMap: rlUInt8_t) -> rlReturnVal_t;
    #[no_mangle]
    fn rlDriverDeInit() -> rlReturnVal_t;
    #[no_mangle]
    fn rlDriverGetHandle() -> *mut rlDriverData_t;
    #[no_mangle]
    fn rlDriverGetPlatformId() -> rlUInt8_t;
    #[no_mangle]
    fn rlDriverIsDeviceMapValid(deviceMap: rlUInt8_t) -> rlReturnVal_t;
    #[no_mangle]
    fn rlDriverCmdInvoke(deviceMap: rlUInt8_t, inMsg: rlDriverMsg_t,
                         outMsg: *mut rlDriverMsg_t) -> rlReturnVal_t;
    #[no_mangle]
    fn rlDriverConfigureCrc(crcType: rlCrcType_t) -> rlReturnVal_t;
    #[no_mangle]
    fn rlDriverConfigureAckTimeout(ackTimeout: rlUInt32_t) -> rlReturnVal_t;
    #[no_mangle]
    fn rlDriverFillPayload(msgId: rlUInt16_t, sbcID: rlUInt16_t,
                           payloadPtr: *mut rlPayloadSb_t,
                           data: *mut rlUInt8_t, inLen: rlUInt16_t);
    #[no_mangle]
    fn rlDriverConstructInMsg(msgId: rlUInt16_t, inMsg: *mut rlDriverMsg_t,
                              payloadPtr: *mut rlPayloadSb_t);
    #[no_mangle]
    fn rlDriverExecuteGetApi(deviceMap: rlUInt8_t, msgId: rlUInt16_t,
                             sbcID: rlUInt16_t, msgData: *mut rlUInt8_t,
                             inLen: rlUInt16_t) -> rlReturnVal_t;
    #[no_mangle]
    fn rlDriverExecuteSetApi(deviceMap: rlUInt8_t, msgId: rlUInt16_t,
                             sbcID: rlUInt16_t, msgData: *mut rlUInt8_t,
                             inLen: rlUInt16_t) -> rlReturnVal_t;
    #[no_mangle]
    fn rlDriverSetRetryCount(retryCnt: rlUInt8_t) -> rlReturnVal_t;
}
pub type rlUInt8_t = libc::c_uchar;
pub type rlUInt16_t = libc::c_ushort;
pub type rlUInt32_t = libc::c_uint;
pub type rlInt8_t = libc::c_char;
pub type rlInt32_t = libc::c_int;
pub type rlComIfHdl_t = *mut libc::c_void;
pub type rlOsiMsgQHdl_t = *mut libc::c_void;
pub type rlOsiSemHdl_t = *mut libc::c_void;
pub type rlOsiMutexHdl_t = *mut libc::c_void;
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
/* ! \brief
* Communication Interface Handle
*/
/* ! \brief
* OS Message Queue Object Handle
*/
/* ! \brief
* OS Semaphore Object Handle
*/
/* ! \brief
* OS Mutex Object Handle
*/
/* ! \brief
* OS Time data type
*/
pub type rlOsiTime_t = rlUInt32_t;
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
pub type RL_P_OSI_SPAWN_ENTRY
    =
    Option<unsafe extern "C" fn(_: *const libc::c_void) -> ()>;
/* ! \brief
* mmWaveLink Event Handler callback
*/
pub type RL_P_EVENT_HANDLER
    =
    Option<unsafe extern "C" fn(_: rlUInt8_t, _: *mut libc::c_void) -> ()>;
#[derive(Copy, Clone)]
#[repr(C)]
pub struct rlComIfCbs {
    pub rlComIfOpen: Option<unsafe extern "C" fn(_: rlUInt8_t, _: rlUInt32_t)
                                -> rlComIfHdl_t>,
    pub rlComIfRead: Option<unsafe extern "C" fn(_: rlComIfHdl_t,
                                                 _: *mut rlUInt8_t,
                                                 _: rlUInt16_t) -> rlInt32_t>,
    pub rlComIfWrite: Option<unsafe extern "C" fn(_: rlComIfHdl_t,
                                                  _: *mut rlUInt8_t,
                                                  _: rlUInt16_t)
                                 -> rlInt32_t>,
    pub rlComIfClose: Option<unsafe extern "C" fn(_: rlComIfHdl_t)
                                 -> rlInt32_t>,
}
/* ! \brief
* Communication interface(SPI, MailBox, UART etc) callback functions
*/
pub type rlComIfCbs_t = rlComIfCbs;
#[derive(Copy, Clone)]
#[repr(C)]
pub struct rlOsiMutexCbs {
    pub rlOsiMutexCreate: Option<unsafe extern "C" fn(_: *mut rlOsiMutexHdl_t,
                                                      _: *mut rlInt8_t)
                                     -> rlInt32_t>,
    pub rlOsiMutexLock: Option<unsafe extern "C" fn(_: *mut rlOsiMutexHdl_t,
                                                    _: rlOsiTime_t)
                                   -> rlInt32_t>,
    pub rlOsiMutexUnLock: Option<unsafe extern "C" fn(_: *mut rlOsiMutexHdl_t)
                                     -> rlInt32_t>,
    pub rlOsiMutexDelete: Option<unsafe extern "C" fn(_: *mut rlOsiMutexHdl_t)
                                     -> rlInt32_t>,
}
/* * @fn rlComIfHdl_t (*rlComIfOpen)(rlUInt8_t deviceIndex, rlUInt32_t flags)
    *
    *...@brief Open Communication interface
    *   @param[in] deviceIndex - Communication Interface to Open
    *   @param[in] flags - Flags to configure the interface
    *
    *   @return rlComIfHdl_t Handle to access the communication interface
    *
    *   Open Communication interface
    */
    /* DesignId : MMWL_DesignId_004 */
    /* Requirements : AUTORADAR_REQ-785 */
/* * @fn rlInt32_t (*rlComIfRead)(rlComIfHdl_t fd, rlUInt8_t *pBuff, rlUInt16_t len)
    *
    *   @brief Read Data from Communication interface
    *   @param[in] fd - Handle to access the communication interface
    *   @param[out] pBuff - Buffer to store data from communication interface
    *   @param[in] len - Read size in bytes
    *
    *   @return rlInt32_t Length of received data
    *
    *   Read Data from Communication interface
    */
    /* DesignId : MMWL_DesignId_004 */
    /* Requirements : AUTORADAR_REQ-785 */
/* * @fn rlInt32_t (*rlComIfWrite)(rlComIfHdl_t fd, rlUInt8_t *pBuff, rlUInt16_t len)
    *
    *   @brief Write Data over Communication interface
    *   @param[in] fd - Handle to access the communication interface
    *   @param[in] pBuff - Buffer containing data to write over communication interface
    *   @param[in] len - write size in bytes
    *
    *   @return rlInt32_t Length of data sent
    *
    *   Write Data over Communication interface
    */
    /* DesignId :  */
    /* Requirements : AUTORADAR_REQ-785 */
/* * @fn rlInt32_t (*rlComIfClose)(rlComIfHdl_t fd)
    *
    *   @brief Close the Communication interface
    *   @param[in] fd - Handle to access the communication interface
    *
    *   @return rlInt32_t Success - 0, Failure - Error code
    *
    *   Close the Communication interface
    */
    /* DesignId : MMWL_DesignId_004 */
    /* Requirements : AUTORADAR_REQ-785 */
/* ! \brief
* OS mutex callback functions
*/
pub type rlOsiMutexCbs_t = rlOsiMutexCbs;
#[derive(Copy, Clone)]
#[repr(C)]
pub struct rlOsiSemCbs {
    pub rlOsiSemCreate: Option<unsafe extern "C" fn(_: *mut rlOsiSemHdl_t,
                                                    _: *mut rlInt8_t)
                                   -> rlInt32_t>,
    pub rlOsiSemWait: Option<unsafe extern "C" fn(_: *mut rlOsiSemHdl_t,
                                                  _: rlOsiTime_t)
                                 -> rlInt32_t>,
    pub rlOsiSemSignal: Option<unsafe extern "C" fn(_: *mut rlOsiSemHdl_t)
                                   -> rlInt32_t>,
    pub rlOsiSemDelete: Option<unsafe extern "C" fn(_: *mut rlOsiSemHdl_t)
                                   -> rlInt32_t>,
}
/* * @fn rlInt32_t (*rlOsiMutexCreate)(rlOsiMutexHdl_t* mutexHdl, rlInt8_t* name)
    *
    *   @brief Create Mutex Object
    *   @param[in] name      - Name to associate with Mutex Object
    *   @param[out] mutexHdl - Pointer to Mutex object
    *
    *   @return rlInt32_t Success - 0, Failure - Error code
    *
    *   Create Mutex Object
    */
    /* DesignId : MMWL_DesignId_004 */
    /* Requirements : AUTORADAR_REQ-784 */
/* * @fn rlInt32_t (*rlOsiMutexLock)(rlOsiMutexHdl_t* mutexHdl, rlOsiTime_t timeout)
    *
    *   @brief Lock Mutex Object
    *   @param[in] mutexHdl - Pointer to Mutex object
    *   @param[in] timeout  - Maximum Time to wait for Mutex Lock
    *
    *   @return rlInt32_t Success - 0, Failure - Error code
    *
    *   OSI Lock Mutex Object.
    *   mmWaveLink requires this Mutex object to be locked to avoid any parallel API
    *   call for the same device instance. In one of the case when mmWaveLink is
    *   underway for a command-response and device sends Async-event message in between,
    *   so at this instance mmWaveLink will try to lock the same Mutex again. Expectation
    *   is that command-response sequence should complete first and that flow unlocks this Mutex
    *   and then only at other context mmWaveLink will cater the Async-event (HostIrq) request.
    *   mmWaveLink passes timeout as max value, where it expects to lock the Mutex for max period.
    *   Any non-zero return value will be treated as error and mmWaveLink will generate
    *   async-event message [SBID RL_RET_CODE_RADAR_OSIF_ERROR] with payload of error code (-10).
    *   This error async-event will be generated only in the interrupt context while catering any
    *   AWR device's async-event message whereas during the CMD-RSP flow that API will return with
    *   error code (-10).
    */
    /* DesignId : MMWL_DesignId_004 */
    /* Requirements : AUTORADAR_REQ-784 */
/* * @fn rlInt32_t (*rlOsiMutexUnLock)(rlOsiMutexHdl_t* mutexHdl)
    *
    *   @brief Unlock Mutex Object
    *   @param[in] mutexHdl - Pointer to Mutex object
    *
    *   @return rlInt32_t Success - 0, Failure - Error code
    *
    *   Unlock Mutex Object. Any non-zero return value will be treated as error and mmWaveLink
    *   will return its own error code (-10).
    */
    /* DesignId : MMWL_DesignId_004 */
    /* Requirements : AUTORADAR_REQ-784 */
/* * @fn rlInt32_t (*rlOsiMutexDelete)(rlOsiMutexHdl_t* mutexHdl)
    *
    *   @brief Destroy Mutex Object
    *   @param[in] mutexHdl - Pointer to Mutex object
    *
    *   @return rlInt32_t Success - 0, Failure - Error code
    *
    *   Destroy Mutex Object
    */
    /* DesignId : MMWL_DesignId_004 */
    /* Requirements : AUTORADAR_REQ-784 */
/* ! \brief
* OS semaphore callback functions
*/
pub type rlOsiSemCbs_t = rlOsiSemCbs;
#[derive(Copy, Clone)]
#[repr(C)]
pub struct rlOsiMsgQCbs {
    pub rlOsiSpawn: Option<unsafe extern "C" fn(_: RL_P_OSI_SPAWN_ENTRY,
                                                _: *const libc::c_void,
                                                _: rlUInt32_t) -> rlInt32_t>,
}
/* * @fn rlInt32_t (*rlOsiSemCreate)(rlOsiSemHdl_t* semHdl, rlInt8_t* name)
    *
    *   @brief Create Semaphore Object
    *   @param[in] name      - Name to associate with Sem Object
    *   @param[out] semHdl   - Pointer to Sem object
    *
    *   @return rlInt32_t Success - 0, Failure - Error code
    *
    *   Create Semaphore Object
    */
    /* DesignId : MMWL_DesignId_004 */
    /* Requirements : AUTORADAR_REQ-784 */
/* * @fn rlInt32_t (*rlOsiSemWait)(rlOsiSemHdl_t* semHdl, rlOsiTime_t timeout)
    *
    *   @brief Wait for Semaphore
    *   @param[in] semHdl   - Pointer to Sem object
    *   @param[in] timeout  - Maximum Time to wait for Semaphore
    *
    *   @return rlInt32_t Success - 0, Failure - Error code
    *
    *   Wait for Semaphore
    */
    /* DesignId : MMWL_DesignId_004 */
    /* Requirements : AUTORADAR_REQ-784 */
/* * @fn rlInt32_t (*rlOsiSemSignal)(rlOsiSemHdl_t* semHdl)
    *
    *   @brief Release Semaphore
    *   @param[in] semHdl - Pointer to Sem object
    *
    *   @return rlInt32_t Success - 0, Failure - Error code
    *
    *   Release Semaphore
    */
    /* DesignId : MMWL_DesignId_004 */
    /* Requirements : AUTORADAR_REQ-784 */
/* * @fn rlInt32_t (*rlOsiSemDelete)(rlOsiSemHdl_t* semHdl)
    *
    *   @brief Destroy Semaphore Object
    *   @param[in] semHdl - Pointer to Sem object
    *
    *   @return rlInt32_t Success - 0, Failure - Error code
    *
    *   Destroy Semaphore Object
    */
    /* DesignId : MMWL_DesignId_004 */
    /* Requirements : AUTORADAR_REQ-784 */
/* ! \brief
* OS message queue/Spawn callback functions
*/
pub type rlOsiMsgQCbs_t = rlOsiMsgQCbs;
#[derive(Copy, Clone)]
#[repr(C)]
pub struct rlOsiCbs {
    pub mutex: rlOsiMutexCbs_t,
    pub sem: rlOsiSemCbs_t,
    pub queue: rlOsiMsgQCbs_t,
}
/* * @fn rlInt32_t (*rlOsiSpawn)(RL_P_OSI_SPAWN_ENTRY pEntry, void* pValue, rlUInt32_t flags)
    *
    *   @brief Calls a function in a different context
    *   @param[in] pEntry - Pointer to Entry Function
    *   @param[in] pValue - Pointer to data passed to function
    *   @param[in] flags  - Flag to indicate preference
    *
    *   @return rlInt32_t Success - 0, Failure - Error code
    *
    *   Calls a function in a different context. mmWaveLink Driver Interrupt handler function
    *   invokes this interface to switch context so that Interrupt Service Routine is executed
    *   immediately. mmWaveLink uses this callback function to invoke 'rlDriverMsgReadSpawnCtx'
    *   funtion to read the async-event message in different interrupt context. Host should
    *   able to handle the error code generated by 'rlDriverMsgReadSpawnCtx' spawned function.
    */
    /* DesignId : MMWL_DesignId_004 */
    /* Requirements : AUTORADAR_REQ-784 */
/* ! \brief
* OS services callback functions
*/
pub type rlOsiCbs_t = rlOsiCbs;
#[derive(Copy, Clone)]
#[repr(C)]
pub struct rlEventCbs {
    pub rlAsyncEvent: Option<unsafe extern "C" fn(_: rlUInt8_t, _: rlUInt16_t,
                                                  _: rlUInt16_t,
                                                  _: *mut rlUInt8_t) -> ()>,
}
/* ! \brief
    * Mutex callback functions.
    */
/* ! \brief
    * Semaphore callback functions.
    */
/* ! \brief
    * OS message queue/Spawn callback functions.
    */
/* ! \brief
* mmWaveLink Asynchronous event callback function
*/
pub type rlEventCbs_t = rlEventCbs;
#[derive(Copy, Clone)]
#[repr(C)]
pub struct rlTimerCbs {
    pub rlDelay: Option<unsafe extern "C" fn(_: rlUInt32_t) -> rlInt32_t>,
}
/* * @fn void (*rlAsyncEvent)(rlUInt8_t devIndex, rlUInt16_t sbId, rlUInt16_t sbLen,
    *                            rlUInt8_t *payload)
    *
    *   @brief Reports Asynchronous events from mmwave radar device such as
    *   device status, exceptions etc
    *   @param[in] devIndex - Device Index to identify source of event
    *   @param[in] subId -  Sub-block Id
    *   @param[in] subLen -  Sub-block data length
    *   @param[in] payload - Sub-block data starting memory address
    *
    *   Reports Asynchronous events from mmwave radar device such as
    *   device status, exceptions etc
    */
    /* DesignId : MMWL_DesignId_004 */
    /* Requirements : AUTORADAR_REQ-783 */
/* ! \brief
* mmWaveLink Timer callback functions
*/
pub type rlTimerCbs_t = rlTimerCbs;
#[derive(Copy, Clone)]
#[repr(C)]
pub struct rlCmdParserCbs {
    pub rlCmdParser: Option<unsafe extern "C" fn(_: rlUInt8_t, _: rlInt32_t)
                                -> rlInt32_t>,
    pub rlPostCnysStep: Option<unsafe extern "C" fn(_: rlUInt8_t)
                                   -> rlInt32_t>,
}
/* ! \brief
* mmWaveLink callback functions for Command parser
*/
pub type rlCmdParserCbs_t = rlCmdParserCbs;
#[derive(Copy, Clone)]
#[repr(C)]
pub struct rlCrcCbs {
    pub rlComputeCRC: Option<unsafe extern "C" fn(_: *mut rlUInt8_t,
                                                  _: rlUInt32_t, _: rlUInt8_t,
                                                  _: *mut rlUInt8_t)
                                 -> rlInt32_t>,
}
/* ! \brief
* mmWaveLink CRC callback function
* @note : Device SPI protocol Limitation for AWR1243: The CRC length of the message
* or Async-event shall be multiple of 4 bytes to enable reliable retry recovery
* mechanism in case of any checksum failure in a message.
*/
pub type rlCrcCbs_t = rlCrcCbs;
#[derive(Copy, Clone)]
#[repr(C)]
pub struct rlDeviceCtrlCbs {
    pub rlDeviceEnable: Option<unsafe extern "C" fn(_: rlUInt8_t)
                                   -> rlInt32_t>,
    pub rlDeviceDisable: Option<unsafe extern "C" fn(_: rlUInt8_t)
                                    -> rlInt32_t>,
    pub rlDeviceMaskHostIrq: Option<unsafe extern "C" fn(_: rlComIfHdl_t)
                                        -> ()>,
    pub rlDeviceUnMaskHostIrq: Option<unsafe extern "C" fn(_: rlComIfHdl_t)
                                          -> ()>,
    pub rlDeviceWaitIrqStatus: Option<unsafe extern "C" fn(_: rlComIfHdl_t,
                                                           _: rlUInt8_t)
                                          -> rlInt32_t>,
    pub rlCommIfAssertIrq: Option<unsafe extern "C" fn(_: rlUInt8_t)
                                      -> rlUInt16_t>,
    pub rlRegisterInterruptHandler: Option<unsafe extern "C" fn(_: rlUInt8_t,
                                                                _:
                                                                    RL_P_EVENT_HANDLER,
                                                                _:
                                                                    *mut libc::c_void)
                                               -> rlInt32_t>,
}
/* * @fn rlInt32_t (*rlComputeCRC)(rlUInt8_t* data, rlUInt32_t dataLen, rlUInt8_t crcType,
    *                  rlUInt8_t* crc)
    *
    *   @brief Compute CRC on the input message
    *   @param[in] data - input message
    *   @param[in] dataLen - size of input message in bytes
    *   @param[in] crcType  - CRC type (0:RL_CRC_TYPE_16BIT_CCITT, 1:RL_CRC_TYPE_32BIT,
    *                         2:RL_CRC_TYPE_64BIT_ISO)
    *   @param[out] crc   - Computed CRC
    *
    *   @return rlInt32_t Success - 0, Failure - Error code
    *
    *   Compute CRC on the input message
    */
    /* DesignId :  */
    /* Requirements :  */
/* ! \brief
* mmWaveLink Device Control, Interrupt callback functions
*/
pub type rlDeviceCtrlCbs_t = rlDeviceCtrlCbs;
/* * @fn rlInt32_t (*rlDeviceEnable)(rlUInt8_t deviceIndex)
    *
    *   @brief Bring mmWave radar device out of Reset
    *   @param[in] deviceIndex -  Index of the device to be enabled
    *
    *   @return rlInt32_t Success - 0, Failure - Error code
    *
    *   Bring mmWave radar device out of Reset. Implement this function to assert the nReset
    *   Pin in mmWave device. Optionally It might require to assert Sense on Power(SOP) Pins
    */
    /* DesignId : MMWL_DesignId_004 */
    /* Requirements : AUTORADAR_REQ-786 */
/* * @fn rlInt32_t (*rlDeviceDisable)(rlUInt8_t deviceIndex)
    *
    *   @brief Power off mmWave radar device
    *   @param[in] deviceIndex -  Index of the device to be disbaled
    *
    *   @return rlInt32_t Success - 0, Failure - Error code
    *
    *   Power off mmWave radar device. Implement this function to de-assert the Reset
    *   Pin in mmWave device
    */
    /* DesignId : MMWL_DesignId_004 */
    /* Requirements : AUTORADAR_REQ-786 */
/* * @fn void (*rlDeviceMaskHostIrq)(rlComIfHdl_t fd)
    *
    *   @brief Masks Host Interrupt
    *   @param[in] fd -  Handle of the device for which interrupt need to be masked
    *
    *   Masks Host Interrupt. If GPIO Interrupt is Level Triggered,
    *   host need to mask the interrupt until the interrupt is serviced
    */
    /* DesignId : MMWL_DesignId_004 */
    /* Requirements : AUTORADAR_REQ-787 */
/* * @fn void (*rlDeviceUnMaskHostIrq)(rlComIfHdl_t fd)
    *
    *   @brief Unmask Host Interrupt
    *   @param[in] fd -  Handle of the device for which interrupt need to be unmasked
    *
    *   Unmask Host Interrupt. If GPIO Interrupt is Level Triggered,
    *   host need to unmask the interrupt once interrupt is processed
    */
    /* DesignId : MMWL_DesignId_004 */
    /* Requirements : AUTORADAR_REQ-787 */
/* * @fn rlInt32_t (*rlDeviceWaitIrqStatus)(rlComIfHdl_t fd, rlUInt8_t highLow)
    *
    *   @brief Polls Host Interrupt Status
    *   @param[in] fd -  Handle of the device for which interrupt need to be polled
    *   @param[in] highLow - Wait for IRQ Level(high/low)
    *
    *   @return rlInt32_t IRQ Line Low - 0, IRQ Line High - 1
    *
    *   mmWave Radar device asserts host IRQ pin to get Host attention. After receiving host
    *   interrupt, host polls Host Interrupt Status, Low on Host IRQ indicate that mmWave device
    *   has written data on communication Interface. This callback should Wait for the IRQ status.
    *   The function waits for the IRQ Level(low/high) based on second argument returns once the
    *   IRQ Level occurs. If HostIRQ is not toggled within timeout (less than ackTimeout), it
    *   should return '-1' value so that it won't be blocked for infinite duration.
    */
    /* DesignId :MMWL_DesignId_004  */
    /* Requirements : AUTORADAR_REQ-787 */
/* * @fn rlUInt16_t (*rlCommIfAssertIrq)(rlUInt8_t highLow)
    *
    *   @brief It assert/de-assert Host IRQ hig/low on MSS to Host for SPI communication
    *   @param[in] highLow - High/low value for Host IRQ
    *
    *   @return rlUInt16_t Success - 0, Failure - Error code
    *
    *   It assert/de-assert Host IRQ hig/low on MSS to Host for SPI communication
    */
    /* DesignId : MMWL_DesignId_004 */
    /* Requirements : AUTORADAR_REQ-787 */
/* * @fn rlInt32_t (*rlRegisterInterruptHandler)(rlUInt8_t deviceIndex,
    *                                               RL_P_EVENT_HANDLER pHandler, void* pValue)
    *
    *   @brief Register Host Interrupt Handler
    *   @param[in] deviceIndex - Device Index to identify source of Host Interrupt
    *   @param[in] pHandler - Interrupt Handler routine
    *   @param[in] pValue   - To Pass any additional data
    *
    *   @return rlInt32_t Success - 0, Failure - Error code
    *
    *   Register Host Interrupt Handler. The Application should store this function handler
    *   and invoke when it receives host Interrupt. The application need to enable GPIO interrupt
    *   in this callback. Event Handler Callback doesn't process the interrupt in the same context
    *   so it is safe to call this handler from ISR directly.
    */
    /* DesignId : MMWL_DesignId_026 */
    /* Requirements : AUTORADAR_REQ-777 */
/* ! \brief
* mmWaveLink print function prototype
*/
pub type rlPrintFptr
    =
    Option<unsafe extern "C" fn(_: *const rlInt8_t, _: ...) -> rlInt32_t>;
#[derive(Copy, Clone)]
#[repr(C)]
pub struct rlDbgCb {
    pub rlPrint: rlPrintFptr,
    pub dbgLevel: rlUInt8_t,
}
/* ! \brief
* mmWaveLink debug callback structure
*/
pub type rlDbgCb_t = rlDbgCb;
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
/* * @fn rlInt32_t (*rlPrint)(const rlInt8_t* format, ...)
    *
    *   @brief Print input message as per the format in the input arguments
    *   @param[in] format  - Format of message and arguments to be printed
    *   @param[in] ...     - Multiple input arguments to be printed
    *
    *   @return rlInt32_t Success- Length of the message written in user's output console in bytes
    *                     Failure- Negative value
    *
    *   Print input message as per the format in the input arguments
    */
    /* DesignId :  */
    /* Requirements :  */
/* *
     * @brief User needs to set debug level such as error, warning, debug, verbose
     */
/* ! \brief
* mmWaveLink client callback structure
*/
pub type rlClientCbs_t = rlClientCbs;
#[derive(Copy, Clone)]
#[repr(C)]
pub struct rlMssEsmFault {
    pub esmGrp1Err: rlUInt32_t,
    pub esmGrp2Err: rlUInt32_t,
    pub reserved0: rlUInt32_t,
    pub reserved1: rlUInt32_t,
}
/* *
     * @brief  Comunication Interface Callback
     */
/* *
     * @brief  Operating System Callback
     */
/* *
     * @brief  Event Callback, required to receive notification
     */
/* *
     * @brief  Device Control Callback
     */
/* *
     * @brief  Timer Callback, required when ACK is enabled
     */
/* *
     * @brief Call back for parsing the Command received at MSS from the Host
                TI Internal Use only
     */
/* *
     * @brief  CRC Callback, required when CRC is enabled
     */
/* *
     * @brief  CRC Types rlCrcType_t 16/32/64
     */
/* *
     * @brief  ACK wait timeout in Milliseconds, 0 - No ACK
               Configuration of the timeout should consider interrupt
               latency in the processor.
     */
/* *
     * @brief  0x0: mmWaveLink runs on Ext Host, \n
               0x1: mmWaveLink runs on MSS, \n
               0x2: mmWaveLink runs on DSS  \n
     */
/* *
     * @brief  xWR1243 + HOST = 0x0, xWR1443 MSS = 0x1, xWR1642 MSS/DSS = 0x2, \n
               xWR1843 MSS/DSS = 0x3, xWR6843 MSS/DSS = 0x4 \n
     */
/* *
     * @brief  Debug Callback, required to receive Debug information
     */
/* ! \brief
 * Structure to hold the MSS ESM Fault data structure for event RL_DEV_AE_MSS_ESMFAULT_SB
 * @note : The FRC lockstep fatal error(BSS module error) is connected to MSS ESM Group 1 lines,
 *         This fatal error must be handled in Host in AWR1243 device (MSS or DSS in xWR1642,
 *         xWR1843 and xWR6843 devices)
 */
pub type rlMssEsmFault_t = rlMssEsmFault;
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
/* *
     * @brief  Bits Definition (0 -- No Error , 1 -- ESM Error)
                 0  NERROR in sync \n
                 1  RESERVED \n
                 2  DMA MPU Region tests \n
                 3  DMA Parity error \n
                 4  RESERVED \n
                 5  RESERVED \n
                 6  DSS CSI parity error \n
                 7  TPCC parity error \n
                 8  CBUF ECC single bit error \n
                 9  CBUF ECC double bit error \n
                 10 RESERVED \n
                 11 RESERVED \n
                 12 RESERVED \n
                 13 Error response from the Peripheral when a DMA transfer is done \n
                 14 RESERVED \n
                 15 VIM RAM double bit errors \n
                 16 RESERVED \n
                 17 MibSPI double bit error test \n
                 18 DSS TPTC0 read MPU error \n
                 19 RESERVED \n
                 20 VIM RAM single bit errors \n
                 21 RESERVED \n
                 22 FRC Lockstep error \n
                 23 RESERVED \n
                 24 RESERVED \n
                 25 MibSPI single bit error test \n
                 26 TCMB0 RAM single bit errors \n
                 27 STC error \n
                 28 TCMB1 RAM single bit errors \n
                 29 DSS TPTC0 write MPU error \n
                 30 DCC compare error \n
                 31 CR4F self-test error.(test of error path by error forcing)  \n
     */
/* *
     * @brief  Bits Definition \n
                 0  TCMA RAM single bit errors \n
                 1  RESERVED \n
                 2  RESERVED \n
                 3  DSS TPTC1 read MPU error \n
                 4  DSS TPTC1 write MPU error \n
                 5  RESERVED \n
                 6  Access error interrupt from FFT ACC \n
                 7  VIM Self-Test Error \n
                 8  RESERVED \n
                 9  RESERVED \n
                 10 RESERVED \n
                 11 RESERVED \n
                 12 RESERVED \n
                 13 RESERVED \n
                 14 RESERVED \n
                 15 RESERVED \n
                 16 RESERVED \n
                 17 RESERVED \n
                 18 RESERVED \n
                 19 RESERVED \n
                 20 RESERVED \n
                 21 RESERVED \n
                 22 RESERVED \n
                 23 RESERVED \n
                 24 RESERVED \n
                 25 radarSS to MSS ESM G2 Trigger \n
                 26 radarSS Mailbox single bit errors \n
                 27 radarSS Mailbox double bit errors \n
                 28 MSS Mailbox single bit errors \n
                 29 MSS Mailbox double bit errors \n
                 30 RESERVED \n
                 31 RESERVED \n
     */
/* *
     * @brief  Reserved for future use
     */
/* *
     * @brief  Reserved for future use
     */
/* ! \brief
 * Structure to hold the MSS/radarSS CPU Fault data strucutre for
 * event RL_DEV_AE_MSS_CPUFAULT_SB and RL_RF_AE_CPUFAULT_SB
 * @note : All the Monitoring Async events will be sent out periodically at calibMonTimeUnit
 *         frame rate (FTTI). The RadarSS/BSS has a queue to hold max 8 transmit API messages
 *         (AEs or Responses), the host shall service all the AEs before start of the next FTTI
 *         epoch to avoid RadarSS Queue full CPU fault fatal error.
 */
pub type rlCpuFault_t = rlCpuFault;
#[derive(Copy, Clone)]
#[repr(C)]
pub struct rlFwVersionParam {
    pub hwVarient: rlUInt8_t,
    pub hwMajor: rlUInt8_t,
    pub hwMinor: rlUInt8_t,
    pub fwMajor: rlUInt8_t,
    pub fwMinor: rlUInt8_t,
    pub fwBuild: rlUInt8_t,
    pub fwDebug: rlUInt8_t,
    pub fwYear: rlUInt8_t,
    pub fwMonth: rlUInt8_t,
    pub fwDay: rlUInt8_t,
    pub patchMajor: rlUInt8_t,
    pub patchMinor: rlUInt8_t,
    pub patchYear: rlUInt8_t,
    pub patchMonth: rlUInt8_t,
    pub patchDay: rlUInt8_t,
    pub patchBuildDebug: rlUInt8_t,
}
/* *
     * @brief  0x0    RF Processor Undefined Instruction Abort \n
               0x1    RF Processor Instruction pre-fetch Abort \n
               0x2    RF Processor Data Access Abort \n
               0x3    RF Processor Firmware Fatal Error \n
               0x4 - 0xFF Reserved \n
     */
/* *
     * @brief  Reserved for future use
     */
/* *
     * @brief  Valid only in case of FAULT type is 0x3, provides the source \n
                     line number at which fatal error occurred. \n
     */
/* *
     * @brief  The instruction PC address at which Fault occurred
     */
/* *
     * @brief  The return address of the function from which fault function \n
                     has been called (Call stack LR) \n
     */
/* *
     * @brief  The CPSR register value at which fault occurred
     */
/* *
     * @brief  The SP register value at which fault occurred
     */
/* *
     * @brief  The address access at which Fault occurred (valid only for fault \n
                     type 0x0 to 0x2) \n
     */
/* *
     * @brief  The status of Error (Error Cause type - valid only for \n
                     fault type 0x0 to 0x2) \n
                     0x000  BACKGROUND_ERR \n
                     0x001  ALIGNMENT_ERR \n
                     0x002  DEBUG_EVENT \n
                     0x00D  PERMISSION_ERR \n
                     0x008  SYNCH_EXTER_ERR \n
                     0x406  ASYNCH_EXTER_ERR \n
                     0x409  SYNCH_ECC_ERR \n
                     0x408  ASYNCH_ECC_ERR \n
     */
/* *
     * @brief  The Source of the Error (Error Source type- valid only for fault type 0x0 to 0x2)\n
                     0x0  ERR_SOURCE_AXI_MASTER \n
                     0x1  ERR_SOURCE_ATCM \n
                     0x2  ERR_SOURCE_BTCM  \n
     */
/* *
     * @brief  The AXI Error type (Error Source type - valid only for fault type 0x0 to 0x2) \n
                     0x0  AXI_DECOD_ERR \n
                     0x1  AXI_SLAVE_ERR  \n
     */
/* *
      * @brief  The Error Access type (Error Access type- valid only for fault type 0x0 to 0x2) \n
                     0x0  READ_ERR \n
                     0x1  WRITE_ERR \n
     */
/* *
     * @brief  The Error Recovery type (Error Recovery type - Valid only for fault \n
                     type 0x0 to 0x2) \n
                     0x0  UNRECOVERY \n
                     0x1  RECOVERY  \n
     */
/* *
     * @brief  Reserved for future use
     */
/* ! \brief
* mmWaveLink firmware version structure
*/
pub type rlFwVersionParam_t = rlFwVersionParam;
#[derive(Copy, Clone)]
#[repr(C)]
pub struct rlSwVersionParam {
    pub major: rlUInt8_t,
    pub minor: rlUInt8_t,
    pub build: rlUInt8_t,
    pub debug: rlUInt8_t,
    pub year: rlUInt8_t,
    pub month: rlUInt8_t,
    pub day: rlUInt8_t,
    pub reserved: rlUInt8_t,
}
/* *
     * @brief  HW variant number
     */
/* *
     * @brief  HW version major number
     */
/* *
     * @brief  HW version minor number
     */
/* *
     * @brief  FW version major number
     */
/* *
     * @brief  FW version major number
     */
/* *
     * @brief  FW version build number
     */
/* *
     * @brief  FW version debug number
     */
/* *
     * @brief  FW Release Year
     */
/* *
     * @brief  FW Release Month
     */
/* *
     * @brief  FW Release Day
     */
/* *
     * @brief  Patch version major number
     */
/* *
     * @brief  Patch version minor number
     */
/* *
     * @brief  Patch Release Year
     */
/* *
     * @brief  Patch Release Month
     */
/* *
     * @brief  Patch Release Day
     */
/* *
     * @brief  Debug and build version
     *         b3:0   Debug version
     *         b7:4   Build version
     */
/* ! \brief
* mmwavelink software version structure
*/
pub type rlSwVersionParam_t = rlSwVersionParam;
#[derive(Copy, Clone)]
#[repr(C)]
pub struct rlVersion {
    pub master: rlFwVersionParam_t,
    pub rf: rlFwVersionParam_t,
    pub mmWaveLink: rlSwVersionParam_t,
}
/* *
     * @brief  SW version major number
     */
/* *
     * @brief  SW version minor number
     */
/* *
     * @brief  SW version buid number
     */
/* *
     * @brief  SW version debug number
     */
/* *
     * @brief  Software Release Year
     */
/* *
     * @brief  Software Release Month
     */
/* *
     * @brief  Software Release Day
     */
/* *
     * @brief  Reserved for future use
     */
/* ! \brief
* mmwavelink version structure
*/
pub type rlVersion_t = rlVersion;
/* *
     * @brief  Master Sub System version
     */
/* *
     * @brief  RF Sub System version
     */
/* *
     * @brief  mmWaveLink version
     */
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
/* ***************************************************************************************
 * FileName     : rl_device.h
 *
 * Description  : This file defines the functions required to Control mmwave radar Device.
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
/* *****************************************************************************
 * TYPE-DEFINE STRUCT/ENUM/UNION DEFINITIONS
 ******************************************************************************
 */
/* ! \brief
* IQ Swap Selection
*/
/* ! \brief
* Channel Interleave Selection
*/
/* ! \brief
* File Dowload data structure
*/
#[derive(Copy, Clone)]
#[repr(C)]
pub struct rlFileData {
    pub chunkLen: rlUInt32_t,
    pub fData: [rlUInt16_t; 116],
}
pub type rlFileData_t = rlFileData;
/* ! \brief
* mmwave radar device MCU Clock output
*/
#[derive(Copy, Clone)]
#[repr(C)]
pub struct rlMcuClkCfg {
    pub mcuClkCtrl: rlUInt8_t,
    pub mcuClkSrc: rlUInt8_t,
    pub srcClkDiv: rlUInt8_t,
    pub reserved: rlUInt8_t,
}
pub type rlMcuClkCfg_t = rlMcuClkCfg;
/* ! \brief
* mmwave radar device PMIC Clock output
*/
#[derive(Copy, Clone)]
#[repr(C)]
pub struct rlPmicClkCfg {
    pub pmicClkCtrl: rlUInt8_t,
    pub pmicClkSrc: rlUInt8_t,
    pub srcClkDiv: rlUInt8_t,
    pub modeSel: rlUInt8_t,
    pub freqSlope: rlUInt32_t,
    pub minNdivVal: rlUInt8_t,
    pub maxNdivVal: rlUInt8_t,
    pub clkDitherEn: rlUInt8_t,
    pub reserved: rlUInt8_t,
}
pub type rlPmicClkCfg_t = rlPmicClkCfg;
/* ! \brief
* mmwave radar device latent fault test
*/
#[derive(Copy, Clone)]
#[repr(C)]
pub struct rllatentFault {
    pub testEn1: rlUInt32_t,
    pub testEn2: rlUInt32_t,
    pub repMode: rlUInt8_t,
    pub testMode: rlUInt8_t,
    pub reserved: rlUInt16_t,
}
pub type rllatentFault_t = rllatentFault;
/* ! \brief
* mmwave radar periodicity test config
*/
#[derive(Copy, Clone)]
#[repr(C)]
pub struct rlperiodicTest {
    pub periodicity: rlUInt32_t,
    pub testEn: rlUInt32_t,
    pub repMode: rlUInt8_t,
    pub reserved0: rlUInt8_t,
    pub reserved1: rlUInt16_t,
}
pub type rlperiodicTest_t = rlperiodicTest;
/* ! \brief
* mmwave radar test pattern config
*/
#[derive(Copy, Clone)]
#[repr(C)]
pub struct rltestPattern {
    pub testPatGenCtrl: rlUInt8_t,
    pub testPatGenTime: rlUInt8_t,
    pub testPatrnPktSize: rlUInt16_t,
    pub numTestPtrnPkts: rlUInt32_t,
    pub testPatRx0Icfg: rlUInt32_t,
    pub testPatRx0Qcfg: rlUInt32_t,
    pub testPatRx1Icfg: rlUInt32_t,
    pub testPatRx1Qcfg: rlUInt32_t,
    pub testPatRx2Icfg: rlUInt32_t,
    pub testPatRx2Qcfg: rlUInt32_t,
    pub testPatRx3Icfg: rlUInt32_t,
    pub testPatRx3Qcfg: rlUInt32_t,
    pub reserved: rlUInt32_t,
}
pub type rltestPattern_t = rltestPattern;
/* ! \brief
* mmwave radar data format config
*/
#[derive(Copy, Clone)]
#[repr(C)]
pub struct rlDevDataFmtCfg {
    pub rxChannelEn: rlUInt16_t,
    pub adcBits: rlUInt16_t,
    pub adcFmt: rlUInt16_t,
    pub iqSwapSel: rlUInt8_t,
    pub chInterleave: rlUInt8_t,
    pub reserved: rlUInt32_t,
}
pub type rlDevDataFmtCfg_t = rlDevDataFmtCfg;
/* ! \brief
* mmwave radar data path config.
*/
#[derive(Copy, Clone)]
#[repr(C)]
pub struct rlDevDataPathCfg {
    pub intfSel: rlUInt8_t,
    pub transferFmtPkt0: rlUInt8_t,
    pub transferFmtPkt1: rlUInt8_t,
    pub cqConfig: rlUInt8_t,
    pub cq0TransSize: rlUInt8_t,
    pub cq1TransSize: rlUInt8_t,
    pub cq2TransSize: rlUInt8_t,
    pub reserved: rlUInt8_t,
}
pub type rlDevDataPathCfg_t = rlDevDataPathCfg;
/* ! \brief
* mmwave radar data path lane enable
*/
#[derive(Copy, Clone)]
#[repr(C)]
pub struct rlDevLaneEnable {
    pub laneEn: rlUInt16_t,
    pub reserved: rlUInt16_t,
}
pub type rlDevLaneEnable_t = rlDevLaneEnable;
/* ! \brief
* DataPath clock configuration
*/
#[derive(Copy, Clone)]
#[repr(C)]
pub struct rlDevDataPathClkCfg {
    pub laneClkCfg: rlUInt8_t,
    pub dataRate: rlUInt8_t,
    pub reserved: rlUInt16_t,
}
pub type rlDevDataPathClkCfg_t = rlDevDataPathClkCfg;
/* ! \brief
* LVDS Lane configuration
*/
#[derive(Copy, Clone)]
#[repr(C)]
pub struct rlDevLvdsLaneCfg {
    pub laneFmtMap: rlUInt16_t,
    pub laneParamCfg: rlUInt16_t,
}
pub type rlDevLvdsLaneCfg_t = rlDevLvdsLaneCfg;
/* ! \brief
* Continous streaming mode configuration
*/
#[derive(Copy, Clone)]
#[repr(C)]
pub struct rlDevContStreamingModeCfg {
    pub contStreamModeEn: rlUInt16_t,
    pub reserved: rlUInt16_t,
}
pub type rlDevContStreamingModeCfg_t = rlDevContStreamingModeCfg;
/* ! \brief
* CSI2 configuration
*/
#[derive(Copy, Clone)]
#[repr(C)]
pub struct rlDevCsi2Cfg {
    pub lanePosPolSel: rlUInt32_t,
    pub reserved1: rlUInt32_t,
}
pub type rlDevCsi2Cfg_t = rlDevCsi2Cfg;
/* ! \brief
* mmwave radar high speed clock configuration
*/
#[derive(Copy, Clone)]
#[repr(C)]
pub struct rlDevHsiClk {
    pub hsiClk: rlUInt16_t,
    pub reserved: rlUInt16_t,
}
pub type rlDevHsiClk_t = rlDevHsiClk;
/* ! \brief
* mmwave radar high speed Data path configuraiton
*/
#[derive(Copy, Clone)]
#[repr(C)]
pub struct rlDevHsiCfg {
    pub datafmt: *mut rlDevDataFmtCfg_t,
    pub dataPath: *mut rlDevDataPathCfg_t,
    pub dataPathClk: *mut rlDevDataPathClkCfg_t,
}
pub type rlDevHsiCfg_t = rlDevHsiCfg;
/* ! \brief
* mmwave radar device config
*/
#[derive(Copy, Clone)]
#[repr(C)]
pub struct rlDevConfig {
    pub aeCrcConfig: rlUInt8_t,
    pub reserved0: rlUInt8_t,
    pub reserved1: rlUInt16_t,
    pub reserved2: rlUInt32_t,
    pub reserved3: rlUInt32_t,
}
pub type rlDevMiscCfg_t = rlDevConfig;
/* ! \brief
* mmwave radar Driver Global Structure
*/
pub type rlDriverData_t = rlDriverData;
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
* mmwave Logging functions
*/
pub type rlLogCtx_t = loggingFunctions;
#[derive(Copy, Clone)]
#[repr(C)]
pub struct loggingFunctions {
    pub rlPrintAr: [rlPrintFptr; 5],
}
/* ! \brief
* Communication handle and device-index for deifferent devices connected to Host
*/
pub type rlComDevInx_t = rlComDevInx;
#[derive(Copy, Clone)]
#[repr(C)]
pub struct rlComDevInx {
    pub comIfHdl: [rlComIfHdl_t; 4],
    pub rlDevIndex: [rlUInt8_t; 4],
}
/* *
     * @brief  Communication Interface Handles
     */
/* *
     * @brief  stores device Index
     */
/* ! \brief
* mmwave radar Driver Function Params
*/
pub type rlFunctionParams_t = rlFunctionParams;
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
* RHCP Async Event structure
*/
pub type rlAsyncEvt_t = rlAsyncEvt;
#[derive(Copy, Clone)]
#[repr(C)]
pub struct rlAsyncEvt {
    pub evtMsg: rlRhcpMsg_t,
}
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
 * FileName     : rl_device.c
 *
 * Description  : This file defines the functions required to Control mmwave radar Device.
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
 * 0.5.2    23Sep2016   Kaushal Kukkar    AUTORADAR-541   xWR1642 Support
 *
 * 0.6.0    15Nov2016   Kaushal Kukkar    AUTORADAR-666   Logging Feature
 *                      Kaushal Kukkar    AUTORADAR-716   Cascade API change
 *
 * 0.7.0    11May2017   Kaushal Kukkar    MMWSDK-362      LDRA static analysis Issue Fix
 *
 * 0.8.6    24Jul2017   Jitendra Gupta    MMWL-19         MSS Test pattern, Monitoring APIs
 *                      Kaushal Kukkar    MMWL-23         Big Endian Support
 *                      Jitendra Gupta    MMWL-26         MSS Data path Get APIs
 *
 * 0.9.1       -        Jitendra Gupta    MMWL-5          Code size optimization
 *
 * 1.2.3.9  21Mar2019   Pavan Penikalapti MMWL-179        Added design & requirement ID for 
 *                                                        rlDeviceSetRetryCount
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
/* * @fn rlReturnVal_t rlDevicePowerOn(rlUInt8_t deviceMap, rlClientCbs_t clientCb)
*
*   @brief Bring mmwave Device Out of Reset
*   @param[in] deviceMap - bitmap of all the connected Device
*   @param[in] clientCb - Client Callbacks for OS/SPI/Interrupts etc
*
*   @return rlReturnVal_t Success - 0, Failure - Error Code
*
*   Bring mmwave Device Out of Reset. Application should wait for async event of
*   subBlock RL_DEV_AE_MSSPOWERUPDONE_SB (in case of AWR1243) before issuing any other APIs
*/
/* DesignId : MMWL_DesignId_004 */
/* Requirements : AUTORADAR_REQ-707 */
#[no_mangle]
pub unsafe extern "C" fn rlDevicePowerOn(mut deviceMap: rlUInt8_t,
                                         mut clientCb: rlClientCbs_t)
 -> rlReturnVal_t {
    let mut retVal: rlReturnVal_t = 0;
    let mut index: rlUInt8_t = 0 as libc::c_uint as rlUInt8_t;
    /* get rlDriver global structure pointer */
    let mut rlDrvData: *mut rlDriverData_t = rlDriverGetHandle();
    /* if driver is already initialized */
    if !rlDrvData.is_null() &&
           (*rlDrvData).isDriverInitialized as libc::c_uint ==
               1 as libc::c_uint {
        /* DeInitialize Device */
        retVal = rlDevicePowerOff()
    } else { retVal = 0 as libc::c_int }
    /* if there is no error found from above conditioning then go ahead and poweron
        mmwavelink & device */
    if retVal == 0 as libc::c_int {
        /* Initialize Host Communication Protocol Driver */
        if rlDriverInit(deviceMap, clientCb) < 0 as libc::c_int {
            /* if driver Init failed then set return error code */
            retVal += -(4 as libc::c_int)
        } else {
            loop 
                 /* Power up mmwave Device */
                 /* loop for all devices connected to device/Host */
                 {
                if deviceMap as libc::c_uint &
                       (1 as libc::c_uint) << index as libc::c_int !=
                       0 as libc::c_uint {
                    /* Enable the 12xx device where it will power up the device and read
                        first Async Event */
                    if clientCb.devCtrlCb.rlDeviceEnable.expect("non-null function pointer")(index)
                           < 0 as libc::c_int {
                        /* set return error code */
                        retVal += -(4 as libc::c_int);
                        /* if device enable is failed the de-init mmwavelink driver */
                        rlDriverDeInit();
                    }
                    /* Reset device Index in DeviceMap for which device has been enabled */
                    deviceMap =
                        (deviceMap as libc::c_uint &
                             !((1 as libc::c_uint) << index as libc::c_int))
                            as rlUInt8_t
                }
                /* increment device index */
                index = index.wrapping_add(1);
                if !(deviceMap as libc::c_uint != 0 as libc::c_uint &&
                         (index as libc::c_uint) < 4 as libc::c_uint) {
                    break ;
                }
            }
        }
    }
    return retVal;
}
/* * @fn rlReturnVal_t rlDeviceAddDevices(rlUInt8_t deviceMap)
*
*   @brief Bring mmwave Device Out of Reset
*   @param[in] deviceMap - bitmap of devices to be connected
*
*   @return rlReturnVal_t Success - 0, Failure - Error Code
*
*   Bring mmwave Device Out of Reset. Application should wait for async event subBlock
*   RL_DEV_AE_MSSPOWERUPDONE_SB before issuing any other APIs. This API is
*   valid only for casecade mode of AWR1243 mmWave device when mmWaveLink instance
*   is running on External Host Processor.
*/
/* DesignId : MMWL_DesignId_021 */
/* Requirements : AUTORADAR_REQ-757 */
#[no_mangle]
pub unsafe extern "C" fn rlDeviceAddDevices(mut deviceMap: rlUInt8_t)
 -> rlReturnVal_t {
    let mut retVal: rlReturnVal_t = 0;
    /* add device for requested deviceMap */
    retVal = rlDriverAddDevice(deviceMap);
    return retVal;
}
/* * @fn rlReturnVal_t rlDeviceRemoveDevices(rlUInt8_t deviceMap)
*
*   @brief Removes connected mmwave devices
*   @param[in] deviceMap - Bitmap of mmwave devices to be disconnected
*
*   @return rlReturnVal_t Success - 0, Failure - Error Code
*
*   @brief Removes mmwave devices and also shuts down the devices. This API is
*   valid only for casecade mode of AWR1243 mmWave device when mmWaveLink instance
*   is running on External Host Processor.
*/
/* DesignId : MMWL_DesignId_034 */
/* Requirements : AUTORADAR_REQ-757 */
#[no_mangle]
pub unsafe extern "C" fn rlDeviceRemoveDevices(mut deviceMap: rlUInt8_t)
 -> rlReturnVal_t {
    let mut retVal: rlReturnVal_t = 0 as libc::c_int;
    let mut index: rlUInt8_t = 0 as libc::c_uint as rlUInt8_t;
    /* get rlDriver global structure pointer */
    let mut rlDrvData: *mut rlDriverData_t = rlDriverGetHandle();
    /* if driver is already initialized and deviceDisable API is not NULL */
    if !rlDrvData.is_null() &&
           (*rlDrvData).isDriverInitialized as libc::c_uint ==
               1 as libc::c_uint &&
           (*rlDrvData).clientCtx.devCtrlCb.rlDeviceDisable.is_some() {
        let mut lclDeviceMap: rlUInt8_t = deviceMap;
        loop  {
            /* loop for device index connected to device/Host */
            if lclDeviceMap as libc::c_uint &
                   (1 as libc::c_uint) << index as libc::c_int !=
                   0 as libc::c_uint {
                /* Enable the 12xx device where it will power up the
                 * device and read first Async Event
                 */
                if (*rlDrvData).clientCtx.devCtrlCb.rlDeviceDisable.expect("non-null function pointer")(index)
                       < 0 as libc::c_int {
                    /* Device Power Off Failed */
                    retVal += -(4 as libc::c_int)
                }
                /* Reset device Index in DeviceMap for which device has been disabled */
                lclDeviceMap =
                    (lclDeviceMap as libc::c_uint &
                         !((1 as libc::c_uint) << index as libc::c_int)) as
                        rlUInt8_t
            }
            /* increment device index */
            index = index.wrapping_add(1);
            if !(lclDeviceMap as libc::c_uint != 0 as libc::c_uint &&
                     (index as libc::c_uint) < 4 as libc::c_uint) {
                break ;
            }
        }
        /* Remove the devices from Driver */
        retVal += rlDriverRemoveDevices(deviceMap)
    } else {
        /* set return error code */
        retVal += -(3 as libc::c_int)
    }
    return retVal;
}
/* * @fn rlReturnVal_t rlDevicePowerOff(void)
*
*   @brief Shutdown mmwave Device
*
*   @return rlReturnVal_t Success - 0, Failure - Error Code
*
*   @brief Stops mmwave operations and also shuts down mmwave device
*/
/* DesignId :  MMWL_DesignId_005 */
/* Requirements : AUTORADAR_REQ-711 */
#[no_mangle]
pub unsafe extern "C" fn rlDevicePowerOff() -> rlReturnVal_t {
    let mut retVal: rlReturnVal_t = 0 as libc::c_int;
    let mut index: rlUInt8_t = 0 as libc::c_uint as rlUInt8_t;
    /* get rlDriver global structure pointer */
    let mut rlDrvData: *mut rlDriverData_t = rlDriverGetHandle();
    /* if driver is already initialized and deviceDisable API is not NULL */
    if !rlDrvData.is_null() &&
           (*rlDrvData).isDriverInitialized as libc::c_uint ==
               1 as libc::c_uint &&
           (*rlDrvData).clientCtx.devCtrlCb.rlDeviceDisable.is_some() {
        let mut deviceMap: rlUInt8_t = (*rlDrvData).deviceMap;
        loop 
             /* disable all connected devices */
             /* if device Index is valid out of deviceMap */
             {
            if deviceMap as libc::c_uint &
                   (1 as libc::c_uint) << index as libc::c_int !=
                   0 as libc::c_uint {
                /* Enable the 12xx device where it will power up the
                 * device and read first Async Event
                 */
                if (*rlDrvData).clientCtx.devCtrlCb.rlDeviceDisable.expect("non-null function pointer")(index)
                       < 0 as libc::c_int {
                    /* Device Power Off Failed */
                    retVal += -(4 as libc::c_int)
                }
                /* Reset device Index in DeviceMap for which device has been disabled */
                deviceMap =
                    (deviceMap as libc::c_uint &
                         !((1 as libc::c_uint) << index as libc::c_int)) as
                        rlUInt8_t
            }
            /* increment device index */
            index = index.wrapping_add(1);
            if !(deviceMap as libc::c_uint != 0 as libc::c_uint ||
                     (index as libc::c_uint) < 4 as libc::c_uint) {
                break ;
            }
        }
        /* DeInitialize Host Communication Protocol Driver */
        retVal += rlDriverDeInit()
    } else {
        /* set return error code */
        retVal += -(3 as libc::c_int)
    }
    return retVal;
}
/* * @fn rlReturnVal_t rlDeviceRfStart(rlUInt8_t deviceMap)
*
*   @brief Enables mmwave RF/Analog Sub system
*   @param[in] deviceMap - Bitmap of devices to send the message
*
*   @return rlReturnVal_t Success - 0, Failure - Error Code
*
*   @brief It enables RF/Analog Subsystem.
*/
/* DesignId : MMWL_DesignId_027 */
/* Requirements : AUTORADAR_REQ-761 */
#[no_mangle]
pub unsafe extern "C" fn rlDeviceRfStart(mut deviceMap: rlUInt8_t)
 -> rlReturnVal_t {
    let mut retVal: rlReturnVal_t = 0;
    /* check if deviceIndex is out of defined value */
    if rlDriverIsDeviceMapValid(deviceMap) != 0 as libc::c_int {
        /* set return error code */
        retVal = -(2 as libc::c_int)
    } else {
        /* Package the command with given data and send it to device */
        retVal =
            rlDriverExecuteSetApi(deviceMap,
                                  0x200 as libc::c_uint as rlUInt16_t,
                                  0 as libc::c_uint as rlUInt16_t,
                                  0 as *mut rlUInt8_t,
                                  0 as libc::c_uint as rlUInt16_t)
    }
    return retVal;
}
/* * @fn rlReturnVal_t rlDeviceFileDownload(rlUInt8_t deviceMap,
*                          rlFileData_t* data, rlUInt16_t remChunks)
*
*   @brief Download mmwave Firmware/Patches over SPI
*   @param[in] deviceMap - Bitmap of devices to send the message
*   @param[in] data  - File Chunk
*   @param[in] remChunks -  Number of remaining Chunks
*
*   @return rlReturnVal_t Success - 0, Failure - Error Code
*
*   Download mmwave Firmware/Patches over SPI. This API is valid only for AWR1243
*   mmWave device when mmWaveLink instance is running on External Host Processor.
*/
/* DesignId :  MMWL_DesignId_006 */
/* Requirements : AUTORADAR_REQ-708 */
#[no_mangle]
pub unsafe extern "C" fn rlDeviceFileDownload(mut deviceMap: rlUInt8_t,
                                              mut data: *mut rlFileData_t,
                                              mut remChunks: rlUInt16_t)
 -> rlReturnVal_t {
    let mut retVal: rlReturnVal_t = 0;
    /* check if deviceIndex is out of defined value */
    if rlDriverIsDeviceMapValid(deviceMap) != 0 as libc::c_int ||
           data.is_null() {
        /* set error code if DeviceMAP is invalid or data pointer is null */
        retVal = -(2 as libc::c_int)
    } else if 0 as libc::c_uint != rlDriverGetPlatformId() as libc::c_uint {
        /* This API is valid only when mmWaveLink instance is running on
       External Host Processor */
        /* set error code of Platform is not set to HOST */
        retVal = -(12 as libc::c_int)
    } else {
        /* LDRA waiver   8 D - DD data flow anomalies found- */
        /* Initialize in-message structure to zero */
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
        /* Initialize out-message structure to zero */
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
        /* Initialize in-payload sub-block structure to zero */
        let mut inPayloadSb: rlPayloadSb_t =
            {
                let mut init =
                    rlPayloadSb{sbid: 0 as libc::c_int as rlUInt16_t,
                                len: 0,
                                pSblkData: 0 as *mut rlUInt8_t,};
                init
            };
        /* Construct command packet */
        rlDriverConstructInMsg(0x204 as libc::c_uint as rlUInt16_t,
                               &mut inMsg, &mut inPayloadSb);
        /* set the remaining chunk in command message */
        inMsg.remChunks = remChunks;
        /* Fill in-message Payload */
        /* AR_CODE_REVIEW MR:R.11.1  <APPROVED> "conversion required." */
        /*LDRA_INSPECTED 95 S */
        rlDriverFillPayload(0x204 as libc::c_uint as rlUInt16_t,
                            0 as libc::c_uint as rlUInt16_t, &mut inPayloadSb,
                            &mut *(*data).fData.as_mut_ptr().offset(0 as
                                                                        libc::c_uint
                                                                        as
                                                                        isize)
                                as *mut rlUInt16_t as *mut rlUInt8_t,
                            (*data).chunkLen as rlUInt16_t);
        /* Send Command to mmWave Radar Device */
        /* LDRA waiver 45 D - can't be NULL */
        /* LDRA waiver 45 D - can't be NULL */
        retVal = rlDriverCmdInvoke(deviceMap, inMsg, &mut outMsg)
    }
    return retVal;
}
/* * @fn rlReturnVal_t rlDeviceGetMssVersion(rlUInt8_t deviceMap, rlFwVersionParam_t *data)
*
*   @brief Get mmWave Master SS version
*   @param[in] deviceMap - Bitmap of devices to send the message
*   @param[out] data  - Container for Version Information
*
*   @return rlReturnVal_t Success - 0, Failure - Error Code
*
*    Get mmWave Master SS version. This API is valid only for AWR1243 mmWave
*    device when mmWaveLink instance is running on External Host Processor.
*/
#[no_mangle]
pub unsafe extern "C" fn rlDeviceGetMssVersion(mut deviceMap: rlUInt8_t,
                                               mut data:
                                                   *mut rlFwVersionParam_t)
 -> rlReturnVal_t {
    let mut retVal: rlReturnVal_t = 0;
    /* check for NULL pointer */
    if data.is_null() {
        /* set error code if data pointer is passed NULL */
        retVal = -(14 as libc::c_int)
    } else if 0 as libc::c_uint != rlDriverGetPlatformId() as libc::c_uint {
        /* This API is valid only when mmWaveLink instance is running on
       External Host Processor */
        /* set error code of Platform is not set to HOST */
        retVal = -(12 as libc::c_int)
    } else {
        /* Package the command with given data and send it to device */
        retVal =
            rlDriverExecuteGetApi(deviceMap,
                                  0x207 as libc::c_uint as rlUInt16_t,
                                  0 as libc::c_uint as rlUInt16_t,
                                  data as *mut rlUInt8_t,
                                  0 as libc::c_uint as rlUInt16_t)
    }
    return retVal;
}
/* * @fn rlReturnVal_t rlDeviceGetRfVersion(rlUInt8_t deviceMap, rlFwVersionParam_t *data)
*
*   @brief Get mmWave RF ROM and patch version
*   @param[in] deviceMap - Bitmap of devices to send the message
*   @param[out] data  - Container for Version Information
*
*   @return rlReturnVal_t Success - 0, Failure - Error Code
*
*    Get mmWave RF ROM and patch version
*/
#[no_mangle]
pub unsafe extern "C" fn rlDeviceGetRfVersion(mut deviceMap: rlUInt8_t,
                                              mut data:
                                                  *mut rlFwVersionParam_t)
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
                                  0 as libc::c_uint as rlUInt16_t,
                                  data as *mut rlUInt8_t,
                                  0 as libc::c_uint as rlUInt16_t)
    }
    return retVal;
}
/* * @fn rlReturnVal_t rlDeviceGetVersion(rlUInt8_t deviceMap, rlVersion_t* data)
*
*   @brief Get mmWave Hardware, Firmware/patch and mmWaveLink version
*   @param[in] deviceMap - Bitmap of devices to send the message
*   @param[out] data  - Container for Version Information
*
*   @return rlReturnVal_t Success - 0, Failure - Error Code
*
*    Get mmWave Hardware, Firmware/patch and mmWaveLink version
*/
/* DesignId : MMWL_DesignId_007 */
/* Requirements : AUTORADAR_REQ-709 */
#[no_mangle]
pub unsafe extern "C" fn rlDeviceGetVersion(mut deviceMap: rlUInt8_t,
                                            mut data: *mut rlVersion_t)
 -> rlReturnVal_t {
    let mut retVal: rlReturnVal_t = 0;
    /* check if deviceIndex is out of defined value */
    if rlDriverIsDeviceMapValid(deviceMap) != 0 as libc::c_int ||
           data.is_null() {
        /* set error code */
        retVal = -(2 as libc::c_int)
    } else {
        /* If mmWaveLink is runing on ext host */
        if 0 as libc::c_uint == rlDriverGetPlatformId() as libc::c_uint {
            /* get mmwave MSS ROM or Application version */
            retVal = rlDeviceGetMssVersion(deviceMap, &mut (*data).master)
        } else {
            /* if mmwavelink is not running on Host then no need to get MSS version */
            retVal = 0 as libc::c_int
        }
        if retVal == 0 as libc::c_int {
            /* get RF (RadarSS) version */
            retVal += rlDeviceGetRfVersion(deviceMap, &mut (*data).rf)
        }
        if retVal == 0 as libc::c_int {
            /* get mmwavelink library version */
            retVal += rlDeviceGetMmWaveLinkVersion(&mut (*data).mmWaveLink)
        }
    }
    return retVal;
}
/* * @fn rlReturnVal_t rlDeviceGetMmWaveLinkVersion(rlSwVersionParam_t* data)
*
*   @brief Get mmWaveLink Version
*   @param[out] data  - Container for Version Information
*
*   @return rlReturnVal_t Success - 0, Failure - Error Code
*
*    Get mmWaveLink Version
*/
/* DesignId : MMWL_DesignId_008 */
/* Requirements : AUTORADAR_REQ-709 */
#[no_mangle]
pub unsafe extern "C" fn rlDeviceGetMmWaveLinkVersion(mut data:
                                                          *mut rlSwVersionParam_t)
 -> rlReturnVal_t {
    let mut retVal: rlReturnVal_t = 0;
    if !data.is_null() {
        /* mmWaveLink Version */
        /* mmWaveLink SW Major verison */
        (*data).major = 1 as libc::c_uint as rlUInt8_t;
        /* mmWaveLink SW Minor verison */
        (*data).minor = 2 as libc::c_uint as rlUInt8_t;
        /* mmWaveLink SW Build verison */
        (*data).build = 6 as libc::c_uint as rlUInt8_t;
        /* mmWaveLink SW Debug verison */
        (*data).debug = 6 as libc::c_uint as rlUInt8_t;
        /* mmWaveLink SW Release Year */
        (*data).year = 20 as libc::c_uint as rlUInt8_t;
        /* mmWaveLink SW Release Month */
        (*data).month = 8 as libc::c_uint as rlUInt8_t;
        /* mmWaveLink SW Release date */
        (*data).day = 27 as libc::c_uint as rlUInt8_t;
        /* Set error code */
        retVal = 0 as libc::c_int
    } else {
        /* set error code */
        retVal = -(2 as libc::c_int)
    }
    return retVal;
}
/* * @fn rlReturnVal_t rlDeviceSetDataFmtConfig(rlUInt8_t deviceMap, rlDevDataFmtCfg_t* data)
*
*   @brief Sets LVDS/CSI2 Data output format
*   @param[in] deviceMap - Bitmap of devices to send the message
*   @param[in] data  - Container for LVDS/CSI2 Data output format
*
*   @return rlReturnVal_t Success - 0, Failure - Error Code
*
*    Sets LVDS/CSI2 Data output format. This API is valid only for AWR1243
*    mmWave device when mmWaveLink instance is running on External Host Processor
*/
/* DesignId : MMWL_DesignId_104 */
/* Requirements : AUTORADAR_REQ-762 */
#[no_mangle]
pub unsafe extern "C" fn rlDeviceSetDataFmtConfig(mut deviceMap: rlUInt8_t,
                                                  mut data:
                                                      *mut rlDevDataFmtCfg_t)
 -> rlReturnVal_t {
    let mut retVal: rlReturnVal_t = 0;
    /* check if deviceIndex is out of defined value */
    if rlDriverIsDeviceMapValid(deviceMap) != 0 as libc::c_int ||
           data.is_null() {
        /* set error code if DeviceMAP is invalid or data pointer is null */
        retVal = -(2 as libc::c_int)
    } else if 0 as libc::c_uint != rlDriverGetPlatformId() as libc::c_uint {
        /* This API is valid only when mmWaveLink instance is running on
       External Host Processor */
        /* set error code of Platform is not set to HOST */
        retVal = -(12 as libc::c_int)
    } else {
        /* Package the command with given data and send it to device */
        retVal =
            rlDriverExecuteSetApi(deviceMap,
                                  0x202 as libc::c_uint as rlUInt16_t,
                                  0x1 as libc::c_uint as rlUInt16_t,
                                  data as *mut rlUInt8_t,
                                  ::std::mem::size_of::<rlDevDataFmtCfg_t>()
                                      as libc::c_ulong as rlUInt16_t)
    }
    return retVal;
}
/* * @fn rlReturnVal_t rlDeviceGetDataFmtConfig(rlUInt8_t deviceMap,
*                                        rlDevDataFmtCfg_t* data)
*
*   @brief Gets LVDS/CSI2 Data output format
*   @param[in] deviceMap - Bitmap of devices to send the message
*   @param[in] data  - Container for LVDS/CSI2 Data output format
*
*   @return rlReturnVal_t Success - 0, Failure - Error Code
*
*   Gets LVDS/CSI2 Data output format. This API is valid only for AWR1243
*   mmWave device when mmWaveLink instance is running on External Host Processor
*/
/* DesignId : MMWL_DesignId_112 */
/* Requirements : AUTORADAR_REQ-762 */
#[no_mangle]
pub unsafe extern "C" fn rlDeviceGetDataFmtConfig(mut deviceMap: rlUInt8_t,
                                                  mut data:
                                                      *mut rlDevDataFmtCfg_t)
 -> rlReturnVal_t {
    let mut retVal: rlReturnVal_t = 0;
    /* check if deviceIndex is out of defined value */
    if rlDriverIsDeviceMapValid(deviceMap) != 0 as libc::c_int ||
           data.is_null() {
        /* set error code if DeviceMAP is invalid or data pointer is null */
        retVal = -(2 as libc::c_int)
    } else if 0 as libc::c_uint != rlDriverGetPlatformId() as libc::c_uint {
        /* This API is valid only when mmWaveLink instance is running on
       External Host Processor */
        /* set error code of Platform is not set to HOST */
        retVal = -(12 as libc::c_int)
    } else {
        /* Package the command with given data and send it to device */
        retVal =
            rlDriverExecuteGetApi(deviceMap,
                                  0x203 as libc::c_uint as rlUInt16_t,
                                  0x1 as libc::c_uint as rlUInt16_t,
                                  data as *mut rlUInt8_t,
                                  0 as libc::c_uint as rlUInt16_t)
    }
    return retVal;
}
/* * @fn rlReturnVal_t rlDeviceSetDataPathConfig(rlUInt8_t deviceMap, rlDevDataPathCfg_t* data)
*
*   @brief Sets LVDS/CSI2 Path Configuration
*   @param[in] deviceMap - Bitmap of devices to send the message
*   @param[in] data  - Container for Path Configuration
*
*   @return rlReturnVal_t Success - 0, Failure - Error Code
*
*   Sets LVDS/CSI2 Data path configuration. This API is valid only for AWR1243
*   mmWave device when mmWaveLink instance is running on External Host Processor
*/
/* DesignId : MMWL_DesignId_105 */
/* Requirements : AUTORADAR_REQ-763 */
#[no_mangle]
pub unsafe extern "C" fn rlDeviceSetDataPathConfig(mut deviceMap: rlUInt8_t,
                                                   mut data:
                                                       *mut rlDevDataPathCfg_t)
 -> rlReturnVal_t {
    let mut retVal: rlReturnVal_t = 0;
    /* check if deviceIndex is out of defined value */
    if rlDriverIsDeviceMapValid(deviceMap) != 0 as libc::c_int ||
           data.is_null() {
        /* set error code if DeviceMAP is invalid or data pointer is null */
        retVal = -(2 as libc::c_int)
    } else if 0 as libc::c_uint != rlDriverGetPlatformId() as libc::c_uint {
        /* This API is valid only when mmWaveLink instance is running on
       External Host Processor */
        /* set error code of Platform is not set to HOST */
        retVal = -(12 as libc::c_int)
    } else {
        /* Package the command with given data and send it to device */
        retVal =
            rlDriverExecuteSetApi(deviceMap,
                                  0x202 as libc::c_uint as rlUInt16_t,
                                  0x2 as libc::c_uint as rlUInt16_t,
                                  data as *mut rlUInt8_t,
                                  ::std::mem::size_of::<rlDevDataPathCfg_t>()
                                      as libc::c_ulong as rlUInt16_t)
    }
    return retVal;
}
/* * @fn rlReturnVal_t rlDeviceGetDataPathConfig(rlUInt8_t deviceMap,
*                                        rlDevDataPathCfg_t* data)
*
*   @brief Gets data path Configuration
*   @param[in] deviceMap - Bitmap of devices to send the message
*   @param[in] data  - Container for Path Configuration
*
*   @return rlReturnVal_t Success - 0, Failure - Error Code
*
*   Gets data path Configuration. This API is valid only for AWR1243 mmWave
*   device when mmWaveLink instance is running on External Host Processor
*/
/* DesignId : MMWL_DesignId_113 */
/* Requirements : AUTORADAR_REQ-763 */
#[no_mangle]
pub unsafe extern "C" fn rlDeviceGetDataPathConfig(mut deviceMap: rlUInt8_t,
                                                   mut data:
                                                       *mut rlDevDataPathCfg_t)
 -> rlReturnVal_t {
    let mut retVal: rlReturnVal_t = 0;
    /* check if deviceIndex is out of defined value */
    if rlDriverIsDeviceMapValid(deviceMap) != 0 as libc::c_int ||
           data.is_null() {
        /* set error code if DeviceMAP is invalid or data pointer is null */
        retVal = -(2 as libc::c_int)
    } else if 0 as libc::c_uint != rlDriverGetPlatformId() as libc::c_uint {
        /* This API is valid only when mmWaveLink instance is running on
       External Host Processor */
        /* set error code of Platform is not set to HOST */
        retVal = -(12 as libc::c_int)
    } else {
        /* Package the command with given data and send it to device */
        retVal =
            rlDriverExecuteGetApi(deviceMap,
                                  0x203 as libc::c_uint as rlUInt16_t,
                                  0x2 as libc::c_uint as rlUInt16_t,
                                  data as *mut rlUInt8_t,
                                  0 as libc::c_uint as rlUInt16_t)
    }
    return retVal;
}
/* * @fn rlReturnVal_t rlDeviceSetLaneConfig(rlUInt8_t deviceMap, rlDevLaneEnable_t* data)
*
*   @brief Sets Lane enable Configuration
*   @param[in] deviceMap - Bitmap of devices to send the message
*   @param[in] data  - Container for lane enable Configuration
*
*   @return rlReturnVal_t Success - 0, Failure - Error Code
*
*   Sets Lane enable configuration. This API is valid only for AWR1243 mmWave
*   device when mmWaveLink instance is running on External Host Processor.
*/
/* DesignId : MMWL_DesignId_033 */
/* Requirements : AUTORADAR_REQ-756, AUTORADAR_REQ-764 */
#[no_mangle]
pub unsafe extern "C" fn rlDeviceSetLaneConfig(mut deviceMap: rlUInt8_t,
                                               mut data:
                                                   *mut rlDevLaneEnable_t)
 -> rlReturnVal_t {
    let mut retVal: rlReturnVal_t = 0;
    /* check if deviceIndex is out of defined value */
    if rlDriverIsDeviceMapValid(deviceMap) != 0 as libc::c_int ||
           data.is_null() {
        /* set error code if DeviceMAP is invalid or data pointer is null */
        retVal = -(2 as libc::c_int)
    } else if 0 as libc::c_uint != rlDriverGetPlatformId() as libc::c_uint {
        /* This API is valid only when mmWaveLink instance is running on
       External Host Processor */
        /* set error code of Platform is not set to HOST */
        retVal = -(12 as libc::c_int)
    } else {
        /* Package the command with given data and send it to device */
        retVal =
            rlDriverExecuteSetApi(deviceMap,
                                  0x202 as libc::c_uint as rlUInt16_t,
                                  0x3 as libc::c_uint as rlUInt16_t,
                                  data as *mut rlUInt8_t,
                                  ::std::mem::size_of::<rlDevLaneEnable_t>()
                                      as libc::c_ulong as rlUInt16_t)
    }
    return retVal;
}
/* * @fn rlReturnVal_t rlDeviceGetLaneConfig(rlUInt8_t deviceMap,
*                                        rlDevLaneEnable_t* data)
*
*   @brief Gets Lane enable Configuration
*   @param[in] deviceMap - Bitmap of devices to send the message
*   @param[out] data - Container for lane enable Configuration
*
*   @return rlReturnVal_t Success - 0, Failure - Error Code
*
*   Gets Lane enable Configuration. This API is valid only for AWR1243 mmWave
*   device when mmWaveLink instance is running on External Host Processor.
*/
/* DesignId : MMWL_DesignId_115 */
/* Requirements : AUTORADAR_REQ-765, AUTORADAR_REQ-764 */
#[no_mangle]
pub unsafe extern "C" fn rlDeviceGetLaneConfig(mut deviceMap: rlUInt8_t,
                                               mut data:
                                                   *mut rlDevLaneEnable_t)
 -> rlReturnVal_t {
    let mut retVal: rlReturnVal_t = 0;
    /* check if deviceIndex is out of defined value */
    if rlDriverIsDeviceMapValid(deviceMap) != 0 as libc::c_int ||
           data.is_null() {
        /* set error code if DeviceMAP is invalid or data pointer is null */
        retVal = -(2 as libc::c_int)
    } else if 0 as libc::c_uint != rlDriverGetPlatformId() as libc::c_uint {
        /* This API is valid only when mmWaveLink instance is running on
       External Host Processor */
        /* set error code of Platform is not set to HOST */
        retVal = -(12 as libc::c_int)
    } else {
        /* Package the command with given data and send it to device */
        retVal =
            rlDriverExecuteGetApi(deviceMap,
                                  0x203 as libc::c_uint as rlUInt16_t,
                                  0x3 as libc::c_uint as rlUInt16_t,
                                  data as *mut rlUInt8_t,
                                  0 as libc::c_uint as rlUInt16_t)
    }
    return retVal;
}
/* * @fn rlReturnVal_t rlDeviceSetDataPathClkConfig(rlUInt8_t deviceMap,
*                                                  rlDevDataPathClkCfg_t* data)
*
*   @brief Sets LVDS Clock Configuration
*   @param[in] deviceMap - Bitmap of devices to send the message
*   @param[in] data  - Container for LVDS Clock Configuration
*
*   @return rlReturnVal_t Success - 0, Failure - Error Code
*
*   @brief Sets LVDS Clock Configuration. This API is valid only for AWR1243
*   mmWave device when mmWaveLink instance is running on External Host Processor.
*/
/* DesignId : MMWL_DesignId_106 */
/* Requirements : AUTORADAR_REQ-764 */
#[no_mangle]
pub unsafe extern "C" fn rlDeviceSetDataPathClkConfig(mut deviceMap:
                                                          rlUInt8_t,
                                                      mut data:
                                                          *mut rlDevDataPathClkCfg_t)
 -> rlReturnVal_t {
    let mut retVal: rlReturnVal_t = 0;
    /* check if deviceIndex is out of defined value */
    if rlDriverIsDeviceMapValid(deviceMap) != 0 as libc::c_int ||
           data.is_null() {
        /* set error code if DeviceMAP is invalid or data pointer is null */
        retVal = -(2 as libc::c_int)
    } else if 0 as libc::c_uint != rlDriverGetPlatformId() as libc::c_uint {
        /* This API is valid only when mmWaveLink instance is running on
       External Host Processor */
        /* set error code of Platform is not set to HOST */
        retVal = -(12 as libc::c_int)
    } else {
        /* Package the command with given data and send it to device */
        retVal =
            rlDriverExecuteSetApi(deviceMap,
                                  0x202 as libc::c_uint as rlUInt16_t,
                                  0x4 as libc::c_uint as rlUInt16_t,
                                  data as *mut rlUInt8_t,
                                  ::std::mem::size_of::<rlDevDataPathClkCfg_t>()
                                      as libc::c_ulong as rlUInt16_t)
    }
    return retVal;
}
/* * @fn rlReturnVal_t rlDeviceGetDataPathClkConfig(rlUInt8_t deviceMap,
*                                        rlProfileCfg_t* data)
*
*   @brief Gets LVDS Clock Configuration
*   @param[in] deviceMap - Bitmap of devices to send the message
*   @param[out] data - Container for LVDS Clock Configuration
*
*   @return rlReturnVal_t Success - 0, Failure - Error Code
*
*   Gets LVDS Clock Configuration. This API is valid only for AWR1243 mmWave
*   device when mmWaveLink instance is running on External Host Processor
*/
/* DesignId : MMWL_DesignId_114 */
/* Requirements : AUTORADAR_REQ-764 */
#[no_mangle]
pub unsafe extern "C" fn rlDeviceGetDataPathClkConfig(mut deviceMap:
                                                          rlUInt8_t,
                                                      mut data:
                                                          *mut rlDevDataPathClkCfg_t)
 -> rlReturnVal_t {
    let mut retVal: rlReturnVal_t = 0;
    /* check if deviceIndex is out of defined value */
    if rlDriverIsDeviceMapValid(deviceMap) != 0 as libc::c_int ||
           data.is_null() {
        /* set error code if DeviceMAP is invalid or data pointer is null */
        retVal = -(2 as libc::c_int)
    } else if 0 as libc::c_uint != rlDriverGetPlatformId() as libc::c_uint {
        /* This API is valid only when mmWaveLink instance is running on
       External Host Processor */
        /* set error code of Platform is not set to HOST */
        retVal = -(12 as libc::c_int)
    } else {
        /* Package the command with given data and send it to device */
        retVal =
            rlDriverExecuteGetApi(deviceMap,
                                  0x203 as libc::c_uint as rlUInt16_t,
                                  0x4 as libc::c_uint as rlUInt16_t,
                                  data as *mut rlUInt8_t,
                                  0 as libc::c_uint as rlUInt16_t)
    }
    return retVal;
}
/* * @fn rlReturnVal_t rlDeviceSetLvdsLaneConfig(rlUInt8_t deviceMap, rlDevLvdsDataCfg_t* data)
*
*   @brief Sets LVDS Lane data format Configuration
*   @param[in] deviceMap - Bitmap of devices to send the message
*   @param[in] data  - Container for LVDS Lane Configuration
*
*   @return rlReturnVal_t Success - 0, Failure - Error Code
*
*   Sets LVDS Lane data format Configuration. This API is valid only for AWR1243
*   mmWave device when mmWaveLink instance is running on External Host Processor
*/
/* DesignId : MMWL_DesignId_030 */
/* Requirements : AUTORADAR_REQ-767 */
#[no_mangle]
pub unsafe extern "C" fn rlDeviceSetLvdsLaneConfig(mut deviceMap: rlUInt8_t,
                                                   mut data:
                                                       *mut rlDevLvdsLaneCfg_t)
 -> rlReturnVal_t {
    let mut retVal: rlReturnVal_t = 0;
    /* check if deviceIndex is out of defined value */
    if rlDriverIsDeviceMapValid(deviceMap) != 0 as libc::c_int ||
           data.is_null() {
        /* set error code if DeviceMAP is invalid or data pointer is null */
        retVal = -(2 as libc::c_int)
    } else if 0 as libc::c_uint != rlDriverGetPlatformId() as libc::c_uint {
        /* This API is valid only when mmWaveLink instance is running on
       External Host Processor */
        /* set error code of Platform is not set to HOST */
        retVal = -(12 as libc::c_int)
    } else {
        /* Package the command with given data and send it to device */
        retVal =
            rlDriverExecuteSetApi(deviceMap,
                                  0x202 as libc::c_uint as rlUInt16_t,
                                  0x5 as libc::c_uint as rlUInt16_t,
                                  data as *mut rlUInt8_t,
                                  ::std::mem::size_of::<rlDevLvdsLaneCfg_t>()
                                      as libc::c_ulong as rlUInt16_t)
    }
    return retVal;
}
/* * @fn rlReturnVal_t rlDeviceGetLvdsLaneConfig(rlUInt8_t deviceMap,
*                                        rlProfileCfg_t* data)
*
*   @brief Gets LVDS Lane Configuration
*   @param[in] deviceMap - Bitmap of devices to send the message
*   @param[out] data - Container for LVDS Lane Configuration
*
*   @return rlReturnVal_t Success - 0, Failure - Error Code
*
*   Gets LVDS Lane Configuration. This API is valid only for AWR1243 mmWave
*   device when mmWaveLink instance is running on External Host Processor
*/
/* DesignId : MMWL_DesignId_116 */
/* Requirements : AUTORADAR_REQ-767 */
#[no_mangle]
pub unsafe extern "C" fn rlDeviceGetLvdsLaneConfig(mut deviceMap: rlUInt8_t,
                                                   mut data:
                                                       *mut rlDevLvdsLaneCfg_t)
 -> rlReturnVal_t {
    let mut retVal: rlReturnVal_t = 0;
    /* check if deviceIndex is out of defined value */
    if rlDriverIsDeviceMapValid(deviceMap) != 0 as libc::c_int ||
           data.is_null() {
        /* set error code if DeviceMAP is invalid or data pointer is null */
        retVal = -(2 as libc::c_int)
    } else if 0 as libc::c_uint != rlDriverGetPlatformId() as libc::c_uint {
        /* This API is valid only when mmWaveLink instance is running on
       External Host Processor */
        /* set error code of Platform is not set to HOST */
        retVal = -(12 as libc::c_int)
    } else {
        /* Package the command with given data and send it to device */
        retVal =
            rlDriverExecuteGetApi(deviceMap,
                                  0x203 as libc::c_uint as rlUInt16_t,
                                  0x5 as libc::c_uint as rlUInt16_t,
                                  data as *mut rlUInt8_t,
                                  0 as libc::c_uint as rlUInt16_t)
    }
    return retVal;
}
/* * @fn rlReturnVal_t rlDeviceSetContStreamingModeConfig(rlUInt8_t deviceMap,
                                                         rlDevContStreamingModeCfg_t* data)
*
*   @brief Sets Continous Streaming Mode Configuration
*   @param[in] deviceMap - Bitmap of devices to send the message
*   @param[in] data  - Container for Continous Streaming Mode Configuration
*
*   @return rlReturnVal_t Success - 0, Failure - Error Code
*
*   @brief This function configures the transfer of captured ADC samples continuously without
*          missing any sample to an external host. This API is valid only for AWR1243 mmWave
*          device when mmWaveLink instance is running on External Host Processor.
*   @note : Continuous streaming mode is useful for RF lab characterization and debug. In this
*           mode, the device is configured to transmit a single continuous wave tone at a specific
*           RF frequency continuously
*/
/* DesignId : MMWL_DesignId_040 */
/* Requirements : AUTORADAR_REQ-829 */
#[no_mangle]
pub unsafe extern "C" fn rlDeviceSetContStreamingModeConfig(mut deviceMap:
                                                                rlUInt8_t,
                                                            mut data:
                                                                *mut rlDevContStreamingModeCfg_t)
 -> rlReturnVal_t {
    let mut retVal: rlReturnVal_t = 0;
    /* check if deviceIndex is out of defined value */
    if rlDriverIsDeviceMapValid(deviceMap) != 0 as libc::c_int ||
           data.is_null() {
        /* set error code if DeviceMAP is invalid or data pointer is null */
        retVal = -(2 as libc::c_int)
    } else if 0 as libc::c_uint != rlDriverGetPlatformId() as libc::c_uint {
        /* This API is valid only when mmWaveLink instance is running on
       External Host Processor */
        /* set error code of Platform is not set to HOST */
        retVal = -(12 as libc::c_int)
    } else {
        /* Package the command with given data and send it to device */
        retVal =
            rlDriverExecuteSetApi(deviceMap,
                                  0x202 as libc::c_uint as rlUInt16_t,
                                  0x6 as libc::c_uint as rlUInt16_t,
                                  data as *mut rlUInt8_t,
                                  ::std::mem::size_of::<rlDevContStreamingModeCfg_t>()
                                      as libc::c_ulong as rlUInt16_t)
    }
    return retVal;
}
/* * @fn rlReturnVal_t rlDeviceGetContStreamingModeConfig(rlUInt8_t deviceMap,
*                                        rlDevContStreamingModeCfg_t* data)
*
*   @brief Gets continuous Streaming Mode Configuration
*   @param[in] deviceMap - Bitmap of devices to send the message
*   @param[out] data - Container for continuous Streaming Mode Configuration
*
*   @return rlReturnVal_t Success - 0, Failure - Error Code
*
*   Gets continuous Streaming Mode Configuration. This API is valid only for
*   AWR1243 mmWave device when mmWaveLink instance is running on External
*   Host Processor
*/
/* DesignId : MMWL_DesignId_121 */
/* Requirements : AUTORADAR_REQ-829 */
#[no_mangle]
pub unsafe extern "C" fn rlDeviceGetContStreamingModeConfig(mut deviceMap:
                                                                rlUInt8_t,
                                                            mut data:
                                                                *mut rlDevContStreamingModeCfg_t)
 -> rlReturnVal_t {
    let mut retVal: rlReturnVal_t = 0;
    /* check if deviceIndex is out of defined value */
    if rlDriverIsDeviceMapValid(deviceMap) != 0 as libc::c_int ||
           data.is_null() {
        /* set error code if DeviceMAP is invalid or data pointer is null */
        retVal = -(2 as libc::c_int)
    } else if 0 as libc::c_uint != rlDriverGetPlatformId() as libc::c_uint {
        /* This API is valid only when mmWaveLink instance is running on
       External Host Processor */
        /* set error code of Platform is not set to HOST */
        retVal = -(12 as libc::c_int)
    } else {
        /* Package the command with given data and send it to device */
        retVal =
            rlDriverExecuteGetApi(deviceMap,
                                  0x203 as libc::c_uint as rlUInt16_t,
                                  0x6 as libc::c_uint as rlUInt16_t,
                                  data as *mut rlUInt8_t,
                                  0 as libc::c_uint as rlUInt16_t)
    }
    return retVal;
}
/* * @fn rlReturnVal_t rlDeviceSetCsi2Config(rlUInt8_t deviceMap, rlDevCsi2Cfg_t* data)
*
*   @brief Sets CSI2 data format Configuration
*   @param[in] deviceMap - Bitmap of devices to send the message
*   @param[in] data  - Container for CSI2 Configuration
*
*   @return rlReturnVal_t Success - 0, Failure - Error Code
*
*   Sets CSI2 data format Configuration. This API is valid only for AWR1243
*   mmWave device when mmWaveLink instance is running on External Host Processor
*/
/* DesignId : MMWL_DesignId_048 */
/* Requirements : AUTORADAR_REQ-756, AUTORADAR_REQ-766 */
#[no_mangle]
pub unsafe extern "C" fn rlDeviceSetCsi2Config(mut deviceMap: rlUInt8_t,
                                               mut data: *mut rlDevCsi2Cfg_t)
 -> rlReturnVal_t {
    let mut retVal: rlReturnVal_t = 0;
    /* check if deviceIndex is out of defined value */
    if rlDriverIsDeviceMapValid(deviceMap) != 0 as libc::c_int ||
           data.is_null() {
        /* set error code if DeviceMAP is invalid or data pointer is null */
        retVal = -(2 as libc::c_int)
    } else if 0 as libc::c_uint != rlDriverGetPlatformId() as libc::c_uint {
        /* This API is valid only when mmWaveLink instance is running on
       External Host Processor */
        /* set error code of Platform is not set to HOST */
        retVal = -(12 as libc::c_int)
    } else {
        /* Package the command with given data and send it to device */
        retVal =
            rlDriverExecuteSetApi(deviceMap,
                                  0x202 as libc::c_uint as rlUInt16_t,
                                  0x7 as libc::c_uint as rlUInt16_t,
                                  data as *mut rlUInt8_t,
                                  ::std::mem::size_of::<rlDevCsi2Cfg_t>() as
                                      libc::c_ulong as rlUInt16_t)
    }
    return retVal;
}
/* * @fn rlReturnVal_t rlDeviceGetCsi2Config(rlUInt8_t deviceMap,
*                                            rlDevCsi2Cfg_t* data)
*
*   @brief Gets Csi2 data format Configuration
*   @param[in] deviceMap - Bitmap of devices to send the message
*   @param[out] data - Container for CSI2 Configuration
*
*   @return rlReturnVal_t Success - 0, Failure - Error Code
*
*  Gets CSI2 data format Configuration. This API is valid only for AWR1243
*  mmWave device when mmWaveLink instance is running on External Host Processor.
*/
/* DesignId : MMWL_DesignId_111 */
/* Requirements : AUTORADAR_REQ-756, AUTORADAR_REQ-766 */
#[no_mangle]
pub unsafe extern "C" fn rlDeviceGetCsi2Config(mut deviceMap: rlUInt8_t,
                                               mut data: *mut rlDevCsi2Cfg_t)
 -> rlReturnVal_t {
    let mut retVal: rlReturnVal_t = 0;
    /* check if deviceIndex is out of defined value */
    if rlDriverIsDeviceMapValid(deviceMap) != 0 as libc::c_int ||
           data.is_null() {
        /* set error code if DeviceMAP is invalid or data pointer is null */
        retVal = -(2 as libc::c_int)
    } else if 0 as libc::c_uint != rlDriverGetPlatformId() as libc::c_uint {
        /* This API is valid only when mmWaveLink instance is running on
       External Host Processor */
        /* set error code of Platform is not set to HOST */
        retVal = -(12 as libc::c_int)
    } else {
        /* Package the command with given data and send it to device */
        retVal =
            rlDriverExecuteGetApi(deviceMap,
                                  0x203 as libc::c_uint as rlUInt16_t,
                                  0x7 as libc::c_uint as rlUInt16_t,
                                  data as *mut rlUInt8_t,
                                  0 as libc::c_uint as rlUInt16_t)
    }
    return retVal;
}
/* * @fn rlReturnVal_t rlDeviceSetHsiConfig(rlUInt8_t deviceMap,
*                                          rlDevHsiCfg_t* data)
*
*    @brief: This function sets the High Speed Interface(LVDS/CSI2) clock, lane,
*                 data rate and data format
*    @param[in] deviceMap - Connected device Index
*    @param[in] data         - HSI Config data
*
*    @return rlReturnVal_t Success - 0, Failure - Error Code
*
*    @brief: This function sets the High Speed Interface(LVDS/CSI2) clock, lane,
*    data rate and data format. This API is valid only for AWR1243
*   mmWave device when mmWaveLink instance is running on External Host Processor
*/
/* DesignId : MMWL_DesignId_020 */
/* Requirements : AUTORADAR_REQ-756 */
#[no_mangle]
pub unsafe extern "C" fn rlDeviceSetHsiConfig(mut deviceMap: rlUInt8_t,
                                              mut data: *mut rlDevHsiCfg_t)
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
    let mut sbcCnt: rlUInt16_t = 0 as libc::c_uint as rlUInt16_t;
    /* Initialize Command and Response Sub Blocks */
    let mut inPayloadSb: [rlPayloadSb_t; 3] =
        [{
             let mut init =
                 rlPayloadSb{sbid: 0 as libc::c_int as rlUInt16_t,
                             len: 0,
                             pSblkData: 0 as *mut rlUInt8_t,};
             init
         }, rlPayloadSb_t{sbid: 0, len: 0, pSblkData: 0 as *mut rlUInt8_t,},
         rlPayloadSb_t{sbid: 0, len: 0, pSblkData: 0 as *mut rlUInt8_t,}];
    /* check if deviceIndex is out of defined value */
    if rlDriverIsDeviceMapValid(deviceMap) != 0 as libc::c_int ||
           data.is_null() {
        /* set error code if DeviceMAP is invalid or data pointer is null */
        retVal = -(2 as libc::c_int)
    } else {
        /* Data Format config SubBlock */
        if !(*data).datafmt.is_null() {
            /* Fill in-message Payload */
            rlDriverFillPayload(0x202 as libc::c_uint as rlUInt16_t,
                                0x1 as libc::c_uint as rlUInt16_t,
                                &mut *inPayloadSb.as_mut_ptr().offset(sbcCnt
                                                                          as
                                                                          isize),
                                (*data).datafmt as *mut rlUInt8_t,
                                ::std::mem::size_of::<rlDevDataFmtCfg_t>() as
                                    libc::c_ulong as rlUInt16_t);
            /* increament Sub-block count */
            sbcCnt = sbcCnt.wrapping_add(1)
        }
        /* Data Path Config SubBlock */
        if !(*data).dataPath.is_null() {
            /* Fill in-message Payload */
            rlDriverFillPayload(0x202 as libc::c_uint as rlUInt16_t,
                                0x2 as libc::c_uint as rlUInt16_t,
                                &mut *inPayloadSb.as_mut_ptr().offset(sbcCnt
                                                                          as
                                                                          isize),
                                (*data).dataPath as *mut rlUInt8_t,
                                ::std::mem::size_of::<rlDevDataPathCfg_t>() as
                                    libc::c_ulong as rlUInt16_t);
            /* increament Sub-block count */
            sbcCnt = sbcCnt.wrapping_add(1)
        }
        /* Data Path clock Config SubBlock */
        if !(*data).dataPathClk.is_null() {
            /* Fill in-message Payload */
            rlDriverFillPayload(0x202 as libc::c_uint as rlUInt16_t,
                                0x4 as libc::c_uint as rlUInt16_t,
                                &mut *inPayloadSb.as_mut_ptr().offset(sbcCnt
                                                                          as
                                                                          isize),
                                (*data).dataPathClk as *mut rlUInt8_t,
                                ::std::mem::size_of::<rlDevDataPathClkCfg_t>()
                                    as libc::c_ulong as rlUInt16_t);
            /* increament Sub-block count */
            sbcCnt = sbcCnt.wrapping_add(1)
        }
        /* Construct command packet */
        rlDriverConstructInMsg(0x202 as libc::c_uint as rlUInt16_t,
                               &mut inMsg,
                               &mut *inPayloadSb.as_mut_ptr().offset(0 as
                                                                         libc::c_uint
                                                                         as
                                                                         isize));
        if sbcCnt as libc::c_uint > 0 as libc::c_uint {
            /* setting num of sub-block to inMsg */
            inMsg.opcode.nsbc = sbcCnt;
            /* Send Command to mmWave Radar Device */
            retVal = rlDriverCmdInvoke(deviceMap, inMsg, &mut outMsg)
        } else {
            /* set error code if application doesn't pass any subBlock data */
            retVal = -(2 as libc::c_int)
        }
    }
    return retVal;
}
/* * @fn rlReturnVal_t rlDeviceSetHsiClk(rlUInt8_t deviceMap, rlDevHsiClk_t* data)
*
*   @brief Sets High Speed Interface Clock
*   @param[in] deviceMap - Bitmap of devices to send the message
*   @param[in] data  - Container for HSI Clock
*
*   @return rlReturnVal_t Success - 0, Failure - Error Code
*
*    @brief Sets High Speed Interface Clock

*/
/* DesignId : MMWL_DesignId_101 */
/* Requirements : AUTORADAR_REQ-765 */
#[no_mangle]
pub unsafe extern "C" fn rlDeviceSetHsiClk(mut deviceMap: rlUInt8_t,
                                           mut data: *mut rlDevHsiClk_t)
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
            rlDriverExecuteSetApi(deviceMap,
                                  0x4 as libc::c_uint as rlUInt16_t,
                                  0x5 as libc::c_uint as rlUInt16_t,
                                  data as *mut rlUInt8_t,
                                  ::std::mem::size_of::<rlDevHsiClk_t>() as
                                      libc::c_ulong as rlUInt16_t)
    }
    return retVal;
}
/* * @fn rlReturnVal_t rlDeviceMcuClkConfig(rlUInt8_t deviceMap, rlMcuClkCfg_t * data)
*
*   @brief Sets the configurations to setup the desired frequency of the MCU Clock
*   @param[in] deviceMap - Bitmap of devices to send the message
*   @param[in] data  - Container for MCU clock Configuration
*
*   @return rlReturnVal_t Success - 0, Failure - Error Code
*
*   Sets the configurations to setup the desired frequency of the MCU Clock. This
*   API is valid only for AWR1243 mmWave device when mmWaveLink instance is running
*   on External Host Processor.
*/
/* DesignId : MMWL_DesignId_069 */
/* Requirements : AUTORADAR_REQ-713 */
#[no_mangle]
pub unsafe extern "C" fn rlDeviceMcuClkConfig(mut deviceMap: rlUInt8_t,
                                              mut data: *mut rlMcuClkCfg_t)
 -> rlReturnVal_t {
    let mut retVal: rlReturnVal_t = 0;
    /* check if deviceIndex is out of defined value */
    if rlDriverIsDeviceMapValid(deviceMap) != 0 as libc::c_int ||
           data.is_null() {
        /* set error code if DeviceMAP is invalid or data pointer is null */
        retVal = -(2 as libc::c_int)
    } else if 0 as libc::c_uint != rlDriverGetPlatformId() as libc::c_uint {
        /* This API is valid only when mmWaveLink instance is running on
    External Host Processor */
        /* set error code of Platform is not set to HOST */
        retVal = -(12 as libc::c_int)
    } else {
        /* Package the command with given data and send it to device */
        retVal =
            rlDriverExecuteSetApi(deviceMap,
                                  0x202 as libc::c_uint as rlUInt16_t,
                                  0 as libc::c_uint as rlUInt16_t,
                                  data as *mut rlUInt8_t,
                                  ::std::mem::size_of::<rlMcuClkCfg_t>() as
                                      libc::c_ulong as rlUInt16_t)
    }
    return retVal;
}
/* * @fn rlReturnVal_t rlDevicePmicClkConfig(rlUInt8_t deviceMap, rlPmicClkCfg_t * data)
*
*   @brief Sets the configurations for PMIC clock
*   @param[in] deviceMap - Bitmap of devices to send the message
*   @param[in] data  - Container for PMIC clock Configuration
*
*   @return rlReturnVal_t Success - 0, Failure - Error Code
*
*   Sets the configurations for PMIC clock. This API is valid only for AWR1243
*   mmWave device when mmWaveLink instance is running on External Host Processor.
*/
/* DesignId : MMWL_DesignId_070 */
/* Requirements : AUTORADAR_REQ-907 */
#[no_mangle]
pub unsafe extern "C" fn rlDevicePmicClkConfig(mut deviceMap: rlUInt8_t,
                                               mut data: *mut rlPmicClkCfg_t)
 -> rlReturnVal_t {
    let mut retVal: rlReturnVal_t = 0;
    /* check if deviceIndex is out of defined value */
    if rlDriverIsDeviceMapValid(deviceMap) != 0 as libc::c_int ||
           data.is_null() {
        /* set error code if DeviceMAP is invalid or data pointer is null */
        retVal = -(2 as libc::c_int)
    } else if 0 as libc::c_uint != rlDriverGetPlatformId() as libc::c_uint {
        /* This API is valid only when mmWaveLink instance is running on
    External Host Processor */
        /* set error code of Platform is not set to HOST */
        retVal = -(12 as libc::c_int)
    } else {
        /* Package the command with given data and send it to device */
        retVal =
            rlDriverExecuteSetApi(deviceMap,
                                  0x202 as libc::c_uint as rlUInt16_t,
                                  0x8 as libc::c_uint as rlUInt16_t,
                                  data as *mut rlUInt8_t,
                                  ::std::mem::size_of::<rlPmicClkCfg_t>() as
                                      libc::c_ulong as rlUInt16_t)
    }
    return retVal;
}
/* * @fn rlReturnVal_t rlDeviceLatentFaultTests(rlUInt8_t deviceMap, rllatentFault_t * data)
*
*   @brief Sets the configurations for latent fault test
*   @param[in] deviceMap - Bitmap of devices to send the message
*   @param[in] data  - Container for latent fault test Configuration
*
*   @return rlReturnVal_t Success - 0, Failure - Error Code
*
*   Sets the configurations for latent fault test. This API is valid only for AWR1243
*   mmWave device when mmWaveLink instance is running on External Host Processor. This API
*   should not be issued when functional frames are running, these are destructive tests.
*
*   @note 1: The MSS latent self tests are destructive tests, which would cause corruption in
*           ongoing SPI/mailbox transactions and which generates N-Error signal while performing
*           ESM G2 error checks. The MIBSPI ECC tests (b13,b14) can be destructive tests if there 
*           is an ongoing MIBSPI communication. It is recommended not to run these self tests in 
*           functional mode of operation. \n
*   @note 2: It is recommended to wait for the latent fault test report asynchronous event after 
*           issuing this API. The MSS latent self tests cannot be issued back to back without 
*           waiting for the test report event.
*/
/* DesignId : MMWL_DesignId_071 */
/* Requirements : AUTORADAR_REQ-908 */
#[no_mangle]
pub unsafe extern "C" fn rlDeviceLatentFaultTests(mut deviceMap: rlUInt8_t,
                                                  mut data:
                                                      *mut rllatentFault_t)
 -> rlReturnVal_t {
    let mut retVal: rlReturnVal_t = 0;
    /* check if deviceIndex is out of defined value */
    if rlDriverIsDeviceMapValid(deviceMap) != 0 as libc::c_int ||
           data.is_null() {
        /* set error code if DeviceMAP is invalid or data pointer is null */
        retVal = -(2 as libc::c_int)
    } else if 0 as libc::c_uint != rlDriverGetPlatformId() as libc::c_uint {
        /* This API is valid only when mmWaveLink instance is running on
    External Host Processor */
        /* set error code of Platform is not set to HOST */
        retVal = -(12 as libc::c_int)
    } else {
        /* Package the command with given data and send it to device */
        retVal =
            rlDriverExecuteSetApi(deviceMap,
                                  0x202 as libc::c_uint as rlUInt16_t,
                                  0xa as libc::c_uint as rlUInt16_t,
                                  data as *mut rlUInt8_t,
                                  ::std::mem::size_of::<rllatentFault_t>() as
                                      libc::c_ulong as rlUInt16_t)
    }
    return retVal;
}
/* * @fn rlReturnVal_t rlDeviceEnablePeriodicTests(rlUInt8_t deviceMap, rlperiodicTest_t * data)
*
*   @brief Sets the configurations for periodic test
*   @param[in] deviceMap - Bitmap of devices to send the message
*   @param[in] data  - Container for periodic test Configuration
*
*   @return rlReturnVal_t Success - 0, Failure - Error Code
*
*   Sets the configurations for periodic test. This API is valid only for AWR1243
*   mmWave device when mmWaveLink instance is running on External Host Processor.
*/
/* DesignId : MMWL_DesignId_072 */
/* Requirements : AUTORADAR_REQ-909 */
#[no_mangle]
pub unsafe extern "C" fn rlDeviceEnablePeriodicTests(mut deviceMap: rlUInt8_t,
                                                     mut data:
                                                         *mut rlperiodicTest_t)
 -> rlReturnVal_t {
    let mut retVal: rlReturnVal_t = 0;
    /* check if deviceIndex is out of defined value */
    if rlDriverIsDeviceMapValid(deviceMap) != 0 as libc::c_int ||
           data.is_null() {
        /* set error code if DeviceMAP is invalid or data pointer is null */
        retVal = -(2 as libc::c_int)
    } else if 0 as libc::c_uint != rlDriverGetPlatformId() as libc::c_uint {
        /* This API is valid only when mmWaveLink instance is running on
    External Host Processor */
        /* set error code of Platform is not set to HOST */
        retVal = -(12 as libc::c_int)
    } else {
        /* Package the command with given data and send it to device */
        retVal =
            rlDriverExecuteSetApi(deviceMap,
                                  0x202 as libc::c_uint as rlUInt16_t,
                                  0x9 as libc::c_uint as rlUInt16_t,
                                  data as *mut rlUInt8_t,
                                  ::std::mem::size_of::<rlperiodicTest_t>() as
                                      libc::c_ulong as rlUInt16_t)
    }
    return retVal;
}
/* * @fn rlReturnVal_t rlDeviceSetTestPatternConfig(rlUInt8_t deviceMap, rltestPattern_t * data)
*
*   @brief Setup for test pattern to be generated
*   @param[in] deviceMap - Bitmap of devices to send the message
*   @param[in] data  - Container for periodic test Configuration
*
*   @return rlReturnVal_t Success - 0, Failure - Error Code
*
*   Set the configurations to setup the test pattern to be generated and transferred
*   over the selected high speed interface (LVDS/CSI2). This API is valid only for
*   AWR1243 mmWave device when mmWaveLink instance is running on External Host Processor.
*/
/* DesignId : MMWL_DesignId_103 */
/* Requirements : AUTORADAR_REQ-910 */
#[no_mangle]
pub unsafe extern "C" fn rlDeviceSetTestPatternConfig(mut deviceMap:
                                                          rlUInt8_t,
                                                      mut data:
                                                          *mut rltestPattern_t)
 -> rlReturnVal_t {
    let mut retVal: rlReturnVal_t = 0;
    /* check if deviceIndex is out of defined value */
    if rlDriverIsDeviceMapValid(deviceMap) != 0 as libc::c_int ||
           data.is_null() {
        /* set error code if DeviceMAP is invalid or data pointer is null */
        retVal = -(2 as libc::c_int)
    } else if 0 as libc::c_uint != rlDriverGetPlatformId() as libc::c_uint {
        /* This API is valid only when mmWaveLink instance is running on
    External Host Processor */
        /* set error code of Platform is not set to HOST */
        retVal = -(12 as libc::c_int)
    } else {
        /* Package the command with given data and send it to device */
        retVal =
            rlDriverExecuteSetApi(deviceMap,
                                  0x202 as libc::c_uint as rlUInt16_t,
                                  0xb as libc::c_uint as rlUInt16_t,
                                  data as *mut rlUInt8_t,
                                  ::std::mem::size_of::<rltestPattern_t>() as
                                      libc::c_ulong as rlUInt16_t)
    }
    return retVal;
}
/* * @fn rlReturnVal_t rlDeviceSetMiscConfig(rlUInt8_t deviceMap, rlDevMiscCfg_t *data)
*
*   @brief Setup misc. device configurations
*   @param[in] data  - Container for device Configuration
*   @param[in] deviceMap - Bitmap of devices to send the message
*
*   @return rlReturnVal_t Success - 0, Failure - Error Code
*
*   Set misc. device configurations of MasterMSS, where currently it set CRC type for async event
*   message sent by MSS to Host. This API is valid only for AWR1243 mmWave device when mmWaveLink
*   instance is running on External Host Processor.
*/
/* DesignId : MMWL_DesignId_117 */
/* Requirements : AUTORADAR_REQ-886 */
#[no_mangle]
pub unsafe extern "C" fn rlDeviceSetMiscConfig(mut deviceMap: rlUInt8_t,
                                               mut data: *mut rlDevMiscCfg_t)
 -> rlReturnVal_t {
    let mut retVal: rlReturnVal_t = 0;
    /* check if deviceIndex is out of defined value */
    if rlDriverIsDeviceMapValid(deviceMap) != 0 as libc::c_int ||
           data.is_null() {
        /* set error code if DeviceMAP is invalid or data pointer is null */
        retVal = -(2 as libc::c_int)
    } else if 0 as libc::c_uint != rlDriverGetPlatformId() as libc::c_uint {
        /* This API is valid only when mmWaveLink instance is running on
    External Host Processor */
        /* set error code of Platform is not set to HOST */
        retVal = -(12 as libc::c_int)
    } else {
        /* Package the command with given data and send it to device */
        retVal =
            rlDriverExecuteSetApi(deviceMap,
                                  0x202 as libc::c_uint as rlUInt16_t,
                                  0xc as libc::c_uint as rlUInt16_t,
                                  data as *mut rlUInt8_t,
                                  ::std::mem::size_of::<rlDevMiscCfg_t>() as
                                      libc::c_ulong as rlUInt16_t)
    }
    return retVal;
}
/* * @fn rlReturnVal_t rlDeviceGetCpuFault(rlUInt8_t deviceMap, rlCpuFault_t *data)
*
*   @brief Get MasterSS CPU fault status.
*   @param[in] deviceMap - Bitmap of devices to send the message
*   @param[in] data  - Container for MasterSS CPU fault status
*
*   @return rlReturnVal_t Success - 0, Failure - Error Code
*
*   This API gets the MasterSS CPU fault status. This API is valid only for AWR1243.
*   mmWave device when mmWaveLink instance is running on External Host Processor.
*/
/* DesignId : MMWL_DesignId_124 */
/* Requirements : AUTORADAR_REQ-1040 */
#[no_mangle]
pub unsafe extern "C" fn rlDeviceGetCpuFault(mut deviceMap: rlUInt8_t,
                                             mut data: *mut rlCpuFault_t)
 -> rlReturnVal_t {
    let mut retVal: rlReturnVal_t = 0;
    /* check if deviceIndex is out of defined value */
    if rlDriverIsDeviceMapValid(deviceMap) != 0 as libc::c_int ||
           data.is_null() {
        /* set error code if DeviceMAP is invalid or data pointer is null */
        retVal = -(2 as libc::c_int)
    } else if 0 as libc::c_uint != rlDriverGetPlatformId() as libc::c_uint {
        /* This API is valid only when mmWaveLink instance is running on
    External Host Processor */
        /* set error code of Platform is not set to HOST */
        retVal = -(12 as libc::c_int)
    } else {
        /* Package the command with given data and send it to device */
        retVal =
            rlDriverExecuteGetApi(deviceMap,
                                  0x207 as libc::c_uint as rlUInt16_t,
                                  0x1 as libc::c_uint as rlUInt16_t,
                                  data as *mut rlUInt8_t,
                                  ::std::mem::size_of::<rlCpuFault_t>() as
                                      libc::c_ulong as rlUInt16_t)
    }
    return retVal;
}
/* * @fn rlReturnVal_t rlDeviceGetEsmFault(rlUInt8_t deviceMap, rlMssEsmFault_t *data)
*
*   @brief Get MasterSS ESM fault status.
*   @param[in] deviceMap - Bitmap of devices to send the message
*   @param[in] data  - Container for MasterSS ESM fault status
*
*   @return rlReturnVal_t Success - 0, Failure - Error Code
*
*   This API gets the MasterSS ESM fault status. This API is valid only for AWR1243
*   mmWave device when mmWaveLink instance is running on External Host Processor.
*/
/* DesignId : MMWL_DesignId_124 */
/* Requirements : AUTORADAR_REQ-1040 */
#[no_mangle]
pub unsafe extern "C" fn rlDeviceGetEsmFault(mut deviceMap: rlUInt8_t,
                                             mut data: *mut rlMssEsmFault_t)
 -> rlReturnVal_t {
    let mut retVal: rlReturnVal_t = 0;
    /* check if deviceIndex is out of defined value */
    if rlDriverIsDeviceMapValid(deviceMap) != 0 as libc::c_int ||
           data.is_null() {
        /* set error code if DeviceMAP is invalid or data pointer is null */
        retVal = -(2 as libc::c_int)
    } else if 0 as libc::c_uint != rlDriverGetPlatformId() as libc::c_uint {
        /* This API is valid only when mmWaveLink instance is running on
    External Host Processor */
        /* set error code of Platform is not set to HOST */
        retVal = -(12 as libc::c_int)
    } else {
        /* Package the command with given data and send it to device */
        retVal =
            rlDriverExecuteGetApi(deviceMap,
                                  0x207 as libc::c_uint as rlUInt16_t,
                                  0x2 as libc::c_uint as rlUInt16_t,
                                  data as *mut rlUInt8_t,
                                  ::std::mem::size_of::<rlMssEsmFault_t>() as
                                      libc::c_ulong as rlUInt16_t)
    }
    return retVal;
}
/* * @fn rlReturnVal_t rlDeviceConfigureCrc(rlCrcType_t crcType)
*
*   @brief  Configures the CRC Type in mmWaveLink Driver
*   @param[in] crcType - CRC Types, 0 - No CRC
*
*   @return rlReturnVal_t Success - 0, Failure - Error Code
*
*   Configures the CRC Type in mmWaveLink Driver
*/
/* DesignId : MMWL_DesignId_028 */
/* Requirements : AUTORADAR_REQ-710, AUTORADAR_REQ-804 */
#[no_mangle]
pub unsafe extern "C" fn rlDeviceConfigureCrc(mut crcType: rlCrcType_t)
 -> rlReturnVal_t {
    /* set CRC Type passed by application to rlDriver */
    return rlDriverConfigureCrc(crcType);
}
/* * @fn rlReturnVal_t rlDeviceConfigureAckTimeout(rlUInt32_t ackTimeout)
*
*   @brief  Configures the Acknowledgement timeout in mmWaveLink Driver
*   @param[in] ackTimeout - ACK timeout, 0 - No ACK
*
*   @return rlReturnVal_t Success - 0, Failure - Error Code
*
*   Configures the Acknowledgement timeout in mmWaveLink Driver, 0 - Disable ACK
*/
/* DesignId : MMWL_DesignId_028 */
/* Requirements : AUTORADAR_REQ_710, AUTORADAR_REQ-804 */
#[no_mangle]
pub unsafe extern "C" fn rlDeviceConfigureAckTimeout(mut ackTimeout:
                                                         rlUInt32_t)
 -> rlReturnVal_t {
    /* Set Ack Timeout value passed by applicatio to rlDriver */
    return rlDriverConfigureAckTimeout(ackTimeout);
}
/* * @fn rlReturnVal_t rlDeviceSetRetryCount(rlUInt8_t retryCnt)
*
*   @brief: Set the command retry count
*   @param[in] retryCnt - Retry count [0: no retry, n: no. of retries]
*                         value limit to RL_API_CMD_RETRY_COUNT(3)
*
*   @return rlReturnVal_t Success - 0, Failure - Error Code(-2)
*
*   Set the retry count to global rlDriverData_t structure.
*/
/* DesignId : MMWL_DesignId_123 */
/* Requirements : AUTORADAR_REQ_781 */
#[no_mangle]
pub unsafe extern "C" fn rlDeviceSetRetryCount(mut retryCnt: rlUInt8_t)
 -> rlReturnVal_t {
    /* Set command retry count to rlDriver */
    return rlDriverSetRetryCount(retryCnt);
}
/* *****************************************************************************
 * FUNCTION PROTOTYPES
 ******************************************************************************
 */
/* *
 *  @defgroup Device Device
 *  @brief Radar Device Management Module.
 *
 *  The Device module has interface for Enabling and controlling Radar device.
 *  Configures the callbacks(SPI, Interrupt, OS etc) for communication with device
 *  It also allows Firmware download over SPI.
 *
 *    Related Files
 *   - rl_device.c
 *  @addtogroup Device
 *  @{
 */
/* Device Interface Functions */
/* RF/DSP Start Functions */
/* File Download Functions */
/* Get Version Functions */
/* MCU Clock configuration Functions */
/* PMIC Clock configuration Functions */
/* Latetnt fault test configuration Functions */
/* Periodic test configuration Functions */
/* Test pattern configuration Functions */
/* mmWaveLink Protocol configuration Functions */
/* Continuous streaming mode Functions */
/* Get different fault status functions */
/* !
 Close the Doxygen group.
 @}
 */
/* *
 *  @defgroup Data_Path Data Path
 *  @brief mmWave Radar Data Path(LVDS/CSI2) Module.
 *
 *  The Data path module has interface for Enabling and controlling high speed
 *  data interface such as CSI2 and LVDS. Configures the data format, data rate,
 *  lane parameters. \n
 *  Below diagram shows the data transfer for different data formats and lanes
 *  on high speed interface
 *
 *  @image html data_path_lanes.png
 *
 *    Related Files
 *   - rl_device.c
 *  @addtogroup Data_Path
 *  @{
 */
/*data Path(LVDS/CSI2) configuration Functions */
#[no_mangle]
pub unsafe extern "C" fn sanity_echo(mut v: libc::c_int) -> libc::c_int {
    return v;
}
/*
 * END OF rl_device.c FILE
 */
