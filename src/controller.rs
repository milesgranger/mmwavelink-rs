#![allow(
    dead_code,
    mutable_transmutes,
    non_camel_case_types,
    non_snake_case,
    non_upper_case_globals,
    unused_assignments,
    unused_mut
)]

use crate::driver::memcpy;

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
pub type rlInt32_t = core::ffi::c_int;
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
/* ***************************************************************************************
 * FileName     : rl_controller.c
 *
 * Description  : This file defines the functions to construct Radar Messages.
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
 * 0.1.0    12May2015   Kaushal Kukkar                    Initial Version
 *
 * 0.6.0    15Nov2016   Kaushal Kukkar    AUTORADAR-666   Logging Feature
 *
 * 0.9.1                Jitendra Gupta    MMWL-5          Code size optimization
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
/* * @fn rlReturnVal_t rlAppendSubBlock(rlUInt8_t rhcpPayload[],
*                       rlUInt16_t sbId, rlUInt16_t sbLen, rlUInt8_t *sbData)
*
*   @brief Appends sub block data to payload buffer
*   @param[out] rhcpPayload - payload buffer
*   @param[in] sbId - sub block Id
*   @param[in] sbLen - sub block Length
*   @param[in] sbData - sub block data
*
*   @return rlReturnVal_t Success - 0, Failure - Error Code
*
*   Appends sub block data to payload buffer
*/
/* DesignId :  */
/* Requirements : AUTORADAR_REQ-772 */
#[no_mangle]
pub unsafe extern "C" fn rlAppendSubBlock(
    mut rhcpPayload: *mut rlUInt8_t,
    mut sblkId: rlUInt16_t,
    mut sbLen: rlUInt16_t,
    mut sbData: *mut rlUInt8_t,
) -> rlReturnVal_t {
    let mut retVal: rlReturnVal_t = 0;
    /* check for null pointer */
    if rhcpPayload.is_null()
        || sbLen as core::ffi::c_uint
            > (256 as core::ffi::c_uint).wrapping_sub(
                (4 as core::ffi::c_uint)
                    .wrapping_add(12 as core::ffi::c_uint)
                    .wrapping_add(8 as core::ffi::c_uint),
            )
    {
        /* set error code */
        retVal = -(9 as core::ffi::c_int)
    } else {
        /* Append SB Id  */
        /*AR_CODE_REVIEW MR:R.11.2 <APPROVED> "The Payload SbId is of type UINT16,
         * so pointer conversion type to UINT16*
         * is required" */
        /*LDRA_INSPECTED 94 S */
        /*LDRA_INSPECTED 95 S */
        /*LDRA_INSPECTED 87 S */
        *(rhcpPayload.offset(0 as core::ffi::c_uint as isize) as *mut rlUInt16_t) = sblkId;
        /* Append SB Len */
        /*AR_CODE_REVIEW MR:R.11.2 <APPROVED> "The Payload SbLen is of type UINT16,
         * so pointer conversion type to UINT16*
         * is required" */
        /*LDRA_INSPECTED 94 S */
        /*LDRA_INSPECTED 95 S */
        /*LDRA_INSPECTED 87 S */
        *(rhcpPayload.offset((0 as core::ffi::c_uint).wrapping_add(2 as core::ffi::c_uint) as isize)
            as *mut rlUInt16_t) = (sbLen as core::ffi::c_uint)
            .wrapping_add(2 as core::ffi::c_uint)
            .wrapping_add(2 as core::ffi::c_uint) as rlUInt16_t;
        /* Append SB Payload */
        if sbLen as core::ffi::c_uint > 0 as core::ffi::c_uint && !sbData.is_null() {
            memcpy(
                &mut *rhcpPayload.offset(
                    (0 as core::ffi::c_uint)
                        .wrapping_add(2 as core::ffi::c_uint)
                        .wrapping_add(2 as core::ffi::c_uint) as isize,
                ) as *mut rlUInt8_t as *mut core::ffi::c_void,
                sbData as *const core::ffi::c_void,
                sbLen as _,
            );
            retVal = 0 as core::ffi::c_int
        } else if sbLen as core::ffi::c_uint == 0 as core::ffi::c_uint && sbData.is_null() {
            retVal = 0 as core::ffi::c_int
        } else {
            retVal = -(2 as core::ffi::c_int)
        }
    }
    return retVal;
}
/* For most of get command sub-block length is zero and sbData is NULL */
/* * @fn rlReturnVal_t rlAppendDummy(rlUInt8_t rhcpPayload[], rlUInt8_t dummyLen)
*
*   @brief Appends dummy bytes to Payload buffer
*   @param[out] rhcpPayload - payload buffer
*   @param[in] dummyLen - numnber of dummy bytes
*
*   @return rlReturnVal_t Success - 0, Failure - Error Code
*
*   Appends dummy bytes to Payload buffer
*/
/* DesignId :  */
/* Requirements :  */
#[no_mangle]
pub unsafe extern "C" fn rlAppendDummy(
    mut rhcpPayload: *mut rlUInt8_t,
    mut dummyLen: rlUInt8_t,
) -> rlReturnVal_t {
    let mut indx: rlUInt8_t = 0;
    /* In the given array fill dummy byte for the requested length */
    indx = 0 as core::ffi::c_uint as rlUInt8_t;
    while (indx as core::ffi::c_int) < dummyLen as core::ffi::c_int {
        /* fill dummy byte */
        *rhcpPayload.offset(indx as isize) = 0xff as core::ffi::c_uint as rlUInt8_t;
        indx = indx.wrapping_add(1)
    }
    return 0 as core::ffi::c_int;
}
/* * @fn rlReturnVal_t rlGetSubBlock(rlUInt8_t rhcpPayload[],
*                 rlUInt16_t* sbId, rlUInt16_t* sbLen, rlUInt8_t* sbData)
*
*   @brief Reads sub block data from payload buffer
*   @param[in] rhcpPayload - payload buffer
*   @param[out] sbId - sub block Id
*   @param[out] sbLen - sub block Length
*   @param[out] sbData - sub block data
*
*   @return rlReturnVal_t Success - 0, Failure - Error Code
*
*   Reads sub block data from payload buffer
*/
/* DesignId :  */
/* Requirements :  */
#[no_mangle]
pub unsafe extern "C" fn rlGetSubBlock(
    mut rhcpPayload: *mut rlUInt8_t,
    mut sbcId: *mut rlUInt16_t,
    mut sbLen: *mut rlUInt16_t,
    mut sbData: *mut rlUInt8_t,
) -> rlReturnVal_t {
    let mut retVal: rlReturnVal_t = 0;
    let mut payloadBuf: *mut rlUInt8_t = 0 as *mut rlUInt8_t;
    /* check for NULL prointer for all input parameters */
    if rhcpPayload.is_null() || sbcId.is_null() || sbLen.is_null() || sbData.is_null() {
        /* set error code */
        retVal = -(9 as core::ffi::c_int)
    } else {
        /* Get SB Id */
        /*AR_CODE_REVIEW MR:R.11.2 <APPROVED> "The Payload SbId is of type UINT16,
         * so pointer conversion type to UINT16*
         * is required" */
        /*LDRA_INSPECTED 94 S */
        /*LDRA_INSPECTED 95 S */
        /*LDRA_INSPECTED 87 S */
        *sbcId = *(rhcpPayload.offset(0 as core::ffi::c_uint as isize) as *mut rlUInt16_t);
        /* Get SB Len */
        /*AR_CODE_REVIEW MR:R.11.2 <APPROVED> "The Payload SbLen is of type UINT16,
         * so pointer conversion type to UINT16*
         * is required" */
        /*LDRA_INSPECTED 94 S */
        /*LDRA_INSPECTED 95 S */
        /*LDRA_INSPECTED 87 S */
        *sbLen = *(rhcpPayload
            .offset((0 as core::ffi::c_uint).wrapping_add(2 as core::ffi::c_uint) as isize)
            as *mut rlUInt16_t);
        /* check if sub-block length is beyond defined limit */
        if *sbLen as core::ffi::c_uint
            > (256 as core::ffi::c_uint).wrapping_sub(
                (4 as core::ffi::c_uint)
                    .wrapping_add(12 as core::ffi::c_uint)
                    .wrapping_add(8 as core::ffi::c_uint),
            )
        {
            /* set error code */
            retVal = -(9 as core::ffi::c_int)
        } else {
            payloadBuf = &mut *rhcpPayload.offset(
                (0 as core::ffi::c_uint)
                    .wrapping_add(2 as core::ffi::c_uint)
                    .wrapping_add(2 as core::ffi::c_uint) as isize,
            ) as *mut rlUInt8_t;
            /* Get SB Payload */
            if *sbLen as core::ffi::c_uint > 0 as core::ffi::c_uint
                && !sbData.is_null()
                && !payloadBuf.is_null()
            {
                /* copy input payload to sub-block data buffer */
                memcpy(
                    sbData as *mut core::ffi::c_void,
                    payloadBuf as *const core::ffi::c_void,
                    (*sbLen as core::ffi::c_uint)
                        .wrapping_sub((2 as core::ffi::c_uint).wrapping_add(2 as core::ffi::c_uint))
                        as _,
                );
            }
            retVal = 0 as core::ffi::c_int
        }
    }
    return retVal;
}
/* * @fn void rlGetSubBlockId(rlUInt8_t rhcpPayload[], rlUInt16_t* sbId)
*
*   @brief Reads sub block Id from payload buffer
*   @param[in] rhcpPayload - payload buffer
*   @param[out] sbId - sub block Id
*
*   @return NONE
*
*   Reads sub block Id from payload buffer
*/
/* DesignId :  */
/* Requirements :  */
#[no_mangle]
pub unsafe extern "C" fn rlGetSubBlockId(
    mut rhcpPayload: *const rlUInt8_t,
    mut sbcId: *mut rlUInt16_t,
) {
    /* Get SB Id */
    /*AR_CODE_REVIEW MR:R.11.2 <APPROVED> "The Payload SbId is of type UINT16,
     * so pointer conversion type to UINT16*
     * is required" */
    /*LDRA_INSPECTED 94 S */
    /*LDRA_INSPECTED 95 S */
    /*LDRA_INSPECTED 87 S */
    *sbcId = *(rhcpPayload.offset(0 as core::ffi::c_uint as isize) as *mut rlUInt16_t);
}
/* ***************************************************************************************
 * FileName     : rl_controller.h
 *
 * Description  : This file defines the functions to construct Radar Messages.
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
/* *****************************************************************************
 * GLOBAL VARIABLES/DATA-TYPES DEFINITIONS
 ******************************************************************************
 */
/* *****************************************************************************
 * FUNCTION PROTOTYPES
 ******************************************************************************
 */
/* Radar Configuration Functions */
/* * @fn void rlGetSubBlockLen(rlUInt8_t rhcpPayload[], rlUInt16_t* sbLen)
*
*   @brief Reads sub block Length from payload buffer
*   @param[in] rhcpPayload - payload buffer
*   @param[out] sbLen - sub block Len
*
*   @return NONE
*
*   Reads sub block Length from payload buffer
*/
/* DesignId :  */
/* Requirements :  */
#[no_mangle]
pub unsafe extern "C" fn rlGetSubBlockLen(
    mut rhcpPayload: *const rlUInt8_t,
    mut sbcLen: *mut rlUInt16_t,
) {
    /* Get SB Len */
    /*AR_CODE_REVIEW MR:R.11.2 <APPROVED> "The Payload SbLen is of type UINT16,
     * so pointer conversion type to UINT16*
     * is required" */
    /*LDRA_INSPECTED 94 S */
    /*LDRA_INSPECTED 95 S */
    /*LDRA_INSPECTED 87 S */
    *sbcLen = *(rhcpPayload
        .offset((0 as core::ffi::c_uint).wrapping_add(2 as core::ffi::c_uint) as isize)
        as *mut rlUInt16_t);
}
/*
 * END OF rl_controller.c FILE
 */
