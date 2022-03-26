use core::slice;
use core::{convert::TryInto, ffi::c_void};
use mmwavelink_sys::ffi;

use crate::error::{Error, Result};

/// Communication interface(SPI, MailBox, UART etc) callback functions.
#[derive(Debug, Copy, Default, Clone)]
#[repr(transparent)]
pub struct CommunicationInterfaceCallbacks(ffi::rlComIfCbs);

impl CommunicationInterfaceCallbacks {
    pub fn new(
        com_open: unsafe extern "C" fn(device_idx: u8, flags: u32) -> ffi::rlComIfHdl_t,
        com_read: unsafe extern "C" fn(fd: ffi::rlComIfHdl_t, p_buff: *mut u8, len: u16) -> i32,
        com_write: unsafe extern "C" fn(fd: ffi::rlComIfHdl_t, p_buff: *mut u8, len: u16) -> i32,
        com_close: unsafe extern "C" fn(fd: ffi::rlComIfHdl_t) -> i32,
    ) -> Self {
        Self(ffi::rlComIfCbs {
            rlComIfOpen: Some(com_open),
            rlComIfRead: Some(com_read),
            rlComIfWrite: Some(com_write),
            rlComIfClose: Some(com_close),
        })
    }
}

#[derive(Debug, Default, Copy, Clone)]
#[repr(transparent)]
pub struct OsiCallbacks(ffi::rlOsiCbs);

#[derive(Debug, Default, Copy, Clone)]
#[repr(transparent)]
pub struct DeviceControlCallbacks(ffi::rlDeviceCtrlCbs);

impl DeviceControlCallbacks {
    pub fn new(
        device_enable: Option<unsafe extern "C" fn(device_index: u8) -> i32>,
        device_disable: Option<unsafe extern "C" fn(device_index: u8) -> i32>,
        device_mask_host_irq: Option<unsafe extern "C" fn(fd: *mut c_void)>,
        device_unmask_host_irq: Option<unsafe extern "C" fn(fd: *mut c_void)>,
        device_wait_irq_status: Option<unsafe extern "C" fn(fd: *mut c_void, high_low: u8) -> i32>,
        comm_if_assert_irq: Option<unsafe extern "C" fn(high_low: u8) -> u16>,
        register_interrupt_handler: Option<
            unsafe extern "C" fn(
                device_index: u8,
                handler: Option<unsafe extern "C" fn(u8, *mut c_void)>,
                *mut c_void,
            ) -> i32,
        >,
    ) -> Self {
        Self(ffi::rlDeviceCtrlCbs {
            rlDeviceEnable: device_enable,
            rlDeviceDisable: device_disable,
            rlDeviceMaskHostIrq: device_mask_host_irq,
            rlDeviceUnMaskHostIrq: device_unmask_host_irq,
            rlDeviceWaitIrqStatus: device_wait_irq_status,
            rlCommIfAssertIrq: comm_if_assert_irq,
            rlRegisterInterruptHandler: register_interrupt_handler,
        })
    }
}

#[derive(Debug, Default, Copy, Clone)]
#[repr(transparent)]
pub struct DebugCallback(ffi::rlDbgCb);

impl DebugCallback {
    pub fn new(debug_cb: unsafe extern "C" fn(format: *const u8, ...) -> i32, dbg_lvl: u8) -> Self {
        Self(ffi::rlDbgCb {
            rlPrint: Some(debug_cb),
            dbgLevel: dbg_lvl,
        })
    }
}

#[derive(Debug, Copy, Default, Clone)]
#[repr(transparent)]
pub struct ClientCallBacks(ffi::rlClientCbs);

#[repr(u8)]
#[derive(Debug, Copy, Clone)]
pub enum Platform {
    ExtHost = 0x0,
    MSS = 0x1,
    DSS = 0x2,
}

#[repr(u8)]
#[derive(Debug, Copy, Clone)]
pub enum DeviceType {
    XWR1243Host = 0x0,
    XWR1443Mss = 0x1,
    XWR1642MssDss = 0x2,
    XWR1843MssDss = 0x3,
    XWR6843MssDss = 0x4,
}

#[repr(u8)]
#[derive(Debug, Copy, Clone)]
pub enum CrcType {
    Sixteen = 16_u8,
    ThirtyTwo = 32_u8,
    SixtyFour = 64_u8,
}

pub fn u32_from_le_bytes(payload: *mut u8, len: u16) -> Result<u32> {
    if len != 4 {
        todo!("Handle when len doesn't equal 4");
    }
    let bytes_slice = unsafe { slice::from_raw_parts(payload as _, len as _) };
    match bytes_slice.try_into() {
        Ok(bytes) => Ok(u32::from_le_bytes(bytes)),
        Err(_) => todo!("Failed to convert bytes slice to array of 4"),
    }
}

impl ClientCallBacks {
    pub fn new(
        communication_interface_cb: CommunicationInterfaceCallbacks,
        osi_cb: OsiCallbacks,
        event_cb: Option<
            unsafe extern "C" fn(dev_idx: u8, sub_id: u16, sub_len: u16, payload: *mut u8),
        >,
        device_ctrl_cb: DeviceControlCallbacks,
        timer_cb: Option<unsafe extern "C" fn(delay: u32) -> i32>,
        crc_cb: Option<
            unsafe extern "C" fn(data: *mut u8, len: u32, crc_type: u8, crc: *mut u8) -> i32,
        >,
        crc_type: CrcType,
        ack_timeout: u32,
        platform: Platform,
        device_type: DeviceType,
        dbg_callback: DebugCallback,
    ) -> Self {
        Self(ffi::rlClientCbs {
            comIfCb: communication_interface_cb.0,
            osiCb: osi_cb.0,
            eventCb: ffi::rlEventCbs {
                rlAsyncEvent: event_cb,
            },
            devCtrlCb: device_ctrl_cb.0,
            timerCb: ffi::rlTimerCbs { rlDelay: timer_cb },
            cmdParserCb: ffi::rlCmdParserCbs::default(), // TI internal use only
            crcCb: ffi::rlCrcCbs {
                rlComputeCRC: crc_cb,
            },
            crcType: crc_type as u8,
            ackTimeout: ack_timeout,
            platform: platform as u8,
            arDevType: device_type as u8,
            dbgCb: dbg_callback.0,
        })
    }
}

/// Sanity check calls to mmwavelink C api work
pub fn sanity_echo(v: i32) -> i32 {
    unsafe { ffi::sanity_echo(v) }
}

/// Bring mmwave Device Out of Reset
pub fn device_power_on(device_map: u8, client_cb: ClientCallBacks) -> Result<()> {
    Error::maybe_device_error(unsafe { ffi::rlDevicePowerOn(device_map, client_cb.0) }, ())
}

/// Shutdown mmwave device.
pub fn device_power_off() -> Result<()> {
    Error::maybe_device_error(unsafe { ffi::rlDevicePowerOff() }, ())
}
