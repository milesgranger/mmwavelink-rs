use mmwavelink_sys::ffi;

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
pub struct EventCallback(ffi::rlEventCbs);

#[derive(Debug, Default, Copy, Clone)]
#[repr(transparent)]
pub struct DeviceControlCallbacks(ffi::rlDeviceCtrlCbs);

#[derive(Debug, Default, Copy, Clone)]
#[repr(transparent)]
pub struct TimerCallback(ffi::rlTimerCbs);

#[derive(Debug, Default, Copy, Clone)]
#[repr(transparent)]
pub struct CrcCallback(ffi::rlCrcCbs);

#[derive(Debug, Default, Copy, Clone)]
#[repr(transparent)]
pub struct DebugCallback(ffi::rlDbgCb);

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
    Sixteen = 16,
    ThirtyTwo = 32,
    SixtyFour = 64,
}

impl ClientCallBacks {
    pub fn new(
        communication_interface_cb: CommunicationInterfaceCallbacks,
        osi_cb: OsiCallbacks,
        event_cb: EventCallback,
        device_ctrl_cb: DeviceControlCallbacks,
        timer_cb: TimerCallback,
        crc_cb: CrcCallback,
        crc_type: CrcType,
        ack_timeout: u32,
        platform: Platform,
        device_type: DeviceType,
        dbg_callback: DebugCallback,
    ) -> Self {
        Self(ffi::rlClientCbs {
            comIfCb: communication_interface_cb.0,
            osiCb: osi_cb.0,
            eventCb: event_cb.0,
            devCtrlCb: device_ctrl_cb.0,
            timerCb: timer_cb.0,
            cmdParserCb: ffi::rlCmdParserCbs::default(), // TI internal use only
            crcCb: crc_cb.0,
            crcType: crc_type as u8,
            ackTimeout: ack_timeout,
            platform: platform as u8,
            arDevType: device_type as u8,
            dbgCb: dbg_callback.0,
        })
    }
}

/// Bring mmwave Device Out of Reset
pub fn device_power_on(device_map: u8, client_cb: ffi::rlClientCbs) -> i32 {
    unsafe { ffi::rlDevicePowerOn(device_map, client_cb) }
}
