use snafu::prelude::*;

#[derive(Debug, Snafu)]
pub enum Error {
    #[snafu(display("Sensor erorr, ret code: {code}"))]
    SensorError { code: i32 },
    #[snafu(display("Device erorr, ret code: {code}"))]
    DeviceError { code: i32 },
}

impl Error {
    pub fn new_device_error(code: i32) -> Self {
        Self::DeviceError { code }
    }
    pub fn new_sensor_error(code: i32) -> Self {
        Self::SensorError { code }
    }
    pub fn maybe_sensor_error<T>(code: i32, or: T) -> Result<T> {
        if code != 0 {
            Err(Error::SensorError { code })
        } else {
            Ok(or)
        }
    }
    pub fn maybe_device_error<T>(code: i32, or: T) -> Result<T> {
        if code != 0 {
            Err(Error::DeviceError { code })
        } else {
            Ok(or)
        }
    }
}

pub type Result<T, E = Error> = core::result::Result<T, E>;
