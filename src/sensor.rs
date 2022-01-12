//! Sensor module
//!
//! Minimal documentation contained here, likely best to use in conjunction
//! with official SDK provided documentation in ti/control/mmwavelink/docs/doxygen/html
//!
use mmwavelink_sys as ffi;

/// Sets the Rx and Tx Channel Configuration.
pub fn set_channel_config(device_map: u8, data: &mut ffi::rlChanCfg) -> i32 {
    unsafe { ffi::rlSetChannelConfig(device_map, data as *mut _) }
}

/// Sets ADC Output Configuration
pub fn set_adc_out_config(device_map: u8, data: &mut ffi::rlAdcOutCfg) -> i32 {
    unsafe { ffi::rlSetAdcOutConfig(device_map, data as *mut _) }
}

/// Sets Low Power Mode Configuration.
pub fn set_low_power_mode_config(device_map: u8, data: &mut ffi::rlLowPowerModeCfg) -> i32 {
    unsafe { ffi::rlSetLowPowerModeConfig(device_map, data as *mut _) }
}

/// Initializes the RF/Analog Subsystem.
pub fn rf_init(device_map: u8) -> i32 {
    unsafe { ffi::rlRfInit(device_map) }
}

/// Gets Chirp profile Configuration.
pub fn get_profile_config(device_map: u8, profile_id: u16, data: &mut ffi::rlProfileCfg) -> i32 {
    unsafe { ffi::rlGetProfileConfig(device_map, profile_id, data as *mut _) }
}

/// Sets Chirp profile Configuration
pub fn set_profile_config(device_map: u8, cnt: u16, data: &mut ffi::rlProfileCfg) -> i32 {
    unsafe { ffi::rlSetProfileConfig(device_map, cnt, data as *mut _) }
}

/// Gets Chirp Configuration.
pub fn get_chirp_config(
    device_map: u8,
    chirp_start_idx: u16,
    chirp_end_idx: u16,
    data: &mut ffi::rlChirpCfg,
) -> i32 {
    unsafe { ffi::rlGetChirpConfig(device_map, chirp_start_idx, chirp_end_idx, data as *mut _) }
}

/// Sets Chirp Configuration
pub fn set_chirp_config(device_map: u8, cnt: u16, data: &mut ffi::rlChirpCfg) -> i32 {
    unsafe { ffi::rlSetChirpConfig(device_map, cnt, data as *mut _) }
}

/// Injects chirp configuration to be programmed dynamically.
pub fn set_multi_chirp_config(device_map: u8, cnt: u16, data: &mut ffi::rlChirpCfg) -> i32 {
    let mut ptr = data as *mut _;
    unsafe { ffi::rlSetMultiChirpCfg(device_map, cnt, &mut ptr as *mut _) }
}

/// Gets Frame Configuration.
pub fn get_frame_config(device_map: u8, data: &mut ffi::rlFrameCfg) -> i32 {
    unsafe { ffi::rlGetFrameConfig(device_map, data as *mut _) }
}

/// Sets Frame Configuration.
pub fn set_frame_config(device_map: u8, data: &mut ffi::rlFrameCfg) -> i32 {
    unsafe { ffi::rlSetFrameConfig(device_map, data as *mut _) }
}

/// Triggers Transmission of Frames.
pub fn sensor_start(device_map: u8) -> i32 {
    unsafe { ffi::rlSensorStart(device_map) }
}

/// Stops Transmission of Frames.
pub fn sensor_stop(device_map: u8) -> i32 {
    unsafe { ffi::rlSensorStop(device_map) }
}

/// Gets Advance Frame Configuration.
pub fn get_adv_frame_config(device_map: u8, data: &mut ffi::rlAdvFrameCfg) -> i32 {
    unsafe { ffi::rlGetAdvFrameConfig(device_map, data as *mut _) }
}

/// Sets Advance Frame Configuration.
pub fn set_adv_frame_config(device_map: u8, data: &mut ffi::rlAdvFrameCfg) -> i32 {
    unsafe { ffi::rlSetAdvFrameConfig(device_map, data as *mut _) }
}

/// Sets Continous mode Configuration.
pub fn set_cont_mode_config(device_map: u8, data: &mut ffi::rlContModeCfg) -> i32 {
    unsafe { ffi::rlSetContModeConfig(device_map, data as *mut _) }
}

/// Enable/Disable Continous mode.
pub fn enable_cont_mode(device_map: u8, data: &mut ffi::rlContModeEn) -> i32 {
    unsafe { ffi::rlEnableContMode(device_map, data as *mut _) }
}

/// Sets Binary Phase Modulation Common Configuration.
pub fn set_bpm_common_config(device_map: u8, data: &mut ffi::rlBpmCommonCfg) -> i32 {
    unsafe { ffi::rlSetBpmCommonConfig(device_map, data as *mut _) }
}

/// Sets Binary Phase Modulation Chirp Configuration.
pub fn set_bpm_chirp_config(device_map: u8, data: &mut ffi::rlBpmChirpCfg) -> i32 {
    unsafe { ffi::rlSetBpmChirpConfig(device_map, data as *mut _) }
}

/// Sets Binary Phase Modulation configuration for multiple Chirp.
pub fn set_multi_bpm_chirp_config(device_map: u8, cnt: u16, data: &mut ffi::rlBpmChirpCfg) -> i32 {
    let mut ptr = data as *mut _;
    unsafe { ffi::rlSetMultiBpmChirpConfig(device_map, cnt, &mut ptr as *mut _) }
}

/// Configures the Test Source.
pub fn set_test_source_config(device_map: u8, data: &mut ffi::rlTestSource) -> i32 {
    unsafe { ffi::rlSetTestSourceConfig(device_map, data as *mut _) }
}

/// Enables the Test Source.
pub fn test_source_enable(device_map: u8, data: &mut ffi::rlTestSourceEnable_t) -> i32 {
    unsafe { ffi::rlTestSourceEnable(device_map, data as *mut _) }
}

/// Gets Time and Temperature information report.
pub fn rf_get_temperature_report(device_map: u8, data: &mut ffi::rlRfTempData_t) -> i32 {
    unsafe { ffi::rlRfGetTemperatureReport(device_map, data as *mut _) }
}

/// Gets Digital Front end statistics such as Residual DC, RMS power in I and Q chains for different Receive channels for different selected profiles. It also includes Cross correlation between I and Q chains.
pub fn rf_dfe_rx_statistics_report(device_map: u8, data: &mut ffi::rlDfeStatReport_t) -> i32 {
    unsafe { ffi::rlRfDfeRxStatisticsReport(device_map, data as *mut _) }
}

/// Configure dynamic power saving feature.
pub fn rf_dynamic_power_save(device_map: u8, data: &mut ffi::rlDynPwrSave_t) -> i32 {
    unsafe { ffi::rlRfDynamicPowerSave(device_map, data as *mut _) }
}

///Set different RadarSS device configurations
pub fn rf_set_device_cfg(device_map: u8, data: &mut ffi::rlRfDevCfg_t) -> i32 {
    unsafe { ffi::rlRfSetDeviceCfg(device_map, data as *mut _) }
}

/*
///Configure GP ADC data parameters
pub fn set_gp_adc_config (u8 device_map, rlGpAdcCfg_t &mut data: )
/// Enable/Disable phase shift configurations per chirp in each of the TXs.
pub fn rf_set_phase_shift_config (u8 device_map, rlUInt16_t cnt, rlRfPhaseShiftCfg_t &mut data: )
/// Enable/Disable PA loopback for all enabled profiles.
pub fn rf_set_p_a_loopback_config (u8 device_map, rlRfPALoopbackCfg_t &mut data: )
/// Enable/Disable Phase shift loopback for all enabled profiles.
pub fn rf_set_p_s_loopback_config (u8 device_map, rlRfPSLoopbackCfg_t &mut data: )
/// Enable/Disable RF IF loopback for all enabled profiles. This is used for debug to check if both TX and RX chains are working correctly.
pub fn rf_set_i_f_loopback_config (u8 device_map, rlRfIFLoopbackCfg_t &mut data: )
/// Set Programmable Filter coefficient RAM.
pub fn rf_set_prog_filt_coeff_ram (u8 device_map, rlRfProgFiltCoeff_t &mut data: )
/// Set Programmable Filter configuration.
pub fn rf_set_prog_filt_config (u8 device_map, rlRfProgFiltConf_t &mut data: )
/// Sets misc feature such as per chirp phase shifter.
pub fn rf_set_misc_config (u8 device_map, rlRfMiscConf_t &mut data: )
/// Set Calibration monitoring time unit.
pub fn rf_set_cal_mon_time_unit_config (u8 device_map, rlRfCalMonTimeUntConf_t &mut data: )
/// Set Calibration monitoring Frequency Limit.
pub fn rf_set_cal_mon_freq_limit_config (u8 device_map, rlRfCalMonFreqLimitConf_t &mut data: )
/// Set RF Init Calibration Mask bits and report type.
pub fn rf_init_calib_config (u8 device_map, rlRfInitCalConf_t &mut data: )
/// Set RF one time & periodic calibration of various RF/analog aspects and trigger.
pub fn rf_run_time_calib_config (u8 device_map, rlRunTimeCalibConf_t &mut data: )
/// Overwrite RX gain temperature Lookup Table(LUT) in Radar SS.
pub fn rx_gain_temp_lut_set (u8 device_map, rlRxGainTempLutData_t &mut data: )
/// Overwrites TX gain temperature based Lookup table (LUT)
pub fn tx_gain_temp_lut_set (u8 device_map, rlTxGainTempLutData_t &mut data: )
/// Sets the limits for RF frequency transmission for each TX and also TX power limits.
pub fn rf_tx_freq_pwr_limit_config (u8 device_map, rlRfTxFreqPwrLimitMonConf_t &mut data: )
/// This API is used to introduce loopback chirps within the functional frames.
pub fn set_loop_bck_burst_cfg (u8 device_map, rlLoopbackBurst_t &mut data: )
/// Injects chirp configuration to be programmed dynamically.
pub fn set_dyn_chirp_cfg (u8 device_map, rlUInt16_t segCnt, rlDynChirpCfg_t **data)
/// Triggers copy of chirp config from SW to HW RAM.
pub fn set_dyn_chirp_en (u8 device_map, rlDynChirpEnCfg_t &mut data: )
/// Injects per-chirp phase shifter configuration to be applied dynamically.
pub fn set_dyn_per_chirp_ph_shifter_cfg (u8 device_map, rlUInt16_t segCnt, rlDynPerChirpPhShftCfg_t **data)
/// Read calibration data from the device.
pub fn rf_calib_data_store (u8 device_map, rlCalibrationData_t &mut data: )
/// Injects calibration data to the device.
pub fn rf_calib_data_restore (u8 device_map, rlCalibrationData_t &mut data: )
/// Sets different Rx gain/phase offset.
pub fn rf_inter_rx_gain_phase_config (u8 device_map, rlInterRxGainPhConf_t &mut data: )
/// Get radarSS bootup status.
pub fn get_rf_bootup_status (u8 device_map, rlRfBootStatusCfg_t &mut data: )
/// Sets Inter-chip turn on and turn off times or various RF blocks.
pub fn set_inter_chirp_blk_ct (u8 device_map, rlInterChirpBlkCtrlCfg_t &mut data: )
/// Triggers the next sub-frame in software triggered sub-frame mode.
pub fn set_sub_frame_start (u8 device_map, rlSubFrameStartCfg_t &mut data: )
/// Read calibration data from the device.
pub fn rf_ph_shift_calib_data_store (u8 device_map, rlPhShiftCalibrationData_t &mut data: )
/// Injects phase shifter calibration data to the device.
pub fn rf_ph_shift_calib_data_restore (u8 device_map, rlPhShiftCalibrationData_t &mut data: )
/// Get device die ID status.
pub fn get_rf_die_id (u8 device_map, rlRfDieIdCfg_t &mut data: )
/// Get RadarSS ESM fault status.
pub fn rf_get_esm_fault (u8 device_map, rlBssEsmFault_t &mut data: )
/// Get RadarSS CPU fault status.
pub fn rf_get_cpu_fault (u8 device_map, rlCpuFault_t &mut data: )
/// Control bandwidth of the APLL and Synthesizer.
pub fn rf_apll_synth_bw_ctl_config (u8 device_map, rlRfApllSynthBwControl_t &mut data: )
/// Sets the power save mode Configuration.
pub fn set_power_save_mode_config (u8 device_map, rlPowerSaveModeCfg_t &mut data: )

 */
