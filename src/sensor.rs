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
pub fn get_frame_config(device_map: u8, data: &mut FrameConfig) -> i32 {
    unsafe { ffi::rlGetFrameConfig(device_map, &mut data.0 as *mut _) }
}

/// Sets Frame Configuration.
pub fn set_frame_config(device_map: u8, data: &mut FrameConfig) -> i32 {
    unsafe { ffi::rlSetFrameConfig(device_map, &mut data.0 as *mut _) }
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
pub fn get_adv_frame_config(device_map: u8, data: &mut AdvanceFrameConfig) -> i32 {
    unsafe { ffi::rlGetAdvFrameConfig(device_map, &mut data.0 as *mut _) }
}

/// Sets Advance Frame Configuration.
pub fn set_adv_frame_config(device_map: u8, data: &mut AdvanceFrameConfig) -> i32 {
    unsafe { ffi::rlSetAdvFrameConfig(device_map, &mut data.0 as *mut _) }
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

///Configure GP ADC data parameters
pub fn set_gp_adc_config(device_map: u8, data: &mut ffi::rlGpAdcCfg) -> i32 {
    unsafe { ffi::rlSetGpAdcConfig(device_map, data as *mut _) }
}

/// Enable/Disable phase shift configurations per chirp in each of the TXs.
pub fn rf_set_phase_shift_config(
    device_map: u8,
    cnt: u16,
    data: &mut ffi::rlRfPhaseShiftCfg,
) -> i32 {
    unsafe { ffi::rlRfSetPhaseShiftConfig(device_map, cnt, data as *mut _) }
}

/// Enable/Disable PA loopback for all enabled profiles.
pub fn rf_set_p_a_loopback_config(device_map: u8, data: &mut ffi::rlRfPALoopbackCfg) -> i32 {
    unsafe { ffi::rlRfSetPALoopbackConfig(device_map, data as *mut _) }
}

/// Enable/Disable Phase shift loopback for all enabled profiles.
pub fn rf_set_p_s_loopback_config(device_map: u8, data: &mut ffi::rlRfPSLoopbackCfg) -> i32 {
    unsafe { ffi::rlRfSetPSLoopbackConfig(device_map, data as *mut _) }
}

/// Enable/Disable RF IF loopback for all enabled profiles. This is used for debug to check if both TX and RX chains are working correctly.
pub fn rf_set_i_f_loopback_config(device_map: u8, data: &mut ffi::rlRfIFLoopbackCfg) -> i32 {
    unsafe { ffi::rlRfSetIFLoopbackConfig(device_map, data as *mut _) }
}

/// Set Programmable Filter coefficient RAM.
pub fn rf_set_prog_filt_coeff_ram(device_map: u8, data: &mut ffi::rlRfProgFiltCoeff) -> i32 {
    unsafe { ffi::rlRfSetProgFiltCoeffRam(device_map, data as *mut _) }
}

/// Set Programmable Filter configuration.
pub fn rf_set_prog_filt_config(device_map: u8, data: &mut ffi::rlRfProgFiltConf) -> i32 {
    unsafe { ffi::rlRfSetProgFiltConfig(device_map, data as *mut _) }
}

/// Sets misc feature such as per chirp phase shifter.
pub fn rf_set_misc_config(device_map: u8, data: &mut ffi::rlRfMiscConf) -> i32 {
    unsafe { ffi::rlRfSetMiscConfig(device_map, data as *mut _) }
}

/// Set Calibration monitoring time unit.
pub fn rf_set_cal_mon_time_unit_config(
    device_map: u8,
    data: &mut ffi::rlRfCalMonTimeUntConf,
) -> i32 {
    unsafe { ffi::rlRfSetCalMonTimeUnitConfig(device_map, data as *mut _) }
}

/// Set Calibration monitoring Frequency Limit.
pub fn rf_set_cal_mon_freq_limit_config(
    device_map: u8,
    data: &mut ffi::rlRfCalMonFreqLimitConf,
) -> i32 {
    unsafe { ffi::rlRfSetCalMonFreqLimitConfig(device_map, data as *mut _) }
}

/// Set RF Init Calibration Mask bits and report type.
pub fn rf_init_calib_config(device_map: u8, data: &mut ffi::rlRfInitCalConf) -> i32 {
    unsafe { ffi::rlRfInitCalibConfig(device_map, data as *mut _) }
}

/// Set RF one time & periodic calibration of various RF/analog aspects and trigger.
pub fn rf_run_time_calib_config(device_map: u8, data: &mut ffi::rlRunTimeCalibConf) -> i32 {
    unsafe { ffi::rlRfRunTimeCalibConfig(device_map, data as *mut _) }
}

/// Overwrite RX gain temperature Lookup Table(LUT) in Radar SS.
pub fn rx_gain_temp_lut_set(device_map: u8, data: &mut ffi::rlRxGainTempLutData) -> i32 {
    unsafe { ffi::rlRxGainTempLutSet(device_map, data as *mut _) }
}

/// Overwrites TX gain temperature based Lookup table (LUT)
pub fn tx_gain_temp_lut_set(device_map: u8, data: &mut ffi::rlTxGainTempLutData) -> i32 {
    unsafe { ffi::rlTxGainTempLutSet(device_map, data as *mut _) }
}

/// Sets the limits for RF frequency transmission for each TX and also TX power limits.
pub fn rf_tx_freq_pwr_limit_config(
    device_map: u8,
    data: &mut ffi::rlRfTxFreqPwrLimitMonConf,
) -> i32 {
    unsafe { ffi::rlRfTxFreqPwrLimitConfig(device_map, data as *mut _) }
}

/// This API is used to introduce loopback chirps within the functional frames.
pub fn set_loop_bck_burst_cfg(device_map: u8, data: &mut ffi::rlLoopbackBurst) -> i32 {
    unsafe { ffi::rlSetLoopBckBurstCfg(device_map, data as *mut _) }
}

/// Injects chirp configuration to be programmed dynamically.
pub fn set_dyn_chirp_cfg(device_map: u8, seg_cnt: u16, data: &mut ffi::rlDynChirpCfg) -> i32 {
    let mut ptr = data as *mut _;
    unsafe { ffi::rlSetDynChirpCfg(device_map, seg_cnt, &mut ptr as *mut _) }
}

/// Triggers copy of chirp config from SW to HW RAM.
pub fn set_dyn_chirp_en(device_map: u8, data: &mut ffi::rlDynChirpEnCfg) -> i32 {
    unsafe { ffi::rlSetDynChirpEn(device_map, data as *mut _) }
}

/// Injects per-chirp phase shifter configuration to be applied dynamically.
pub fn set_dyn_per_chirp_ph_shifter_cfg(
    device_map: u8,
    seg_cnt: u16,
    data: &mut ffi::rlDynPerChirpPhShftCfg,
) -> i32 {
    let mut ptr = data as *mut _;
    unsafe { ffi::rlSetDynPerChirpPhShifterCfg(device_map, seg_cnt, &mut ptr as *mut _) }
}

/// Read calibration data from the device.
pub fn rf_calib_data_store(device_map: u8, data: &mut ffi::rlCalibrationData) -> i32 {
    unsafe { ffi::rlRfCalibDataStore(device_map, data as *mut _) }
}

/// Injects calibration data to the device.
pub fn rf_calib_data_restore(device_map: u8, data: &mut ffi::rlCalibrationData) -> i32 {
    unsafe { ffi::rlRfCalibDataRestore(device_map, data as *mut _) }
}

/// Sets different Rx gain/phase offset.
pub fn rf_inter_rx_gain_phase_config(device_map: u8, data: &mut ffi::rlInterRxGainPhConf) -> i32 {
    unsafe { ffi::rlRfInterRxGainPhaseConfig(device_map, data as *mut _) }
}
/// Get radarSS bootup status.
pub fn get_rf_bootup_status(device_map: u8, data: &mut ffi::rlRfBootStatusCfg) -> i32 {
    unsafe { ffi::rlGetRfBootupStatus(device_map, data as *mut _) }
}
/// Sets Inter-chip turn on and turn off times or various RF blocks.
pub fn set_inter_chirp_blk_ct(device_map: u8, data: &mut ffi::rlInterChirpBlkCtrlCfg) -> i32 {
    unsafe { ffi::rlSetInterChirpBlkCtrl(device_map, data as *mut _) }
}
/// Triggers the next sub-frame in software triggered sub-frame mode.
pub fn set_sub_frame_start(device_map: u8, data: &mut ffi::rlSubFrameStartCfg) -> i32 {
    unsafe { ffi::rlSetSubFrameStart(device_map, data as *mut _) }
}
/// Read calibration data from the device.
pub fn rf_ph_shift_calib_data_store(
    device_map: u8,
    data: &mut ffi::rlPhShiftCalibrationData,
) -> i32 {
    unsafe { ffi::rlRfPhShiftCalibDataStore(device_map, data as *mut _) }
}
/// Injects phase shifter calibration data to the device.
pub fn rf_ph_shift_calib_data_restore(
    device_map: u8,
    data: &mut ffi::rlPhShiftCalibrationData,
) -> i32 {
    unsafe { ffi::rlRfPhShiftCalibDataRestore(device_map, data as *mut _) }
}
/// Get device die ID status.
pub fn get_rf_die_id(device_map: u8, data: &mut ffi::rlRfDieIdCfg) -> i32 {
    unsafe { ffi::rlGetRfDieId(device_map, data as *mut _) }
}
/// Get RadarSS ESM fault status.
pub fn rf_get_esm_fault(device_map: u8, data: &mut ffi::rlBssEsmFault) -> i32 {
    unsafe { ffi::rlRfGetEsmFault(device_map, data as *mut _) }
}
/// Get RadarSS CPU fault status.
pub fn rf_get_cpu_fault(device_map: u8, data: &mut ffi::rlCpuFault) -> i32 {
    unsafe { ffi::rlRfGetCpuFault(device_map, data as *mut _) }
}
/// Control bandwidth of the APLL and Synthesizer.
pub fn rf_apll_synth_bw_ctl_config(device_map: u8, data: &mut ffi::rlRfApllSynthBwControl) -> i32 {
    unsafe { ffi::rlRfApllSynthBwCtlConfig(device_map, data as *mut _) }
}
/// Sets the power save mode Configuration.
pub fn set_power_save_mode_config(device_map: u8, data: &mut PowerSaveModeConfig) -> i32 {
    unsafe { ffi::rlSetPowerSaveModeConfig(device_map, &mut data.0 as *mut _) }
}

/// API over `mmwavelink::ffi::rlCpuFault`
#[derive(Debug, Clone, Copy, Default)]
#[repr(transparent)]
pub struct CpuFault(ffi::rlCpuFault);

impl CpuFault {
    pub fn new(
        fault_type: u8,
        line_num: u16,
        fault_lr: u32,
        fault_prev_lr: u32,
        fault_spsr: u32,
        fault_sp: u32,
        fault_addr: u32,
        fault_err_status: u16,
        fault_err_src: u8,
        fault_axi_err_type: u8,
        fault_acc_type: u8,
        fault_recovery_type: u8,
    ) -> Self {
        Self(ffi::rlCpuFault {
            faultType: fault_type,
            reserved0: 0,
            lineNum: line_num,
            faultLR: fault_lr,
            faultPrevLR: fault_prev_lr,
            faultSpsr: fault_spsr,
            faultSp: fault_sp,
            faultAddr: fault_addr,
            faultErrStatus: fault_err_status,
            faultErrSrc: fault_err_src,
            faultAxiErrType: fault_axi_err_type,
            faultAccType: fault_acc_type,
            faultRecovType: fault_recovery_type,
            reserved1: 0,
        })
    }
}

/// API over `mmwavelink::ffi::rlPowerSaveModeCfg`
#[repr(transparent)]
pub struct PowerSaveModeConfig(ffi::rlPowerSaveModeCfg);

impl PowerSaveModeConfig {
    pub fn new(low_power_state_transition_cmd: u16) -> Self {
        Self(ffi::rlPowerSaveModeCfg {
            lowPwrStateTransCmd: low_power_state_transition_cmd,
            reserved0: 0,
            reserved: [0, 0, 0, 0],
        })
    }
}

/// API over `mmwavelink::ffi::rlFrameCfg`
#[repr(transparent)]
pub struct FrameConfig(ffi::rlFrameCfg);

impl FrameConfig {
    pub fn new(
        chirp_start_idx: u16,
        chirp_end_idx: u16,
        n_loops: u16,
        n_frames: u16,
        n_adc_samples: u16,
        frame_periodicity: u32,
        trigger_select: u16,
        n_dummy_chirps_at_end: u8,
        frame_trigger_delay: u32,
    ) -> Self {
        Self(ffi::rlFrameCfg {
            reserved0: 0,
            chirpStartIdx: chirp_start_idx,
            chirpEndIdx: chirp_end_idx,
            numLoops: n_loops,
            numFrames: n_frames,
            numAdcSamples: n_adc_samples,
            framePeriodicity: frame_periodicity,
            triggerSelect: trigger_select,
            reserved1: 0,
            numDummyChirpsAtEnd: n_dummy_chirps_at_end,
            frameTriggerDelay: frame_trigger_delay,
        })
    }
}

/// API over `mmwavelink::ffi::rlAdvFrameCfg
#[repr(transparent)]
pub struct AdvanceFrameConfig(ffi::rlAdvFrameCfg);

impl AdvanceFrameConfig {
    pub fn new(
        frame_sequence: AdvanceFrameSequenceConfig,
        frame_data: AdvanceFrameDataConfig,
    ) -> Self {
        Self(ffi::rlAdvFrameCfg {
            frameSeq: frame_sequence.0,
            frameData: frame_data.0,
        })
    }
}

/// API over `mmwavelink::ffi::rlAdvFrameSeqCfg`
#[repr(transparent)]
pub struct AdvanceFrameSequenceConfig(ffi::rlAdvFrameSeqCfg);

impl AdvanceFrameSequenceConfig {
    pub fn new(
        n_sub_frames: u8,
        force_profile: u8,
        loop_back_config: u8,
        sub_frame_trigger: u8,
        sub_frame_config: [ffi::rlSubFrameCfg; 4usize],
        n_frames: u16,
        trigger_select: u16,
        frame_trigger_delay: u32,
    ) -> Self {
        Self(ffi::rlAdvFrameSeqCfg {
            numOfSubFrames: n_sub_frames,
            forceProfile: force_profile,
            loopBackCfg: loop_back_config,
            subFrameTrigger: sub_frame_trigger,
            subFrameCfg: sub_frame_config,
            numFrames: n_frames,
            triggerSelect: trigger_select,
            frameTrigDelay: frame_trigger_delay,
            reserved0: 0,
            reserved1: 0,
        })
    }
}

/// API over `ffi::rlAdvFrameDataCfg`
#[repr(transparent)]
pub struct AdvanceFrameDataConfig(ffi::rlAdvFrameDataCfg);

impl AdvanceFrameDataConfig {
    pub fn new(n_sub_frames: u8, sub_frame_data_config: [ffi::rlSubFrameDataCfg; 4usize]) -> Self {
        Self(ffi::rlAdvFrameDataCfg {
            numSubFrames: n_sub_frames,
            reserved0: 0,
            reserved1: 0,
            subframeDataCfg: sub_frame_data_config,
        })
    }
}
