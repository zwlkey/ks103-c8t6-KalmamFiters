// Most of the functionality of this library is based on the VL53L1X API
// provided by ST (STSW-IMG007), and some of the explanatory comments are quoted
// or paraphrased from the API source code, API user manual (UM2356), and
// VL53L1X datasheet.

#include "VL53L1X.h"
#include "vl53lxx_i2c.h"
#include "delay.h"

u8 address = 0x52;
u8 distance_mode;
RangingData ranging_data;

ResultBuffer results;

uint16_t io_timeout;
u8 did_timeout;
uint16_t timeout_start_ms;

uint16_t fast_osc_frequency;
uint16_t osc_calibrate_val;

u8 calibrated=1;
uint8_t saved_vhv_init;
uint8_t saved_vhv_timeout;
	
uint8_t last_status; // status of last I2C transmission

// The Arduino two-wire interface uses a 7-bit number for the address,
// and sets the last bit correctly based on reads and writes
const uint8_t AddressDefault = 0x29;

// value used in measurement timing budget calculations
// assumes PresetMode is LOWPOWER_AUTONOMOUS
//
// vhv = LOWPOWER_AUTO_VHV_LOOP_DURATION_US + LOWPOWERAUTO_VHV_LOOP_BOUND
//       (tuning parm default) * LOWPOWER_AUTO_VHV_LOOP_DURATION_US
//     = 245 + 3 * 245 = 980
// TimingGuard = LOWPOWER_AUTO_OVERHEAD_BEFORE_A_RANGING +
//               LOWPOWER_AUTO_OVERHEAD_BETWEEN_A_B_RANGING + vhv
//             = 1448 + 2100 + 980 = 4528
const uint32_t TimingGuard = 4528;

// value in DSS_CONFIG__TARGET_TOTAL_RATE_MCPS register, used in DSS
// calculations
const uint16_t TargetRate = 0x0A00;
	
void setAddress(uint8_t new_addr)
{
  writeReg(VL53L1_I2C_SLAVE__DEVICE_ADDRESS, new_addr & 0x7F);
  address = new_addr;
}

u8 getAddress() { return address; }
u8 getDistanceMode() { return distance_mode; }

// Initialize sensor using settings taken mostly from VL53L1_DataInit() and
// VL53L1_StaticInit().
// If io_2v8 (optional) is true or not given, the sensor is configured for 2V8
// mode.
u8 init(u8 io_2v8)
{
  if(readReg16Bit(VL53L1_IDENTIFICATION__MODEL_ID) != 0xEACC)  return 0;

  writeReg(VL53L1_SOFT_RESET, 0x00);
  delay_ms(100);
  writeReg(VL53L1_SOFT_RESET, 0x01);
	delay_ms(10);

	//检测系统是否就绪
  if( (readReg(VL53L1_FIRMWARE__SYSTEM_STATUS) & 0x01) == 0 ) return 0;

  init_reg(io_2v8);

  return 1;
}

void init_reg(u8 io_2v8)
{
  // sensor uses 1V8 mode for I/O by default; switch to 2V8 mode if necessary
  if (io_2v8)
  {
		writeReg(VL53L1_PAD_I2C_HV__EXTSUP_CONFIG,
    readReg(VL53L1_PAD_I2C_HV__EXTSUP_CONFIG) | 0x01);
  }

  // store oscillator info for later use
  fast_osc_frequency = readReg16Bit(VL53L1_OSC_MEASURED__FAST_OSC__FREQUENCY);
  osc_calibrate_val = readReg16Bit(VL53L1_RESULT__OSC_CALIBRATE_VAL);

  // VL53L1_DataInit() end

  // VL53L1_StaticInit() begin

  // Note that the API does not actually apply the configuration settings below
  // when VL53L1_StaticInit() is called: it keeps a copy of the sensor's
  // register contents in memory and doesn't actually write them until a
  // measurement is started. Writing the configuration here means we don't have
  // to keep it all in memory and avoids a lot of redundant writes later.

  // the API sets the preset mode to LOWPOWER_AUTONOMOUS here:
  // VL53L1_set_preset_mode() begin

  // VL53L1_preset_mode_standard_ranging() begin

  // values labeled "tuning parm default" are from vl53l1_tuning_parm_defaults.h
  // (API uses these in VL53L1_init_tuning_parm_storage_struct())

  // static config
  // API resets PAD_I2C_HV__EXTSUP_CONFIG here, but maybe we don't want to do
  // that? (seems like it would disable 2V8 mode)
  writeReg16Bit(VL53L1_DSS_CONFIG__TARGET_TOTAL_RATE_MCPS, TargetRate); // should already be this value after reset
  writeReg(VL53L1_GPIO__TIO_HV_STATUS, 0x02);
  writeReg(VL53L1_SIGMA_ESTIMATOR__EFFECTIVE_PULSE_WIDTH_NS, 8); // tuning parm default
  writeReg(VL53L1_SIGMA_ESTIMATOR__EFFECTIVE_AMBIENT_WIDTH_NS, 16); // tuning parm default
  writeReg(VL53L1_ALGO__CROSSTALK_COMPENSATION_VALID_HEIGHT_MM, 0x01);
  writeReg(VL53L1_ALGO__RANGE_IGNORE_VALID_HEIGHT_MM, 0xFF);
  writeReg(VL53L1_ALGO__RANGE_MIN_CLIP, 0); // tuning parm default
  writeReg(VL53L1_ALGO__CONSISTENCY_CHECK__TOLERANCE, 2); // tuning parm default

  // general config
  writeReg16Bit(VL53L1_SYSTEM__THRESH_RATE_HIGH, 0x0000);
  writeReg16Bit(VL53L1_SYSTEM__THRESH_RATE_LOW, 0x0000);
  writeReg(VL53L1_DSS_CONFIG__APERTURE_ATTENUATION, 0x38);

  // timing config
  // most of these settings will be determined later by distance and timing
  // budget configuration
  writeReg16Bit(VL53L1_RANGE_CONFIG__SIGMA_THRESH, 360); // tuning parm default
  writeReg16Bit(VL53L1_RANGE_CONFIG__MIN_COUNT_RATE_RTN_LIMIT_MCPS, 192); // tuning parm default

  // dynamic config

  writeReg(VL53L1_SYSTEM__GROUPED_PARAMETER_HOLD_0, 0x01);
  writeReg(VL53L1_SYSTEM__GROUPED_PARAMETER_HOLD_1, 0x01);
  writeReg(VL53L1_SD_CONFIG__QUANTIFIER, 2); // tuning parm default

  // VL53L1_preset_mode_standard_ranging() end

  // from VL53L1_preset_mode_timed_ranging_*
  // GPH is 0 after reset, but writing GPH0 and GPH1 above seem to set GPH to 1,
  // and things don't seem to work if we don't set GPH back to 0 (which the API
  // does here).
  writeReg(VL53L1_SYSTEM__GROUPED_PARAMETER_HOLD, 0x00);
  writeReg(VL53L1_SYSTEM__SEED_CONFIG, 1); // tuning parm default

  // from VL53L1_config_low_power_auto_mode
  writeReg(VL53L1_SYSTEM__SEQUENCE_CONFIG, 0x8B); // VHV, PHASECAL, DSS1, RANGE
  writeReg16Bit(VL53L1_DSS_CONFIG__MANUAL_EFFECTIVE_SPADS_SELECT, 200 << 8);
  writeReg(VL53L1_DSS_CONFIG__ROI_MODE_CONTROL, 2); // REQUESTED_EFFFECTIVE_SPADS

  // VL53L1_set_preset_mode() end

  // default to long range, 50 ms timing budget
  // note that this is different than what the API defaults to
  setDistanceMode(Long);
  setMeasurementTimingBudget(50000);

  // VL53L1_StaticInit() end

  // the API triggers this change in VL53L1_init_and_start_range() once a
  // measurement is started; assumes MM1 and MM2 are disabled
  writeReg16Bit(VL53L1_ALGO__PART_TO_PART_RANGE_OFFSET_MM,
  readReg16Bit(VL53L1_MM_CONFIG__OUTER_OFFSET_MM) * 4);
}

uint8_t _I2CBuffer[20];

// Write an 8-bit register
void writeReg(uint16_t reg, uint8_t value)
{
	vl53l1Write(address,reg,1,&value);
}

// Write a 16-bit register
void writeReg16Bit(uint16_t reg, uint16_t value)
{
	_I2CBuffer[0] = value >> 8;
	_I2CBuffer[1] = value & 0x00FF;
	
	vl53l1Write(address,reg,2,_I2CBuffer);
}

// Write a 32-bit register
void writeReg32Bit(uint16_t reg, uint32_t value)
{
	_I2CBuffer[0] = (value >> 24) & 0xFF;
	_I2CBuffer[1] = (value >> 16) & 0xFF;
	_I2CBuffer[2] = (value >> 8)  & 0xFF;
	_I2CBuffer[3] = (value >> 0 ) & 0xFF;
	
	vl53l1Write(address,reg,4,_I2CBuffer);
}

// Read an 8-bit register
uint8_t readReg(uint16_t reg)
{
  uint8_t value;

	vl53l1Read(address,reg,1,&value);

  return value;
}

// Read a 16-bit register
uint16_t readReg16Bit(uint16_t reg)
{
  uint16_t value;

	vl53l1Read(address,reg,2,_I2CBuffer);
	value = ((uint16_t)_I2CBuffer[0]<<8) + (uint16_t)_I2CBuffer[1];

  return value;
}

// Read a 32-bit register
uint32_t readReg32Bit(uint16_t reg)
{
  uint32_t value;

	vl53l1Read(address,reg,4,_I2CBuffer);
	value = ((uint32_t)_I2CBuffer[0]<<24) + ((uint32_t)_I2CBuffer[1]<<16) +
					((uint32_t)_I2CBuffer[2]<<8)  + (uint32_t)_I2CBuffer[3];

  return value;
}

// set distance mode to Short, Medium, or Long
// based on VL53L1_SetDistanceMode()
u8 setDistanceMode(u8 mode)
{
  // save existing timing budget
  uint32_t budget_us = getMeasurementTimingBudget();

  switch (mode)
  {
    case Short:
      // from VL53L1_preset_mode_standard_ranging_short_range()

      // timing config
      writeReg(VL53L1_RANGE_CONFIG__VCSEL_PERIOD_A, 0x07);
      writeReg(VL53L1_RANGE_CONFIG__VCSEL_PERIOD_B, 0x05);
      writeReg(VL53L1_RANGE_CONFIG__VALID_PHASE_HIGH, 0x38);

      // dynamic config
      writeReg(VL53L1_SD_CONFIG__WOI_SD0, 0x07);
      writeReg(VL53L1_SD_CONFIG__WOI_SD1, 0x05);
      writeReg(VL53L1_SD_CONFIG__INITIAL_PHASE_SD0, 6); // tuning parm default
      writeReg(VL53L1_SD_CONFIG__INITIAL_PHASE_SD1, 6); // tuning parm default

      break;

    case Medium:
      // from VL53L1_preset_mode_standard_ranging()

      // timing config
      writeReg(VL53L1_RANGE_CONFIG__VCSEL_PERIOD_A, 0x0B);
      writeReg(VL53L1_RANGE_CONFIG__VCSEL_PERIOD_B, 0x09);
      writeReg(VL53L1_RANGE_CONFIG__VALID_PHASE_HIGH, 0x78);

      // dynamic config
      writeReg(VL53L1_SD_CONFIG__WOI_SD0, 0x0B);
      writeReg(VL53L1_SD_CONFIG__WOI_SD1, 0x09);
      writeReg(VL53L1_SD_CONFIG__INITIAL_PHASE_SD0, 10); // tuning parm default
      writeReg(VL53L1_SD_CONFIG__INITIAL_PHASE_SD1, 10); // tuning parm default

      break;

    case Long: // long
      // from VL53L1_preset_mode_standard_ranging_long_range()

      // timing config
      writeReg(VL53L1_RANGE_CONFIG__VCSEL_PERIOD_A, 0x0F);
      writeReg(VL53L1_RANGE_CONFIG__VCSEL_PERIOD_B, 0x0D);
      writeReg(VL53L1_RANGE_CONFIG__VALID_PHASE_HIGH, 0xB8);

      // dynamic config
      writeReg(VL53L1_SD_CONFIG__WOI_SD0, 0x0F);
      writeReg(VL53L1_SD_CONFIG__WOI_SD1, 0x0D);
      writeReg(VL53L1_SD_CONFIG__INITIAL_PHASE_SD0, 14); // tuning parm default
      writeReg(VL53L1_SD_CONFIG__INITIAL_PHASE_SD1, 14); // tuning parm default

      break;

    default:
      // unrecognized mode - do nothing
      return false;
  }

  // reapply timing budget
  setMeasurementTimingBudget(budget_us);

  // save mode so it can be returned by getDistanceMode()
  distance_mode = mode;

  return true;
}

// Set the measurement timing budget in microseconds, which is the time allowed
// for one measurement. A longer timing budget allows for more accurate
// measurements.
// based on VL53L1_SetMeasurementTimingBudgetMicroSeconds()
u8 setMeasurementTimingBudget(uint32_t budget_us)
{
  // assumes PresetMode is LOWPOWER_AUTONOMOUS

  if (budget_us <= TimingGuard) { return 0; }

  uint32_t range_config_timeout_us = budget_us -= TimingGuard;
  if (range_config_timeout_us > 1100000) { return 0; } // FDA_MAX_TIMING_BUDGET_US * 2

  range_config_timeout_us /= 2;

  // VL53L1_calc_timeout_register_values() begin

  uint32_t macro_period_us;

  // "Update Macro Period for Range A VCSEL Period"
  macro_period_us = calcMacroPeriod(readReg(VL53L1_RANGE_CONFIG__VCSEL_PERIOD_A));

  // "Update Phase timeout - uses Timing A"
  // Timeout of 1000 is tuning parm default (TIMED_PHASECAL_CONFIG_TIMEOUT_US_DEFAULT)
  // via VL53L1_get_preset_mode_timing_cfg().
  uint32_t phasecal_timeout_mclks = timeoutMicrosecondsToMclks(1000, macro_period_us);
  if (phasecal_timeout_mclks > 0xFF) { phasecal_timeout_mclks = 0xFF; }
  writeReg(VL53L1_PHASECAL_CONFIG__TIMEOUT_MACROP, phasecal_timeout_mclks);

  // "Update MM Timing A timeout"
  // Timeout of 1 is tuning parm default (LOWPOWERAUTO_MM_CONFIG_TIMEOUT_US_DEFAULT)
  // via VL53L1_get_preset_mode_timing_cfg(). With the API, the register
  // actually ends up with a slightly different value because it gets assigned,
  // retrieved, recalculated with a different macro period, and reassigned,
  // but it probably doesn't matter because it seems like the MM ("mode
  // mitigation"?) sequence steps are disabled in low power auto mode anyway.
  writeReg16Bit(VL53L1_MM_CONFIG__TIMEOUT_MACROP_A, encodeTimeout(
    timeoutMicrosecondsToMclks(1, macro_period_us)));

  // "Update Range Timing A timeout"
  writeReg16Bit(VL53L1_RANGE_CONFIG__TIMEOUT_MACROP_A, encodeTimeout(
    timeoutMicrosecondsToMclks(range_config_timeout_us, macro_period_us)));

  // "Update Macro Period for Range B VCSEL Period"
  macro_period_us = calcMacroPeriod(readReg(VL53L1_RANGE_CONFIG__VCSEL_PERIOD_B));

  // "Update MM Timing B timeout"
  // (See earlier comment about MM Timing A timeout.)
  writeReg16Bit(VL53L1_MM_CONFIG__TIMEOUT_MACROP_B, encodeTimeout(
    timeoutMicrosecondsToMclks(1, macro_period_us)));

  // "Update Range Timing B timeout"
  writeReg16Bit(VL53L1_RANGE_CONFIG__TIMEOUT_MACROP_B, encodeTimeout(
    timeoutMicrosecondsToMclks(range_config_timeout_us, macro_period_us)));

  // VL53L1_calc_timeout_register_values() end

  return 1;
}

// Get the measurement timing budget in microseconds
// based on VL53L1_SetMeasurementTimingBudgetMicroSeconds()
uint32_t getMeasurementTimingBudget()
{
  // assumes PresetMode is LOWPOWER_AUTONOMOUS and these sequence steps are
  // enabled: VHV, PHASECAL, DSS1, RANGE

  // VL53L1_get_timeouts_us() begin

  // "Update Macro Period for Range A VCSEL Period"
  uint32_t macro_period_us = calcMacroPeriod(readReg(VL53L1_RANGE_CONFIG__VCSEL_PERIOD_A));

  // "Get Range Timing A timeout"

  uint32_t range_config_timeout_us = timeoutMclksToMicroseconds(decodeTimeout(
    readReg16Bit(VL53L1_RANGE_CONFIG__TIMEOUT_MACROP_A)), macro_period_us);

  // VL53L1_get_timeouts_us() end

  return  2 * range_config_timeout_us + TimingGuard;
}

// Start continuous ranging measurements, with the given inter-measurement
// period in milliseconds determining how often the sensor takes a measurement.
void startContinuous(uint32_t period_ms)
{
  // from VL53L1_set_inter_measurement_period_ms()
  writeReg32Bit(VL53L1_SYSTEM__INTERMEASUREMENT_PERIOD, period_ms * osc_calibrate_val);

  writeReg(VL53L1_SYSTEM__INTERRUPT_CLEAR, 0x01); // sys_interrupt_clear_range
  writeReg(VL53L1_SYSTEM__MODE_START, 0x40); // mode_range__timed
}

// Stop continuous measurements
// based on VL53L1_stop_range()
void stopContinuous()
{
  writeReg(VL53L1_SYSTEM__MODE_START, 0x80); // mode_range__abort

  // VL53L1_low_power_auto_data_stop_range() begin

  calibrated = 0;

  // "restore vhv configs"
  if (saved_vhv_init != 0)
  {
    writeReg(VL53L1_VHV_CONFIG__INIT, saved_vhv_init);
  }
  if (saved_vhv_timeout != 0)
  {
     writeReg(VL53L1_VHV_CONFIG__TIMEOUT_MACROP_LOOP_BOUND, saved_vhv_timeout);
  }

  // "remove phasecal override"
  writeReg(VL53L1_PHASECAL_CONFIG__OVERRIDE, 0x00);

  // VL53L1_low_power_auto_data_stop_range() end
}

// Returns a range reading in millimeters when continuous mode is active
// (readRangeSingleMillimeters() also calls this function after starting a
// single-shot range measurement)
uint16_t read(u8 blocking)
{
  readResults();

  if (!calibrated)
  {
    setupManualCalibration();
    calibrated = 1;
  }

  updateDSS();

  getRangingData();

  writeReg(VL53L1_SYSTEM__INTERRUPT_CLEAR, 0x01); // sys_interrupt_clear_range

  return ranging_data.range_mm;
}

// Private Methods /////////////////////////////////////////////////////////////

// "Setup ranges after the first one in low power auto mode by turning off
// FW calibration steps and programming static values"
// based on VL53L1_low_power_auto_setup_manual_calibration()
void setupManualCalibration()
{
  // "save original vhv configs"
  saved_vhv_init = readReg(VL53L1_VHV_CONFIG__INIT);
  saved_vhv_timeout = readReg(VL53L1_VHV_CONFIG__TIMEOUT_MACROP_LOOP_BOUND);

  // "disable VHV init"
  writeReg(VL53L1_VHV_CONFIG__INIT, saved_vhv_init & 0x7F);

  // "set loop bound to tuning param"
  writeReg(VL53L1_VHV_CONFIG__TIMEOUT_MACROP_LOOP_BOUND,
    (saved_vhv_timeout & 0x03) + (3 << 2)); // tuning parm default (LOWPOWERAUTO_VHV_LOOP_BOUND_DEFAULT)

  // "override phasecal"
  writeReg(VL53L1_PHASECAL_CONFIG__OVERRIDE, 0x01);
  writeReg(VL53L1_CAL_CONFIG__VCSEL_START, readReg(VL53L1_PHASECAL_RESULT__VCSEL_START));
}

// read measurement results into buffer
void readResults()
{
	vl53l1Read(address,VL53L1_RESULT__RANGE_STATUS,17,_I2CBuffer);
	
  results.range_status = _I2CBuffer[0];

  results.stream_count = _I2CBuffer[2];

  results.dss_actual_effective_spads_sd0  = (uint16_t)_I2CBuffer[3] << 8; // high byte
  results.dss_actual_effective_spads_sd0 |=           _I2CBuffer[4];      // low byte

  results.ambient_count_rate_mcps_sd0  = (uint16_t)_I2CBuffer[7] << 8; // high byte
  results.ambient_count_rate_mcps_sd0 |=           _I2CBuffer[8];      // low byte

  results.final_crosstalk_corrected_range_mm_sd0  = (uint16_t)_I2CBuffer[13] << 8; // high byte
  results.final_crosstalk_corrected_range_mm_sd0 |=           _I2CBuffer[14];      // low byte

  results.peak_signal_count_rate_crosstalk_corrected_mcps_sd0  = (uint16_t)_I2CBuffer[15] << 8; // high byte
  results.peak_signal_count_rate_crosstalk_corrected_mcps_sd0 |=           _I2CBuffer[16];      // low byte
}

// perform Dynamic SPAD Selection calculation/update
// based on VL53L1_low_power_auto_update_DSS()
void updateDSS()
{
  uint16_t spadCount = results.dss_actual_effective_spads_sd0;

  if (spadCount != 0)
  {
    // "Calc total rate per spad"

    uint32_t totalRatePerSpad =
      (uint32_t)results.peak_signal_count_rate_crosstalk_corrected_mcps_sd0 +
      results.ambient_count_rate_mcps_sd0;

    // "clip to 16 bits"
    if (totalRatePerSpad > 0xFFFF) { totalRatePerSpad = 0xFFFF; }

    // "shift up to take advantage of 32 bits"
    totalRatePerSpad <<= 16;

    totalRatePerSpad /= spadCount;

    if (totalRatePerSpad != 0)
    {
      // "get the target rate and shift up by 16"
      uint32_t requiredSpads = ((uint32_t)TargetRate << 16) / totalRatePerSpad;

      // "clip to 16 bit"
      if (requiredSpads > 0xFFFF) { requiredSpads = 0xFFFF; }

      // "override DSS config"
      writeReg16Bit(VL53L1_DSS_CONFIG__MANUAL_EFFECTIVE_SPADS_SELECT, requiredSpads);
      // DSS_CONFIG__ROI_MODE_CONTROL should already be set to REQUESTED_EFFFECTIVE_SPADS

      return;
    }
  }

  // If we reached this point, it means something above would have resulted in a
  // divide by zero.
  // "We want to gracefully set a spad target, not just exit with an error"

   // "set target to mid point"
   writeReg16Bit(VL53L1_DSS_CONFIG__MANUAL_EFFECTIVE_SPADS_SELECT, 0x8000);
}

// get range, status, rates from results buffer
// based on VL53L1_GetRangingMeasurementData()
void getRangingData()
{
  // VL53L1_copy_sys_and_core_results_to_range_results() begin

  uint16_t range = results.final_crosstalk_corrected_range_mm_sd0;

  // "apply correction gain"
  // gain factor of 2011 is tuning parm default (VL53L1_TUNINGPARM_LITE_RANGING_GAIN_FACTOR_DEFAULT)
  // Basically, this appears to scale the result by 2011/2048, or about 98%
  // (with the 1024 added for proper rounding).
  ranging_data.range_mm = ((uint32_t)range * 2011 + 0x0400) / 0x0800;

  // VL53L1_copy_sys_and_core_results_to_range_results() end

  // set range_status in ranging_data based on value of RESULT__RANGE_STATUS register
  // mostly based on ConvertStatusLite()
  switch(results.range_status)
  {
    case 17: // MULTCLIPFAIL
    case 2: // VCSELWATCHDOGTESTFAILURE
    case 1: // VCSELCONTINUITYTESTFAILURE
    case 3: // NOVHVVALUEFOUND
      // from SetSimpleData()
      ranging_data.range_status = VL53L1_RANGESTATUS_HARDWARE_FAIL;
      break;

    case 13: // USERROICLIP
     // from SetSimpleData()
      ranging_data.range_status = VL53L1_RANGESTATUS_MIN_RANGE_FAIL;
      break;

    case 18: // GPHSTREAMCOUNT0READY
      ranging_data.range_status = VL53L1_RANGESTATUS_SYNCRONISATION_INT;
      break;

    case 5: // RANGEPHASECHECK
      ranging_data.range_status = VL53L1_RANGESTATUS_OUTOFBOUNDS_FAIL;
      break;

    case 4: // MSRCNOTARGET
      ranging_data.range_status = VL53L1_RANGESTATUS_SIGNAL_FAIL;
      break;

    case 6: // SIGMATHRESHOLDCHECK
      ranging_data.range_status = VL53L1_RANGESTATUS_SIGMA_FAIL;
      break;

    case 7: // PHASECONSISTENCY
      ranging_data.range_status = VL53L1_RANGESTATUS_WRAP_TARGET_FAIL;
      break;

    case 12: // RANGEIGNORETHRESHOLD
      ranging_data.range_status = VL53L1_RANGESTATUS_XTALK_SIGNAL_FAIL;
      break;

    case 8: // MINCLIP
      ranging_data.range_status = VL53L1_RANGESTATUS_RANGE_VALID_MIN_RANGE_CLIPPED;
      break;

    case 9: // RANGECOMPLETE
      // from VL53L1_copy_sys_and_core_results_to_range_results()
      if (results.stream_count == 0)
      {
        ranging_data.range_status = VL53L1_RANGESTATUS_RANGE_VALID_NO_WRAP_CHECK_FAIL;
      }
      else
      {
        ranging_data.range_status = VL53L1_RANGESTATUS_RANGE_VALID;
      }
      break;

    default:
      ranging_data.range_status = VL53L1_RANGESTATUS_NONE;
  }

  // from SetSimpleData()
  ranging_data.peak_signal_count_rate_MCPS =
    countRateFixedToFloat(results.peak_signal_count_rate_crosstalk_corrected_mcps_sd0);
  ranging_data.ambient_count_rate_MCPS =
    countRateFixedToFloat(results.ambient_count_rate_mcps_sd0);
}

// Decode sequence step timeout in MCLKs from register value
// based on VL53L1_decode_timeout()
uint32_t decodeTimeout(uint16_t reg_val)
{
  return ((uint32_t)(reg_val & 0xFF) << (reg_val >> 8)) + 1;
}

// Encode sequence step timeout register value from timeout in MCLKs
// based on VL53L1_encode_timeout()
uint16_t encodeTimeout(uint32_t timeout_mclks)
{
  // encoded format: "(LSByte * 2^MSByte) + 1"

  uint32_t ls_byte = 0;
  uint16_t ms_byte = 0;

  if (timeout_mclks > 0)
  {
    ls_byte = timeout_mclks - 1;

    while ((ls_byte & 0xFFFFFF00) > 0)
    {
      ls_byte >>= 1;
      ms_byte++;
    }

    return (ms_byte << 8) | (ls_byte & 0xFF);
  }
  else { return 0; }
}

// Convert sequence step timeout from macro periods to microseconds with given
// macro period in microseconds (12.12 format)
// based on VL53L1_calc_timeout_us()
uint32_t timeoutMclksToMicroseconds(uint32_t timeout_mclks, uint32_t macro_period_us)
{
  return ((uint64_t)timeout_mclks * macro_period_us + 0x800) >> 12;
}

// Convert sequence step timeout from microseconds to macro periods with given
// macro period in microseconds (12.12 format)
// based on VL53L1_calc_timeout_mclks()
uint32_t timeoutMicrosecondsToMclks(uint32_t timeout_us, uint32_t macro_period_us)
{
  return (((uint32_t)timeout_us << 12) + (macro_period_us >> 1)) / macro_period_us;
}

// Calculate macro period in microseconds (12.12 format) with given VCSEL period
// assumes fast_osc_frequency has been read and stored
// based on VL53L1_calc_macro_period_us()
uint32_t calcMacroPeriod(uint8_t vcsel_period)
{
  // from VL53L1_calc_pll_period_us()
  // fast osc frequency in 4.12 format; PLL period in 0.24 format
  uint32_t pll_period_us = ((uint32_t)0x01 << 30) / fast_osc_frequency;

  // from VL53L1_decode_vcsel_period()
  uint8_t vcsel_period_pclks = (vcsel_period + 1) << 1;

  // VL53L1_MACRO_PERIOD_VCSEL_PERIODS = 2304
  uint32_t macro_period_us = (uint32_t)2304 * pll_period_us;
  macro_period_us >>= 6;
  macro_period_us *= vcsel_period_pclks;
  macro_period_us >>= 6;

  return macro_period_us;
}

float countRateFixedToFloat(uint16_t count_rate_fixed) { return (float)count_rate_fixed / (1 << 7); }
u8 dataReady() { return (readReg(VL53L1_GPIO__TIO_HV_STATUS) & 0x01) == 0; }
uint16_t readRangeContinuousMillimeters(u8 blocking) { return read(blocking); } // alias of read()
