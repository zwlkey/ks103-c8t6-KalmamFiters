#ifndef VL53L1X_H
#define VL53L1X_H

#include "sys.h"
#include "VL53L1_register_map.h"

/** @defgroup VL53L1_define_RangeStatus_group Defines the Range Status
*	@{
*/
#define	 VL53L1_RANGESTATUS_RANGE_VALID				0
/*!<The Range is valid. */
#define	 VL53L1_RANGESTATUS_SIGMA_FAIL				1
/*!<Sigma Fail. */
#define	 VL53L1_RANGESTATUS_SIGNAL_FAIL				2
/*!<Signal fail. */
#define	 VL53L1_RANGESTATUS_RANGE_VALID_MIN_RANGE_CLIPPED	3
/*!<Target is below minimum detection threshold. */
#define	 VL53L1_RANGESTATUS_OUTOFBOUNDS_FAIL			4
/*!<Phase out of valid limits -  different to a wrap exit. */
#define	 VL53L1_RANGESTATUS_HARDWARE_FAIL			5
/*!<Hardware fail. */
#define	 VL53L1_RANGESTATUS_RANGE_VALID_NO_WRAP_CHECK_FAIL	6
/*!<The Range is valid but the wraparound check has not been done. */
#define	VL53L1_RANGESTATUS_WRAP_TARGET_FAIL			7
/*!<Wrapped target - no matching phase in other VCSEL period timing. */
#define	VL53L1_RANGESTATUS_PROCESSING_FAIL			8
/*!<Internal algo underflow or overflow in lite ranging. */
#define	VL53L1_RANGESTATUS_XTALK_SIGNAL_FAIL			9
/*!<Specific to lite ranging. */
#define	VL53L1_RANGESTATUS_SYNCRONISATION_INT			10
/*!<1st interrupt when starting ranging in back to back mode. Ignore data. */
#define	VL53L1_RANGESTATUS_RANGE_VALID_MERGED_PULSE		11
/*!<All Range ok but object is result of multiple pulses merging together.
* Used by RQL for merged pulse detection
*/
#define	VL53L1_RANGESTATUS_TARGET_PRESENT_LACK_OF_SIGNAL	12
/*!<Used  by RQL  as different to phase fail. */
#define	VL53L1_RANGESTATUS_MIN_RANGE_FAIL			13
/*!<User ROI input is not valid e.g. beyond SPAD Array.*/
#define	VL53L1_RANGESTATUS_RANGE_INVALID			14
/*!<lld returned valid range but negative value ! */
#define	 VL53L1_RANGESTATUS_NONE				255
/*!<No Update. */

/** @} VL53L1_define_RangeStatus_group */

  enum DistanceMode { Short, Medium, Long, Unknown };

	typedef struct 
	{
		uint16_t range_mm;
		uint8_t range_status; //RangeStatus range_status
		float peak_signal_count_rate_MCPS;
		float ambient_count_rate_MCPS;
	}RangingData;

	extern RangingData ranging_data;

	void setAddress(uint8_t new_addr);
	uint8_t getAddress(void);

	u8 init(u8 io_2v8);
	void init_reg(u8 io_2v8);

	void writeReg(uint16_t reg, uint8_t value);
	void writeReg16Bit(uint16_t reg, uint16_t value);
	void writeReg32Bit(uint16_t reg, uint32_t value);
	uint8_t readReg(uint16_t reg);
	uint16_t readReg16Bit(uint16_t reg);
	uint32_t readReg32Bit(uint16_t reg);

	u8 setDistanceMode(u8 mode);
	u8 getDistanceMode(void);

	u8 setMeasurementTimingBudget(uint32_t budget_us);
	uint32_t getMeasurementTimingBudget(void);

	void startContinuous(uint32_t period_ms);
	void stopContinuous(void);
	uint16_t read(u8 blocking);//(bool blocking = true);
	uint16_t readRangeContinuousMillimeters(u8 blocking);

	// check if sensor has new reading available
	// assumes interrupt is active low (GPIO_HV_MUX__CTRL bit 4 is 1)
	u8 dataReady(void);

	static const char * rangeStatusToString(uint8_t status);

	// for storing values read from RESULT__RANGE_STATUS (0x0089)
	// through RESULT__PEAK_SIGNAL_COUNT_RATE_CROSSTALK_CORRECTED_MCPS_SD0_LOW
	// (0x0099)
	typedef struct
	{
		uint8_t range_status;
	// uint8_t report_status: not used
		uint8_t stream_count;
		uint16_t dss_actual_effective_spads_sd0;
 // uint16_t peak_signal_count_rate_mcps_sd0: not used
		uint16_t ambient_count_rate_mcps_sd0;
 // uint16_t sigma_sd0: not used
 // uint16_t phase_sd0: not used
		uint16_t final_crosstalk_corrected_range_mm_sd0;
		uint16_t peak_signal_count_rate_crosstalk_corrected_mcps_sd0;
	}ResultBuffer;

	// making this static would save RAM for multiple instances as long as there
	// aren't multiple sensors being read at the same time (e.g. on separate
	// I2C buses)

	// Record the current time to check an upcoming timeout against
	//void startTimeout() { timeout_start_ms = millis(); }

	// Check if timeout is enabled (set to nonzero value) and has expired
	//bool checkTimeoutExpired() {return (io_timeout > 0) && ((uint16_t)(millis() - timeout_start_ms) > io_timeout); }

	void setupManualCalibration(void);
	void readResults(void);
	void updateDSS(void);
	void getRangingData(void);

	static uint32_t decodeTimeout(uint16_t reg_val);
	static uint16_t encodeTimeout(uint32_t timeout_mclks);
	static uint32_t timeoutMclksToMicroseconds(uint32_t timeout_mclks, uint32_t macro_period_us);
	static uint32_t timeoutMicrosecondsToMclks(uint32_t timeout_us, uint32_t macro_period_us);
	uint32_t calcMacroPeriod(uint8_t vcsel_period);

	// Convert count rate from fixed point 9.7 format to float
	float countRateFixedToFloat(uint16_t count_rate_fixed);

#endif
