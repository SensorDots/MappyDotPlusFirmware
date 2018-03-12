/*
 * vl53l0x_profiles.h
 *
 * Created: 23/11/2017 12:21:05 AM
 *  Author: Blair Wyatt
 */ 


#ifndef VL53L0X_PROFILES_H_
#define VL53L0X_PROFILES_H_



typedef struct {
	uint8_t range_ignore_threshold;
	uint32_t range_ignore_threshold_value;
	uint32_t signal_rate_final_range;
	uint32_t sigma_final_range;
	uint32_t timing_budget_ms;
	uint8_t vcsel_period_pre_range;
	uint8_t vscel_period_final_range;

} VL53L0X_Measurement_Mode;

/* High Accuracy */
#define HIGH_ACCURACY_RANGE_IGNORE_THRESHOLD 0
#define HIGH_ACCURACY_SIGNAL_RATE_FINAL_RANGE (0.25*65536)
#define HIGH_ACCURACY_SIGMA_FINAL_RANGE (18*65536)
#define HIGH_ACCURACY_TIMING_BUDGET 200000 //5Hz
#define HIGH_ACCURACY_VCSEL_PERIOD_PRE_RANGE 14
#define HIGH_ACCURACY_VCSEL_PERIOD_FINAL_RANGE 10

/* Long Range */
#define LONG_RANGE_RANGE_IGNORE_THRESHOLD 0
#define LONG_RANGE_SIGNAL_RATE_FINAL_RANGE (0.1*65536)
#define LONG_RANGE_SIGMA_FINAL_RANGE (60*65536)
#define LONG_RANGE_TIMING_BUDGET 33000 //30Hz
#define LONG_RANGE_VCSEL_PERIOD_PRE_RANGE 18
#define LONG_RANGE_VCSEL_PERIOD_FINAL_RANGE 14

/* High Speed */
#define HIGH_SPEED_RANGE_IGNORE_THRESHOLD 0
#define HIGH_SPEED_SIGNAL_RATE_FINAL_RANGE (0.25*65536)
#define HIGH_SPEED_SIGMA_FINAL_RANGE (32*65536)
#define HIGH_SPEED_TIMING_BUDGET 20000+1600 //50Hz (give/fudge a little extra time/accuracy to get 50Hz as it runs a bit fast; around 55Hz)
#define HIGH_SPEED_VCSEL_PERIOD_PRE_RANGE 14
#define HIGH_SPEED_VCSEL_PERIOD_FINAL_RANGE 10

/* Default */
#define DEFAULT_RANGE_IGNORE_THRESHOLD 1
#define DEFAULT_CHECKENABLE_RANGE_IGNORE_THRESHOLD_VALUE (1.5*0.023*65536)
#define DEFAULT_SIGNAL_RATE_FINAL_RANGE (0.25*65536)
#define DEFAULT_SIGMA_FINAL_RANGE (18*65536)
#define DEFAULT_TIMING_BUDGET 33000 //30Hz
#define DEFAULT_VCSEL_PERIOD_PRE_RANGE 14
#define DEFAULT_VCSEL_PERIOD_FINAL_RANGE 10

#endif /* VL53L0X_PROFILES_H_ */