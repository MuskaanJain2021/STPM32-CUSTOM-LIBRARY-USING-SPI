#ifndef STPM32_CALIB_H
#define STPM32_CALIB_H


#include "STPM3X_Def.h"
#include <stdint.h>
#include <stdbool.h>
#include "esp_err.h"



/* ---------- ADC constants ---------- */
#define STPM32_ADC_BITS        24 // 24-bit ADC resolution
#define STPM32_ADC_RMS_FULL   8388608.0f   // 2^23corrected for signed 24-bit
#define STPM32_ADC_VREF        1.18f // internal reference voltage (V)
#define STPM32_VFS 0.3f// Full scale voltage at ADC input (V), corresponds to 0x7FFFFF reading. This is determined by the voltage divider formed by Rtop and Rbot and the ADC reference voltage. VFS = VREF * (Rtop + Rbot) / Rbot
/* ---------- Hardware values ---------- */
#define STPM32_RSHUNT          0.0003f // 0.3milli ohm
#define STPM32_RTOP            1000000.0f// 1 mega ohm
#define STPM32_RBOT            270000.0f// 270 kilo ohm

/* ---------- Scaling constants ---------- */
// #define STPM32_KV  \
//     ((STPM32_ADC_VREF / STPM32_ADC_FULL) * \
//     ((STPM32_RTOP + STPM32_RBOT) / STPM32_RBOT))

// #define STPM32_KI(gain) \
//     ((STPM32_ADC_VREF / STPM32_ADC_FULL) / \
//     (STPM32_RSHUNT * (gain)))

// #define STPM32_KP(gain) \
//     (STPM32_KV * STPM32_KI(gain))

/* ---------- Calibration structure ---------- */
typedef struct {
    float kv;// Voltage scaling factor (V/LSB)
    float ki;// Current scaling factor (A/LSB)
    float kp;// Power scaling factor (W/LSB)
    int32_t power_offset_raw;// Raw power offset in LSB units
} stpm32_calib_t;
extern stpm32_calib_t stpm32_calib_ph1;

/*========================Event Based Apis=====================================*/
typedef struct {
    int64_t acc;// Accumulator for raw power values
    uint16_t count;// Number of samples accumulated
    uint16_t target;// Target sample count for averaging
    bool done;// Flag indicating completion of offset accumulation
} stpm32_offset_state_t;

extern stpm32_offset_state_t power_offset_state;

/**
 * @brief  Initializes power offset accumulation state.
 *
 * @param[in,out] p_state  Pointer to offset state structure.
 * @param[in]     samples  Number of samples required to complete averaging.
 *
 * @return None.
 */

void stpm32_offset_start(stpm32_offset_state_t *s, uint16_t samples);

/**
 * @brief  Accumulates raw power samples for offset computation.
 *
 * Adds signed power register value to accumulator until target
 * sample count is reached.
 *
 * @param[in,out] s    Pointer to offset state structure.
 * @param[in]     power_reg  Raw 24-bit DSP power register value.
 *
 * @return None.
 */
void stpm32_offset_process(stpm32_offset_state_t *s,uint32_t power_reg);

/**
 * @brief  Decodes DFE gain setting.
 * 
 * @param[in] gain_bits  DFE gain configuration bits.
 *
 * @return Gain multiplier as floating-point value.
 */
float stpm32_dfe_gain(uint8_t gain_bits);

/**
 * @brief  Initializes calibration scaling factors.
 * Computes voltage, current, and power scaling constants
 * based on hardware configuration and DFE gain.
 * @param[out]   c   Pointer to calibration structure.
 * @param[in]  gain_bits   DFE gain configuration bits.
 *
 * @return None.
 */
void stpm32_calib_init(stpm32_calib_t *c,uint8_t gain_bits);


/**
 * @brief  Returns computed power offset.
 *
 * @param[in] s  Pointer to completed offset state structure.
 *
 * @return Averaged raw power offset value(LSB units).
 */
int32_t stpm32_offset_get(const stpm32_offset_state_t *s);


/**
 * @brief  Converts raw power register value to watts.
 *
 * @param[in] power_reg  Raw DSP power register value.
 * @param[in] cal      Pointer to calibration data.
 *
 * @return Instantaneous power in watts.
 */
float stpm32_read_power_w(uint32_t power_reg,const stpm32_calib_t *cal);

/**
 * @brief  Measures raw power offset using blocking averaging.
 *
 * Intended for test or calibration use only.
 *
 * @param[in] power_reg  Raw 24-bit DSP power register value.
 * @param[in] samples    Number of samples to average.
 *
 * @return Averaged signed raw power offset (LSB units).
 */
int32_t stpm32_measure_power_offset(uint32_t power_reg,uint16_t samples);
/**
 * @brief  Decodes RMS register value to signed 24-bit integer.
 *
 * @param[in] reg  Raw 24-bit RMS register value.
 *
 * @return usigned RMS value (LSB units).
 */

uint32_t stpm32_rms_decode(uint32_t reg);


/**
 * @brief  Converts raw VRMS register value to volts.
 *
 * @param[in] vrms_reg  Raw 24-bit VRMS register value.
 * @param[in] cal     Pointer to calibration data.
 *
 * @return RMS voltage in volts.
 */
float stpm32_read_vrms(uint32_t vrms_reg,const stpm32_calib_t *cal);

/**
 * @brief  Converts raw IRMS register value to amperes.
 *
 * @param[in] irms_reg  Raw 24-bit IRMS register value.
 * @param[in] cal     Pointer to calibration data.
 *
 * @return RMS current in amperes.
 */
float stpm32_read_irms(uint32_t irms_reg,const stpm32_calib_t *cal);

/* ================= Global power update ================= */
//void stpm32_update_power(void);


#endif /* STPM32_CALIB_H */
