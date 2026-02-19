#include "stpm32_calib.h"
#include "STPM3X_Def.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

//float latest_power_w = 0.0f;
stpm32_offset_state_t power_offset_state;

/* ================= Phase-2 calibration ================= */
stpm32_calib_t stpm32_calib_ph1 = {
    .kv = 1.0f,
    .ki = 1.0f,
    .kp = 1.0f,
    .power_offset_raw = 0
};

// int32_t stpm32_rms_decode(uint32_t reg)
// {
//     reg &= 0x00FFFFFF;
//     if (reg & 0x00800000)
//         reg |= 0xFF000000;   // sign extend
//     return (int32_t)reg;
// }//;large rms will be negative when interpreted as signed, but we want the magnitude for scaling, so we return unsigned value and let the caller handle the sign if needed.

uint32_t stpm32_rms_decode(uint32_t reg)
{
    return reg & 0x00FFFFFF;
}



/* ================= Global power update ================= */
// void stpm32_update_power(void)
// {
//     uint32_t reg;

//     if (stpm32_read_reg(PH2_REG5_ADDR, &reg, STPM32_CRC_NONE) != ESP_OK) {
//         return;
//     }

//     int32_t raw = stpm32_power_decode(reg);
//     raw -= stpm32_calib_ph2.power_offset_raw;

//     latest_power_w = raw * stpm32_calib_ph2.kp;
// }

float stpm32_read_power_w(uint32_t power_reg,const stpm32_calib_t *cal)
{
    uint32_t reg;
    if (stpm32_read_reg(power_reg, &reg, STPM32_CRC_NONE) != ESP_OK)
        return 0.0f;

    int32_t raw = stpm32_power_decode(reg);
    
    raw -= cal->power_offset_raw;

    return raw * cal->kp;
}

/* ================= Offset measurement ================= */
/* Must be called with NO AC input */
void stpm32_offset_start(stpm32_offset_state_t *s, uint16_t samples)
{
    s->acc = 0;
    s->count = 0;
    s->target = samples;
    s->done = false;
}


void stpm32_offset_process(stpm32_offset_state_t *s,
                           uint32_t power_reg)
{
    if (s->done)
        return;

    uint32_t reg;
    stpm32_read_reg(power_reg, &reg, STPM32_CRC_NONE);

    s->acc += stpm32_power_decode(reg);
    s->count++;

    if (s->count >= s->target) {
        s->done = true;
    }
}

/*=========================Get computed offset (call only when done == true)====================================*/
int32_t stpm32_offset_get(const stpm32_offset_state_t *s)
{
    if (!s || !s->done || s->target == 0)
        return 0;

    return (int32_t)(s->acc / s->target);
}

//corrected gain decoding to match DFE_CR1_I_GAIN encoding in STPM3X_Def.h
//This is independent  of offset measurement and can be used to decode gain for any purpose. 
//Even if offset measurement is not used, this can be used to decode gain for correct scaling of voltage and current readings.
float stpm32_dfe_gain(uint8_t gain_bits)
{
    switch (gain_bits & 0x03)
    {
        case 0: return 2.0f;
        case 1: return 4.0f;
        case 2: return 8.0f;
        case 3: return 16.0f;
        default: return 2.0f;
    }
}

void stpm32_calib_init(stpm32_calib_t *c,
                                     uint8_t gain_bits)
{
    gain_bits &= 0x03; 
    float gain = stpm32_dfe_gain(gain_bits); 

    c->kv = (STPM32_ADC_VREF / STPM32_ADC_RMS_FULL) *
            ((STPM32_RTOP + STPM32_RBOT) / STPM32_RBOT);

    c->ki = (STPM32_ADC_VREF / STPM32_ADC_RMS_FULL) /
            (STPM32_RSHUNT * gain);

    c->kp = c->kv * c->ki;
}

int32_t stpm32_measure_power_offset(uint32_t power_reg,
                                    uint16_t samples)
{
    if (samples == 0)
        return 0;

    int64_t acc = 0;
    uint16_t valid = 0;

    for (uint16_t i = 0; i < samples; i++) {

        uint32_t reg;
        if (stpm32_read_reg(power_reg, &reg, STPM32_CRC_NONE) == ESP_OK)
        {
            int32_t raw = stpm32_power_decode(reg);
            acc += raw;
            valid++;
        }

        vTaskDelay(pdMS_TO_TICKS(5));  // allow DSP update
    }

    if (valid == 0)
        return 0;

    return (int32_t)(acc / valid);
}


float stpm32_read_vrms(uint32_t vrms_reg,
                       const stpm32_calib_t *cal)
{
    uint32_t reg;
    if (stpm32_read_reg(vrms_reg, &reg, STPM32_CRC_NONE) != ESP_OK)
        return 0.0f;

    return stpm32_rms_decode(reg) * cal->kv;
}

float stpm32_read_irms(uint32_t irms_reg,
                                     const stpm32_calib_t *cal)
{
    uint32_t reg;
    if (stpm32_read_reg(irms_reg, &reg, STPM32_CRC_NONE) != ESP_OK)
        return 0.0f;

    return stpm32_rms_decode(reg) * cal->ki;
}
