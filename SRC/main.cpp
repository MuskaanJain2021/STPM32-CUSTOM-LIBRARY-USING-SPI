#include <Arduino.h>
#include "spi_stpm32.h"
#include "STPM3X_REF.h"
#include "STPM3X_Def.h"
#include "esp32-hal.h"
#include "stpm32_calib.h"
#include "stpm32_logger.h"

#define STPM32_PERIOD_MS 1000
static uint32_t last_sample_ms = 0;

static inline uint32_t dsp_cr1_pack(const dsp_cr1_lsw_cfg_t *lsw, const dsp_cr1_msw_cfg_t *msw)
{
    return ((uint32_t)dsp_cr1_msw_pack(msw) << 16) | dsp_cr1_lsw_pack(lsw);
}

static inline void dsp_cr1_set_envref1(uint32_t *reg, uint8_t value)
{
    if (value)
        *reg |= DSP_CR1_ENVREF1_Msk; // set bit
    else
        *reg &= ~DSP_CR1_ENVREF1_Msk; // clear bit
}

static inline uint16_t stpm32_vrms_ph1(uint32_t reg)
{
    return (uint16_t)((reg & DSP_RMS_V1_Msk) >> DSP_RMS_V1_Pos);
}

static inline uint32_t stpm32_irms_ph1(uint32_t reg)
{
    return (uint32_t)((reg & DSP_RMS_C1_Msk) >> DSP_RMS_C1_Pos);
}

static inline uint16_t stpm32_vrms_ph2(uint32_t reg)
{
    return (uint16_t)((reg & DSP_RMS_V2_Msk) >> DSP_RMS_V2_Pos);
}

static inline uint32_t stpm32_irms_ph2(uint32_t reg)
{
    return (uint32_t)((reg & DSP_RMS_C2_Msk) >> DSP_RMS_C2_Pos);
}

void setup()
{
    Serial.begin(115200);
    delay(1000);
    stpm32_logger_set_level(STPM32_LOG_ERROR);

    STPM32_LOGI("ESP32 <=> STPM32 POWER METER INIT");
    STPM32_LOGI("Bring-up + Zero-Input Calibration");

    /* ---------- GPIO SAFE STATES ---------- */
    stpm32_gpio_init();
    gpio_set_level(STPM32_SYNC_GPIO, 0);

    /* ---------- Init SPI ---------- */
    ESP_ERROR_CHECK(stpm32_spi_init());
    delay(100);

    /* ---------- Power cycle STPM ---------- */
    stpm32_power_cycle();
    delay(40);

    /* ---------- Global reset ---------- */
    stpm32_sync_pulse(3);
    delay(50);

    // safe: ensure CS HIGH
    CS_HIGH();

    // Optional: clear SR1 flags
    uint32_t sr1;
    if (stpm32_read_sr1(&sr1) == ESP_OK && sr1)
        stpm32_clear_sr1();
    stpm32_sync_pulse(1);
    esp_rom_delay_us(10);
    /* ---------- Read DSP_CR1 default ---------- */
    uint32_t dsp_cr1_default;
    if (stpm32_read_reg(DSP_CR1_ADDR, &dsp_cr1_default, STPM32_CRC_NONE) == ESP_OK)
        STPM32_LOGI("DSP_CR1 default = 0x%08lX", dsp_cr1_default);
    else
        STPM32_LOGE("STPM32 NOT RESPONSIVE - Failed to read DSP_CR1");

    /* ---------- SPI sanity test ---------- */
    uint8_t tx[4] = {0xFF, 0x00, 0xAA, 0x55};
    uint8_t rx[4] = {0};
    CS_LOW();
    delayMicroseconds(2);
    stpm32_spi_rw(tx, rx, sizeof(tx));
    CS_HIGH();
    STPM32_LOGD("SPI RAW TEST TX: %02X %02X %02X %02X", tx[0], tx[1], tx[2], tx[3]);
    STPM32_LOGD("SPI RAW TEST RX: %02X %02X %02X %02X", rx[0], rx[1], rx[2], rx[3]);

    /* ---------- Clear SR1 ---------- */
    // uint32_t sr1;
    if (stpm32_read_sr1(&sr1) == ESP_OK && sr1)
        stpm32_clear_sr1();

    stpm32_sync_pulse(1);
    esp_rom_delay_us(10);
    /* ================= DFE CONFIG ================= */
    dfe_cr1_cfg_t dfe1 = {.gain1 = 2, .enc1 = 1, .env1 = 1};
    dfe_cr2_cfg_t dfe2 = {.gain2 = 2, .enc2 = 1, .env2 = 1};
    ESP_ERROR_CHECK(stpm32_write_reg(DFE_CR1_ADDR, dfe_cr1_pack(&dfe1), STPM32_CRC_NONE));
    ESP_ERROR_CHECK(stpm32_write_reg(DFE_CR2_ADDR, dfe_cr2_pack(&dfe2), STPM32_CRC_NONE));
    stpm32_sync_pulse(1);
    delayMicroseconds(20);

    STPM32_LOGI("Waiting for DFE ready...");
    uint32_t dfe_status = 0;
    uint32_t dfe_start = millis();
    while (millis() - dfe_start < 500)
    {
        stpm32_sync_pulse(1);
        esp_rom_delay_us(10);
        stpm32_read_reg(STPM32_DFE_STATUS, &dfe_status, STPM32_CRC_NONE);
        if ((dfe_status & DFE_STATUS_V_OK_Msk) && (dfe_status & DFE_STATUS_I_OK_Msk))
        {
            STPM32_LOGI("DFE READY");
            break;
        }
        delay(10);
    }
    if (!((dfe_status & DFE_STATUS_V_OK_Msk) && (dfe_status & DFE_STATUS_I_OK_Msk)))
    {
        STPM32_LOGE("DFE not ready (timeout)");
        return;
    }

    /* ================= DSP CONFIG ================= */
    dsp_cr1_lsw_cfg_t lsw = {.tc1 = 4, .envref1 = 1, .clears = 0, .clrss_to1 = 0};
    dsp_cr1_msw_cfg_t msw = {.lcs1 = 0, .lps1 = 0, .lpw1 = 0, .roc1 = 0, .bhpfc1 = 1, .bhpfv1 = 1, .apm1 = 1};
    ESP_ERROR_CHECK(stpm32_write_reg(DSP_CR_LSB_ADDR, dsp_cr1_lsw_pack(&lsw), STPM32_CRC_NONE));
    ESP_ERROR_CHECK(stpm32_write_reg(DSP_CR1_MSW_ADDR, dsp_cr1_msw_pack(&msw), STPM32_CRC_NONE));
    delayMicroseconds(100);
    stpm32_sync_pulse(1);

    dsp_cr3_msw_cfg_t cr3 = {.en_cum = 1};
    ESP_ERROR_CHECK(stpm32_write_reg(DSP_CR3_MSW_ADDR, dsp_cr3_msw_pack(&cr3), STPM32_CRC_NONE));
    stpm32_sync_pulse(1);

    /* ================= START DSP PIPELINE ================= */
    STPM32_LOGI("Starting DSP pipeline...");
    esp_rom_delay_us(5000);

    uint32_t status;
    uint32_t dsp_start = millis();
    stpm32_state_t stpm_stat = STPM_STATE_OFF;
    while (millis() - dsp_start < 1000)
    {
        stpm32_sync_pulse(1);
        esp_rom_delay_us(10);
        ESP_ERROR_CHECK(stpm32_read_reg(STPM32_DSP_STATUS, &status, STPM32_CRC_NONE));
        if (status & DSP_STATUS_VALID_Msk)
        {
            stpm_stat = STPM_STATE_RUNNING;
            STPM32_LOGI("DSP RUNNING & DATA VALID");
            break;
        }
        delay(25);
    }
    if (!(status & DSP_STATUS_VALID_Msk))
    {
        stpm_stat = STPM_STATE_ERROR;
        STPM32_LOGW("DSP running but no valid waveform detected");
        return;
    }

    /* ================= OFFSET CALIBRATION ================= */
    stpm32_offset_start(&power_offset_state, 200);
    stpm32_calib_init(&stpm32_calib_ph1, 0);

    /* ================= READ VRMS, IRMS, POWER ================= */
    uint32_t vrms_raw, irms_raw, power_raw;
    if (millis() - last_sample_ms >= 1000)
    { // every 1 second
        last_sample_ms = millis();

        stpm32_sync_pulse(1);
        esp_rom_delay_us(20);

        float vrms = stpm32_read_vrms(PH1_VRMS_ADDR, &stpm32_calib_ph1);
        float irms = stpm32_read_irms(PH1_IRMS_ADDR, &stpm32_calib_ph1);
        float power = stpm32_read_power_w(PH1_REG5_ADDR, &stpm32_calib_ph1);

        STPM32_LOGI("PH1: VRMS=%.3f V, IRMS=%.3f A, PWR=%.3f W", vrms, irms, power);
    }
}

void loop()
{
}

// #include <Arduino.h>
// #include "spi_stpm32.h"
// #include "STPM3X_REF.h"
// #include "STPM3X_Def.h"
// #include "esp32-hal.h"
// #include "stpm32_calib.h"
// #include "stpm32_logger.h"

// #define STPM32_PERIOD_MS 1000
// // stpm32_state_t stpm_stat = STPM_STATE_OFF;
// static uint32_t last_sample_ms = 0;

// /* ===================== RAW SPI TEST ===================== */
// void stpm32_spi_byte_test(void)
// {
//     uint8_t tx[4] = {0xFF, 0x00, 0xAA, 0x55};
//     uint8_t rx[4] = {0};

//     CS_LOW();
//     delayMicroseconds(2);
//     stpm32_spi_rw(tx, rx, sizeof(tx));
//     CS_HIGH();

//     STPM32_LOGD("SPI RAW TEST");
//     STPM32_LOGD("TX: %02X %02X %02X %02X",
//                 tx[0], tx[1], tx[2], tx[3]);

//     STPM32_LOGD("RX: %02X %02X %02X %02X",
//                 rx[0], rx[1], rx[2], rx[3]);
// }

// static inline uint32_t dsp_cr1_pack(const dsp_cr1_lsw_cfg_t *lsw, const dsp_cr1_msw_cfg_t *msw)
// {
//     return ((uint32_t)dsp_cr1_msw_pack(msw) << 16) | dsp_cr1_lsw_pack(lsw);
// }

// static inline void dsp_cr1_set_envref1(uint32_t *reg, uint8_t value)
// {
//     if (value)
//         *reg |= DSP_CR1_ENVREF1_Msk; // set bit
//     else
//         *reg &= ~DSP_CR1_ENVREF1_Msk; // clear bit
// }

// static inline uint16_t stpm32_vrms_ph1(uint32_t reg)
// {
//     return (uint16_t)((reg & DSP_RMS_V1_Msk) >> DSP_RMS_V1_Pos);
// }

// static inline uint32_t stpm32_irms_ph1(uint32_t reg)
// {
//     return (uint32_t)((reg & DSP_RMS_C1_Msk) >> DSP_RMS_C1_Pos);
// }

// static inline uint16_t stpm32_vrms_ph2(uint32_t reg)
// {
//     return (uint16_t)((reg & DSP_RMS_V2_Msk) >> DSP_RMS_V2_Pos);
// }

// static inline uint32_t stpm32_irms_ph2(uint32_t reg)
// {
//     return (uint32_t)((reg & DSP_RMS_C2_Msk) >> DSP_RMS_C2_Pos);
// }

// /* ===================== SETUP ===================== */
// void setup()
// {
//     Serial.begin(115200);
//     delay(1000);

//     STPM32_LOGI("ESP32 <=> STPM32 POWER METER INIT");
//     STPM32_LOGI("Bring-up + Zero-Input Calibration");

//     /* ---------- GPIO SAFE STATES ---------- */
//     stpm32_gpio_init();

//     gpio_set_level(STPM32_SYNC_GPIO, 0);

//     /* ---------- Init SPI ---------- */
//     ESP_ERROR_CHECK(stpm32_spi_init());
//     delay(10);

//     /* ---------- Power cycle STPM ---------- */
//     stpm32_power_cycle();
//     delay(40);
//     /*------------------------------global reset----------------------------------*/
//     stpm32_sync_pulse(3); // 3 pulses = global reset
//     delay(20);

//     /*Read DSP_CR1 TO ENSURE CHIP IS IDLE AND HOLDS DEFAULT VALUE AT POWER ON*/
//     uint32_t dsp_cr1_default;
//     esp_err_t err = stpm32_read_reg(DSP_CR1_ADDR, &dsp_cr1_default, STPM32_CRC_NONE);
//     if (err != ESP_OK)
//     {
//         STPM32_LOGE("Failed to read DSP_CR1: 0x%02X", err);
//         // optionally retry or continue safely
//     }
//     else
//     {
//         STPM32_LOGI("DSP_CR1 default = 0x%08lX", dsp_cr1_default);
//     }

//     /* ---------- SPI sanity check : read DSP_CR1 ---------- */

//     // // --- Step 1: Read default DSP_CR1 (power-on value) ---
//     // ESP_ERROR_CHECK(stpm32_read_reg(DSP_CR1_ADDR, &dsp_cr1_default, STPM32_CRC_NONE));

//     // Serial.printf("DSP_CR1 default = 0x%08lX\n", dsp_cr1_default);

//     // dsp_cr1_lsw_cfg_t lsw_cfg = {.tc1 = 4, .envref1 = 1, .clears = 1, .clrss_to1 = 5};
//     // ESP_ERROR_CHECK(stpm32_write_reg(DSP_CR_LSB_ADDR, dsp_cr1_lsw_pack(&lsw_cfg), STPM32_CRC_NONE));
//     // dsp_cr1_msw_cfg_t msw_cfg = {.lcs1 = 1, .lps1 = 1, .lpw1 = 8, .roc1 = 1, .bhpfc1 = 1, .bhpfv1 = 1};
//     // ESP_ERROR_CHECK(stpm32_write_reg(DSP_CR1_MSW_ADDR, dsp_cr1_msw_pack(&msw_cfg), STPM32_CRC_NONE));

//     // dsp_cr1_pack(&lsw_cfg, &msw_cfg);
//     // // read back to verify
//     // uint32_t dsp_cr1_verify;
//     // ESP_ERROR_CHECK(stpm32_read_reg(DSP_CR1_ADDR, &dsp_cr1_verify, STPM32_CRC_NONE));
//     // if (dsp_cr1_verify == dsp_cr1_pack(&lsw_cfg, &msw_cfg))
//     //     Serial.println("SPI read/write test PASSED");
//     // else
//     //     Serial.printf("SPI read/write test FAILED: 0x%08lX != 0x%08lX\n", dsp_cr1_verify, dsp_cr1_pack(&lsw_cfg, &msw_cfg));
//     stpm32_spi_byte_test();

//     uint32_t sr1;
//     if (stpm32_read_sr1(&sr1) == ESP_OK)
//     {
//         if (sr1)
//             stpm32_clear_sr1();
//     }

//     /* ================= DFE CONFIG ================= */
//     dfe_cr1_cfg_t dfe1 = {.gain1 = 2, .enc1 = 1, .env1 = 1};
//     dfe_cr2_cfg_t dfe2 = {.gain2 = 2, .enc2 = 1, .env2 = 1};

//     ESP_ERROR_CHECK(stpm32_write_reg(DFE_CR1_ADDR, dfe_cr1_pack(&dfe1), STPM32_CRC_NONE));
//     ESP_ERROR_CHECK(stpm32_write_reg(DFE_CR2_ADDR, dfe_cr2_pack(&dfe2), STPM32_CRC_NONE));
//     stpm32_sync_pulse(1);
//     delayMicroseconds(20);

//     STPM32_LOGI("Waiting for DFE ready...");

//     uint32_t dfe_status = 0;
//     uint32_t dfe_start = millis();

//     while (millis() - dfe_start < 500) // 500 timeout
//     {
//         stpm32_sync_pulse(1);
//         esp_rom_delay_us(10);

//         stpm32_read_reg(STPM32_DFE_STATUS, &dfe_status, STPM32_CRC_NONE);

//         if ((dfe_status & DFE_STATUS_V_OK_Msk) &&
//             (dfe_status & DFE_STATUS_I_OK_Msk))
//         {
//             STPM32_LOGI("DFE READY");
//             break;
//         }

//         delay(10);
//     }

//     if (!((dfe_status & DFE_STATUS_V_OK_Msk) &&
//           (dfe_status & DFE_STATUS_I_OK_Msk)))
//     {
//         STPM32_LOGE("DFE not ready (timeout)");
//         return; // STOP initialization
//     }

//     /*=======================Enable REF,TC, FUND FIRST==========================*/
//     // Read DSP_CR1 first
//     // uint32_t dsp_cr1_default;
//     // ESP_ERROR_CHECK(stpm32_read_reg(DSP_CR1_ADDR, &dsp_cr1_default, STPM32_CRC_NONE));

//     // // Enable ENVREF1
//     // dsp_cr1_set_envref1(&dsp_cr1_default, 1);

//     // // Enable TC1 bits
//     // dsp_cr1_default |= DSP_CR1_TC1_Msk;

//     // // Enable APM1 (fundamental/active measurement)
//     // dsp_cr1_default |= DSP_CR1_APM1_Msk;

//     // // Split into LSW and MSW for write
//     // uint16_t dsp_cr1_lsw = dsp_cr1_default & 0xFFFF;         // lower 16 bits
//     // uint16_t dsp_cr1_msw = (dsp_cr1_default >> 16) & 0xFFFF; // upper 16 bits

//     // // Write LSW first, then MSW
//     // ESP_ERROR_CHECK(stpm32_write_reg(DSP_CR_LSB_ADDR, dsp_cr1_lsw, STPM32_CRC_NONE));
//     // ESP_ERROR_CHECK(stpm32_write_reg(DSP_CR1_MSW_ADDR, dsp_cr1_msw, STPM32_CRC_NONE));

//     // delayMicroseconds(20);

//     /* ================= DSP CONFIG ================= */
//     // 1️. Configure DSP_CR1
//     dsp_cr1_lsw_cfg_t lsw = {.tc1 = 4, .envref1 = 1, .clears = 0, .clrss_to1 = 0}; /// tc1 = 4 corrected
//     dsp_cr1_msw_cfg_t msw = {.lcs1 = 0, .lps1 = 0, .lpw1 = 0, .roc1 = 0, .bhpfc1 = 1, .bhpfv1 = 1, .apm1 = 1};
//     uint32_t dsp_cr1_val = dsp_cr1_pack(&lsw, &msw);
//     STPM32_LOGI("DSP_CR1 value = 0x%08lX\n", dsp_cr1_val);

//     // 2️. Write LSW then MSW
//     ESP_ERROR_CHECK(stpm32_write_reg(DSP_CR_LSB_ADDR, dsp_cr1_lsw_pack(&lsw), STPM32_CRC_NONE));
//     ESP_ERROR_CHECK(stpm32_write_reg(DSP_CR1_MSW_ADDR, dsp_cr1_msw_pack(&msw), STPM32_CRC_NONE));

//     // 3️. Wait a tiny bit for DSP internal update
//     delayMicroseconds(100);

//     // 4️. Latch / SYNC pulse
//     stpm32_sync_pulse(1);
//     // 5️. Enable cumulative energy (DSP_CR3)
//     dsp_cr3_msw_cfg_t cr3 = {.en_cum = 1};
//     ESP_ERROR_CHECK(stpm32_write_reg(DSP_CR3_MSW_ADDR, dsp_cr3_msw_pack(&cr3), STPM32_CRC_NONE));
//     stpm32_sync_pulse(1);

//     // 6️.Verify DSP_CR1
//     uint32_t dsp_cr1_verify;
//     ESP_ERROR_CHECK(stpm32_read_reg(DSP_CR1_ADDR, &dsp_cr1_verify, STPM32_CRC_NONE));
//     STPM32_LOGI("DSP_CR1 after write = 0x%08lX\n", dsp_cr1_verify);

//     // uint32_t val;
//     // stpm32_read_reg_dbg(DSP_CR1_ADDR, &val, STPM32_CRC_NONE);
//     // Serial.printf("After latch DSP_CR1 = 0x%06lX\n", val);

//     /* ================= START DSP PIPELINE ================= */
//     STPM32_LOGI("Starting DSP pipeline...");
//     esp_rom_delay_us(5000);

//     uint32_t status;
//     uint32_t dsp_start = millis();
//     while (millis() - dsp_start < 1000) // wait max 1 sec
//     {
//         stpm32_sync_pulse(1);
//         esp_rom_delay_us(10);
//         ESP_ERROR_CHECK(stpm32_read_reg(STPM32_DSP_STATUS, &status, STPM32_CRC_NONE));

//         if (status & DSP_STATUS_VALID_Msk)
//             break;

//         delay(25);
//     }

//     if (!(status & DSP_STATUS_VALID_Msk))
//     {
//         stpm_stat = STPM_STATE_ERROR;
//         STPM32_LOGW("DSP running but no valid waveform detected");
//     }
//     else
//     {
//         stpm_stat = STPM_STATE_RUNNING;
//         STPM32_LOGI("DSP RUNNING & DATA VALID");
//     }

//     // when ac signal is present, DSP_STATUS_VALID bit will be set. If not set, it could be due to no input signal or some issue in DSP configuration or synchronization. In either case we should not proceed with reading measurements as they would be invalid. We can choose to continue and read measurements but they may not be meaningful until we have valid data. For this test code, we will just log the status and continue, but in a real application you might want to handle this case more robustly (e.g. retry configuration, alert user, etc).
//     //  if (status & DSP_STATUS_VALID_Msk)
//     //      stpm_stat = STPM_STATE_RUNNING;
//     //  else
//     //      stpm_stat = STPM_STATE_ERROR;

//     stpm32_offset_start(&power_offset_state, 200);

//     /* ============================Calibration ========================== */
//     stpm32_calib_init(&stpm32_calib_ph1, 0);
// }

// /* ===================== LOOP ===================== */
// void loop()
// {
//     // /*=======================Period control ================================= */
//     // if (millis() - last_sample_ms < STPM32_PERIOD_MS)
//     //     return;

//     // last_sample_ms = millis();

//     // /* ================= OFFSET CALIBRATION ================= */
//     // if (!power_offset_state.done)
//     // {
//     //     uint32_t raw_power;
//     //     stpm32_sync_pulse(1);
//     //     esp_rom_delay_us(10);

//     //     ESP_ERROR_CHECK(stpm32_read_reg(PH1_REG5_ADDR, &raw_power, STPM32_CRC_NONE));
//     //     stpm32_offset_process(&power_offset_state, raw_power);

//     //     if (power_offset_state.done)
//     //     {
//     //         stpm32_calib_ph1.power_offset_raw = stpm32_offset_get(&power_offset_state);
//     //         STPM32_LOGI("Power offset calibrated: %ld (raw)", stpm32_calib_ph1.power_offset_raw);
//     //     }

//     //     return; // wait until offset is done
//     // }

//     // /* ================ CHECK DSP STATE BEFORE MEASUREMENT ================ */
//     // if (stpm_stat != STPM_STATE_RUNNING)
//     // {
//     //     STPM32_LOGW("Skipping measurement: DSP not running or no AC signal");
//     //     return;
//     // }

//     // /* ================= NORMAL MEASUREMENT ================= */
//     // stpm32_sync_pulse(1);
//     // esp_rom_delay_us(20); // allow DSP snapshot settle

//     // float vrms = stpm32_read_vrms(PH1_VRMS_ADDR, &stpm32_calib_ph1);
//     // float irms = stpm32_read_irms(PH1_IRMS_ADDR, &stpm32_calib_ph1);
//     // float pwr = stpm32_read_power_w(PH1_REG5_ADDR, &stpm32_calib_ph1);

//     // STPM32_LOGI("VRMS=%.3f V IRMS=%.3f A PWR=%.3f W", vrms, irms, pwr);
// }
