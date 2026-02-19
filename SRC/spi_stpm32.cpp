#include "spi_stpm32.h"
#include "driver/gpio.h"
#include "esp_log.h"
#include <Arduino.h>
#include "FreeRTOSConfig.h"
#include "freertos/task.h"
#include "STPM3X_Def.h"
#include "driver/gpio.h"
#include "esp_rom_sys.h"

spi_device_handle_t stpm32_spi_handle = NULL;
static const char *TAG = "STPM32";

// Global variable to track STPM32 state
stpm32_state_t stpm_stat = STPM_STATE_OFF;

// void stpm32_latch(void)
// {
//     gpio_set_level(STPM32_SPI_CS_GPIO, 1);
//     esp_rom_delay_us(2);

//     gpio_set_level(STPM32_SYNC_GPIO, 0);
//     esp_rom_delay_us(2);

//     gpio_set_level(STPM32_SYNC_GPIO, 1);
//     esp_rom_delay_us(5);
// }

void stpm32_sync_raw(uint8_t count)
{
    gpio_set_level(STPM32_SPI_CS_GPIO, 1);

    for (uint8_t i = 0; i < count; i++)
    {
        gpio_set_level(STPM32_SYNC_GPIO, 0);
        esp_rom_delay_us(STPM_SYN_PULSE_US);

        gpio_set_level(STPM32_SYNC_GPIO, 1);
        esp_rom_delay_us(STPM_SYN_GAP_US);
    }
}

void stpm32_sync_pulse(uint8_t count)
{
    if (count < 1 || count > 3)
        return;
    stpm32_sync_raw(count);
}

void stpm32_sync_snapshot(void)
{
    stpm32_sync_raw(1);
}

void stpm32_sync_global_reset(void)
{
    stpm32_sync_raw(3);
}

esp_err_t stpm32_spi_init(void)
{
    esp_err_t ret;

    spi_bus_config_t buscfg;
    memset(&buscfg, 0, sizeof(buscfg));

    buscfg.mosi_io_num = STPM32_SPI_MOSI_GPIO;
    buscfg.miso_io_num = STPM32_SPI_MISO_GPIO;
    buscfg.sclk_io_num = STPM32_SPI_SCLK_GPIO;
    buscfg.quadwp_io_num = -1;
    buscfg.quadhd_io_num = -1;
    buscfg.max_transfer_sz = 5;

    spi_device_interface_config_t devcfg;
    memset(&devcfg, 0, sizeof(devcfg));

    devcfg.clock_speed_hz = 1000000; // 1 MHz
    devcfg.mode = 3;
    devcfg.spics_io_num = -1;
    devcfg.queue_size = 1;
    devcfg.flags = 0;

    ret = spi_bus_initialize(VSPI_HOST, &buscfg, SPI_DMA_CH_AUTO);
    if (ret != ESP_OK)
    {
        return ret;
    }

    ret = spi_bus_add_device(VSPI_HOST, &devcfg, &stpm32_spi_handle);
    if (ret != ESP_OK)
    {
        return ret;
    }

    ESP_LOGI(TAG, "STPM32 SPI initialized");
    return ESP_OK;
}

// stpm32_err_t stpm32_spi_init_wrapper(void)
// {
//     for (uint8_t attempt = 0; attempt < STPM32_SPI_INIT_RETRIES; attempt++)
//     {
//         esp_err_t ret = stpm32_spi_init();

//         if (ret == ESP_OK)
//         {
//             stpm_stat = STPM_STATE_RESET; // STPM32 ready to reset/init DSP
//             ESP_LOGI(TAG, "SPI initialized successfully on attempt %d", attempt + 1);
//             return STPM_OK;
//         }

//         // SPI init failed
//         stpm_stat = STPM_STATE_ERROR;
//         ESP_LOGW(TAG, "SPI init attempt %d failed: %d", attempt + 1, ret);
//         vTaskDelay(pdMS_TO_TICKS(STPM32_SPI_INIT_DELAY_MS));
//     }

//     // All attempts failed
//     ESP_LOGE(TAG, "SPI failed after %d attempts", STPM32_SPI_INIT_RETRIES);
//     return STPM_ERR_SPI;
// }

#define STPM32_SPI_INIT_RETRIES 3
#define STPM32_SPI_INIT_DELAY_MS 50

stpm32_err_t stpm32_spi_init_wrapper(void)
{
    esp_err_t ret;

    for (uint8_t attempt = 0; attempt < STPM32_SPI_INIT_RETRIES; attempt++)
    {
        ret = stpm32_spi_init(); // call your basic SPI init

        if (ret == ESP_OK)
        {
            stpm_stat = STPM_STATE_RESET; // ready to reset/init DSP
            ESP_LOGI(TAG, "STPM32 SPI initialized successfully on attempt %d", attempt + 1);
            return STPM_OK;
        }

        // SPI init failed
        stpm_stat = STPM_STATE_ERROR;
        ESP_LOGW(TAG, "SPI init attempt %d failed: %d", attempt + 1, ret);
        vTaskDelay(pdMS_TO_TICKS(STPM32_SPI_INIT_DELAY_MS));
    }

    // All attempts failed
    stpm_stat = STPM_STATE_ERROR;
    ESP_LOGE(TAG, "STPM32 SPI init failed after %d attempts", STPM32_SPI_INIT_RETRIES);
    return STPM_ERR_SPI;
}

static stpm32_err_t stpm32_spi_transfer_raw(uint8_t *tx, uint8_t *rx, size_t len)
{
    if (!tx || !rx)
        return STPM_ERR_INVALID_ARG;

    esp_err_t ret = spi_transfer(tx, rx, len);
    if (ret != ESP_OK)
    {
        ESP_LOGE(TAG, "SPI raw transfer failed");
        return STPM_ERR_SPI;
    }

    return STPM_OK;
}

uint8_t stpm3x_crc8_compute(const uint8_t *data, uint8_t length)
{
    uint8_t crc = STPM3X_CRC8_INIT;
    uint8_t i;
    uint8_t bit;

    for (i = 0U; i < length; i++)
    {
        crc ^= data[i];

        for (bit = 0U; bit < 8U; bit++)
        {
            if ((crc & 0x80U) != 0U)
            {
                crc = (uint8_t)((crc << 1) ^ STPM3X_CRC8_POLY);
            }
            else
            {
                crc <<= 1;
            }
        }
    }

    return crc;
}
// corrected
static void stpm32_build_frame(const stpm32_packet_t *pkt,
                               stpm32_spi_frame_t *frame)
{
    uint8_t i = 0;

    /* Address */
    frame->buf[i++] = (pkt->op == STPM32_OP_READ)
                          ? (pkt->addr | 0x80U)
                          : (pkt->addr & 0x7FU);

    if (pkt->op == STPM32_OP_WRITE)
    {
        frame->buf[i++] = (uint8_t)(pkt->data & 0xFFU);
        frame->buf[i++] = (uint8_t)((pkt->data >> 8) & 0xFFU);
        frame->buf[i++] = (uint8_t)((pkt->data >> 16) & 0xFFU);
    }
    else
    {
        frame->buf[i++] = 0xFFU;
        frame->buf[i++] = 0xFFU;
        frame->buf[i++] = 0xFFU;
    }

    if (pkt->crc == STPM32_CRC_CRC8_07)
    {
        frame->buf[i++] = stpm3x_crc8_compute(frame->buf, i);
    }

    frame->len = i;
}

esp_err_t spi_transfer(uint8_t *tx, uint8_t *rx, size_t len)
{
    spi_transaction_t t;
    memset(&t, 0, sizeof(t));

    t.length = len * 8;
    t.tx_buffer = tx;
    t.rx_buffer = rx;

    return spi_device_transmit(stpm32_spi_handle, &t);
}

esp_err_t stpm32_spi_transfer(stpm32_packet_t *pkt)
{

    if (!pkt)
        return STPM_ERR_INVALID_ARG;

    if (stpm_stat != STPM_STATE_RUNNING && stpm_stat != STPM_STATE_DSP_WAIT)
    {
        ESP_LOGW(TAG, "STPM32 not ready for SPI transfer! Current state: %d", stpm_stat);
        return STPM_ERR_DSP_NOT_VALID;
    }

    stpm32_spi_frame_t frame;
    uint8_t rx[6] = {0};
    stpm32_build_frame(pkt, &frame);

    if (frame.len > sizeof(rx))
        return STPM_ERR_TIMEOUT;
    ;

    CS_LOW();
    stpm32_err_t err = stpm32_spi_transfer_raw(frame.buf, rx, frame.len);
    CS_HIGH();

    if (err != STPM_OK)
        return err;

    if (pkt->op == STPM32_OP_READ)
    {
        if (pkt->crc == STPM32_CRC_CRC8_07)
        {
            uint8_t expected_len = 5;
            uint8_t computed_crc = stpm3x_crc8_compute(rx, expected_len - 1);
            uint8_t received_crc = rx[expected_len - 1];

            if (computed_crc != received_crc)
                return STPM_ERR_CRC;
        }

        pkt->data = ((uint32_t)rx[1]) |
                    ((uint32_t)rx[2] << 8) |
                    ((uint32_t)rx[3] << 16);
    }

#ifdef STPM32_DEBUG
    printf("RX: ");
    for (int i = 0; i < frame.len; i++)
        printf("%02X ", rx[i]);
    printf("\n");
#endif

    return STPM_OK;
}

stpm32_err_t stpm32_spi_bus_txrx(uint8_t *tx, uint8_t *rx, size_t len)
{

    if (!tx || !rx)
        return STPM_ERR_INVALID_ARG;

    if (stpm_stat != STPM_STATE_RUNNING && stpm_stat != STPM_STATE_DSP_WAIT)
        return STPM_ERR_DSP_NOT_VALID;

    spi_transaction_t t = {0};
    t.length = len * 8;
    t.tx_buffer = tx;
    t.rx_buffer = rx;

    gpio_set_level(STPM32_SPI_CS_GPIO, 0);
    esp_err_t ret = spi_device_transmit(stpm32_spi_handle, &t);
    gpio_set_level(STPM32_SPI_CS_GPIO, 1);

    return (ret == ESP_OK) ? STPM_OK : STPM_ERR_SPI;
}

// esp_err_t stpm32_read_reg(uint8_t reg, uint32_t *value, stpm32_crc_mode_t crc_mode)
// {
//     if (!value)
//         return ESP_ERR_INVALID_ARG;

//     stpm32_packet_t pkt_cmd = {.op = STPM32_OP_READ, .addr = reg, .data = 0, .crc = crc_mode};
//     stpm32_packet_t pkt_data = {.op = STPM32_OP_READ, .addr = reg, .data = 0, .crc = crc_mode};

//     esp_err_t ret = stpm32_spi_transfer(&pkt_cmd);
//     if (ret != ESP_OK)
//         return ret;

//     esp_rom_delay_us(4);

//     ret = stpm32_spi_transfer(&pkt_data);
//     if (ret != ESP_OK)
//         return ret;

//     *value = pkt_data.data;

//     return ESP_OK;
// }
stpm32_err_t stpm32_read_reg(uint8_t reg, uint32_t *value, stpm32_crc_mode_t crc_mode)
{
    if (!value)
        return STPM_ERR_INVALID_ARG; // Map invalid pointer to your enum

    // Check if STPM32 is ready
    if (stpm_stat != STPM_STATE_RUNNING && stpm_stat != STPM_STATE_DSP_WAIT)
    {
        ESP_LOGW(TAG, "STPM32 not ready for read! Current state: %d", stpm_stat);
        return STPM_ERR_DSP_NOT_VALID; // Not ready
    }

    stpm32_packet_t pkt_cmd = {.op = STPM32_OP_READ, .addr = reg, .data = 0, .crc = crc_mode};
    stpm32_packet_t pkt_data = {.op = STPM32_OP_READ, .addr = reg, .data = 0, .crc = crc_mode};

    // First SPI transfer (command)
    esp_err_t ret = stpm32_spi_transfer(&pkt_cmd);
    if (ret != ESP_OK)
    {
        ESP_LOGE(TAG, "SPI transfer (cmd) failed for reg 0x%02X", reg);
        return STPM_ERR_SPI;
    }

    esp_rom_delay_us(4);

    // Second SPI transfer (data)
    ret = stpm32_spi_transfer(&pkt_data);
    if (ret != ESP_OK)
    {
        ESP_LOGE(TAG, "SPI transfer (data) failed for reg 0x%02X", reg);
        return STPM_ERR_SPI;
    }

    // Copy received data
    *value = pkt_data.data & 0x00FFFFFFU; // Mask to 24-bit

    return STPM_OK;
}

esp_err_t stpm32_read_reg_dbg(uint8_t reg, uint32_t *value, stpm32_crc_mode_t crc_mode)
{
    esp_err_t ret = stpm32_read_reg(reg, value, crc_mode);

    if (ret == ESP_OK)
    {
        Serial.printf("\nSPI READ REG 0x%02X = 0x%06lX\n", reg, *value);
    }
    else
    {
        Serial.printf("\nSPI READ ERROR 0x%02X\n", reg);
    }

    return ret;
}

esp_err_t stpm32_read_sr1(uint32_t *value)
{
    if (value == NULL)
        return ESP_ERR_INVALID_ARG;
    stpm32_sync_snapshot();
    esp_rom_delay_us(10);

    esp_err_t ret = stpm32_read_reg(DSP_SR1_ADDR, value, STPM32_DEFAULT_CRC);
    if (ret != ESP_OK)
        return ret;

    *value &= 0x00FFFFFFU;
    return ESP_OK;
}

esp_err_t stpm32_spi_rw(uint8_t *tx, uint8_t *rx, size_t len)
{
    spi_transaction_t t = {0};

    t.length = len * 8;
    t.tx_buffer = tx;
    t.rx_buffer = rx;
#ifdef STPM32_DEBUG
    printf("RX: ");
    for (int i = 0; i < len; i++)
        printf("%02X ", rx[i]);
    printf("\n");
#endif

    return spi_device_transmit(stpm32_spi_handle, &t);
}
stpm32_err_t stpm32_write_reg(uint8_t reg,
                              uint32_t value,
                              stpm32_crc_mode_t crc_mode)
{
    if (reg == 0xFF)
        return STPM_ERR_INVALID_ARG;

    // Check if STPM32 is ready
    if (stpm_stat != STPM_STATE_RUNNING && stpm_stat != STPM_STATE_DSP_WAIT)
    {
        ESP_LOGW(TAG, "STPM32 not ready for write! Current state: %d", stpm_stat);
        return STPM_ERR_DSP_NOT_VALID;
    }

    stpm32_packet_t pkt = {
        .op = STPM32_OP_WRITE,
        .addr = reg,
        .data = value & 0x00FFFFFF,
        .crc = crc_mode};

    esp_err_t ret = stpm32_spi_transfer(&pkt);
    if (ret != ESP_OK)
    {
        ESP_LOGE(TAG, "SPI transfer (write) failed for reg 0x%02X", reg);
        return STPM_ERR_SPI;
    }

    return STPM_OK;
}
stpm32_err_t stpm32_clear_sr1(void)
{
    if (stpm_stat != STPM_STATE_RUNNING && stpm_stat != STPM_STATE_DSP_WAIT)
    {
        ESP_LOGW(TAG, "Cannot clear SR1: DSP not ready (state=%d)", stpm_stat);
        return STPM_ERR_DSP_NOT_VALID;
    }

    uint32_t sr1;
    stpm32_sync_snapshot();
    esp_rom_delay_us(10);

    stpm32_err_t err = stpm32_read_reg(DSP_SR1_ADDR, &sr1, STPM32_DEFAULT_CRC);
    if (err != STPM_OK)
    {
        ESP_LOGE(TAG, "Failed to read SR1 before clearing: %d", err);
        stpm_stat = STPM_STATE_ERROR;
        return err;
    }

    // Clear latched faults
    err = stpm32_write_reg(DSP_SR1_ADDR, STPM32_SR1_CLEAR_ALL_MASK, STPM32_DEFAULT_CRC);
    if (err != STPM_OK)
    {
        ESP_LOGE(TAG, "Failed to clear SR1: %d", err);
        stpm_stat = STPM_STATE_ERROR;
        return err;
    }

    stpm32_sync_snapshot();
    esp_rom_delay_us(10);

    ESP_LOGI(TAG, "SR1 cleared successfully");
    return STPM_OK;
}

esp_err_t stpm32_gpio_init(void)
{
    esp_err_t ret;
    gpio_config_t gpio_cfg = {0};

    gpio_cfg.pin_bit_mask = (1ULL << STPM32_SPI_CS_GPIO);
    gpio_cfg.mode = GPIO_MODE_OUTPUT;
    gpio_cfg.pull_up_en = GPIO_PULLUP_DISABLE;
    gpio_cfg.pull_down_en = GPIO_PULLDOWN_DISABLE;
    gpio_cfg.intr_type = GPIO_INTR_DISABLE;

    ret = gpio_config(&gpio_cfg);
    if (ret != ESP_OK)
        return ret;

    gpio_cfg.pin_bit_mask = (1ULL << STPM32_SYNC_GPIO);
    gpio_cfg.mode = GPIO_MODE_OUTPUT;
    gpio_cfg.pull_up_en = GPIO_PULLUP_DISABLE;
    gpio_cfg.pull_down_en = GPIO_PULLDOWN_DISABLE;
    gpio_cfg.intr_type = GPIO_INTR_DISABLE;

    ret = gpio_config(&gpio_cfg);
    if (ret != ESP_OK)
        return ret;
    gpio_cfg.pin_bit_mask = (1ULL << STPM32_POWER_EN_GPIO);
    gpio_cfg.mode = GPIO_MODE_OUTPUT;
    gpio_cfg.pull_up_en = GPIO_PULLUP_DISABLE;
    gpio_cfg.pull_down_en = GPIO_PULLDOWN_DISABLE;
    gpio_cfg.intr_type = GPIO_INTR_DISABLE;

    ret = gpio_config(&gpio_cfg);
    if (ret != ESP_OK)
        return ret;

    gpio_set_level(STPM32_SPI_CS_GPIO, 1);
    gpio_set_level(STPM32_SYNC_GPIO, 1);
    gpio_set_level(STPM32_POWER_EN_GPIO, STPM32_POWER_ON_LEVEL);
    vTaskDelay(pdMS_TO_TICKS(20)); // 20ms safe boot time

    return ESP_OK;
}
void stpm32_power_cycle(void)
{
    stpm_stat = STPM_STATE_ERROR; // mark error during power off
    gpio_set_level(STPM32_SPI_CS_GPIO, 1);
    gpio_set_level(STPM32_SYNC_GPIO, 1);

    gpio_set_level(STPM32_POWER_EN_GPIO, STPM32_POWER_OFF_LEVEL);
    vTaskDelay(pdMS_TO_TICKS(50));

    gpio_set_level(STPM32_POWER_EN_GPIO, STPM32_POWER_ON_LEVEL);
    vTaskDelay(pdMS_TO_TICKS(40));

    stpm_stat = STPM_STATE_RESET; // ready to reinit DSP
}
