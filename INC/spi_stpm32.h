#ifndef SPI_STPM32_H
#define SPI_STPM32_H

#include <stdint.h>
#include "driver/spi_master.h"
#include "esp_err.h"
#ifdef __cplusplus
extern "C" {
#endif

extern spi_device_handle_t stpm32_spi_handle;

/* ================= CRC ================= */
/**
 * @brief Polynomial for CRC-8 calculation (x^8 + x^2 + x + 1)
 */
#define STPM3X_CRC8_POLY   (0x07U)

/**
 * @brief Initial value for CRC-8 calculation
 */
#define STPM3X_CRC8_INIT   (0x00U)

/* ================= SPI (VSPI) GPIOs ================= */
/** MOSI pin for STPM32 SPI */
#define STPM32_SPI_MOSI_GPIO     GPIO_NUM_23
/** MISO pin for STPM32 SPI */
#define STPM32_SPI_MISO_GPIO     GPIO_NUM_19
/** SPI clock pin for STPM32 */
#define STPM32_SPI_SCLK_GPIO     GPIO_NUM_18
/** SPI chip-select pin for STPM32 */
#define STPM32_SPI_CS_GPIO       GPIO_NUM_25

/* ================= STPM32 SYNC / IRQ ================= */
/** SYNC / IRQ pin for STPM32 */
#define STPM32_SYNC_GPIO         GPIO_NUM_16

/* ================= POWER CONTROL ================= */
/** STPM32 power enable pin */
#define STPM32_POWER_EN_GPIO     GPIO_NUM_17
/** Level to turn STPM32 power ON */
#define STPM32_POWER_ON_LEVEL    (1U)
/** Level to turn STPM32 power OFF */
#define STPM32_POWER_OFF_LEVEL   (0U)

#define STPM32_SR1_CLEAR_ALL_MASK   0x0000FF30

/// ================= TIMING CONSTANTS ================= */
static const uint32_t STPM_SYN_PULSE_US = 4;
static const uint32_t STPM_SYN_GAP_US   = 4;

#define STPM32_SPI_INIT_RETRIES 3
#define STPM32_SPI_INIT_DELAY_MS 50
/* ================= CHIP SELECT MACROS ================= */
/** Pull CS low */
#define CS_LOW()   do { gpio_set_level(STPM32_SPI_CS_GPIO, 0); } while(0)

/** Pull CS high */
#define CS_HIGH()   do { gpio_set_level(STPM32_SPI_CS_GPIO, 1); } while(0)


/* ================= DEFAULT CRC MODE ================= */
#define STPM32_DEFAULT_CRC   STPM32_CRC_CRC8_07


/* ================= INTERFACE ENUMS ================= */
/** STPM32 communication interface */
typedef enum
{
    STPM32_IF_SPI,  /**< SPI interface */
    STPM32_IF_UART  /**< UART interface (not implemented) */
} stpm32_interface_t;

/** STPM32 operation type */
typedef enum
{
    STPM32_OP_READ,  /**< Read register */
    STPM32_OP_WRITE  /**< Write register */
} stpm32_op_t;

/** CRC mode */
typedef enum
{
    STPM32_CRC_NONE = 0,       /**< No CRC */
    STPM32_CRC_CRC8_07         /**< CRC-8, polynomial 0x07 (SMBus) */
} stpm32_crc_mode_t;

//Device state machine states
typedef enum {
    STPM_STATE_OFF,
    STPM_STATE_RESET,
    STPM_STATE_DFE_WAIT,
    STPM_STATE_DSP_WAIT,
    STPM_STATE_RUNNING,
    STPM_STATE_ERROR
} stpm32_state_t;
extern stpm32_state_t stpm_stat;
//Error codes for calibration and measurement functions. Not to be confused with SPI transaction errors.
typedef enum {
    STPM_OK = 0,
    STPM_ERR_SPI,
    STPM_ERR_CRC,
    STPM_ERR_TIMEOUT,
    STPM_ERR_DFE_NOT_READY,
    STPM_ERR_DSP_NOT_VALID,
    STPM_ERR_INVALID_ARG,
} stpm32_err_t;

/* ================= SPI FRAMES / PACKETS ================= */
/**
 * @brief Raw SPI frame (low-level)
 *
 * Contains the SPI bytes to be transmitted or received.
 */
typedef struct
{
    uint8_t buf[6];   /**< SPI frame buffer: addr + 3 bytes data + optional CRC */
    uint8_t len;      /**< Number of valid bytes in buffer */
} stpm32_spi_frame_t;

/**
 * @brief Logical STPM32 packet
 *
 * Encapsulates a register operation with optional CRC.
 */
typedef struct
{
    stpm32_op_t        op;    /**< READ or WRITE operation */
    uint8_t            addr;  /**< 7-bit register address */
    uint32_t           data;  /**< 24-bit payload (LSB aligned) */
    stpm32_crc_mode_t  crc;   /**< CRC mode to use */
} stpm32_packet_t;

/* ================= SPI API ================= */
/**
 * @brief Initialize SPI peripheral for STPM32
 *
 * Configures SPI bus, GPIOs, and adds device to SPI master.
 *
 * @return ESP_OK on success, error code otherwise
 */
esp_err_t stpm32_spi_init(void);


stpm32_err_t stpm32_spi_init_wrapper(void);

/**
 * @brief Initialize GPIOs required for STPM32 operation.
 *
 * Configures:
 *  - CS pin as output with HIGH level (prevents boot failure)
 *  - SYNC/IRQ pin as input
 *
 * @note CS must be HIGH at boot to avoid STPM32 misbehavior.
 * @note SYNC/IRQ pin is a strap pin from STPM32 ,do not pull-up.
 *
 * @return ESP_OK if GPIOs configured successfully, otherwise an ESP-IDF error code.
 */
esp_err_t stpm32_gpio_init(void);


/**
 * @brief Perform SPI write/read transaction
 *
 * @param tx Pointer to transmit buffer
 * @param rx Pointer to receive buffer
 * @param len Number of bytes to transfer
 *
 * @return ESP_OK on success, error code otherwise
 */
esp_err_t stpm32_spi_rw(uint8_t *tx, uint8_t *rx, size_t len);


/**
 * @brief Latch STPM32 DSP measurements
 *
 * Generates a SYNC pulse to force the STPM32 to capture a coherent
 * snapshot of all measurement registers.
 *
 * @note  Timing per datasheet: t_rel > 40 ns, t_lpw >= 4 us
 *        Safe delays are used in microseconds.
 */
///void stpm32_latch(void);
void stpm32_sync_pulse(uint8_t count);
void stpm32_sync_snapshot(void);   // 1 pulse
void stpm32_sync_global_reset(void);      // 3 pulse'
/**
 * @brief Read a 24-bit register from STPM32
 *
 * @param reg Register address
 * @param value Pointer to store register value
 * @param crc_mode CRC mode to use
 *
 * @return ESP_OK on success, error code otherwise
 */
//esp_err_t stpm32_read_reg(uint8_t reg, uint32_t *value, stpm32_crc_mode_t crc_mode);
stpm32_err_t stpm32_read_reg(uint8_t reg, uint32_t *value, stpm32_crc_mode_t crc_mode);
/**
 * @brief Write a 24-bit register to STPM32
 *
 * @param reg Register address
 * @param value 24-bit value to write
 * @param crc_mode CRC mode to use
 *
 * @return ESP_OK on success, error code otherwise
 */
stpm32_err_t stpm32_write_reg(uint8_t reg,uint32_t value,stpm32_crc_mode_t crc_mode);

/* Convenience inline wrappers using default CRC */
static inline esp_err_t stpm32_read_reg_simple(uint8_t reg, uint32_t *value)
{
    return stpm32_read_reg(reg, value, STPM32_DEFAULT_CRC);
}

static inline esp_err_t stpm32_write_reg_simple(uint8_t reg, uint32_t value)
{
    return stpm32_write_reg(reg, value, STPM32_DEFAULT_CRC);
}

/**
 * @brief Debug version of register read
 *
 * @param reg Register address
 * @param value Pointer to store value
 * @param crc_mode CRC mode to use
 *
 * @return ESP_OK on success, error code otherwise
 */
esp_err_t stpm32_read_reg_dbg(uint8_t reg, uint32_t *value, stpm32_crc_mode_t crc_mode);

/* ================= CRC API ================= */
/**
 * @brief Compute CRC-8 over a data buffer
 *
 * @param data Pointer to input data
 * @param length Number of bytes to process
 *
 * @return Computed CRC-8 value
 */
uint8_t stpm3x_crc8_compute(const uint8_t *data , uint8_t length);

/* ================= POWER CONTROL ================= */
/**
 * @brief Power-cycle the STPM32
 *
 * Toggles the power enable pin off/on.
 */
void stpm32_power_cycle(void);

/* ================= STATUS REGISTER ================= */
/**
 * @brief Clear SR1 register
 */
stpm32_err_t stpm32_clear_sr1(void);

/**
 * @brief Read SR1 register
 *
 * @param value Pointer to store SR1 value
 *
 * @return ESP_OK on success, error code otherwise
 */
esp_err_t stpm32_read_sr1(uint32_t *value);


/**
 * @brief Transfer a logical STPM32 packet over SPI
 *
 * Handles chip-select, CRC, and read/write decoding.
 *
 * @param pkt Pointer to STPM32 packet structure
 *
 * @return ESP_OK on success, otherwise an SPI driver error
 */
esp_err_t stpm32_spi_transfer(stpm32_packet_t *pkt);
esp_err_t spi_transfer(uint8_t *tx, uint8_t *rx, size_t len);

stpm32_err_t stpm32_spi_bus_txrx(uint8_t *tx, uint8_t *rx, size_t len);//only for transport layer testing, not for crc and parsing testing.Device state machine states.



#ifdef __cplusplus
}
#endif

#endif /* SPI_STPM32_H */
