#ifndef STPM32_LOGGER_H
#define STPM32_LOGGER_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif


/* ===============================
   Log Level Configuration
   =============================== */

typedef enum
{
    STPM32_LOG_NONE = 0,
    STPM32_LOG_ERROR,
    STPM32_LOG_WARN,
    STPM32_LOG_INFO,
    STPM32_LOG_DEBUG
} stpm32_log_level_t;

/* ===============================
   Configuration
   =============================== */

#ifndef STPM32_LOG_LEVEL
#define STPM32_LOG_LEVEL STPM32_LOG_DEBUG
#endif


void stpm32_logger_set_level(stpm32_log_level_t level);
stpm32_log_level_t stpm32_logger_get_level(void);

/* ===============================
   Logging Macros
   =============================== */
#if STPM32_LOG_LEVEL >= STPM32_LOG_ERROR
#define STPM32_LOGE(fmt, ...) \
    do { stpm32_log_error(__FILE__, __LINE__, fmt, ##__VA_ARGS__); } while (0)
#else
#define STPM32_LOGE(...) \
    do { } while (0)
#endif

#if STPM32_LOG_LEVEL >= STPM32_LOG_WARN
#define STPM32_LOGW(fmt, ...) \
    do { stpm32_log_warn(__FILE__, __LINE__, fmt, ##__VA_ARGS__); } while (0)
#else
#define STPM32_LOGW(...) \
    do { } while (0)
#endif

#if STPM32_LOG_LEVEL >= STPM32_LOG_INFO
#define STPM32_LOGI(fmt, ...) \
    do { stpm32_log_info(__FILE__, __LINE__, fmt, ##__VA_ARGS__); } while (0)
#else
#define STPM32_LOGI(...) \
    do { } while (0)
#endif

#if STPM32_LOG_LEVEL >= STPM32_LOG_DEBUG
#define STPM32_LOGD(fmt, ...) \
    do { stpm32_log_debug(__FILE__, __LINE__, fmt, ##__VA_ARGS__); } while (0)
#else
#define STPM32_LOGD(...) \
    do { } while (0)
#endif


void stpm32_log_error(const char *file, int line, const char *fmt, ...);
void stpm32_log_warn(const char *file, int line, const char *fmt, ...);
void stpm32_log_info(const char *file, int line, const char *fmt, ...);
void stpm32_log_debug(const char *file, int line, const char *fmt, ...);

#ifdef __cplusplus
}
#endif

#endif /* STPM32_LOGGER_H */
