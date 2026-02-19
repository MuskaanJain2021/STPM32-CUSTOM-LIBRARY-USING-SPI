#include "stpm32_logger.h"
#include <stdarg.h>
#include <stdio.h>
#include "esp_log.h"
#include "esp_timer.h"
#ifdef CONFIG_FREERTOS_HZ
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#endif


#define STPM32_LOG_TAG        "STPM32"
#define STPM32_LOG_BUFFER_SZ  256

static stpm32_log_level_t current_level = STPM32_LOG_LEVEL;


void stpm32_logger_set_level(stpm32_log_level_t level)
{
    current_level = level;
}

stpm32_log_level_t stpm32_logger_get_level(void)
{
    return current_level;
}


static void stpm32_log_base(esp_log_level_t esp_level,
                            const char *file,
                            int line,
                            const char *fmt,
                            va_list args)
{
    if (!fmt)
        return;

    char msg[STPM32_LOG_BUFFER_SZ];

    /* Format user message safely */
    int len = vsnprintf(msg, sizeof(msg), fmt, args);

    if (len < 0)
        return;

    /* Optional: timestamp in ms */
    uint32_t time_ms = (uint32_t)(esp_timer_get_time() / 1000ULL);

#ifdef CONFIG_FREERTOS_HZ
    const char *task = pcTaskGetName(NULL);
#else
    const char *task = "main";
#endif

    esp_log_write(esp_level,
                  STPM32_LOG_TAG,
                  "[%lu ms][%s] %s:%d: %s",
                  time_ms,
                  task,
                  file,
                  line,
                  msg);
}


void stpm32_log_error(const char *file, int line, const char *fmt, ...)
{
    if (current_level < STPM32_LOG_ERROR)
        return;

    va_list args;
    va_start(args, fmt);
    stpm32_log_base(ESP_LOG_ERROR, file, line, fmt, args);
    va_end(args);
}

void stpm32_log_warn(const char *file, int line, const char *fmt, ...)
{
    if (current_level < STPM32_LOG_WARN)
        return;

    va_list args;
    va_start(args, fmt);
    stpm32_log_base(ESP_LOG_WARN, file, line, fmt, args);
    va_end(args);
}

void stpm32_log_info(const char *file, int line, const char *fmt, ...)
{
    if (current_level < STPM32_LOG_INFO)
        return;

    va_list args;
    va_start(args, fmt);
    stpm32_log_base(ESP_LOG_INFO, file, line, fmt, args);
    va_end(args);
}

void stpm32_log_debug(const char *file, int line, const char *fmt, ...)
{
    if (current_level < STPM32_LOG_DEBUG)
        return;

    va_list args;
    va_start(args, fmt);
    stpm32_log_base(ESP_LOG_DEBUG, file, line, fmt, args);
    va_end(args);
}
