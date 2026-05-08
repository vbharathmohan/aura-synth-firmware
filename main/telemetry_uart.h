#ifndef TELEMETRY_UART_H
#define TELEMETRY_UART_H

#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

/* Instrument MCU -> CrowPanel one-way telemetry (UART TX only). */
bool telemetry_uart_init(void);
void telemetry_uart_start(void);

#ifdef __cplusplus
}
#endif

#endif /* TELEMETRY_UART_H */

