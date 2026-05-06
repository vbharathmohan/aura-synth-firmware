/**
 * panel_input.h — 3×3 button matrix + 2 sliders + 1 pot (INTEGRATION_MODE).
 *
 * Call panel_input_init() once at boot, then panel_input_poll() from the
 * sensor polling task (or any loop) at any interval; scans run internally
 * at ~100 Hz when enough time has passed.
 */

#ifndef PANEL_INPUT_H
#define PANEL_INPUT_H

#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

void panel_input_init(void);
void panel_input_poll(void);

#ifdef __cplusplus
}
#endif

#endif /* PANEL_INPUT_H */
