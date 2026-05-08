# CrowPanel Display Firmware (Separate Project)

This is a standalone ESP-IDF app for the **CrowPanel ESP32-S3**.

It does **only**:
- UART RX (from the instrument board)
- GUI render on the CrowPanel display

It does **not** include instrument code (audio, sensors, ToF, panel scanning).

## Build / Flash (CrowPanel only)

From this folder:

```bash
cd crowpanel_display
idf.py set-target esp32s3
idf.py -p COM_CROW build flash monitor
```

## UART Wiring

- Instrument TX (`GPIO8`) -> CrowPanel RX (`GPIO44`)
- GND <-> GND

Telemetry expected format:

`AURA,1,mode,track,instr,wavePct,rec,play,loop_ms,head_ms,t0,t1,t2,t3,scopeHex`

