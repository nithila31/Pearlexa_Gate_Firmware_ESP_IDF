# Release Notes

## v1.0.0

Initial self-contained local-component version.

### Included

- `pearlexa_connect` local ESP-IDF component inside `components/pearlexa_connect`.
- Gate-specific application in `main/main.c`.
- BLE provisioning, Wi-Fi, MQTT, OTA, and app lifecycle callbacks kept in the reusable component.
- Gate open/close/stop behavior kept in `main.c`.
- Safer momentary gate command pulses instead of permanent output holding.
- Root CMake project name fixed to avoid spaces.

### Intended workflow

Upload the whole folder as one private GitHub repository. No SSH/private component dependency setup is required.
