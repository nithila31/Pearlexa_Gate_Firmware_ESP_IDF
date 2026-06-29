# pearlexa_connect

Reusable Pearlexa ESP-IDF connectivity component.

This component owns the universal Part A logic:

- BLE provisioning mode
- Wi-Fi storage and connection
- Wi-Fi RSSI/status publishing
- MQTT connection and routing
- OTA version check and OTA install
- OTA-safe app lifecycle callbacks

Device-specific Part B logic stays outside the library in the product `main.c`:

- GPIO pin map
- relay/output behavior
- sensor/state inputs
- product MQTT JSON command handling
- product self-test and safe-stop behavior

## Folder layout

```text
pearlexa_connect/
  CMakeLists.txt
  idf_component.yml
  include/pearlexa_connect.h
  src/pearlexa_connect.c
  examples/gate_basic/
```

## Git versioning

Use semantic tags:

```bash
git tag v1.1.3
git push origin v1.1.3
```

Device projects can then pin a known working release in `main/idf_component.yml`:

```yaml
dependencies:
  pearlexa_connect:
    git: git@github.com:Pearlexa/pearlexa_connect.git
    version: v1.1.3
```

For development, place this folder directly in the device project's `components/` folder.

This device project currently uses a local copy at `components/pearlexa_connect`.
