# Pearlexa Gate Firmware - Local `pearlexa_connect` Component - v1.0.0

This is a **self-contained ESP-IDF project** for the Pearlexa gate controller.

The common Pearlexa connectivity code is kept locally inside the project as an ESP-IDF component:

```text
components/pearlexa_connect/
```

The product-specific gate behavior is kept in:

```text
main/main.c
```

This means the whole folder can be uploaded to GitHub as one private firmware repository and cloned/built on any PC without SSH component dependency setup.

## Folder structure

```text
Pearlexa_Gate_Local_Library_Project_v1.0.0/
├── CMakeLists.txt
├── README.md
├── RELEASE_NOTES.md
├── VERSION
├── partitions.csv
├── sdkconfig
├── main/
│   ├── CMakeLists.txt
│   └── main.c
└── components/
    └── pearlexa_connect/
        ├── CMakeLists.txt
        ├── idf_component.yml
        ├── include/
        │   └── pearlexa_connect.h
        └── src/
            └── pearlexa_connect.c
```

## Architecture

### `components/pearlexa_connect`

This is the reusable Pearlexa library.

It contains:

- BLE provisioning
- Wi-Fi save/connect/reconnect
- MQTT connection and publish/subscribe routing
- OTA check/install flow
- OTA rollback validation callback support
- App lifecycle callbacks
- Config mode handling
- Wi-Fi RSSI publishing

### `main/main.c`

This is device-specific.

For the gate controller, it contains:

- Gate GPIO pin map
- Open / Close / Stop pulse behavior
- Gate state input reading
- Gate command JSON handling
- Gate self-test callback
- Gate safe-stop before OTA
- Pearlexa device config values

For another Pearlexa product, such as a smart bulb, relay board, fan controller, or sensor, keep `components/pearlexa_connect` the same and replace only `main/main.c`.

## Build

Open ESP-IDF PowerShell/terminal inside this folder and run:

```bash
idf.py set-target esp32c3
idf.py reconfigure
idf.py build
```

Flash example:

```bash
idf.py -p COMx flash monitor
```

Replace `COMx` with your actual serial port.

## Git upload

This project is intended to be uploaded as one complete private GitHub repository.

```bash
git init
git add .
git commit -m "Initial Pearlexa gate firmware with local pearlexa_connect v1.0.0"
```

Then create a private repo on GitHub and push:

```bash
git branch -M main
git remote add origin https://github.com/Pearlexa/Pearlexa_Gate_Firmware.git
git push -u origin main
```

Optional version tag:

```bash
git tag v1.0.0
git push origin v1.0.0
```

## Important rule

Do not edit common Wi-Fi/MQTT/BLE/OTA code inside `main.c`.

- Common reusable code goes into `components/pearlexa_connect`.
- Gate-specific code goes into `main/main.c`.

