# LD2450 distance reader

This PlatformIO project reads the tracked targets from an LD2450 connected to a Wemos D1 mini and displays it on a self-hosted website.
It can track up to 3 people displayed as dots of different colors.
Built for a Wemos D1 Mini

## Wiring

Use a common ground and connect:

| LD2450 | Wemos D1 mini |
| --- | --- |
| TX | D7 (GPIO13) |
| RX | D8 (GPIO15) |
| GND | G |
| VCC | 5V |

The LD2450 UART is 3.3 V logic. Power the sensor according to the specific board's documentation, and do not connect a 5 V UART signal to the D1 mini.

## Build and monitor

1. Open this folder in VS Code with PlatformIO installed.
2. Build and upload the `d1_mini` environment.
3. Open the serial monitor at `115200` baud.

## Visualizer Wi-Fi

On startup, the device first tries saved home Wi-Fi credentials without opening a setup portal. If it cannot connect, it creates one access point named `LD2450-Visualizer`; connect to it and open `http://192.168.4.1`. The visualizer page includes a `Wi-Fi setup` link that opens the WiFiManager configuration portal when needed.

When connected to home Wi-Fi, use `http://ld2450.local` or the device IP address shown by your router.

Output looks like:

```text
distance_m=1.247
```

The sensor's default UART speed is 256000 baud. The sketch uses the D1 mini's hardware UART permanently on D7/D8. A separate transmit-only software serial channel sends log output through the normal USB serial TX pin, so the monitor remains at 115200 without interrupting sensor reception.

The sketch parses the LD2450 detection frame (`AA FF 03 00 ... 55 CC`) and tracks up to three targets. The web visualizer shows a colored dot for each active target, and its fixed distance readout and history graph use the nearest target. A zero distance means no target is currently detected.

The LD2450 reports a 2D position for each target, not only a distance. `x_mm` is the target's left/right position relative to the sensor, and `y_mm` is its position along the sensor's forward axis. The display uses the LD2450's native lateral convention. The reported distance is calculated from those coordinates. Coordinates use a sign bit, which the sketch decodes before calculating the distance.