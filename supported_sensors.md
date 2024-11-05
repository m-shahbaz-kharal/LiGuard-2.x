# Supported Sensors
`LiGuard` supports various LIDAR and RGB sensors. To use these sensors, you need to install the `LiGuard` with `sensor-sdks` extra dependencies.

To install `LiGuard` with `sensor-sdks` support, run:
```bash
pip install LiGuard[sensor-sdks]
```

The following table lists the supported sensors and their corresponding extra dependencies:

## RGB Sensors
| Detector | Extra Dependencies |
|----------|--------------------|
| Spinnaker Cameras   | `spinnaker-python` |

## LIDAR Sensors
| Detector | Extra Dependencies |
|----------|--------------------|
| Ouster LIDARs | `ouster-sdk==0.12.0` |