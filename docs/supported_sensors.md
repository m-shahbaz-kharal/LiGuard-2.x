# Supported Sensors
`LiGuard` supports various LIDAR and RGB sensors. To use these sensors, you need to install official their official SDKs and dependencies. The following table lists the supported sensors and their corresponding extra dependencies:

## RGB Sensors
| Detector | Extra Dependencies | How to install |
|----------|--------------------| ---------------|
| Spinnaker Cameras   | `spinnaker-python` | go to [this link](https://www.teledynevisionsolutions.com/support/support-center/software-firmware-downloads/iis/spinnaker-sdk-download/spinnaker-sdk--download-files/?pn=Spinnaker+SDK&vn=Spinnaker+SDK), and download and install the appropriate version of the Spinnaker SDK for your environment. |

To use the RGB sensor in your data processing pipeline, add the following entry under `sensors` in your pipeline configuration.
```yaml
camera: # camera sensor configurations, at this point only Flir cameras are supported, support for other cameras is coming soon
    enabled: True # set True to stream point clouds from sensor, please set False `data/camera` is enabled.
    hostname: '192.168.1.3' # sensor ip address or hostname (please see manufacturer instructions for more details)
    manufacturer: 'Flir' # sensor manufacturer
    model: 'BFS-PGE-16S2C-CS' # sensor model
    serial_number: '00000000' # sensor serial number
    camera_matrix: [1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0] # camera matrix (K)
    distortion_coeffs: [0, 0, 0, 0, 0] # distortion coefficients (D)
    T_lidar_camera: [[1, 0, 0, 0], [0, 1, 0, 0], [0, 0, 1, 0], [0, 0, 0, 1]] # 4x4 transformation matrix from camera to lidar
```

## LIDAR Sensors
| Detector | Extra Dependencies | How to install |
|----------|--------------------| ---------------|
| Ouster LIDARs | `ouster-sdk` | `pip install ouster-sdk` |

To use the LIDAR sensor in your data processing pipeline, add the following entry under `sensors` in your pipeline configuration.
```yaml
lidar: # lidar sensor configurations, at this point only Ouster lidars are supported, support for other lidars is coming soon
    enabled: True # set True to stream point clouds from sensor, please set False `data/lidar` is enabled.
    hostname: '192.168.1.2' # sensor ip address or hostname (please see manufacturer instructions for more details)
    manufacturer: 'Ouster' # sensor manufacturer
    model: 'OS1-64' # sensor model
    serial_number: '000000000000' # sensor serial number
```