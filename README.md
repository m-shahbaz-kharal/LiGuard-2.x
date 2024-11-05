# Introduction
`LiGuard` is a framework for interactive research on LIDAR (and accompanying image) data focusing on intelligent transportation system (ITS) applications. It provides a graphical user interface (GUI) that facilitates the creation of LIDAR data processing pipelines, enabling users to interactively enable, disable, or adjust function parameters and visualize the results.

`LiGuard` provides, out of the box, data readers for many common dataset formats including support for reading calibration and label data. Moreover, it provides built-in features ranging from basic data preprocessors to advanced object detection and tracking algorithms. Additionally, it offers a straightforward standard for adding custom functions or algorithms, allowing users to integrate unique components into their pipelines. Your contributions and collaborations are encouraged! Please visit [our GitHub repository](https://github.com/m-shahbaz-kharal/LiGuard-GUI) for more information.

# Installation
Requirements:
- Windows 10 or later
- Python 3.8, 3.9, or 3.10

Install `LiGuard` with pip (from PyPI):
```bash
pip install LiGuard
```

* `LiGuard` also support various deep object detectors, see [Supported Deep Object Detectors](supported_deep_detectors.md) for more information.
* `LiGuard` also supports various sensors to support live data streaming, see [Supported Sensors](supported_sensors.md) for more information.


Run interactive `LiGuard` GUI:
```bash
liguard-gui
```

or run `LiGuard` bulk data processor using specific configuration (created using `liguard-gui`):
```bash
liguard-blk path/to/config.yml
```

# Getting Started

Create a simple LIDAR data processing pipeline:
1. In the `Configuration` windows, click the `New` button to create a new configuration.
2. Under the `data` dropdown:
    1. Select the `main_dir` (aka root directory) by clicking the `...` button next to it and click open.
    2. Enter the name of `lidar_subdir` in the provided input field just below the `main_dir` input field.
    3. Under the `lidar` dropdown, make sure the `enabled` checkbox is checked and the `pcd_type` matches the file extention (for example, `.bin`, `.pcd`, or `.npy`).
3. Click `Save As` button to save the configuration.
4. Click `Apply` to see the LIDAR data visualization.
5. Under the `proc`->`lidar`->`crop` dropdown in the `Configuration` window, check the `enabled` checkbox.
6. Click `Apply` to see the cropped LIDAR data.
7. Explore various other built-in functions in the configuration window to familiarize yourself with additional features.

# Documentation
The documentation for `LiGuard` is available at [GitHub Pages](https://m-shahbaz-kharal.github.io/LiGuard).

# License
Distributed under the MIT License. See [LICENSE](LICENSE.txt) for more information.