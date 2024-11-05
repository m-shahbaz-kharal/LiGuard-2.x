# Introduction
`LiGuard` is a framework for interactive research on LIDAR (and accompanying image) data focusing on intelligent transportation system (ITS) applications. It provides a graphical user interface (GUI) that facilitates the creation of LIDAR data processing pipelines, enabling users to interactively enable, disable, or adjust function parameters and visualize the results.

`LiGuard` provides, out of the box, data readers for many common dataset formats including support for reading calibration and label data. Moreover, it provides built-in features ranging from basic data preprocessors to advanced object detection and tracking algorithms. Additionally, it offers a straightforward standard for adding custom functions or algorithms, allowing users to integrate unique components into their pipelines. Your contributions and collaborations are encouraged! Please visit [our GitHub repository](https://github.com/m-shahbaz-kharal/LiGuard-2.x) for more information.

# Quick Start
Requirements:
- Windows 10 or later
- Python 3.8, 3.9, or 3.10

Install `LiGuard` with pip (from PyPI):
```bash
pip install LiGuard
```

Run `LiGuard` by executing the following command in the terminal:
```bash
liguard-gui
```
Test an example pipeline:
1. In the `Configuration` windows, click the `Open` button.
2. Navigate to `examples/pipelines` and open `sample_pipeline.yml`.
3. Explore various functions under `proc` dropdown in the `Configuration` window. For example, under `proc/lidar/crop`, check the `enabled` checkbox, and click `Apply` to see the cropped LIDAR data.
4. Press `left arrow` or `right arrow` to navigate through the frames. A list of all key bindings is available [here](docs/visualizer_key_bindings.md).
5. To save the pipeline, click the `Save`/`Save As` button in the `Configuration` window.

To learn how to create advanced custom pipelines, see [Tutorial: Implementing A Point-Cloud Processing Pipeline](https://m-shahbaz-kharal.github.io/LiGuard-2.x/tutorial_your_first_pipeline.html).

# Documentation
A documentation for `LiGuard` is available at [GitHub Pages](https://m-shahbaz-kharal.github.io/LiGuard-2.x).

# License
Distributed under the MIT License. See [LICENSE](LICENSE.txt) for more information.