**LiGuard: A Research-Purposed Framework for Processing LIDAR and Image Data**

[Introduction](#introduction) | [Installation](#installation) | [Usage](#usage) | [Documentation](#documentation) | [Contributing](#contributing) | [License](#license)

# Introduction
`LiGuard` is a research-purposed framework for LiDAR (and corresponding image) data. It provides an easy-to-use graphical user interface (GUI) that helps researchers interactively create algorithms by allowing them to dynamically create, enable or disable components, adjust parameters, and instantly visualize results.

`LiGuard` features, out of the box, data reading for many common dataset formats including support for reading calibration and label data. Moreover, it provides (an increasing list of) commonly used algorithm components ranging from basic data preprocessors to advanced object detection and tracking algorithms. Additionally, it establishes a straightforward standard for adding custom functions/algorithms, allowing users to integrate unique components into their pipelines. Pipelines created in `LiGuard` are saved in structured directories, making it easy to share and reproduce results.

![LiGuard Main Interface](../figs/liguard-main.png)
*LiGuard's GUI Layout (from left to right): Configuration Window, Visualization Windows (Point Cloud Feed and Image Feed), and Log Window.*

# Installation
Requirements:
- Windows 10 or later
- Python 3.8, 3.9, 3.10, or 3.11

Install `LiGuard` with pip (from PyPI):
```bash
pip install LiGuard
```

Run `LiGuard` by executing the following command in the terminal:
```bash
liguard-gui
```
# Usage
Test an example pipeline:

1. In the `Configuration` windows, click the `Open` button.
2. Navigate to `examples/simple_pipeline`, click open, and then click `Apply`.
3. Explore various functions under `proc` dropdown in the `Configuration` window. For example, under `proc/lidar/crop`, check the `enabled` checkbox, and click `Apply` to see the cropped LIDAR data.
4. Press `left arrow` or `right arrow` to navigate through the frames. A list of all key bindings is available [here](visualizer_key_bindings.md).
5. To save the pipeline, click the `Save` button in the `Configuration` window.

For more details on pipelines, see [LiGuard Pipelines](liguard_pipelines.md).

# Documentation
A detailed documentation for `LiGuard` is available at [GitHub Pages](https://m-shahbaz-kharal.github.io/LiGuard-2.x).

# Contributing
We welcome contributions to the `LiGuard` framework. Please follow the guidelines below to contribute to the framework:
- Fork the repository.
- Create a new branch for your feature or bug fix.
- Make your changes and add comments.
- Write tests for your changes.
- Run the tests.
- Create a pull request.

# License
MIT License Copyright (c) 2024 Muhammad Shahbaz - see the [LICENSE](../../LICENSE.txt) file for details.