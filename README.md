<p align="center">
    <img src="https://github.com/m-shahbaz-kharal/LiGuard-2.x/raw/dev_2.x/docs/figs/liguard-logo.png" alt="Description" width="400" />
</p>

<p align="center">
    A Research-Purposed Framework for Processing LIDAR and Image Data
</p>

<p align="center">
    <a href="https://m-shahbaz-kharal.github.io/LiGuard-2.x/README.html#installation">Installation</a> | <a href="https://m-shahbaz-kharal.github.io/LiGuard-2.x/README.html#usage">Usage</a> | <a href="https://m-shahbaz-kharal.github.io/LiGuard-2.x/">Documentation</a> | <a href="https://m-shahbaz-kharal.github.io/LiGuard-2.x/README.html#contributing">Contributing</a> | <a href="https://github.com/m-shahbaz-kharal/LiGuard-2.x/blob/dev_2.x/LICENSE.txt">License</a>
</p>

![PyPI Release Version](https://img.shields.io/pypi/v/liguard?label=release)
![Docs Build Status](https://img.shields.io/github/actions/workflow/status/m-shahbaz-kharal/LiGuard-2.x/build_and_deploy_sphinx_docs.yml?label=docs)
![Tests Run Status](https://img.shields.io/github/actions/workflow/status/m-shahbaz-kharal/LiGuard-2.x/run_tests.yml?label=tests)
![GitHub License](https://img.shields.io/github/license/m-shahbaz-kharal/LiGuard-2.x)

# Introduction
`LiGuard` is a research-purposed framework for LiDAR (and corresponding image) data. It provides an easy-to-use graphical user interface (GUI) that helps researchers interactively create algorithms by allowing them to dynamically create, enable or disable components, adjust parameters, and instantly visualize results.

`LiGuard` features, out of the box, data reading for many common dataset formats including support for reading calibration and label data. Moreover, it provides (an increasing list of) commonly used algorithm components ranging from basic data preprocessors to advanced object detection and tracking algorithms. Additionally, it establishes a straightforward standard for adding custom functions/algorithms, allowing users to integrate unique components into their pipelines. Pipelines created in `LiGuard` are saved in structured directories, making it easy to share and reproduce results.

![LiGuard Main Interface](https://raw.githubusercontent.com/m-shahbaz-kharal/LiGuard-2.x/refs/heads/dev_2.x/docs/figs/liguard-main.png)
*LiGuard's GUI Layout (from left to right): Configuration Window, Visualization Windows (Point Cloud Feed and Image Feed), and Log Window.*

# Installation
Requirements:
- OS
  - Windows 10 or later
  - macOS 10.14 Mojave or later
- Python 3.8, 3.9, 3.10, or 3.11

Install `LiGuard` with pip (from PyPI):
```bash
pip install LiGuard
```

Run `LiGuard` by executing the following command in the terminal:
```bash
liguard-gui
```

**Note to macOS Users:** When launching `liguard-gui` for the first time on macOS, `liguard-gui` may request accessibility permissions in order to control this computer. This is a requirement for the keybindings to function properly. To grant these permissions, go to **System Preferences > Privacy & Security > Accessibility** and toggle the button next to **"Terminal"**.

# Usage
Test an example pipeline:

1. In the `Configuration` windows, click the `Open` button.
2. Navigate to `examples/simple_pipeline`, click open, and then click `Apply`.
3. Explore various functions under `proc` dropdown in the `Configuration` window. For example, under `proc/lidar/crop`, check the `enabled` checkbox, and click `Apply` to see the cropped LIDAR data.
4. Press `left arrow` or `right arrow` to navigate through the frames. A list of all key bindings is available [here](https://github.com/m-shahbaz-kharal/LiGuard-2.x/blob/dev_2.x/docs/visualizer_key_bindings.md).
5. To save the pipeline, click the `Save` button in the `Configuration` window.

For more details on pipelines, see [LiGuard Pipelines](https://github.com/m-shahbaz-kharal/LiGuard-2.x/blob/dev_2.x/docs/liguard_pipelines.md).

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
MIT License Copyright (c) 2024 Muhammad Shahbaz - see the [LICENSE](https://github.com/m-shahbaz-kharal/LiGuard-2.x/blob/dev_2.x/LICENSE.txt) file for details.