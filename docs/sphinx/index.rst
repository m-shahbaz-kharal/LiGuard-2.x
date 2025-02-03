.. LiGuard documentation master file, created by
   sphinx-quickstart on Fri Nov 15 03:13:49 2024.
   You can adapt this file completely to your liking, but it should at least
   contain the root `toctree` directive.

Welcome to LiGuard's documentation!
===================================
`LiGuard` is a research-purposed framework for LiDAR (and corresponding image) data. It provides an easy-to-use graphical user interface (GUI) that helps researchers interactively create algorithms by allowing them to dynamically create, enable or disable components, adjust parameters, and instantly visualize results.

`LiGuard` features, out of the box, data reading for many common dataset formats including support for reading calibration and label data. Moreover, it provides (an increasing list of) commonly used algorithm components ranging from basic data preprocessors to advanced object detection and tracking algorithms. Additionally, it establishes a straightforward standard for adding custom functions/algorithms, allowing users to integrate unique components into their pipelines. Pipelines created in `LiGuard` are saved in structured directories, making it easy to share and reproduce results.



.. toctree::
   :maxdepth: 2
   :caption: Contents:

   README
   visualizer_key_bindings
   liguard_pipelines
   supported_deep_detectors
   supported_sensors
   algo_auxiliary

.. toctree::
   :maxdepth: 1
   :caption: Module Docs:

   modules


Indices and tables
==================

* :ref:`genindex`
* :ref:`modindex`
* :ref:`search`
