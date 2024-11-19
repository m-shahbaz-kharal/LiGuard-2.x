---
title: 'LiGuard: Interactively and Rapidly Create Point-Cloud and Image Processing Pipelines'

tags:
  - Python
  - GUI
  - Open3D
  - OpenCV
  - lidar
  - Point-Cloud Processing
  - Image Processing
  - Visualization
authors:
  - name: Muhammad Shahbaz
    orcid: 0009-0003-6377-8274
    affiliation: 1
  - name: Shaurya Agarwal
    orcid: 0000-0001-7754-6341
    affiliation: 1
affiliations:
  - name: University of Central Florida, USA
    index: 1
date: 16 April 2024
bibliography: paper.bib

---
# Summary
There is a growing interest in the development of lidar-based applications in domains like robotics, autonomous driving, traffic monitoring, infrastructure inspection, and many areas of envrionmental surveying and mapping. In many cases, these applications rely on solutions that are not end-to-end but rather a sequential combination (a pipeline) of individual data processing functions including data readers, pre-processors, application specific algorithms/models, post-processors, and visualizers, etc. Creating such multi-step solutions often require extensive revisiting of the proposed processing pipeline, including enabling/disabling certain functions (for ablation studies), fine-tunning parameters, and even completely replacing parts of the solution. Such revisions pose challenges on the speed of research and even on the reuse of it due to interdependencies among functions. To address this issue, `LiGuard` is presented. `LiGuard` is a software framework that provides an easy-to-use graphical user interface (GUI) that helps researchers rapidly build and test their point cloud (and corresponding image) processing pipelines. It allows them to dynamically add/remove/reorder functions, adjust parameters of those functions, and visualize results, in a live and interactive manner compared to other packages where such modifications often require re-executing the program script. Moreover, because it creates all the meta files in strucutured directories, it allows easy sharing of the created pipelines or the individual functions used in those pipelines.

`LiGuard` features, out of the box, data reading for many common dataset formats including support for reading calibration and label data. Moreover, it provides (an increasing list of) commonly used algorithm components ranging from basic data preprocessors to advanced object detection and tracking algorithms. Additionally, it establishes a straightforward standard for adding custom functions/algorithms, allowing users to integrate unique components into their pipelines. For latest updates on supported features, please see [LiGuard’s documentation](https://m-shahbaz-kharal.github.io/LiGuard-2.x/).

Note: `LiGuard` is built on many standard python libraries and packages for data processing and visualization including Open3D [@Zhou2018], OpenCV [@opencv_library], Numpy [@harris2020array], and SciPy [@2020SciPy]. `LiGuard` is designed to be used interactively and is therefore not ideal for processing very large datasets. We provide a utility [`liguard_cmd`](https://github.com/m-shahbaz-kharal/LiGuard-2.x/blob/dev_2.x/liguard/liguard_cmd.py) that processes data slightly faster by removing GUI and implementing multithreading but since the data is still processed sequentially the output generation is as fast as the slowest step in the pipeline.

# Statement of Need
Recent advancements in LiDAR technology have significantly broadened its applicability across various domains, making it a potential sensor for precise measurement. Innovations such as solid-state LiDAR and nanophotonics-based devices have enhanced the performance, reliability, and cost-effectiveness of these sensors, enabling their use in diverse fields including autonomous driving, environmental monitoring, and industrial automation [@10.1002/lpor.202100511]. Additionally, the integration of advanced optics and beam steering technologies has improved the spatial resolution and operational efficiency of LiDAR systems, facilitating their deployment in complex environments for applications ranging from agriculture to urban planning ([@10.3390/s22239293]; [@10.3390/mi13060894]). As the lidar sensors are becoming more and more mainstream, there is an increasing need of software tools that help standardize rapid research and discovery, and allow easy reproducibility of the experiments.

`LiGuard` fulfills a critical need for interactive research in lidar data processing, where iterative adjustments and refinements to processing pipelines are essential and are otherwise time consuming to . While it is not optimized for large-scale datasets such as pointcloudset [@Goelles2021] or focus operating system wide utility and integeration such as ROS [@ros], its user-friendly interface allows researchers to dynamically modify and visualize their workflows, facilitating rapid experimentation and adaptation. PDAL [@pdal_contributors_2024_2616780], a notable library for creating point cloud processing pipelines, though offers much more versatile control over pipelines by providing abstract access, filtering, exploitation, and workflow management capabilities, it lacks interactivity and live parameter tuning which is often crucial in fine-tuning multi-step pipelines. PCL [@Rusu_ICRA2011_PCL], Open3D [@Zhou2018], and other [@brown2012laspy; @pyntcloud; @Hunter:2007; @Lamprecht2019], etc. are low-level libraries that offer fine access to underlying data and are designed to be used as base for the development of versatile 3D applications (`LiGuard` is primarily built on Open3D for its GUI and 3D visualizations needs).

`LiGuard` segregates the data processing pipeline from the application logic, making it a modular, reusable, and extensible framework. It is, at its core, a combination of five sub-modules, (1) Data I/O, (2) Process Configuration, (3) Inter-Process Data Sharing, (4) Data Processing, and (5) Visualization. During its development, a great focus is put towards abstracting away the tasks that foster duplication of efforts, therefore, Data I/O, Inter-Process Data Sharing, and Visualization sub-modules are developed in such a way that they, although allow modifications from development contributors, are running behind the curtains for researchers to relieve them of the burden of reading data efficiently, creating logic to keep the processing as interactive as possible (often requiring understanding of multi-threading), and visualizing (to some extent the) outcomes. This, in turn, allows researchers to focus on two main things of their point-cloud data processing pipelines, the processes they need to execute on the data and the configuration of those processes. `LiGuard` facilitates this by (a) providing YAML formatted configuration files that are later loaded into GUI allowing easy manipulations of the parameters, and (b) allowing researchers to directly (from GUI) create corresponding (to configuration) functions that are automatically called along with configuration parameters and data.

# Architecture
![`LiGuard`'s Architecture\label{fig:liguardarch}](figs/block_diagram.png){ width=80% }

`LiGuard` is built on top of the Open3D [@Zhou2018] and OpenCV [@opencv_library] libraries allowing researchers and contributors to leverage the extensive functionalities provided by these libraries. A high-level architecture is presented in \autoref{fig:liguardarch} consisting of five main components: (1) GUI (purple), (2) Data Handlers (green), (3) Shared Configuration Dictionary (blue), (4) Shared Data Dictionary (orange), and (5) Research Algorithms. The GUI component is responsible for creating an interface for researchers to interact with the framework and for visualizations of both lidar and image data. The Data Handlers component is responsible for reading data from disk/sensor(s). The Shared Configuration Dictionary and Shared Data Dictionary components are responsible for sharing configuration and data, respectively, between different components of the framework. The Research Algorithms component is responsible for implementing the algorithms that process the data. The darker blue rectangular box at the top shows the `pipeline directory` where user created data handler(s) and algo(rithms) reside along all the meta files (.yml). Please note that the research algorithms are analogous to the functions mentioned in [Section: Summary](#summary).

# Usage
A tutorial with technical details is available under [`Usage`](https://github.com/m-shahbaz-kharal/LiGuard-2.x/tree/dev_2.x?tab=readme-ov-file#usage) at our [`Github`](https://github.com/m-shahbaz-kharal/LiGuard-2.x) repository. There, a very simple example pipeline is added (contains only 5 frames of lidar and image data to save space).

# Contributions
M.S and S.A. conceived the idea of `LiGuard`. M.S. developed the framework, wrote the documentation, and wrote the manuscript. S.A. reviewed the manuscript and provided feedback. Both authors have read and agreed to the published version of the manuscript.

# Acknowledgements
We thank [Dr. Karan Sikka](https://www.ksikka.com) for his great support and guidance throughout the development of `LiGuard`.

# References
