from setuptools import setup, find_packages
import glob

setup(
    name="LiGuard",
    version="2.1.2",
    author="Muhammad Shahbaz",
    author_email="m.shahbaz.kharal@outlook.com",
    description=(
        "A research-purposed, GUI-powered, Python-based framework that allows easy "
        "development of dynamic point-cloud (and accompanying image) data processing pipelines."
    ),
    long_description=open("README.md").read(),
    long_description_content_type="text/markdown",
    url="https://github.com/m-shahbaz-kharal/LiGuard-2.x",
    classifiers=[
        "Programming Language :: Python :: 3.8",
        "Programming Language :: Python :: 3.9",
        "Programming Language :: Python :: 3.10",
        "Programming Language :: Python :: 3.11",
        "Development Status :: 3 - Alpha",
        "Environment :: Win32 (MS Windows)",
        "Environment :: X11 Applications",
        "Intended Audience :: Developers",
        "Intended Audience :: Education",
        "Intended Audience :: Other Audience",
        "Intended Audience :: Science/Research",
        "License :: OSI Approved :: MIT License",
        "Natural Language :: English",
        "Operating System :: Microsoft :: Windows",
        "Operating System :: POSIX :: Linux",
        "Topic :: Scientific/Engineering :: Image Processing",
        "Topic :: Scientific/Engineering :: Visualization",
        "Topic :: Multimedia :: Graphics :: Viewers",
        "Topic :: Utilities",
    ],
    keywords="point-cloud, image, processing, pipeline, dynamic, GUI, research, development, framework",
    python_requires=">=3.8",
    install_requires=[
        "pynput",
        "numpy<2.0.0",
        "open3d==0.18.0",
        "opencv-python",
        "pillow",
        "pyyaml",
        "scipy",
        "matplotlib"
    ],
    extras_require={
        "dev": [
            "pytest",
            "sphinx",
            "sphinx-rtd-theme",
        ],
    },
    packages=find_packages(),
    include_package_data=True,
    entry_points={
        "console_scripts": [
            "liguard-gui=liguard.liguard_gui:main",
            "liguard-cmd=liguard.liguard_cmd:main",
            "liguard-profiler=liguard.liguard_profiler:main",
        ]
    },
)