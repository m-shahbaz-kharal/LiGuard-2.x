from setuptools import setup, find_packages

def install_pointpillars_if_deep_detectors_cuda_is_installed():
    import subprocess
    import sys
    
    if 'deep-detectors-cuda' in sys.argv:
        subprocess.run("git clone https://github.com/zhulf0804/PointPillars algo/nn/PointPillars", shell=True, check=True)
        subprocess.run("cd algo/nn/PointPillars/ops && python setup.py develop", shell=True, check=True)

setup(
    name="LiGuard",
    version="2.1.0",
    author="Muhammad Shahbaz",
    author_email="m.shahbaz.kharal@outlook.com",
    description=(
        "A research-purposed, GUI-powered, Python-based framework that allows easy "
        "development of dynamic point-cloud (and accompanying image) data processing pipelines."
    ),
    long_description=open("README.md").read(),
    long_description_content_type="text/markdown",
    url="https://github.com/m-shahbaz-kharal/LiGuard-2.1",
    classifiers=[
        "Programming Language :: Python :: 3.8",
        "Programming Language :: Python :: 3.9",
        "Programming Language :: Python :: 3.10",
        "Programming Language :: Python :: 3.11",
        "License :: OSI Approved :: MIT License",
        "Operating System :: Microsoft :: Windows :: Windows 10",
        "Operating System :: Microsoft :: Windows :: Windows 11",
        "Topic :: Scientific/Engineering :: Image Processing",
        "Topic :: Scientific/Engineering :: Visualization",
    ],
    keywords="point-cloud, image, processing, pipeline, dynamic, GUI, research, development, framework",
    packages=find_packages(),
    python_requires=">=3.8, <=3.11",
    install_requires=[
        "keyboard==0.13.5",
        "numpy==1.26.4",
        "open3d==0.18.0",
        "opencv-python==4.9.0.80",
        "pillow==10.3.0",
        "pyyaml==6.0.1",
        "scipy==1.12.0",
    ],
    cmdclass={
        'install': install_pointpillars_if_deep_detectors_cuda_is_installed,
    },
    extras_require={
        "sensor-sdks": [
            "ouster-sdk==0.12.0",
            "spinnaker-python==4.0.0.116",
        ],
        "deep-detectors-cuda": [
            "torch==1.13.1+cu117",
            "torchvision==0.14.1+cu117",
            "ultralytics==8.2.78",
        ],
        "deep-detectors-cpu": [
            "torch==1.13.1",
            "torchvision==0.14.1",
            "ultralytics==8.2.78",
        ],
        "dev": [
            "pytest==8.1.1",
            "sphinx==7.2.6",
            "sphinx-rtd-theme==2.0.0",
        ],
    },
    entry_points={
        "console_scripts": [
            "liguard-gui=liguard_gui:main",
            "liguard-blk=bulk_process:main",
        ]
    },
)
