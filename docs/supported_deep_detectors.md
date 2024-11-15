# Supported Deep Object Detectors (Experimental)
`LiGuard` allows users to add various pre-trained LIDAR and/or RGB deep object detectors into their data processing pipelines. Each deep detector usually has its dependencies. It is upto the users to install the dependencies of their cusotm deep learning algorithms. In the following, a list of currently supported (out-of-box) deep object detectors is given along their corresponding installation instructions.

## Image Object Detectors
### YOLOv5:
```bash
pip install -U torch==1.13.1+cu117 torchvision==0.14.1+cu117 --extra-index-url https://download.pytorch.org/whl/cu117
pip install -U ultralytics==8.2.78
```
Once installation is complete, add the following entry under `proc/camera/` in `base_config.yml` file inside your pipeline directory.
```yaml
UltralyticsYOLOv5:
    enabled: True
    priority: 1 # priority of process - lower is higher
    model: 'yolov5s' # can be yolov5s, yolov5m, yolov5l, yolov5x https://pytorch.org/hub/ultralytics_yolov5
    class_colors: # the classes you need to detect and their corresponding bbox colors in RGB format 
        Person: [1, 0, 0]
        Bicycle: [0, 1, 0]
        Car: [0, 0, 1]
        Motorcycle: [0, 1, 0]
        Bus: [0, 1, 1]
        Truck: [1, 0, 1]
    score_threshold: 0.5 # minimum score threshold for detection
```

## Point Cloud Object Detectors
### PointPillars:
```bash
pip install -U torch==1.13.1+cu117 torchvision==0.14.1+cu117 --extra-index-url https://download.pytorch.org/whl/cu117
git clone https://github.com/zhulf0804/PointPillars /path/to/liguard/algo/nn/PointPillars
pip install -U -r /path/to/liguard/algo/nn/PointPillars/requirements.txt
cd /path/to/liguard/algo/nn/PointPillars/ops && python setup.py develop
```
Once installation is complete, add the following entry under `proc/lidar/` in `base_config.yml` file inside your pipeline directory.
```yaml
PointPillarDetection:
    enabled: True
    priority: 1 # priority of process - lower is higher
    github_repo_dir: 'algo/nn/PointPillars' # clone https://github.com/zhulf0804/PointPillars to this path and install the requirements
    ckpt_file: 'algo/nn/PointPillars/pretrained/epoch_160.pth' # path to checkpoint file
    score_threshold: 0.5 # minimum score threshold for detection
    # Pedestrians are colored Red, Cars are colored Green, Cyclists are colored Blue
```
