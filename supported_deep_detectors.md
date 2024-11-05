# Supported Deep Object Detectors
`LiGuard` supports various deep object detectors. To use these detectors, you need to install the `LiGuard` with either `deep-detectors-cuda` or `deep-detectors-cpu` extra tag.

To install `LiGuard` with `deep-detectors-cuda` and `cuda` support, run:
```bash
pip install torch==1.13.1+cu117 torchvision==0.14.1+cu117 torchaudio==0.13.1 --extra-index-url https://download.pytorch.org/whl/cu117
pip install LiGuard[deep-detectors-cuda]
```
or to install `LiGuard` with cpu support only, run:
```bash
pip install LiGuard[deep-detectors-cpu]
```

The following table lists all the supported deep object detectors and their corresponding extra dependencies:

## Image Object Detectors
| Detector | Extra Dependencies |
|----------|--------------------|
| YOLOv5 (cuda)  | `torch==1.13.1+cu117` |
| YOLOv5 (cpu)   | `torch==1.13.1` |

## Point Cloud Object Detectors
| Detector | Extra Dependencies |
|----------|--------------------|
| PointPillars (cuda) | `torch==1.13.1+cu117` |
| PointPillars (cpu)  | `torch==1.13.1` |