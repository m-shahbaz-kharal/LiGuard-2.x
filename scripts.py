import subprocess
import sys

def run_if_deep_detectors_cuda():
    # Only run if extra-1 is specified
    if 'deep-detectors-cuda' in sys.argv:
        subprocess.run("git clone https://github.com/zhulf0804/PointPillars algo/nn/PointPillars", shell=True, check=True)
        subprocess.run("cd algo/nn/PointPillars/ops && python setup.py develop", shell=True, check=True)