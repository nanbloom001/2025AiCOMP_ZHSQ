import numpy
import cv2
import ultralytics
from stitching import Stitcher

print(f"Numpy version: {numpy.__version__} (应为 1.x)")
print(f"OpenCV version: {cv2.__version__}")
print("环境验证通过！YOLO 和 Stitching 均可运行。")