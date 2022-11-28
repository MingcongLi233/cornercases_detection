# cornercases_detection
[![Python 3.9+](https://img.shields.io/badge/python-3.9+-blue.svg)](https://www.python.org/downloads/release/python-390/)
[![Pytorch 1.10.2](https://img.shields.io/badge/pytorch-1.10.2-blue.svg)](https://pytorch.org/)

Python implementation of the paper:
- 'CODA: A Real-World Road Corner Case Dataset for Object Detection in Autonomous Driving'([link](https://arxiv.org/pdf/2203.07724.pdf))

Point cloud segmentation algorithem is inspired by the paper:
- 'Fast Range Image-Based Segmentation of Sparse 3D Laser Scans for Online Operation'([link](http://www.ipb.uni-bonn.de/pdfs/bogoslavskyi16iros.pdf))
- code in python ([link](https://github.com/Likarian/python-pointcloud-clustering))

## Requirements
- Python 3.9.13
- PyTorch 1.10.2
- Numpy 1.23.3 
- Open3d 0.15.1
- Matplotlib 3.6.0

## Pipeline
### 1.Point Clustering
**- Preprocessing**

The goals for this step are:

1.Transform the point cloud file form. The point cloud files in KITTI are changed from .bin into .npy range image or changing .pcd file into .npy range image.

2.Downsampling and outliner points removal.To reduce the computation consumption, the point cloud was downsampled; the outliner points were deleted to increase the performance of the clustering algorithm.

**- Ground removal**

To reduce the disturbance of ground points on clustering, a function based on Ransac was applied to the points cloud.

**- Clustering**



#### The first step result
![clustering illustration](doc/point_clustering.jpg)

### 2.Semantic Segmentation
### 3.Object Detection

## Quick Start
### Available Point Cloud Segmentation Algorithem
- Fast Range
- DbSCAN



