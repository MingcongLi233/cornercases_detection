from matplotlib.pyplot import imshow
import open3d as o3d
import numpy as np

def readDepth(input):
    depth_raw = o3d.io.read_image(input)
    depth = np.asarray(depth_raw)
    imshow(depth)
    return depth


