import numpy as np
import open3d as o3d

from .plane_removal import ground_segmentation

def preprocess(lid_path,voxel_size,nb_points,radius):
    """
    1.check the input file type
    2.tranfrom .bin file to .npy file
    3.downsampling and outliers removal
    """
    # check the file type
    if lid_path.endswith(".bin"):
        pcd_arr = bin2pcd(lid_path,voxel_size,nb_points,radius)
    elif lid_path.endswith(".pcd"):
        pcd_arr = o3d.io.read_point_cloud(lid_path)
        pcd_arr = np.asarray(pcd_arr.points)
    else:
        print("wrong input!")


    return pcd_arr  # .npy file

def bin2pcd(lid_path,voxel_size,nb_points,radius):
    # Load binary point cloud
    bin_pcd = np.fromfile(lid_path, dtype=np.float32) #ToDo

    # Reshape and drop reflection values
    points = bin_pcd.reshape((-1,4))[:,0:3]

    # Convert to Open3D point cloud
    o3d_pcd = o3d.geometry.PointCloud(o3d.utility.Vector3dVector(points))


    # pcd Preprocessing
    # downsample 
    voxel_down_pcd = o3d_pcd.voxel_down_sample(voxel_size=voxel_size)

    # Radius oulier removal
    voxel_down_pcd.remove_radius_outlier(nb_points=nb_points, radius=radius)

    pcd_arr = np.asarray(voxel_down_pcd.points)

    return pcd_arr  # .npy
