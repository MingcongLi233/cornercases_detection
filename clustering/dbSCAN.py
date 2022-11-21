import open3d as o3d
import matplotlib.pyplot as plt
import numpy as np

def dbSCAN(xyz_points,epsilon,min_points,vis):
    """
    clustering methede by DbSCAN
    """

    pcd = o3d.utility.Vector3dVector(xyz_points)
    pcd= o3d.geometry.PointCloud(pcd)

    with o3d.utility.VerbosityContextManager(
            o3d.utility.VerbosityLevel.Debug) as cm:
        labels = np.array(
            pcd.cluster_dbscan(eps=epsilon, min_points=min_points, print_progress=True)) # ToDo

    max_label = labels.max()
    print(f"point cloud has {max_label + 1} clusters")
    colors = plt.get_cmap("tab20")(labels / (max_label if max_label > 0 else 1))
    colors[labels < 0] = 0
    pcd.colors = o3d.utility.Vector3dVector(colors[:, :3])
    if vis==True:
        o3d.visualization.draw_geometries([pcd],
                                        zoom=0.35,
                                        front=[-0.4999, -0.1659, -0.8499],
                                        lookat=[2.1813, 2.0619, 2.0999],
                                        up=[0.1204, -0.9852, 0.1215])
    