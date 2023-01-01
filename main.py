import argparse

from clustering.preprocessing import preprocess
from clustering.plane_removal import ground_segmentation
from clustering.fast_range import fast_range_clustering
from clustering.dbSCAN import dbSCAN



if __name__ == "__main__":
    # parser
    parser = argparse.ArgumentParser()
    #main
    parser.add_argument('--lid_path', type=str, default=".\input\dataset\kitti.bin")
    parser.add_argument('--img_path', type=str, default=".\input\dataset\kitti.png")  
    parser.add_argument('--clustering_method', type=str, default='dbscan')

    #preprocessing
    parser.add_argument('--voxel_size', type=float, default=0.05)
    parser.add_argument('--nb_points', type=int, default=32)
    parser.add_argument('--radius', type=float, default=0.05)

    #pcd2range
    parser.add_argument('--width', type=int, default=1080)
    parser.add_argument('--height', type=int, default=64)

    #dbSCAN
    parser.add_argument('--epsilon', type=float, default=0.25)
    parser.add_argument('--min_points', type=int, default=20)

    #plane_removal
    parser.add_argument('--iters', type=int, default=100)

    #fast_range
    parser.add_argument('--init_angle', type=float, default=5.67)
    parser.add_argument('--change_rate', type=float, default=0.33)
    parser.add_argument('--horizontaltheta', type=float, default=30)
    parser.add_argument('--horizontalangle', type=float, default=0.1)
    parser.add_argument('--verticaltheta', type=float, default=20)
    parser.add_argument('--verticalangle', type=float, default=10)

    #visualization
    parser.add_argument('--vis', type=bool, default=True)

    args = parser.parse_args()
    

    # 1. pre-processing
    pcd_arr = preprocess(args.lid_path, args.voxel_size, args.nb_points, args.radius)

    # 2. GPR
    _, seg_cloud = ground_segmentation(pcd_arr, args.iters)
    
    # Clsutering
    if args.clustering_method == 'fastrange':
        fast_range_clustering(
        seg_cloud, args.width, args.height, args.init_angle, args.change_rate,
        args.horizontaltheta, args.horizontalangle, args.verticaltheta, 
        args.verticalangle, args.vis, args.img_path
        )
        # func(seg_cloud, vis=True, save_fig=True, )
    elif args.clustering_method == 'dbscan':
        dbSCAN(seg_cloud, args.epsilon, args.min_points, args.vis)
    else:
        print("wrong methd input!")






