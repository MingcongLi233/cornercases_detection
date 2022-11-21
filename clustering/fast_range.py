"""
The Code is inspired by https://github.com/Likarian/python-pointcloud-clustering
"""
from matplotlib import test
import numpy as np
from .pcd2range import pcd2range
import math
import open3d
import datetime



def fast_range_clustering (npy_file, width, height, init_angle, change_rate, 
horizontaltheta, horizontalangle, verticaltheta, verticalangle,vis):
    """
    Unsupervised point-cloud clustering and show the result
    """
    range_image = pcd2range(npy_file,width,height)

    depth = np.squeeze(range_image)*1000 # keep the same distance ratio

    SegTool =RangeImageLabeling()

    tic = datetime.datetime.now()
    SegResult = SegTool.LabelRangeImage(RangeImage=depth, HorizontalTheta=horizontaltheta, HorizontalAngle=horizontalangle, VerticalTheta=verticaltheta, VerticalAngle=verticalangle ) # ToDo
    tok = datetime.datetime.now()
    print(tok-tic)

    segnum = np.unique(SegResult).shape[0]
    label_colours = np.random.randint(255,size=(segnum+1,3))/255

    im_target_rgb = np.array([label_colours[ c ] for c in SegResult])
    im_target_rgb = im_target_rgb.astype( np.uint8 )

    def custom_draw_geometry_with_custom(pcd):
        vis = open3d.visualization.Visualizer()
        vis.create_window()
        vis.add_geometry(pcd)
        ctr = vis.get_view_control()
        ctr.set_front([ 0.89452025962235249, -0.18810349064390619, 0.40552506942572236 ])
        ctr.set_lookat([ -4803.5406166994117, 2578.7673925914692, 1502.733219030637 ])
        ctr.set_up([ -0.39927129067518657, 0.071776107780855442, 0.91401894225141844 ])
        ctr.set_zoom(0.16)

        vis.run()
        vis.destroy_window()


    ent_pc = depth

    ent_num = np.sum(ent_pc>0)
    ent_q = np.zeros( (ent_num, 3) )
    ent_color = np.zeros( (ent_num, 3) )
    ent_deter = 0

    for i in range(ent_pc.shape[0]):
        for j in range(ent_pc.shape[1]):

            # ToDo
            r1 = math.cos(math.radians(init_angle-change_rate*i))*math.cos(math.radians(j*0.25)) * ent_pc[i, j]
            r2 = math.cos(math.radians(init_angle-change_rate*i))*math.sin(math.radians(j*0.25)) * ent_pc[i, j]
            r3 = math.sin(math.radians(init_angle-change_rate*i)) * ent_pc[i, j]

            if r1+r2+r3 == 0:
                pass
            else:
                ent_q[ent_deter, 0] = r1
                ent_q[ent_deter, 1] = r2
                ent_q[ent_deter, 2] = r3
                ent_color[ent_deter, :] = label_colours[SegResult[i, j]]

                ent_deter += 1



    pcd = open3d.geometry.PointCloud()
    pcd.points = open3d.utility.Vector3dVector(ent_q)
    pcd.colors = open3d.utility.Vector3dVector(ent_color)
    if vis==True:
        image=custom_draw_geometry_with_custom(pcd)
        # return image


class RangeImageLabeling( object ):
    def __init__(self):
        self.Label = 1
        self.queue = list()
    
    def LabelRangeImage(self, RangeImage, HorizontalTheta, HorizontalAngle, VerticalTheta, VerticalAngle):
        self.RangeImage = RangeImage
        self.Rrow = self.RangeImage.shape[0]
        self.Rcol = self.RangeImage.shape[1]
        self.L = np.zeros( self.RangeImage.shape )
        defect = 1*(self.RangeImage == 0)
        self.L = self.L - defect
        
        # change to radian
        self.Htheta = np.deg2rad(HorizontalTheta)
        self.HorizontalAngle = np.deg2rad(HorizontalAngle)
        self.Vtheta = np.deg2rad(VerticalTheta)
        self.VerticalAngle = np.deg2rad(VerticalAngle)

        for row in range(self.Rrow):
            for col in range(self.Rcol):
                if self.L[row,col] == 0:
                    self.LabelComponentBFS(row, col)
                    self.Label += 1

        return self.L.astype(np.int16)

    def LabelComponentBFS( self, r, c ):
        self.queue.append([r, c])
        while len(self.queue) != 0:
            target = self.queue.pop(0)
            target_r = target[0]
            target_c = target[1]
            if self.L[target_r, target_c] == 0:
                self.L[target_r, target_c] = self.Label
                RangeCenter = self.RangeImage[target_r, target_c]

                if RangeCenter > 0:
                    RowNeighborList, ColNeighborList = self.GetNeighbor( target_r, target_c )

                    if len(RowNeighborList) > 0:

                        for RowNeighbor in RowNeighborList:
                            row_neighbor_r = RowNeighbor[0]
                            row_neighbor_c = RowNeighbor[1]

                            RangeNeighbor = self.RangeImage[row_neighbor_r, row_neighbor_c]
                            
                            d1 = max(RangeCenter, RangeNeighbor)
                            d2 = min(RangeCenter, RangeNeighbor)

                            ValueArcTan = np.arctan(( d2 * np.sin(self.HorizontalAngle * (row_neighbor_r - target_r)) ) / ( d1 - d2 * np.cos(self.HorizontalAngle * (row_neighbor_r - target_r)) ))
                            if ValueArcTan > self.Htheta:
                                if [row_neighbor_r, row_neighbor_c] in self.queue:
                                    pass
                                else:
                                    self.queue.append([row_neighbor_r, row_neighbor_c])

                    if len(ColNeighborList) > 0:

                        for ColNeighbor in ColNeighborList:
                            col_neighbor_r = ColNeighbor[0]
                            col_neighbor_c = ColNeighbor[1]

                            RangeNeighbor = self.RangeImage[col_neighbor_r, col_neighbor_c]
                            
                            d1 = max(RangeCenter, RangeNeighbor)
                            d2 = min(RangeCenter, RangeNeighbor)

                            ValueArcTan = np.arctan(( d2 * np.sin(self.VerticalAngle * (col_neighbor_c - target_c)) ) / ( d1 - d2 * np.cos(self.VerticalAngle * (col_neighbor_c - target_c)) ))
                            if ValueArcTan > self.Vtheta:
                                if [col_neighbor_r, col_neighbor_c] in self.queue:
                                    pass
                                else:
                                    self.queue.append([col_neighbor_r, col_neighbor_c])

    def GetNeighbor(self, r, c):

        RowNeighborList = list()
        ColNeighborList = list()

        DownPointRow = r + 1
        RightPointCol = c + 1

        deterR = 0
        deterC = 0

        while True:
            if DownPointRow < self.Rrow:
                if self.RangeImage[DownPointRow, c] > 0:
                    deterR = 1
                else:
                    DownPointRow += 1
            else:
                deterR = 1
                DownPointRow = -1

            if RightPointCol < self.Rcol:
                if self.RangeImage[r, RightPointCol] > 0:
                    deterC = 1
                else:
                    RightPointCol += 1
            else:
                deterC = 1
                RightPointCol = -1

            if (deterR+deterC) == 2:
                break

        if DownPointRow > 0:
            RowNeighborList.append([DownPointRow, c])
        if RightPointCol > 0:
            ColNeighborList.append([r, RightPointCol])
        return RowNeighborList, ColNeighborList
