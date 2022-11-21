import numpy as np
import math



def pcd2range(pcd_arr,width,height):
    """

    Transform  xyz points to .npy range image

    """
    width = width
    height = height

    num=pcd_arr.shape[0]
    range_image=np.zeros(shape=(height,width))

    for i in range(0,num):
        x=pcd_arr[i][0]
        y=pcd_arr[i][1]
        z=pcd_arr[i][2]

        X=pcd_arr[:,0]
        Y=pcd_arr[:,1]


        r=np.sqrt(x**2+y**2+z**2)

        u = getColumn(x,y,width)
        v = getRow(z,r,height)
        if range_image[v, u] > r or range_image[v, u] == 0:
            range_image[v, u]=r
        else:
            continue
    return range_image




def getColumn(x,y,width):
    fi=math.atan2(y,x)
    u=int((1+(fi/math.pi))*width/2)  
    return u-1

def getRow(z,r,height):
    theta=math.asin(z/r)
    v=int((np.deg2rad(6)-theta)*height/np.deg2rad(32))

    return v-1
