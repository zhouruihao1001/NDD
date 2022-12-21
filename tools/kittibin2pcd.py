# Adapted from https://gist.github.com/HTLife/e8f1c4ff1737710e34258ef965b48344

import numpy as np
import struct
import sys
from open3d import *


def print_usage():
    print("usage: python3 kittibin2pcd.py <bin_file> <out_file>")


def convert_kitti_bin_to_pcd(binFilePath):
    size_float = 4
    list_pcd = []
    with open(binFilePath, "rb") as f:
        byte = f.read(size_float * 4)
        while byte:
            x, y, z, intensity = struct.unpack("ffff", byte)
            list_pcd.append([x, y, z])
            byte = f.read(size_float * 4)
    np_pcd = np.asarray(list_pcd)
    pcd = open3d.geometry.PointCloud()
    pcd.points = open3d.utility.Vector3dVector(np_pcd)
    return pcd

# source
bin_file = '../data/000209.bin'

if __name__ == '__main__':
    if len(sys.argv) < 3:
        print_usage()
        exit(0)

    bin_file = str(sys.argv[1])
    pcd_file = str(sys.argv[2])

    # convert
    pcd_ = convert_kitti_bin_to_pcd(bin_file)
    # show
    open3d.visualization.draw_geometries([pcd_])
    # save
    open3d.io.write_point_cloud(pcd_file, pcd_, write_ascii=False, compressed=False, print_progress=False)