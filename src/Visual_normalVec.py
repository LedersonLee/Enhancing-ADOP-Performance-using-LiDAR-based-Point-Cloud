import open3d as o3d
import numpy as np

#input_filename= "/root/catkin_ws/Dataset/normalByPython.ply"
input_filename= "/root/catkin_ws/Dataset/ADOP_dataset/final_bin.ply"
pcd=o3d.io.read_point_cloud(input_filename)

print("Let's start to downsample the point cloud and visualize it\n")
downpcd = pcd.voxel_down_sample(voxel_size=0.05)
o3d.visualization.draw_geometries([downpcd],
                                  zoom=0.3412,
                                  front=[0.4257, -0.2125, -0.8795],
                                  lookat=[2.6172, 2.0475, 1.532],
                                  up=[-0.0694, -0.9768, 0.2024], point_show_normal=True)

print("All done\n")
