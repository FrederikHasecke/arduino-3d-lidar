'''
Quick script to visualize the point cloud data from the demo .npy files
'''

import os
import time
import argparse
import open3d as o3d
import numpy as np

def spherical_to_cartesian(pc_sphere):
    """
    Projects a point cloud from spherical coordinates to cartesian coordinates

    Parameters
    ----------
    pc_sphere : numpy.ndarray

    Returns
    -------
    pc_cartesian: numpy.ndarray
    """
    # convert the spherical coordinates to cartesian coordinates
    pc_cartesian = np.zeros((pc_sphere.shape[0], 3))

    pc_cartesian[:, 0] = pc_sphere[:, 0] * np.cos(pc_sphere[:, 2]) * np.cos(pc_sphere[:, 1])
    pc_cartesian[:, 1] = pc_sphere[:, 0] * np.cos(pc_sphere[:, 2]) * np.sin(pc_sphere[:, 1])
    pc_cartesian[:, 2] = pc_sphere[:, 0] * np.sin(pc_sphere[:, 2])
    
    return pc_cartesian

def convert_to_cartesian(pc):
    """
    The raw point cloud data is stored as a nx3 numpy array, where n is the number of points.

    The first column contains the distance to the point.
    The second column contains the rotation angle in degrees.
    The third column contains the Lidar channel index.

    Parameters
    ----------
    pc : numpy.ndarray

    Returns
    -------
    pc_cartesian: numpy.ndarray
    """

    pc_sphere = np.zeros((pc.shape[0], 3)).astype(np.float32)

    # mm to m
    pc_sphere[:, 0] = pc[:, 0] / 1000


    # each channel is rotated differently, therefore we need a lut to add the offset
    angle_offset = np.array([0, 90, -135, -45 ,135, 45, -90, 180])

    # horizontal angle to radians
    pc_sphere[:, 1] = np.radians((pc[:, 1]+angle_offset[pc[:,2]]-90)%360)

    # channel index to radians
    channel_angle_lut = np.linspace(-15, 15, 8)[::-1]
    pc_sphere[:, 2] = np.radians(channel_angle_lut[pc[:, 2].astype(int)])

    # convert the spherical coordinates to cartesian coordinates
    pc_cartesian = spherical_to_cartesian(pc_sphere)

    return pc_cartesian, pc_sphere




# parse arguments
def parse_args():
    parser = argparse.ArgumentParser(description='Visualize point cloud data from .npy files')
    parser.add_argument('--path', type=str, default='data/sequences/01/lidar_points', help='Path to the directory where the data is stored')
    args = parser.parse_args()
    return args

if __name__ == '__main__':
    # parse arguments
    args = parse_args()

    # get the path of the current file
    current_file_path = os.path.abspath(__file__)

    # get the parent directory of the current file
    current_file_dir = os.path.dirname(os.path.dirname(current_file_path))

    # construct the absolute path to the data directory
    data_dir_path = os.path.join(current_file_dir, os.path.normpath(args.path))

    # list all files in the directory
    files = os.listdir(data_dir_path)

    # sort the files
    files.sort()

    list_of_point_clouds_cartesian = []
    list_of_point_clouds_spherical = []

    # visualise all files in a sequence at 2 fps
    for file in files:
        # visualize the file for 0.5 seconds
        pcd = np.load(os.path.join(data_dir_path, file))

        pc_cartesian, pc_sphere = convert_to_cartesian(pcd)

        list_of_point_clouds_cartesian.append(pc_cartesian)
        list_of_point_clouds_spherical.append(pc_sphere)

    # Create a visualizer
    vis = o3d.visualization.Visualizer()
    vis.create_window()

    # Assume we are using the first pointcloud to set up the initial view
    pcd = o3d.geometry.PointCloud()
    pcd.points = o3d.utility.Vector3dVector(list_of_point_clouds_cartesian[0])

    # Add the point cloud to the visualizer
    vis.add_geometry(pcd)

    # Set the view parameters
    ctr = vis.get_view_control()
    ctr.set_zoom(0.8)
    ctr.set_front([0.0, 0.0, -1.0]) 
    ctr.set_lookat([0.0, 0.0, 0.0]) 

    # Loop over pointcloud sequence and update
    for np_pc in list_of_point_clouds_cartesian[1:]:
        # Update point cloud data and refresh the view
        pcd.points = o3d.utility.Vector3dVector(np_pc)
        vis.update_geometry(pcd)
        vis.poll_events()
        vis.update_renderer()

        # Wait for the required amount of time to achieve the recording speed of 2 fps
        time.sleep(0.05)

    vis.destroy_window() 




