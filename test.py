import open3d as o3d
import cv2
import numpy as np

def depth_and_rgb_to_pcd(depth_image_path, rgb_image_path, pcd_file_path):
    depth_image = cv2.imread(depth_image_path, cv2.IMREAD_ANYDEPTH)
    rgb_image = cv2.imread(rgb_image_path, cv2.IMREAD_COLOR)

    rgbd_image = o3d.geometry.RGBDImage.create_from_color_and_depth(
        o3d.geometry.Image(rgb_image),
        o3d.geometry.Image(depth_image), convert_rgb_to_intensity=(len(rgb_image.shape) != 3), depth_trunc=20000, depth_scale=1000.0
    )
    pcd = o3d.geometry.PointCloud.create_from_rgbd_image(
        rgbd_image,
        o3d.camera.PinholeCameraIntrinsic(
            o3d.camera.PinholeCameraIntrinsicParameters.PrimeSenseDefault)
    )

    o3d.io.write_point_cloud(pcd_file_path, pcd)


import open3d as o3d

def pcd_to_ply(pcd_file_path, ply_file_path):
    # Read the PCD file
    pcd = o3d.io.read_point_cloud(pcd_file_path)

    # Save the point cloud as a PLY file
    o3d.io.write_point_cloud(ply_file_path, pcd)


depth_image_path = "output/real-time/194430103131F51200_12:14:00.117018_depth.png"
rgb_image_path = "output/real-time/194430103131F51200_12:14:00.117018_rgb.png"
pcd_file_path = "output/output.pcd"
ply_file_path = "output/output.ply"

depth_and_rgb_to_pcd(depth_image_path, rgb_image_path, pcd_file_path)
pcd_to_ply(pcd_file_path, ply_file_path)