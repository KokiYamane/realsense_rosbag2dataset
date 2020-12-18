import shutil
import os
import sys
import rosbag
import cv2
from cv_bridge import CvBridge
import open3d as o3d
from tqdm import tqdm
import numpy as np

if os.path.exists('data'):
    shutil.rmtree('data')

os.mkdir('data')
os.mkdir('data/color')
os.mkdir('data/depth')
os.mkdir('data/pcd')

image_color_list = []
image_depth_list = []
timestamp_list = []
for topic, msg, t in rosbag.Bag(sys.argv[1]).read_messages():
    if topic == '/device_0/sensor_1/Color_0/image/data':
        image_color = CvBridge().imgmsg_to_cv2(msg, 'bgr8')
        image_color_list.append(image_color)
        timestamp_list.append(t)

    if topic == '/device_0/sensor_0/Depth_0/image/data':
        image_depth = CvBridge().imgmsg_to_cv2(msg, 'passthrough')
        image_depth_list.append(image_depth)


for timestamp, image_color, image_depth in tqdm(zip(
        timestamp_list, image_color_list, image_depth_list), total=len(timestamp_list)):
    t = timestamp.to_nsec()
    cv2.imwrite(
        'data/color/color{:012}.png'.format(t),
        cv2.resize(
            image_color,
            (image_depth.shape[1],
             image_depth.shape[0])))
    cv2.imwrite('data/depth/depth{:012}.png'.format(t), image_depth)

    color_raw = o3d.io.read_image('data/color/color{:012}.png'.format(t))
    depth_raw = o3d.io.read_image('data/depth/depth{:012}.png'.format(t))
    rgbd_image = o3d.geometry.RGBDImage.create_from_color_and_depth(
        color_raw, depth_raw, convert_rgb_to_intensity=False)
    pcd = o3d.geometry.PointCloud.create_from_rgbd_image(
        rgbd_image,
        o3d.camera.PinholeCameraIntrinsic(
            o3d.camera.PinholeCameraIntrinsicParameters.PrimeSenseDefault))
    o3d.io.write_point_cloud('data/pcd/pcd{:012}.pcd'.format(t), pcd)
