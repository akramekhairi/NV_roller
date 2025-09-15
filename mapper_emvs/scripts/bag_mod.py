#!/usr/bin/env python

import rosbag
import rospy
from sensor_msgs.msg import CameraInfo, Image
import argparse
import os
from tqdm import tqdm

def modify_rosbag(in_path, out_path, time_offset_sec, cam_info_rate):
    """
    Modifies a ROS bag by applying a timestamp offset to a specific topic and
    injecting a new CameraInfo topic.
    """
    # The constant time offset to apply to the /dvs/pose topic
    time_offset = rospy.Duration.from_sec(time_offset_sec)
    pose_topic = '/dvs/pose'
    cam_info_topic = '/dvs/camera_info'

    print("Processing bag file:")
    print(f"  Input:  {in_path}")
    print(f"  Output: {out_path}")
    print(f"Applying a {time_offset.to_sec()}s offset to topic '{pose_topic}'")
    print(f"Injecting '{cam_info_topic}' at {cam_info_rate} Hz")

    # --- Create the CameraInfo message ---
    cam_info_msg = CameraInfo()
    cam_info_msg.header.frame_id = 'dvs_camera'
    cam_info_msg.distortion_model = "plumb_bob"

    # --- Original and New Camera Parameters ---
    # Original intrinsic parameters (focal lengths)
    fx = 292.5200
    fy = 293.1900

    # New principal point from 'centerInfo'
    cx_rect = 1.726100000000000e+02
    cy_rect = 1.301250000000000e+02

    # Extrinsic parameters from 'cameraPose' [R | t]
    # Flattened 3x4 matrix
    camera_pose_flat = [1.0000, 0.0042, 0.0077, -0.2273,
                        0.0040, 0.9998, 0.0187, -0.6761,
                        0.0078, 0.0187, 0.9998, 0.5175]

    # Distortion parameters (D)
    cam_info_msg.D = [-0.362, 0.115, 0.0009576, 0.00000455, 0.130]

    # --- Populate the CameraInfo message fields ---

    # 1. Intrinsic camera matrix (K) for the RECTIFIED image
    # Uses new principal point (cx_rect, cy_rect)
    cam_info_msg.K = [fx, 0.0, cx_rect,
                      0.0, fy, cy_rect,
                      0.0, 0.0, 1.0]

    # 2. Rectification matrix (R)
    # This is the 3x3 rotation part of the cameraPose
    cam_info_msg.R = [camera_pose_flat[0], camera_pose_flat[1], camera_pose_flat[2],
                      camera_pose_flat[4], camera_pose_flat[5], camera_pose_flat[6],
                      camera_pose_flat[8], camera_pose_flat[9], camera_pose_flat[10]]

    # 3. Projection/camera matrix (P) for the RECTIFIED image
    # P = [fx' 0 cx' Tx]
    #     [0 fy' cy' Ty]
    #     [0  0  1   0]

    cam_info_msg.P = [fx, 0.0, cx_rect, 0.0,
                      0.0, fy, cy_rect, 0.0,
                      0.0, 0.0, 1.0, 0.0]

    # We will get width and height from the first image message we see
    last_cam_info_time = None
    cam_info_interval = rospy.Duration.from_sec(1.0 / cam_info_rate)

    with rosbag.Bag(out_path, 'w') as outbag:
        with rosbag.Bag(in_path, 'r') as inbag:
            total_messages = inbag.get_message_count()
            for topic, msg, t in tqdm(inbag.read_messages(), total=total_messages, unit="msgs"):
                # --- Auto-detect image dimensions for CameraInfo ---
                if cam_info_msg.width == 0 and msg._type == 'sensor_msgs/Image':
                    print(f"\nDetected image dimensions from topic '{topic}': {msg.width}x{msg.height}")
                    cam_info_msg.width = msg.width
                    cam_info_msg.height = msg.height

                # --- Handle the /dvs/pose timestamp modification ---
                if topic == pose_topic:
                    new_time = t + time_offset
                    if hasattr(msg, 'header'):
                        msg.header.stamp = new_time
                    outbag.write(topic, msg, new_time)
                else:
                    outbag.write(topic, msg, t)

                # --- Inject the CameraInfo message at the specified rate ---
                if cam_info_msg.width != 0:
                    if last_cam_info_time is None or (t - last_cam_info_time) > cam_info_interval:
                        cam_info_msg.header.stamp = t
                        outbag.write(cam_info_topic, cam_info_msg, t)
                        last_cam_info_time = t

    print(f"\nProcessing complete. Modified bag saved to: {out_path}")


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="Modify a ROS bag file by shifting timestamps for /dvs/pose and adding a /dvs/camera_info topic.")
    parser.add_argument('--inbag', required=True, help="Path to the input ROS bag file.")
    parser.add_argument('--outbag', required=True, help="Path for the output ROS bag file.")
    parser.add_argument(
        '--offset',
        type=float,
        default=-0.0371060371398926,
        help="Time offset in seconds to apply to /dvs/pose timestamps."
    )
    parser.add_argument(
        '--cam-info-rate',
        type=float,
        default=1.0,
        help="Rate (in Hz) at which to publish the new /dvs/camera_info topic."
    )

    args = parser.parse_args()

    if not os.path.exists(args.inbag):
        print(f"Error: Input file not found at '{args.inbag}'")
        exit(1)

    modify_rosbag(args.inbag, args.outbag, args.offset, args.cam_info_rate)