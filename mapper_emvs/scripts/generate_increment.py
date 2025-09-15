#!/usr/bin/env python

import os
import sys
import subprocess
import argparse
import rosbag
from tqdm import tqdm

def find_start_event_index(bag_path, pose_topic, event_topic):
    """
    Finds the index of the first event that occurs at or after the first pose timestamp.
    Also returns the total number of events in the bag.
    """
    print(f"Analyzing bag '{bag_path}' to find start index...")
    first_pose_ts = None
    total_events = 0
    start_idx = -1

    try:
        with rosbag.Bag(bag_path, 'r') as bag:
            for topic, msg, t in bag.read_messages(topics=[pose_topic]):
                first_pose_ts = msg.header.stamp
                print(f"Found first pose timestamp at: {first_pose_ts.to_sec()}")
                break

            if first_pose_ts is None:
                print(f"Error: No messages found on pose topic '{pose_topic}'.")
                return -1, 0

            pbar = tqdm(total=bag.get_message_count([event_topic]), unit=" msgs", desc="Scanning events")
            for topic, msg, t in bag.read_messages(topics=[event_topic]):
                pbar.update(1)
                if not msg.events:
                    continue

                if start_idx == -1:
                    for event in msg.events:
                        total_events += 1
                        if event.ts >= first_pose_ts:
                            start_idx = total_events - 1
                            break
                    if start_idx != -1:
                        print(f"\nStart event index found at: {start_idx}")
                else:
                    total_events += len(msg.events)
            pbar.close()

    except rosbag.bag.ROSBagUnindexedException:
        print("Error: Bag file is unindexed. Please run 'rosbag reindex <bag_file>'")
        sys.exit(1)
    except Exception as e:
        print(f"An error occurred while reading the bag file: {e}")
        sys.exit(1)

    if start_idx == -1:
        print("Warning: Reached end of bag without finding an event after the first pose.")
        start_idx = 0

    print(f"Found a total of {total_events} events in the bag.")
    return start_idx, total_events


def main():
    parser = argparse.ArgumentParser(
        description="Run EMVS in chunks, saving depth maps, point clouds, and poses.",
        formatter_class=argparse.RawTextHelpFormatter
    )
    parser.add_argument('--bag_filename', required=True, help="Path to the input ROS bag file.")
    parser.add_argument('--output_dir', required=True, help="Base directory for all outputs.")
    parser.add_argument('--flagfile', help="Path to a gflags configuration file (.conf).")
    parser.add_argument('--chunk_size', type=int, default=75000, help="Number of events per chunk.")
    parser.add_argument('--step_size', type=int, default=5000, help="Sliding window step size.")
    parser.add_argument('--pose_topic', default='/dvs/pose', help="The pose topic.")
    parser.add_argument('--event_topic', default='/dvs/events', help="The event topic.")
    
    args, passthrough_args = parser.parse_known_args()

    if not os.path.exists(args.bag_filename):
        print(f"Error: Bag file not found at '{args.bag_filename}'")
        sys.exit(1)
    if args.flagfile and not os.path.exists(args.flagfile):
        print(f"Error: Flag file not found at '{args.flagfile}'")
        sys.exit(1)

    # --- Create all output directories ---
    colored_dir = os.path.join(args.output_dir, 'colored')
    depth_maps_dir = os.path.join(args.output_dir, 'depth_maps')
    pointclouds_dir = os.path.join(args.output_dir, 'pointclouds')
    confidence_map_dir = os.path.join(args.output_dir, 'confidence_maps')
    csv_path = os.path.join(args.output_dir, 'average_depths.csv')
    os.makedirs(colored_dir, exist_ok=True)
    os.makedirs(depth_maps_dir, exist_ok=True)
    os.makedirs(pointclouds_dir, exist_ok=True)
    os.makedirs(confidence_map_dir, exist_ok=True)
    print(f"Outputs will be saved to subdirectories in: {args.output_dir}")

    if args.step_size > args.chunk_size:
        print("Warning: step_size is larger than chunk_size. This will skip events.")

    start_idx, total_events = find_start_event_index(
        args.bag_filename, args.pose_topic, args.event_topic
    )
    if start_idx == -1:
        sys.exit(1)

    current_event_idx = start_idx
    frame_counter = 1
    num_chunks = (total_events - start_idx - args.chunk_size) // args.step_size + 1
    
    with open(csv_path, 'w') as csv_file:
        csv_file.write("frame_number,average_depth_meters\n")

        with tqdm(total=num_chunks, unit="chunk") as pbar:
            while current_event_idx + args.chunk_size <= total_events:
                pbar.set_description(f"Processing events {current_event_idx} -> {current_event_idx + args.chunk_size}")

                cmd = ["rosrun", "mapper_emvs", "run_emvs"]
                if args.flagfile: cmd.append(f"--flagfile={args.flagfile}")
                cmd.extend(passthrough_args)
                cmd.append(f"--bag_filename={args.bag_filename}")
                cmd.append(f"--start_event_idx={current_event_idx}")
                cmd.append(f"--num_events={args.chunk_size}")

                try:
                    result = subprocess.run(cmd, check=True, capture_output=True, text=True)

                    avg_depth = float(result.stdout.strip())
                    csv_file.write(f"{frame_counter:04d},{avg_depth}\n")

                    # --- File Management ---
                    frame_name = f"{frame_counter:04d}"
                    
                    # Source files created by C++ executable
                    source_files = {
                        "depth_colored.png": os.path.join(colored_dir, f"{frame_name}.png"),
                        "depth_map.png": os.path.join(depth_maps_dir, f"{frame_name}.png"),
                        "pointcloud.pcd": os.path.join(pointclouds_dir, f"{frame_name}.pcd"),
                        "pose.txt": os.path.join(pointclouds_dir, f"{frame_name}.txt"),
                        "confidence_map.png": os.path.join(confidence_map_dir, f"{frame_name}.png")
                    }

                    # Move and rename all generated files
                    for src, dest in source_files.items():
                        if os.path.exists(src):
                            os.rename(src, dest)
                        else:
                            print(f"\nWarning: Source file '{src}' not found after processing chunk.")
                    
                    # Clean up remaining temporary files
                    for f in ["dsi.npy", "semidense_mask.png", "confidence_map.png"]:
                        if os.path.exists(f):
                            os.remove(f)

                except (subprocess.CalledProcessError, ValueError) as e:
                    print(f"\n--- Error processing chunk starting at event {current_event_idx} ---")
                    if isinstance(e, subprocess.CalledProcessError):
                        print(f"COMMAND: {' '.join(cmd)}")
                        print(f"Return Code: {e.returncode}")
                        print("--- STDOUT --- \n" + e.stdout)
                        print("--- STDERR --- \n" + e.stderr)
                    else:
                        print(f"Could not parse average depth from stdout: {result.stdout}")
                    print("Aborting script.")
                    sys.exit(1)

                current_event_idx += args.step_size
                frame_counter += 1
                pbar.update(1)
    
    print("\nProcessing complete.")
    print(f"All outputs saved to '{args.output_dir}'.")


if __name__ == "__main__":
    main()