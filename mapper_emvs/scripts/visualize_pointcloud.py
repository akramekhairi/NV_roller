import open3d as o3d
import numpy as np
import argparse
import matplotlib.pyplot as plt

class PointCloudVisualizer:
    def __init__(self, cloud):
        self.cloud = cloud
        # Apply transformation to flip the point cloud right side up
        self.cloud.transform([[1, 0, 0, 0], [0, -1, 0, 0], [0, 0, -1, 0], [0, 0, 0, 1]])
        self.points = np.asarray(cloud.points)
    
    def pick_points(self):
        
        vis = o3d.visualization.VisualizerWithEditing()
        vis.create_window()
        vis.add_geometry(self.cloud)
        vis.run()
        picked_points = vis.get_picked_points()
        vis.destroy_window()
        
        if picked_points:
            for point_idx in picked_points:
                point = self.points[point_idx]
                print(f"Selected point {point_idx}:")
                print(f"Depth (Z-value): {point[2]*1000:.5f}")
        
        return picked_points



if __name__ == "__main__":
    parser = argparse.ArgumentParser(description='Analyze and visualize point cloud depth distribution')
    parser.add_argument('-i', '--input', default='pointcloud.pcd', type=str,
                        help='path to the PCD file (default: pointcloud.pcd)')
    args = parser.parse_args()

    cloud = o3d.io.read_point_cloud(args.input)
    
    # Create visualizer instance
    visualizer = PointCloudVisualizer(cloud)
    # Interactive point selection
    visualizer.pick_points()