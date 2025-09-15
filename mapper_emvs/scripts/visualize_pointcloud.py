import open3d as o3d
import numpy as np
import argparse
import matplotlib.pyplot as plt

class PointCloudVisualizer:
    def __init__(self, cloud):
        self.cloud = cloud
        # # Apply transformation to flip the point cloud right side up
        # self.cloud.transform([[1, 0, 0, 0], [0, -1, 0, 0], [0, 0, -1, 0], [0, 0, 0, 1]])
        self.points = np.asarray(cloud.points)
    
    
    def pick_points(self):
        print("\nPoint picking instructions:")
        print("1) Press [shift + left click] to select points")
        print("2) Press [shift + right click] to undo selection")
        print("3) Press 'Q' to close the window when finished")
        
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
                print(f"Depth (Z-value): {point[2]:.5f}")
        
        return picked_points
    
    def color_by_depth(self, cmap_name='hsv_r', min_fixed=-0.05, max_fixed=-0.02):
        """
        Colors the point cloud based on a FIXED depth range (Z-coordinate).
        Points outside this range will be clamped to the min/max colors.
        """
        points = np.asarray(self.cloud.points)
        if points.shape[0] == 0:
            print("Warning: Point cloud is empty. Cannot color by depth.")
            return

        depths = points[:, 2] # Use the Z-coordinate after transformation

        print(f"Using fixed depth range for coloring: Min={min_fixed:.5f}, Max={max_fixed:.5f}")
        actual_min = np.min(depths)
        actual_max = np.max(depths)
        print(f"Actual depth range in cloud: Min={actual_min:.5f}, Max={actual_max:.5f}")


        # Normalize depths based on the FIXED range
        # Ensure max_fixed is actually greater than min_fixed to avoid division by zero
        if max_fixed <= min_fixed:
             print(f"Error: max_fixed ({max_fixed}) must be greater than min_fixed ({min_fixed}). Cannot color.")
             # Assign a default color (e.g., gray) or leave colors unchanged
             gray_color = np.array([0.5, 0.5, 0.5])
             colors = np.tile(gray_color, (points.shape[0], 1))
             self.cloud.colors = o3d.utility.Vector3dVector(colors)
             return

        depths_normalized = (depths - min_fixed) / (max_fixed - min_fixed)

        # Clamp normalized values to the [0, 1] range
        # Points with depth < min_fixed will get the color for 0 (min_fixed color)
        # Points with depth > max_fixed will get the color for 1 (max_fixed color)
        depths_clipped = np.clip(depths_normalized, 0, 1)

        # Create color array using matplotlib colormap
        colors = plt.get_cmap(cmap_name)(depths_clipped)[:, 0:3] # Get RGB

        # Assign colors to point cloud
        self.cloud.colors = o3d.utility.Vector3dVector(colors)



def calculate_depth_stats(cloud):
    points = np.asarray(cloud.points)
    depths = points[:, 2]
    
    max_depth = np.max(depths)
    min_depth = np.min(depths)
    avg_depth = np.mean(depths)
    
    # Create a histogram of depths
    hist, bin_edges = np.histogram(depths, bins=100)
    
    # Find the most common depth (mode)
    mode_depth = bin_edges[np.argmax(hist)]
    
    return max_depth, min_depth, avg_depth, mode_depth, depths

def plot_depth_distribution(depths):
    plt.figure(figsize=(10, 6))
    plt.hist(depths, bins=100, edgecolor='black')
    plt.title('Depth Distribution')
    plt.xlabel('Depth')
    plt.ylabel('Frequency')
    plt.show()

if __name__ == "__main__":
    parser = argparse.ArgumentParser(description='Analyze and visualize point cloud depth distribution')
    parser.add_argument('-i', '--input', default='pointcloud.pcd', type=str,
                        help='path to the PCD file (default: pointcloud.pcd)')
    args = parser.parse_args()

    cloud = o3d.io.read_point_cloud(args.input)

    # Create visualizer instance
    visualizer = PointCloudVisualizer(cloud)
    #visualizer.color_by_depth()

    
    # Calculate and display depth statistics
    max_depth, min_depth, avg_depth, mode_depth, depths = calculate_depth_stats(cloud)
    print(f"Maximum depth: {max_depth:.5f}")
    print(f"Minimum depth: {min_depth:.5f}")
    print(f"Average depth: {avg_depth:.5f}")
    print(f"Most common depth: {mode_depth:.5f}")

    # # Plot depth distribution
    # plot_depth_distribution(depths)
    
    # Interactive point selection
    visualizer.pick_points()
