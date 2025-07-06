import laspy
import open3d as o3d
import numpy as np
import matplotlib.pyplot as plt

def visualize_laz_open3d(file_path, downsample_factor=100, use_color=True):
    """Visualize LAZ file with robust camera setup and coordinate handling"""
    # Read LAZ file
    las = laspy.read(file_path)
    
    # Extract and downsample points
    points = np.vstack((las.x, las.y, las.z)).transpose()
    points = points[::downsample_factor]
    
    # Create point cloud
    pcd = o3d.geometry.PointCloud()
    pcd.points = o3d.utility.Vector3dVector(points)
    
    # Apply colors or elevation-based coloring
    if use_color and hasattr(las, 'red') and hasattr(las, 'green') and hasattr(las, 'blue'):
        colors = np.vstack((las.red, las.green, las.blue)).transpose()
        colors = colors[::downsample_factor]
        pcd.colors = o3d.utility.Vector3dVector(colors / 65535.0)
    else:
        # Color by elevation
        z_min = np.min(points[:, 2])
        z_max = np.max(points[:, 2])
        z_normalized = (points[:, 2] - z_min) / (z_max - z_min + 1e-8)
        colors = plt.cm.viridis(z_normalized)[:, :3]
        pcd.colors = o3d.utility.Vector3dVector(colors)
    
    # Create visualizer with proper camera setup
    vis = o3d.visualization.Visualizer()
    vis.create_window(width=1200, height=900)
    vis.add_geometry(pcd)
    
    # Configure render settings
    render_opt = vis.get_render_option()
    render_opt.point_size = 2.0  # Increase point size
    render_opt.background_color = np.array([0.2, 0.2, 0.2])  # Gray background
    render_opt.light_on = True
    
    # Set camera to properly view the point cloud
    ctr = vis.get_view_control()
    bbox = pcd.get_axis_aligned_bounding_box()
    
    # Calculate camera parameters
    center = bbox.get_center()
    max_bound = bbox.get_max_bound()
    min_bound = bbox.get_min_bound()
    size = np.linalg.norm(max_bound - min_bound)
    
    # Set camera position and target
    camera_params = ctr.convert_to_pinhole_camera_parameters()
    camera_params.extrinsic = np.array([
        [1, 0, 0, center[0] + size],
        [0, 1, 0, center[1] + size],
        [0, 0, 1, center[2] + size * 0.5],
        [0, 0, 0, 1]
    ])
    ctr.convert_from_pinhole_camera_parameters(camera_params)
    
    # Add coordinate frame (scaled to point cloud size)
    coord_size = size * 0.1
    coord_frame = o3d.geometry.TriangleMesh.create_coordinate_frame(size=coord_size, origin=center)
    vis.add_geometry(coord_frame)
    
    # Reset view to ensure visibility
    vis.reset_view_point(True)
    
    # Run visualization
    vis.run()
    vis.destroy_window()

# Example usage
if __name__ == "__main__":
    #laz_file = "outputs-02/12DN1_12_corridor.LAZ"
    #laz_file = "kloosterveen-hoogkerk/12DN1_12.LAZ"
    laz_file = "mocked_powerline_protection.laz" # "07CZ2_03.LAZ"
    visualize_laz_open3d(
        laz_file,
        downsample_factor=10,   # Adjust based on your system
        use_color=False          # Set to True if RGB data exists
    )