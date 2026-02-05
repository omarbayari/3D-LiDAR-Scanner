import numpy as np
import open3d as o3d

if __name__ == "__main__":
    # Input number of scans and define spins per scan
    total_scans = int(input("How many scans did you take: "))
    spins_per_scan = 16

    # Load the point cloud data from the file
    print("Reading the prism point cloud data (pcd)...")
    point_cloud = o3d.io.read_point_cloud("point_array.xyz", format="xyz")

    # Display the point cloud array numerically for inspection
    print("The PCD array:")
    print(np.asarray(point_cloud.points))

    # Visualize the point cloud data in an interactive window
    print("Visualizing the PCD: (spawns a separate interactive window)")
    o3d.visualization.draw_geometries([point_cloud])

    # Assign unique vertex numbers for each yz slice
    yz_slice_vertices = []
    for i in range(0, total_scans * spins_per_scan):
        yz_slice_vertices.append([i])

    # Define the lines to connect vertices within each yz slice
    connection_lines = []
    for i in range(0, total_scans * spins_per_scan, spins_per_scan):
        for j in range(spins_per_scan):
            if j == spins_per_scan - 1:
                connection_lines.append([yz_slice_vertices[i + j], yz_slice_vertices[i]])  # Close the loop
            else:
                connection_lines.append([yz_slice_vertices[i + j], yz_slice_vertices[i + j + 1]])

    # Define the lines connecting vertices between consecutive yz slices
    for i in range(0, total_scans * spins_per_scan - spins_per_scan - 1, spins_per_scan):
        for j in range(spins_per_scan):
            connection_lines.append([yz_slice_vertices[i + j], yz_slice_vertices[i + j + spins_per_scan]])

    # Map the lines to 3D point coordinates
    line_set = o3d.geometry.LineSet(points=o3d.utility.Vector3dVector(np.asarray(point_cloud.points)),
                                     lines=o3d.utility.Vector2iVector(connection_lines))

    # Visualize the point cloud with lines connecting the vertices
    o3d.visualization.draw_geometries([line_set])
