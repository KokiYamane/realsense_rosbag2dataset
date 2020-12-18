import sys
import open3d as o3d

pcd = o3d.io.read_point_cloud(sys.argv[1])
o3d.visualization.draw_geometries([pcd])
