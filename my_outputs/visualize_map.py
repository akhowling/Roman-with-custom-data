import pickle
import numpy as np
import open3d as o3d

m = pickle.load(open("roman_map.pkl","rb"))

points = []
for seg in m.segments:
    if hasattr(seg, "points") and seg.points is not None:
        points.append(seg.points)

if len(points) > 0:
    points = np.vstack(points)
    pcd = o3d.geometry.PointCloud()
    pcd.points = o3d.utility.Vector3dVector(points)
    o3d.visualization.draw_geometries([pcd])
