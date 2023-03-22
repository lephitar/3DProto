import open3d as o3d
import os
import numpy as np
import time

def load_point_cloud(filename):
    pcd = o3d.io.read_point_cloud(filename)
    return pcd


def visualize_point_cloud(pcd):
    o3d.visualization.draw_geometries([pcd])


def animate_explosion(point_cloud, duration=5, explosion_speed=1):
    pcd = point_cloud
    points = np.asarray(pcd.points)
    center = points.mean(axis=0)
    vectors = points - center
    normalized_vectors = vectors / np.linalg.norm(vectors, axis=1, keepdims=True)
    explosion_speed = 0.01
    vis = o3d.visualization.Visualizer()
    vis.create_window()
    vis.add_geometry(pcd)

    start_time = time.time()
    while time.time() - start_time < 3:
        x = 0
    start_time = time.time()
    
    while True:
        elapsed_time = time.time() - start_time
        print(elapsed_time)

        if elapsed_time > duration:
            break

        # Compute the new positions of the points
        new_positions = points + explosion_speed * elapsed_time * normalized_vectors
        pcd.points = o3d.utility.Vector3dVector(new_positions)

        vis.update_geometry(pcd)
        vis.poll_events()
        vis.update_renderer()

    vis.destroy_window()



if __name__ == '__main__':
    pointFile = "p213.pcd"
    print("Current working directory:", os.getcwd())

    pcdFile = load_point_cloud(pointFile)
    animate_explosion(pcdFile)
