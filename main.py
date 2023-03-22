import open3d as o3d
import os
import numpy as np
import time

def load_point_cloud(filename):
    pcd = o3d.io.read_point_cloud(filename)
    return pcd



class ExplosionAnimator:
    def __init__(self, point_cloud, duration=5, explosion_speed=1):
        self.point_cloud = point_cloud
        self.points = np.asarray(self.point_cloud.points)
        self.center = self.points.mean(axis=0)
        self.vectors = self.points - self.center
        self.normalized_vectors = self.vectors / np.linalg.norm(self.vectors, axis=1, keepdims=True)
        self.start_time = time.time()
        self.duration = duration
        self.explosion_speed = explosion_speed

    def step(self):
        elapsed_time = time.time() - self.start_time

        if elapsed_time > self.duration:
            return False

        new_positions = self.points + self.explosion_speed * elapsed_time * self.normalized_vectors
        self.point_cloud.points = o3d.utility.Vector3dVector(new_positions)

        return True

def animate_explosion2(point_cloud, duration=5, explosion_speed=1, fps=30):
    animator = ExplosionAnimator(point_cloud, duration, explosion_speed)
    vis = o3d.visualization.Visualizer()
    vis.create_window()
    vis.add_geometry(point_cloud)

    while animator.step():
        vis.update_geometry(point_cloud)
        vis.poll_events()
        vis.update_renderer()
        time.sleep(1 / fps)

    vis.destroy_window()


def visualize_point_cloud(pcd):
    o3d.visualization.draw_geometries([pcd])


def animate_explosion(point_cloud, duration=10, explosion_speed=0.002):
    pcd = point_cloud
    points = np.asarray(pcd.points)
    center = points.mean(axis=0)
    vectors = points - center
    normalized_vectors = vectors / np.linalg.norm(vectors, axis=1, keepdims=True)
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
        new_positions = points + np.cos(elapsed_time) * explosion_speed * elapsed_time * normalized_vectors
        pcd.points = o3d.utility.Vector3dVector(new_positions)

        vis.update_geometry(pcd)
        vis.poll_events()
        vis.update_renderer()

    vis.destroy_window()



if __name__ == '__main__':
    pointFile = "p213.pcd"
    pcdFile = load_point_cloud(pointFile)
    animate_explosion2(pcdFile, 10, 0.002)

