import numpy as np

# Point Cloud class/struct for convenience
class PointCloudStruct():
    def __init__(self, name, cloud, downsampled_cloud, voxel_size, down_sample_voxel_size):
        self.name = name
        self.geometry = cloud
        self.cloud = cloud
        self.downsampled_cloud = downsampled_cloud
        self.voxel_size = voxel_size
        self.down_sample_voxel_size = down_sample_voxel_size

# Mesh class/struct for convenience
class MeshStruct():
    def __init__(self, name, mesh):
        self.name = name
        self.mesh = mesh
        self.geometry = mesh

# Circular array for convenience
class CircularArray(np.ndarray):
    def __new__(cls, *args, **kwargs):
        return np.asarray(args[0]).view(cls)
    
    def __getitem__(self, index):
        return np.ndarray.__getitem__(self, index % len(self))