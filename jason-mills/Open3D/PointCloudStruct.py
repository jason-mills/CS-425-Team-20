import open3d as o3d


# Point Cloud class/struct for convenience
class PointCloudStruct():
    def __init__(self, name, cloud, downsampled_cloud, voxel_size, down_sample_voxel_size):
        self.name = name
        self.cloud = cloud
        self.downsampled_cloud = downsampled_cloud
        self.voxel_size = voxel_size
        self.down_sample_voxel_size = down_sample_voxel_size