import open3d as o3d
import json


def main():
    with open("config.json") as cf:
        rs_cfg = o3d.t.io.RealSenseSensorConfig(json.load(cf))

    rs = o3d.t.io.RealSenseSensor()
    rs.init_sensor(rs_cfg, 0, "test.bag")
    rs.start_capture(True)  # true: start recording with capture
    for fid in range(150):
        im_rgbd = rs.capture_frame(True, True)  # wait for frames and align them
        # process im_rgbd.depth and im_rgbd.color

    rs.stop_capture()


if __name__ == '__main__':
    main()