#!/usr/bin/env python3
"""
check_cloud.py

Subscribe once to /map/cloud and print sample points and bounds.
Run inside the container after sourcing ROS 2 and workspace:

source /opt/ros/jazzy/setup.bash
source /workspace/V2/install/setup.bash
python3 scripts/check_cloud.py

"""
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2

# sensor_msgs_py provides read_points convenience
try:
    from sensor_msgs_py.point_cloud2 import read_points
except Exception:
    # fallback: try ros2 bag installed package location
    from sensor_msgs.point_cloud2 import read_points

import math

class CloudChecker(Node):
    def __init__(self):
        super().__init__('cloud_checker')
        self.sub = self.create_subscription(
            PointCloud2,
            '/map/cloud',
            self.cb,
            10)
        self.got = False

    def cb(self, msg: PointCloud2):
        if self.got:
            return
        self.got = True
        # read_points returns generator of tuples
        pts = list(read_points(msg, field_names=('x','y','z','rgb'), skip_nans=True))
        total = len(pts)
        print(f'Received {total} points (showing up to 10 samples)')
        sample_count = min(10, total)
        for i in range(sample_count):
            p = pts[i]
            x = float(p[0]) if p[0] is not None else float('nan')
            y = float(p[1]) if p[1] is not None else float('nan')
            z = float(p[2]) if p[2] is not None else float('nan')
            rgb_val = p[3]
            # rgb may be packed float or int
            try:
                rgb_int = int(rgb_val)
            except Exception:
                # when rgb is float-packed, reinterpret bits
                try:
                    import struct
                    rgb_int = struct.unpack('I', struct.pack('f', float(rgb_val)))[0]
                except Exception:
                    rgb_int = 0
            print(f'{i}: x={x:.3f} y={y:.3f} z={z:.3f} rgb=0x{rgb_int:08x}')

        if total > 0:
            xs = [float(p[0]) for p in pts]
            ys = [float(p[1]) for p in pts]
            zs = [float(p[2]) for p in pts]
            print('Bounds:')
            print(f'  x: {min(xs):.3f} .. {max(xs):.3f}')
            print(f'  y: {min(ys):.3f} .. {max(ys):.3f}')
            print(f'  z: {min(zs):.3f} .. {max(zs):.3f}')
        else:
            print('No points available after skipping NaNs')
        rclpy.shutdown()


def main():
    rclpy.init()
    node = CloudChecker()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
