#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
import numpy as np

class CheckDepth(Node):
    def __init__(self):
        super().__init__('check_depth_once')
        self.sub = self.create_subscription(Image, '/uav/depth/image_raw', self.cb, 10)

    def cb(self, msg: Image):
        enc = msg.encoding
        h, w = msg.height, msg.width
        data = np.frombuffer(msg.data, dtype=np.uint16)  # 16UC1 expected
        if data.size != h*w:
            print(f"encoding={enc} shape=({h},{w}) data_elems={data.size} expected={h*w}")
        data = data[:h*w].reshape(h, w)

        nonzero = data[data > 0]
        print(f"encoding={enc} shape=({h},{w})")
        print(f"nonzero_count={nonzero.size} ({100.0*nonzero.size/(h*w):.2f}%)")
        if nonzero.size:
            print(f"min_nonzero={int(nonzero.min())}  max_nonzero={int(nonzero.max())}")
            # show a few samples
            ys = [h//2, h//2, h//4, 3*h//4]
            xs = [w//2, w//4, w//2, w//2]
            samples = [(y,x,int(data[y,x])) for y,x in zip(ys,xs)]
            print("samples(y,x,val)=", samples)
        else:
            print("ALL ZERO DEPTH")

        rclpy.shutdown()

def main():
    rclpy.init()
    node = CheckDepth()
    rclpy.spin(node)

if __name__ == '__main__':
    main()
