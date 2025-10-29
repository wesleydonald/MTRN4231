import math
from geometry_msgs.msg import TransformStamped
import numpy as np
import rclpy
from rclpy.node import Node
from tf2_ros.static_transform_broadcaster import StaticTransformBroadcaster

def quaternion_from_euler(ai, aj, ak):
    ai /= 2.0
    aj /= 2.0
    ak /= 2.0
    ci = math.cos(ai)
    si = math.sin(ai)
    cj = math.cos(aj)
    sj = math.sin(aj)
    ck = math.cos(ak)
    sk = math.sin(ak)
    cc = ci*ck
    cs = ci*sk
    sc = si*ck
    ss = si*sk

    q = np.empty((4, ))
    q[0] = cj*sc - sj*cs
    q[1] = cj*ss + sj*cc
    q[2] = cj*cs - sj*sc
    q[3] = cj*cc + sj*ss

    return q

class CameraFramePublisher(Node):

  def __init__(self):
    super().__init__('camera_frame_publisher')

    # Initialise the transform broadcaster 
    self.tf_static_broadcaster = StaticTransformBroadcaster(self)

    # Publish static transforms once at setup
    self.publish_transform()

  def publish_transform(self):
    t = TransformStamped()

    t.header.stamp = self.get_clock().now().to_msg()
    t.header.frame_id = 'world'
    t.child_frame_id = 'camera_tf'
    
    scale = 0.3
    t.transform.translation.x = scale * 0.0
    t.transform.translation.y = scale * 1.0
    t.transform.translation.z = scale * 2.0

    q = quaternion_from_euler(float(math.pi), float(0.0), float(math.pi))
    t.transform.rotation.x = q[0]
    t.transform.rotation.y = q[1]
    t.transform.rotation.z = q[2]
    t.transform.rotation.w = q[3]

    self.tf_static_broadcaster.sendTransform(t)


def main(args=None):
  rclpy.init(args=args)
  node = CameraFramePublisher()
  rclpy.spin(node)
  rclpy.shutdown()

if __name__ == '__main__':
  main()
