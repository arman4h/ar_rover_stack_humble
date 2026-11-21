import time
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
class Watchdog(Node):
 def __init__(self):
  super().__init__('cmd_vel_watchdog')
  self.declare_parameter('input_topic','/cmd_vel_nav'); self.declare_parameter('output_topic','/cmd_vel')
  self.declare_parameter('timeout_sec',0.5); self.declare_parameter('publish_rate',20.0)
  self.timeout=float(self.get_parameter('timeout_sec').value); self.last=time.monotonic(); self.cmd=Twist()
  self.pub=self.create_publisher(Twist,self.get_parameter('output_topic').value,10)
  self.sub=self.create_subscription(Twist,self.get_parameter('input_topic').value,self.cb,10)
  self.timer=self.create_timer(1.0/float(self.get_parameter('publish_rate').value),self.tick)
 def cb(self,msg): self.cmd=msg; self.last=time.monotonic()
 def tick(self):
  if time.monotonic()-self.last>self.timeout: self.pub.publish(Twist())
  else: self.pub.publish(self.cmd)
def main(args=None):
 rclpy.init(args=args); n=Watchdog()
 try: rclpy.spin(n)
 except KeyboardInterrupt: pass
 n.destroy_node(); rclpy.shutdown()
