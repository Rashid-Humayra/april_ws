
from __future__ import annotations
import math
from enum import Enum, auto
import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool, String
from geometry_msgs.msg import Twist, Vector3Stamped


def clamp(v:float, lo:float, hi:float) -> float:
    return lo if v<lo else hi if v>hi else v


class State(Enum):
    SEARCH = auto()
    ALIGN = auto()
    DRIVE = auto()
    HOLD = auto()
    

class DockingController(Node):
    def __init__(self):
        super().__init__("docking_controller")
        
        #Params
        
        self.declare_parameter("control_hz", 30)
        
        self.declare_parameter("k_alpha", 1.2)
        self.declare_parameter("r_max", 0.35)
        self.declare_parameter("turn_sign", -1.0)
        self.declare_parameter("alpha_align", 0.15) # rad: if less than this- DRIVE
        self.declare_parameter("alpha_hard", 0.45)  # rad: if more than this- stop forward
        self.declare_parameter("pose_timeout", 0.6)
        self.declare_parameter("u_far", 0.12)  # velocity when far
        self.declare_parameter("u_near", 0.08) # velocity when close
        self.declare_parameter("z_slow", 2.0)  # threshold for slowing
        self.declare_parameter("z_stop", 0.80) # stop distance
        self.declare_parameter("u_max", 0.20)  # safety constraint
        self.declare_parameter("search_spin_rate", 0.20)  # rad/s
        self.declare_parameter("search_forward", 0.00)
        
        self.control_hz = float(self.get_parameter("control_hz").value)
        self.k_alpha = float(self.get_parameter("k_alpha").value)
        self.r_max = float(self.get_parameter("r_max").value)
        self.turn_sign = float(self.get_parameter("turn_sign").value)

        self.alpha_align = float(self.get_parameter("alpha_align").value)
        self.alpha_hard = float(self.get_parameter("alpha_hard").value)
        self.pose_timeout = float(self.get_parameter("pose_timeout").value)

        self.u_far = float(self.get_parameter("u_far").value)
        self.u_near = float(self.get_parameter("u_near").value)
        self.z_slow = float(self.get_parameter("z_slow").value)
        self.z_stop = float(self.get_parameter("z_stop").value)
        self.u_max = float(self.get_parameter("u_max").value)

        self.search_spin_rate = float(self.get_parameter("search_spin_rate").value)
        self.search_forward = float(self.get_parameter("search_forward").value)
    
    
        self.sub_valid = self.create_subscription(Bool, "/dock/tag0_valid", self.on_valid, 10)
        self.sub_pose = self.create_subscription(Vector3Stamped, "/dock/tag0", self.on_pose, 10)

        self.pub_cmd = self.create_publisher(Twist, "/cmd_vel", 10)
        self.pub_state = self.create_publisher(String, "/dock/state", 10)
        
        
        
        
        self.state = State.SEARCH
        self.tag_valid = False
        self.last_pose_t = -1e5
        self.x = 0.0
        self.z = 999.0
        self.alpha = 0.0

        self.timer = self.create_timer(1.0 / self.control_hz, self.step)
        self._publish_state()
        
        
        
    def on_valid(self, msg: Bool):
        self.tag_valid = bool(msg.data)

    def on_pose(self, msg: Vector3Stamped):
        self.x = float(msg.vector.x)
        self.alpha = float(msg.vector.y)  # rad
        self.z = float(msg.vector.z)
        self.last_pose_t = self.get_clock().now().nanoseconds / 1e9
            
    def _publish_state(self):
        self.pub_state.publish(String(data=self.state.name))

    def _set_state(self, s: State):
        if s != self.state:
            self.state = s
            self.get_logger().info(f"State -> {self.state.name}")
            self._publish_state()
        
    def _pose_ok(self, now: float) -> bool:
        if not self.tag_valid:
            return False
        if (now - self.last_pose_t) > self.pose_timeout:
            return False
        return True

    def _speed_schedule(self, z: float) -> float:
        return self.u_near if z <= self.z_slow else self.u_far
        
        
    def step(self):
        now = self.get_clock().now().nanoseconds / 1e9
        ok = self._pose_ok(now)
        
        cmd = Twist()
            
            
        ####search when tag not detected
        if not ok:
            self._set_state(State.SEARCH)
            cmd.linear.x = float(self.search_forward)
            cmd.angular.z = float(self.search_spin_rate)
            self.pub_cmd.publish(cmd)
            return
        
        
        ####pose available
        if self.z <= self.z_stop:
            self._set_state(State.HOLD)
            cmd.linear.x = 0.0
            cmd.angular.z = 0.0
            self.pub_cmd.publish(cmd)
            return
        
        ####set from search state to align state when tag pose is ok
        if self.state == State.SEARCH:
            self._set_state(State.ALIGN)
            
        
        r = self.turn_sign * (-self.k_alpha * self.alpha)   #####like a P controller
        r = clamp(r, -self.r_max, self.r_max)
        
        if self.state == State.ALIGN:
            cmd.linear.x = 0.0
            cmd.angular.z = float(r)
            
            if abs(self.alpha) < self.alpha_align:
                self._set_state(State.DRIVE)
                
            self.pub_cmd.publish(cmd)
            return
        
        
        if self.state == State.DRIVE:
            if abs(self.alpha) > self.alpha_hard:
                u = 0.0
            else:
                u = self._speed_schedule(self.z)
                
            u = clamp(u, 0.0, self.u_max)
            
            cmd.linear.x = float(u)
            cmd.angular.z = float(r)
            
            self.pub_cmd.publish(cmd)
            return
        
        cmd.linear.x = 0.0
        cmd.angular.z = 0.0
        self.pub_cmd.publish(cmd)
  

          
def main(args=None):
    rclpy.init(args=args)
    node = DockingController()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
                
            