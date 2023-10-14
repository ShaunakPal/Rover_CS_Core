from itertools import cycle
import rclpy
from rclpy.node import Node
from std_msgs.msg import UInt8
from sensor_msgs.msg import Joy
from geometry_msgs.msg import Twist

class Joystick():
    def __init__(self,):
        self.data = Joy()
        self.joy_x_max = 1
        self.joy_y_max = 1
        self.deadzone = 0.05
        
class DriveNode(Node):

    def __init__(self):
        super().__init__("drive_node")

        # # Define QoS settings
        # qos_profile = rclpy.qos.QoSProfile(
        #     history=rclpy.qos.HistoryPolicy.KEEP_LAST,
        #     depth=10,  # Set the depth of the message queue
        #     reliability=rclpy.qos.ReliabilityPolicy.BEST_EFFORT,
        #     durability=rclpy.qos.DurabilityPolicy.VOLATILE
        # )

        self.mode_change = False
        self.modes = cycle([0,1,2])
        self.mode = next(self.modes)                              # Available modes: {2: 25% speed, 1: 50% speed, 0:100% speed}
        self.drive_mode_pub_ = self.create_publisher(UInt8, "/Drive/mode", 10)      # updates from Joy.buttons[0] press event

        self.v_max = 1.6                            # Maximum linear velocity of rover
        self.w_max = 1.7                            # Maximum angular velocity of rover
        self.cmd_vel_pub_ = self.create_publisher(Twist, "/Drive/cmd_vel", 10)

        self.joy_in = Joystick()
        self.joy_sub_ = self.create_subscription(Joy,"/joy",self.joy_sub_callback,10)
        self.joy_sub_   # prevent unused variable warning

        self.timer = self.create_timer(0.1, self.send_drive_msgs)
        self.get_logger().info("Drive Node Started")

    def joy_sub_callback(self,msg):
        self.joy_in.data = msg

        # Mode change: checks for button press and realease
        if len(self.joy_in.data.buttons)>0:
            if not self.mode_change:
                if msg.buttons[0] == 1:
                    self.mode_change = True
            else:
                if msg.buttons[0] == 0:
                    self.mode = next(self.modes)
                    self.mode_change = False
    
    def send_drive_msgs(self):
        cmd_msg = Twist()
        mode_msg = UInt8()

        mode_msg.data = self.mode
        if self.joy_in:
            cmd_msg = self.map_joy()

        self.drive_mode_pub_.publish(mode_msg)
        self.cmd_vel_pub_.publish(cmd_msg)

        self.get_logger().info(f'Linear velocity: {cmd_msg.linear.x}, Angular velocity: {cmd_msg.angular.z}, Mode:{mode_msg.data}')

    def map_joy(self):
        vel_msg = Twist()       # Twist message object instantiation
       
        vel_msg.linear.x=0.0    # Twist message initialisations
        vel_msg.linear.y=0.0
        vel_msg.linear.z=0.0
        vel_msg.angular.x=0.0
        vel_msg.angular.y=0.0
        vel_msg.angular.z=0.0

        # Modes definition
        if self.mode == 0:
            vel_msg.linear.x=self._map_v(self.v_max)        # Maximum linear velocity: v_max = 1.6 
            vel_msg.angular.z=self._map_w(self.w_max)       # Maximum angular velocity: w_max = 1.7
        elif self.mode == 1:
            vel_msg.linear.x=self._map_v(self.v_max/2)      # Maximum linear velocity: v_max = 0.8 
            vel_msg.angular.z=self._map_w(self.w_max/2)     # Maximum angular velocity: w_max = 0.85
        elif self.mode == 2:
            vel_msg.linear.x=self._map_v(self.v_max/4)     # Maximum linear velocity: v_max = 0.4
            vel_msg.angular.z=self._map_w(self.w_max/4)     # Maximum angular velocity: w_max = 0.43

        return vel_msg
    
    def _map_v(self,v_max):
        v=0.0      
        if len(self.joy_in.data.axes)>0:         # Makes sure if joystick is not connected return v=0.0
            deadzone = self.joy_in.deadzone
            joy_y = self.joy_in.data.axes[1]     # Get axes[0] i.e. left joystick y-axis
            joy_y_max = self.joy_in.joy_y_max    # Output on maximum deflection
            if joy_y>deadzone:
                v = ((v_max/(joy_y_max-deadzone))*(joy_y-joy_y_max)) + v_max
            elif joy_y<-deadzone:
                v = ((v_max/(joy_y_max-deadzone))*(joy_y+joy_y_max)) - v_max
        
        if v>v_max:
            return v_max
        
        return v
    
    def _map_w(self,w_max):
        w=0.0
        if len(self.joy_in.data.axes)>0:        # Makes sure if joystick is not connected return w=0.0
            deadzone = self.joy_in.deadzone
            joy_x = self.joy_in.data.axes[0]    # Get axes[0] i.e. left joystick x-axis 
            joy_x_max = self.joy_in.joy_x_max   # Output on maximum deflection
            if joy_x>deadzone:
                w = ((w_max/(deadzone-joy_x_max))*(joy_x-joy_x_max)) - w_max
            elif joy_x<-deadzone:
                w = ((w_max/(deadzone-joy_x_max))*(joy_x+joy_x_max)) + w_max
        
        if w>w_max:
            return w_max
        
        return w

        
def main(args=None):
    rclpy.init(args=args)
    node = DriveNode()
    rclpy.spin(node)
    rclpy.shutdown()
