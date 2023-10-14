from itertools import cycle
import rclpy
from rclpy.node import Node
from daruka_msgs.msg import ScienceOut
from std_msgs.msg import UInt8
from rclpy.parameter import Parameter
from sensor_msgs.msg import Joy

class Joystick():
    def __init__(self,):
        self.data = Joy()
        self.joy_x_max = 1
        self.joy_y_max = 1
        self.deadzone = 0.05

class ScienceNode(Node):
    def __init__(self):
        super().__init__("science_node")

        self.mode_change = False
        self.modes = cycle([0,1,2])
        self.mode = next(self.modes)        # Available modes: {0: Raman,1: Drill, 2: Sampling}
        self.mode_dict = {0: 'Raman',1: 'Drill', 2: 'Sampling'}
        
        self.science_mode_pub_ = self.create_publisher(UInt8, "/Science/mode", 10)

        self.joy_in = Joystick()
        self.joy_sub_ = self.create_subscription(Joy,"/joy",self.joy_sub_callback,10)

        self.science_out_pub_ = self.create_publisher(ScienceOut, "/Science/cmd_vel", 10)       # Publisher
        self.timer = self.create_timer(0.1, self.send_science_msgs)

        self.pwm_max = 250      #make parameter
        
        self.get_logger().info("Science node started")


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


    def send_science_msgs(self):
        science_out_msg = ScienceOut()
        mode_msg = UInt8()

        mode_msg.data = self.mode
        if self.joy_in:         # checks for joystick input
            science_out_msg = self.map_joy_science()

        self.science_mode_pub_.publish(mode_msg)
        self.science_out_pub_.publish(science_out_msg)
        msg_log=f'''              Mode: {self.mode_dict[self.mode]}
                                                  Auger Vert Vel: {science_out_msg.auger_vert_vel}
                                                  Auger Vert Dir: {science_out_msg.auger_vert_dir}
                                                   Auger Hor Vel: {science_out_msg.auger_hor_vel}
                                                   Auger Hor Dir: {science_out_msg.auger_hor_dir}
                                                     Drill speed: {science_out_msg.drill_vel}
                                                 Drill direction: {science_out_msg.drill_dir}
                                               Carousel Velocity: {science_out_msg.carousel_vel}
                                              Carousel Direction: {science_out_msg.carousel_dir}
                                                     reagent_dir: {science_out_msg.reagent_direction}
                                                  Raman Vert Vel: {science_out_msg.raman_vert_vel}
                                                  Raman Vert Dir: {science_out_msg.raman_vert_dir}
                                                   Raman Hor Vel: {science_out_msg.raman_hor_vel}
                                                   raman Hor dir: {science_out_msg.raman_hor_dir}'''
        self.get_logger().info(msg_log)

    def map_joy_science(self):
        science_out_msg = ScienceOut()

        science_out_msg.carousel_vel = 0      # initialise all values to 0
        science_out_msg.carousel_dir = 0
        science_out_msg.drill_dir = 0
        science_out_msg.drill_vel = 0
        science_out_msg.auger_vert_vel = 0
        science_out_msg.auger_vert_dir = 0
        science_out_msg.auger_hor_vel = 0
        science_out_msg.auger_hor_dir = 0
        science_out_msg.carousel_dir = 0 
        science_out_msg.carousel_vel = 0
        science_out_msg.reagent_direction = 0
        science_out_msg.reagent_speed = 0
        science_out_msg.npk_servo = 0
        science_out_msg.micro_dir = 0 
        science_out_msg.micro_speed = 0


        if self.mode == 2:      #Sampling mode
            science_out_msg.carousel_vel, science_out_msg.carousel_dir = self.map_axis(self.pwm_max, 1)
            science_out_msg.reagent_speed, science_out_msg.reagent_direction = self.map_button(self.pwm_max, [1,2])

        if self.mode == 1:      #drill mode
            science_out_msg.auger_vert_vel, science_out_msg.auger_vert_dir = self.map_axis(self.pwm_max, 3)
            science_out_msg.auger_hor_vel, science_out_msg.auger_hor_dir = self.map_axis(self.pwm_max, 0, -1)
            science_out_msg.drill_vel, science_out_msg.drill_dir = self.map_axis(self.pwm_max, 1 ,1)

        if self.mode == 0:      # Raman mode

            science_out_msg.raman_vert_vel, science_out_msg.raman_vert_dir = self.map_axis(self.pwm_max, 0)
            science_out_msg.raman_hor_vel, science_out_msg.raman_hor_dir = self.map_axis(self.pwm_max, 1, -1)

        return science_out_msg

    #use flip when dealing with horizontal axes
    def map_axis(self, pwm_max, axis, flip = 1 ):  
        motor_pwm = 0
        motor_dir = 0

        if len(self.joy_in.data.axes)>0:         # Makes sure if joystick is not connected return v=0.0
            deadzone = self.joy_in.deadzone
            joy_y = flip * self.joy_in.data.axes[axis]      
            joy_y_max = self.joy_in.joy_y_max    # Output on maximum deflection
            if joy_y>deadzone:
                motor_pwm = int((pwm_max/(joy_y_max-deadzone))*(joy_y-joy_y_max) + pwm_max)     #carousel speed and direction
                motor_dir = 1       
        
            elif joy_y<-deadzone:
                motor_pwm = int(((-1*pwm_max)/(joy_y_max-deadzone))*(joy_y+joy_y_max) + pwm_max)
                motor_dir = 0    
            else:   
                motor_pwm = 0
                motor_dir = 0

        if motor_pwm>pwm_max:
            motor_pwm = pwm_max

        return motor_pwm, motor_dir
    
    def map_button(self, pwm_max, buttons):
        motor_pwm = 0
        motor_dir = 0

        if len(self.joy_in.data.buttons) > 0:
            if self.joy_in.data.buttons[buttons[0]] == 1:        #motor speed and direction
                motor_pwm = 250       #change this to a constant value/ make a parameter
                motor_dir = 1         #up
            elif self.joy_in.data.buttons[buttons[1]] == 1:
                motor_pwm = 250       #change this to a constant value/ make a parameter
                motor_dir = 0         #down
            else:
                motor_pwm = 0
                motor_dir = 0         #stationary
        
        if motor_pwm>pwm_max:
            motor_pwm = pwm_max

        return motor_pwm, motor_dir
    
def main(args=None):
    rclpy.init(args=args)
    node = ScienceNode()
    rclpy.spin(node)
    rclpy.shutdown()
