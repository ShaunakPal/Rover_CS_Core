from smbus2 import SMBus, i2c_msg
import rclpy
from rclpy.node import Node
from daruka_msgs.msg import ScienceOut, ScienceIn


## Sub this out with the BusOp class
bus = SMBus(0)           # Using i2c bus 0
i2c_addr = 0x08          # Address of stm
reg_write = 0            # Register to write data on
reg_read = 2             # Register to read data from
read_length = 13         # Incoming sensor array length


#class BusOp(SMBus, i2c_msg):
#    def __init__(self, i2c_addr = 0x10, reg_write: int = 0, reg_read: int = 0, read_length: int = 13, 
#                 bus: int | str | None = 0, force: bool = False) -> None:
#        self.bus = super().__init__(bus, force)
#        self.i2c_addr = i2c_addr            # Address of stm
#        self.reg_write = reg_write          # Register to write data on
#        self.reg_read = reg_read            # Register to read data from
#        self.read_length = read_length      # Incoming sensor array length
#    
#    def set_write_register(self, reg_write):
#        self.reg_write = reg_write
#
#    def set_read_register(self, reg_read):
#        self.reg_read = reg_read
#    
#    def set_i2c_addr(self, i2c_addr):
#        self.i2c_addr = i2c_addr
#    
#    def set_i2c_addr(self, read_length):
#        self.read_length = read_length
#
#    def i2c_rw(self, msg: ScienceOut, logger: Node.get_logger()):
#        '''Sends the passed ScienceOut() msg, reads and returns the sensor data as a list over i2c bus'''
#        array = [msg.auger_vert_vel, msg.auger_vert_dir,
#                  msg.auger_hor_vel, msg.auger_hor_dir,
#                  msg.drill_vel, msg.drill_dir,
#                  msg.carousel_vel, msg.carousel_dir,
#                  msg.reagent_direction,
#                  msg.raman_vert_vel, msg.raman_vert_dir,
#                  msg.raman_hor_vel, msg.raman_hor_dir
#                ]
#        self.bus.write_i2c_block_data(self.i2c_addr, self.reg_write, array)
#
#        sensor_in = self.bus.read_i2c_block_data(self.i2c_addr, self.reg_read, self.read_length)
#        if logger:
#            logger.info("Sent First Array to STM", once=True)
#            logger.info("Sent: ", array)
#            logger.info("Recieved First Array from STM", once=True)
#
#        return sensor_in


class Sci_I2C_Node(Node):

    def __init__(self):
        super().__init__("Science_i2c_node")
        self.science_out_sub_ = self.create_subscription(ScienceOut, "/Science/cmd_vel", self.sci_out_sub_callback,10)
        self.science_in_pub_ = self.create_publisher(ScienceIn, "/Science/sensor_data", 10)
        
        self.i2c_timer = self.create_timer(0.1, self.i2c_callback)
        
        self.i2c_tx_buff = ScienceOut()
        self.send_pending = 0

        self.get_logger().info("\n------Science I2C Node Started------\n")


    def sci_out_sub_callback(self, msg: ScienceOut):
        '''Gets Science Out message'''
        self.msg_out = msg
        self.send_pending = 1
    
    def i2c_callback(self):
        '''sends ScienceOut to stm over i2c and publishes recieved sensor data to topic'''
        sci_in_msg = ScienceIn()

        if self.send_pending:
            self.i2c_tx_buff = self.msg_out
            self.send_pending = 0
        
        # This block uses list unpacking to assign values to the attributes of the msg 
        # Note: Take care of the order of these attributes
        
        buff = self.i2c_rw(reg_read, reg_write, read_length, self.i2c_tx_buff)
        sci_in_msg.temp = float(buff[0])
        sci_in_msg.humidity = float(buff[1])
        sci_in_msg.pressure = float(buff[2])
        sci_in_msg.n = int(buff[3])
        sci_in_msg.p = int(buff[4])
        sci_in_msg.k = int(buff[5])
        sci_in_msg.soiltemp = int(buff[6])
        sci_in_msg.soilmoist = int(buff[7])
        sci_in_msg.soilph = int(buff[8])
        sci_in_msg.uva = float(buff[9])
        sci_in_msg.uvb = float(buff[10])
        sci_in_msg.uvindex = float(buff[11])
        sci_in_msg.gas_sensor = int(buff[12])

        self.science_in_pub_.publish(sci_in_msg)
    
    def i2c_rw(self, reg_read, reg_write, read_length, msg):
        ''' sends the passed ScienceOut() msg over i2c on bus and reads '''
        array = [msg.auger_vert_vel, msg.auger_vert_dir,
                  msg.auger_hor_vel, msg.auger_hor_dir,
                  msg.drill_vel, msg.drill_dir,
                  msg.carousel_vel, msg.carousel_dir,
                  msg.reagent_direction,
                  msg.raman_vert_vel, msg.raman_vert_dir,
                  msg.raman_hor_vel, msg.raman_hor_dir
                ]
        bus.write_i2c_block_data(i2c_addr, reg_write, array)
        self.get_logger().info("Sent First Array to STM", once=True)
        self.get_logger().info(str(array))

        sensor_in = bus.read_i2c_block_data(i2c_addr, reg_read, read_length)
        self.get_logger().info("Recieved First Array from STM", once=True)

        return sensor_in

def main(args=None):
    # Initialization sequence
    rclpy.init(args=args)
    node = Sci_I2C_Node()
    
    # Runtime Excution statement
    rclpy.spin(node)

    # Termination Sequence
    rclpy.shutdown()
