#!usr/bin/env python

import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32MultiArray


class EncoderDataSubscriber(Node):
    def __init__(self):
        super().__init__('Encoder_Data_Subscriber')
        self.subscriber = self.create_subscription(
            Int32MultiArray,
            '/get_ticks',
            self.encoder_data_callback,
            10
        )
        self.encoder_data_1 = 0
        self.encoder_data_2 = 0
        
    def encoder_data_callback(self, msg):
        self.encoder_data_1 = msg.data[0]
        self.encoder_data_2 = msg.data[1]
        self.get_logger().info(f"Received encoder data: {self.encoder_data_1}, {self.encoder_data_2}")    
    
    
def main(args=None):
    rclpy.init(args=args)
    node = EncoderDataSubscriber()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
    
if __name__=='__main__':
    main()