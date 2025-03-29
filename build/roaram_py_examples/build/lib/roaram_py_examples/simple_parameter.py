#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rcl_interfaces.msg import SetParametersResult
from rclpy.parameter import Parameter

class Simple_parameter(Node): # MODIFY NAME
    def __init__(self):
        super().__init__("simple_parameter") # MODIFY NAME
        self.declare_parameter("simple_int_param", 28)
        self.declare_parameter("simple_str_param", "roarm")

        self.add_on_set_parameters_callback(self.param_change_callback)
    
    def param_change_callback(self, params):
        result = SetParametersResult()
        for param in params:
            if param.name == "simple_int_param" and param.name == Parameter.Type.INTEGER:
                self.get_logger().info(f"參數simple_int_param改變成新的值:{param.value}")
                result.successful = True
            if param.name == "simple_str_param" and param.name == Parameter.Type.STRING:
                self.get_logger().info(f"參數simple_str_param改變成新的字串:{param.value}")
                result.successful = True


def main(args=None):
    rclpy.init(args=args)
    node = Simple_parameter() # MODIFY NAME
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == "__main__":
    main()