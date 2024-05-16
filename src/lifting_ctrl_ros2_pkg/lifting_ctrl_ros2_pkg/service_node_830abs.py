import _thread
import time
import rclpy
from rclpy.node import Node
import argparse
import sys

from .lifting_motor_ctrl import C_LiftingMotorCtrl

from .lifting_ctrl_service_node import C_ROS_Server

from lifting_msg_pkg.msg import LiftMotorMsg

from lifting_msg_pkg.srv  import LiftMotorSrv

def main(args=None):
    rclpy.init(args=args)
    parser = argparse.ArgumentParser()
    parser.add_argument('--id', default=1)     # add_argument()指定程序可以接受的命令行选项
    args_output = parser.parse_args()      # parse_args()从指定的选项中返回一些数据
    server = C_ROS_Server(int(args_output.id))
    server.check_existing_nodes()

    server.StartThread(1)

    rclpy.spin(server)

    rclpy.shutdown()