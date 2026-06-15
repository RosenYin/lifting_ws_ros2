import _thread
import time
import sys
import rclpy
from rclpy.node import Node
import argparse

from .scripts.lifting_motor_ctrl_850pro import C_LiftingMotorCtrl_850pro

from .scripts.lifting_ctrl_service_node_850pro import C_ROS_Server

from lifting_msg_pkg.msg import LiftMotorMsg

from lifting_msg_pkg.srv  import LiftMotorSrv

def main(args=None):
    rclpy.init(args=args)
    
    # 解析命令行参数，但忽略 ROS 2 的系统参数
    custom_args = []
    i = 0
    while i < len(sys.argv):
        if sys.argv[i] == '--id' and i + 1 < len(sys.argv):
            custom_args.extend(['--id', sys.argv[i + 1]])
            i += 2
        elif sys.argv[i].startswith('--ros-args'):
            # 跳过 ROS 2 参数
            i += 1
            while i < len(sys.argv) and sys.argv[i].startswith('-'):
                i += 1
        else:
            i += 1
    
    # 使用 parse_known_args 解析参数
    import argparse
    parser = argparse.ArgumentParser()
    parser.add_argument('--id', default=1, type=int)
    
    args_output, unknown = parser.parse_known_args(custom_args)
    
    # 创建服务器
    server = C_ROS_Server(int(args_output.id))
    server.check_existing_nodes()
    server.StartThread(1)

    rclpy.spin(server)
    rclpy.shutdown()