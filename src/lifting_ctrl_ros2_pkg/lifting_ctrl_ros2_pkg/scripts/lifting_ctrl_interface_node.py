#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
import time

from bt_task_msgs.srv  import LiftInterfaceSrv, LiftInterfaceSrvRequest, LiftInterfaceSrvResponse 
from bt_task_msgs.srv  import LiftMotorSrv, LiftMotorSrvRequest, LiftMotorSrvResponse 
from bt_task_msgs.msg import LiftMotorMsg

class C_Server_Interface(Node):
    def __init__(self) -> None:
        super().__init__('lifting_ctrl_interface_node')
        
        self.motor_srv = self.create_service(
            LiftInterfaceSrv, 
            'LiftingMotorService', 
            self.SververCallbackBlock
        )
        
        self.motor_sub = None
        self.motor_pub = None
        
    def SubCallBack(self, msg):
        if self.motor_pub is None:
            self.motor_pub = self.create_publisher(
                LiftMotorMsg, 
                '/LiftMotorStatePub', 
                3
            )
        self.motor_pub.publish(msg)

    def SubscribeWithWaitForMessage(self, topic_name, timeout=3):
        received_msg = None
        
        def message_callback(msg):
            nonlocal received_msg
            received_msg = msg
        
        try:
            sub = self.create_subscription(
                LiftMotorMsg,
                topic_name,
                message_callback,
                1
            )
            
            # 等待消息或超时
            start_time = time.time()
            while rclpy.ok() and received_msg is None:
                elapsed_time = time.time() - start_time
                if elapsed_time >= timeout:
                    break
                    
                remaining_timeout = timeout - elapsed_time
                rclpy.spin_once(self, timeout_sec=min(0.5, remaining_timeout))
            
            # 清理订阅器
            self.destroy_subscription(sub)
            
            # 返回是否成功接收到消息
            return received_msg is not None
                
        except Exception as e:
            self.get_logger().error(f"Error waiting for message: {e}")
            return False
            
    def SververCallbackBlock(self, request, response):
        self.val = request.val
        self.mode = request.mode
        self.id = request.id
        
        sub_service_name = f'{self.id}' + '/LiftingMotorService'
        sub_topic_name = f'{self.id}' + '/LiftMotorStatePub'
        
        if self.mode == -10:
            if self.SubscribeWithWaitForMessage(sub_topic_name):
                if self.motor_sub is not None and self.motor_pub is not None:
                    self.destroy_publisher(self.motor_pub)
                    self.destroy_subscription(self.motor_sub)
                    
                self.motor_sub = self.create_subscription(
                    LiftMotorMsg, 
                    sub_topic_name, 
                    self.SubCallBack,
                    3
                )
                
                response.message = "Lift interface Start pub"
                response.success = True
                response.code = 13005
            else:
                response.message = "Lift interface Start pub failed"
                response.success = False
                response.code = 13007
            return response
            
        if self.mode == -11:
            if self.motor_sub is not None and self.motor_pub is not None:
                self.destroy_publisher(self.motor_pub)
                self.destroy_subscription(self.motor_sub)
                self.motor_sub = self.motor_pub = None
                response.message = "Lift interface Stop pub successfully"
                response.success = True
                response.code = 13011
            else:
                response.message = "Lift interface Stop pub failed"
                response.success = False
                response.code = 13012
            return response
        
        client = self.create_client(LiftMotorSrv, sub_service_name)
        
        if not client.wait_for_service(timeout_sec=10.0):
            print("service not available")

        req = LiftMotorSrv.Request()
        req.val = self.val
        req.mode = self.mode
        
        future = client.call_async(req)
        rclpy.spin_until_future_complete(self, future)
        
        service_response = future.result()
        
        if service_response.state == 1:
            response.message = "Lift execut command successfully"
            response.success = True
            response.code = 13000
        elif service_response.state == -1:
            response.message = "[error] Lift execut command failed"
            response.success = False
            response.code = 13001
        elif service_response.state == 2:
            response.message = "[error] Lift touched downLimit"
            response.success = False
            response.code = 13002
        elif service_response.state == -2:
            response.message = "[error] Lift touched upLimit"
            response.success = False
            response.code = 13003
        elif service_response.state == -9:
            response.message = "[error] Lift motor(port) Loss"
            response.success = False
            response.code = 13004
        elif service_response.state == -12:
            response.message = "[error] Lift execute timeout!!!"
            response.success = False
            response.code = 13007
        elif service_response.state == -13:
            response.message = "[error] Lift execute timeout & init failed!!!"
            response.success = False
            response.code = 13007
        elif service_response.state == -14:
            response.message = "[error] Lift init failed!!!"
            response.success = False
            response.code = 13008
        else:
            response.message = "[warning] Lift mode code unknown"
            response.success = False
            response.code = 13010
        
        return response

def main(args=None):
    rclpy.init(args=args)
    
    try:
        node = C_Server_Interface()
        rclpy.spin(node)
        
    except Exception as e:
        print(f"Service call failed: {e}")
        
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == "__main__":
    main()
