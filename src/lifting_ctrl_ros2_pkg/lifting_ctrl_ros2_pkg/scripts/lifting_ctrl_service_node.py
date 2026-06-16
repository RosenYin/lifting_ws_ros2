#!/usr/bin/env python3
# 升降柱控制服务端节点
# 同时将接收到的串口数据，publish出去
# 开了一个线程用来pub数据
# import rospy
import _thread
import time
import rclpy
from rclpy.node import Node
import argparse
import sys
from .lifting_motor_ctrl import C_LiftingMotorCtrl
from lifting_msg_pkg.msg import LiftMotorMsg
from lifting_msg_pkg.srv  import LiftMotorSrv

class C_ROS_Server(Node):
    def __init__(self, id:int=None) -> None:
        '''
        初始化节点
        '''
        self.node_name = 'lifting_ctrl_server_node_830abs_' + str(id)
        super().__init__(self.node_name)
        print("寻找已经存在的节点")
        self.motor_id = id
        self.ctrl = C_LiftingMotorCtrl(self.motor_id)
        self.check_existing_nodes()
        self.ctrl.MotorConfigInit()
        # self.declare_parameter('motor_id', 1)
        # self.motor_id = self.get_parameter('motor_id').value
        # self.get_logger().info(f"motor_id is {self.motor_id}")
        
        # 电机控制类实例化 
        self.input_callback_time = time.time()
        lift_port = self.ctrl.GetLiftPortName()
        
        # 设置阻塞式回调函数超时时间，默认为300s，即5分钟
        if('responseTimeout' in self.ctrl.GetMotorJsonConfig().keys()):
            self.response_timeout = self.ctrl.GetMotorJsonConfig()["responseTimeout"]
        else: self.response_timeout = 60
        
        # 设置回调函数模式，如果是1则是立刻返回，0为阻塞式回调
        if('callBackMode' in self.ctrl.GetMotorJsonConfig().keys()):
            self.callBackMode = self.ctrl.GetMotorJsonConfig()["callBackMode"]
        else: self.callBackMode = 1
        # topicHz
        if('topicHz' in self.ctrl.GetMotorJsonConfig().keys()):
            self.topicHz = self.ctrl.GetMotorJsonConfig()["topicHz"]
        else: self.topicHz = -1
        # 设定执行器末端速度，单位mm/s
        self.liftTargetSpd = abs((self.ctrl.GetMotorMaxSpd() * 10000) / (self.ctrl.reductionRatio * 60))
        if('initSpd' in self.ctrl.GetMotorJsonConfig().keys()):
            self.initSpd = self.ctrl.GetMotorJsonConfig()["initSpd"]
        else: self.initSpd = -100
        
        # 初始化位置
        if('initPos' in self.ctrl.GetMotorJsonConfig().keys()):
            self.__initPos = (self.ctrl.GetMotorJsonConfig()["initPos"])
        else: self.__initPos = 0
        
        # 读取电机数据
        self.motor_states = self.ctrl.ReadMotorData()
        # 获取目标位置
        self.target_height = self.ctrl.GetTargetHeight()
        self.target_speed = 0
        self.ctrl.LiftMovePos(self.target_height)
        
        # 初始阶段为初始化模式
        if('initMode' in self.ctrl.GetMotorJsonConfig().keys()):
            self.mode = (self.ctrl.GetMotorJsonConfig()["initMode"])
        else: self.mode = 1
        
        self.fps_error=0
        self.back_height = 0
        if isinstance(self.motor_states, C_LiftingMotorCtrl.motor_msg):
            self.back_height = self.motor_states.back_lift_height
        
        self.motor_msgs = LiftMotorMsg()
        self.motor_pub = self.create_publisher(LiftMotorMsg, 'LiftMotorStatePub', 1)
        if(self.callBackMode == 0):
            self.motor_srv = self.create_service(LiftMotorSrv, 'LiftingMotorService', self.SververCallbackBlock)
        elif(self.callBackMode == 1):
            self.motor_srv = self.create_service(LiftMotorSrv, 'LiftingMotorService', self.SververCallbackImmediately)
        self.first_up_flag = False
        self.first_down_flag = False
        self.previous_dir = 0
        self.command_send = 0
        self.offset = 0
        
        # json文件读取程序启动时候的模式
        if(self.mode != 0):
            self.init_state:bool = False
        else: self.init_state:bool = True
        
        self.overload_flag:bool = False
        self.callLock = self.last_callLock = True
        self.timeoutFlag:bool = False
        self.print_flag_init = True
    
    def check_existing_nodes(self):
        # 从参数服务器获取所有节点的名称
        existing_nodes = self.get_node_names()
        print(existing_nodes)
        if existing_nodes.count(self.node_name)>1:
            self.get_logger().error(f"A node with name '{self.node_name}' already exists.")
            sys.exit(1)
    
    def JudgeFirstUpOrDown(self):
        '''
        判断电机是否切换状态，比如上一次是上升，当前要发送下降，那么判断当前是上升后的第一次下降，反之亦然
        '''
        dir = self.JudgeMotorDirection()
        if(self.command_send == 1):
            self.command_send = 0
            if(dir == 1 and self.previous_dir == -1):
                self.first_up_flag = True
                self.first_down_flag = False
            elif(dir == -1 and self.previous_dir == 1): 
                self.first_up_flag = False
                self.first_down_flag = True
            elif(dir == 1 and self.previous_dir == 1):
                self.first_up_flag = False
            elif(dir == -1 and self.previous_dir == -1):
                self.first_down_flag = False
        if(dir != 0):
            self.previous_dir = dir
        if(self.first_up_flag): 
            self.offset = 3
        if(self.first_down_flag):
            self.offset = -3
        
    def PublishMotorMsgs(self) -> None:
        '''
        将要pub出去的数据赋值
        '''
        if isinstance(self.motor_states, C_LiftingMotorCtrl.motor_msg):
            # 当前话题电机id
            self.motor_msgs.motor_id = self.motor_id
            # 初始化状态
            self.motor_msgs.init_state = self.init_state
            # print("---------------")
            # 电机控制模式
            self.motor_msgs.ctrl_mode = self.motor_states.ctrl_mode
            # 电机反馈电压
            self.motor_msgs.back_vol = self.motor_states.back_vol
            # 电机反馈电流
            self.motor_msgs.back_current = self.motor_states.back_current
            # 电机状态位
            self.motor_msgs.state_bit = self.motor_states.state_bit
                # 电机是否运行，0-停机，1-启动
            self.motor_msgs.run_state = (bool)(self.motor_msgs.state_bit&0x01)
                # 过流
            self.motor_msgs.overflow_i = (bool)(self.motor_msgs.state_bit&0x02)
                # 过压
            self.motor_msgs.overflow_v = (bool)(self.motor_msgs.state_bit&0x04)
                # 编码器故障
            self.motor_msgs.encode_err = (bool)(self.motor_msgs.state_bit&0x8)
                # 位置偏差过大
            self.motor_msgs.pos_bias_over = (bool)(self.motor_msgs.state_bit&0x10)
                # 欠压
            self.motor_msgs.under_vol = (bool)(self.motor_msgs.state_bit&0x20)
                # 过载标志
            self.motor_msgs.over_load = (bool)(self.motor_msgs.state_bit&0x40)
            # 外部控制标志 = 0-pc ； 1-外部 PLC
            self.motor_msgs.external_ctrl_mode = (bool)(self.motor_msgs.state_bit&0x80)
            # 电机转速
            self.motor_msgs.motor_speed = self.motor_states.back_speed
            # 升降柱转速abs(round((self.motor_states.back_speed*10000) / (self.ctrl.reductionRatio * 60))
            self.motor_msgs.back_speed = round((self.motor_states.back_speed*10000) / (self.ctrl.reductionRatio * 60)) # mm/s
            # 电机定位完成
            self.motor_msgs.location_complete = self.motor_states.location_complete
            # 电机反馈位置
            self.motor_msgs.back_pos = self.motor_states.back_pos
            # 升降柱当前高度
            self.motor_msgs.back_height = self.motor_states.back_lift_height
            # 升降柱目标高度
            # self.motor_msgs.target_height = self.ctrl.GetTargetHeight()
            self.motor_msgs.target_height = self.ctrl.GetTargetHeight()
            # 电机目标位置
            self.motor_msgs.target_pos = self.ctrl.GetTargetPos()
            # 电机上限位
            self.motor_msgs.up_limit = (self.motor_states.upLimit)
            # 电机下限位
            self.motor_msgs.down_limit = (self.motor_states.downLimit)
            self.motor_msgs.up_limit_height = self.ctrl.GetUpLimitHeight()
            self.motor_msgs.down_limit_height = self.ctrl.GetDownLimitHeight()
        # 电机是否到达了目标位置，误差小于1认为到达了
        if(abs(self.target_height - self.back_height) < 1): 
            self.motor_msgs.reach_target_pos = True
        else: self.motor_msgs.reach_target_pos = False
        self.motor_msgs.stop_flag = self.ctrl.GetStopFlag()
        self.motor_msgs.fps_error = self.fps_error
            # 发布
        try:
            self.motor_pub.publish(self.motor_msgs)
        except Exception as e:
            print("发布失败",e)
            pass
    
    def JudgeMotorDirection(self):
        '''
        判断电机方向,1上升，-1下降，0不动
        '''
        # 正向，向上升
        if(self.target_height-self.back_height > 0):
            return 1
        # 向下降
        elif(self.target_height-self.back_height < 0):
            return -1
        # 不动
        else:
            return 0 
    
    def JudgeMotorDirectionWithSpeed(self):
        '''
        判断电机方向，1正方向，-1负方向，0不动
        '''
        if(self.target_speed > 0):
            # print("电机正方向速度运动")
            return 1
        elif(self.target_speed < 0):
            # print("电机负方向速度运动")
            return -1
        # 不动
        elif(self.target_speed == 0):
            return 0 
        else: return 2
    
    def SververCallbackImmediately(self, request, response):
        '''
        830ABS 服务端回调函数,立刻反馈数值，不等待电机到达位置就反馈
        '''
        self.input_callback_time = time.time()
        self.command_send = 1

        self.mode = request.mode
        resp = -1
        self.timeoutFlag = False
        init_state_lock = True
        if(self.callLock == self.last_callLock):
            self.callLock = not self.last_callLock
            # 初始化模式
            if(request.mode == 1):
                if(init_state_lock):
                    self.init_state = False
                    init_state_lock = False
                if(self.init_state): resp = 1
                else: resp = -1
            # 位置模式
            elif(request.mode == 0 and self.init_state):
                self.target_height = self.ctrl.SetTargetHeight(request.val)
                self.get_logger().info(f"target height: {self.target_height}")
                # 电机是否到达了目标位置，误差小于1认为到达了，反馈1为到达，反馈-1为未到达
                if(abs(self.target_height - self.back_height) < 2):
                    resp = 1
                else: resp = -1
            # 急停
            elif(request.mode == -2 and self.init_state):
                if isinstance(self.motor_states, C_LiftingMotorCtrl.motor_msg):
                    if(self.motor_states.back_speed == 0):
                        resp = 1
                else: resp = -1
            # -3 指令用来初始化电机，并清空错误帧率
            elif(request.mode == -3):
                resp = 1
                # else: resp = -1
            # 恒定速度
            elif(request.mode == -4 and self.init_state):
                # self.get_logger().info(f"the max speed is: {self.liftTargetSpd}")
                self.ctrl.SetTargetHeight(self.back_height)
                self.target_speed = request.val
                self.get_logger().info(f"----------操作电机 {self.motor_id} 目标速度为 {self.target_speed}")
                if(self.target_speed > abs(self.liftTargetSpd)): self.target_speed=abs(self.liftTargetSpd)
                if(self.target_speed < -abs(self.liftTargetSpd)): self.target_speed=-abs(self.liftTargetSpd)
                self.get_logger().info(f"----------操作电机 {self.motor_id} 最终发送速度为 {self.target_speed}")
                if isinstance(self.motor_states, C_LiftingMotorCtrl.motor_msg):
                    self.get_logger().info(str(self.motor_states.back_speed))
                    if(abs(round((self.motor_states.back_speed*10000) / (self.ctrl.reductionRatio * 60)) - self.target_speed)<20):
                        resp = 1
                else: resp = -1
            elif(request.mode == -5 and self.init_state):
                self.target_speed = request.val
                if(self.target_speed > self.liftTargetSpd): self.target_speed=self.liftTargetSpd
                if(self.target_speed < -self.liftTargetSpd): self.target_speed=-self.liftTargetSpd
                self.target_speed = round((self.target_speed * self.ctrl.reductionRatio * 60)/10000)
                self.ctrl.MotorSetMaxSpd(self.target_speed)
                resp = 1
            elif(request.mode == -6):
                if(self.target_speed ==0 ):
                    self.ctrl.MotorClearPosZero()
                else:
                    resp = -15
                if(abs(self.motor_states.back_pos) <= 1):
                    resp = 1
                else:
                    resp = -1
            # 恒定速度
            elif(request.mode == -7 and self.init_state):
                # self.get_logger().info(f"the max speed is: {self.liftTargetSpd}")
                self.ctrl.SetTargetHeight(self.back_height)
                self.target_speed = request.val
                max_spd = self.ctrl.GetMotorMaxSpd()
                self.get_logger().info(f"----------操作电机 {self.motor_id} 目标速度为{self.target_speed}")
                if(self.target_speed > max_spd): self.target_speed=max_spd
                if(self.target_speed < -max_spd): self.target_speed=-max_spd
                self.get_logger().info(f"----------操作电机 {self.motor_id} 最终发送速度数值为 {self.target_speed}")
                if isinstance(self.motor_states, C_LiftingMotorCtrl.motor_msg):
                    self.get_logger().info(str(self.motor_states.back_speed))
                    if(abs(self.motor_states.back_speed - self.target_speed)<20):
                        resp = 1
                else: resp = -1
            else:
                resp = 0
                if(not self.init_state):
                    resp = -14

            if(resp == -1):
                if(self.init_state == True):
                    if isinstance(self.motor_states, C_LiftingMotorCtrl.motor_msg):
                        if(self.motor_states.upLimit):
                            if(self.mode == 0 and self.JudgeMotorDirection()==-1):
                                resp = -1
                            elif(self.mode == -4 and self.JudgeMotorDirectionWithSpeed()==-1):
                                resp = -1
                            else: resp = -2
                        elif(self.motor_states.downLimit):
                            if(self.mode == 0 and self.JudgeMotorDirection()==1):
                                resp = -1
                            elif(self.mode == -4 and self.JudgeMotorDirectionWithSpeed()==1):
                                resp = -1
                            else: resp = 2
                else:
                    resp = -1
            # 如果错误帧率达到4以上
            if(self.fps_error > 4):
                resp = -9

            if(time.time()  - self.input_callback_time > self.response_timeout):
                self.mode == -2
                self.timeoutFlag = True
                self.get_logger().error("lifting_motor timeout!!!!")
                resp = -12
                if(not self.init_state):
                    resp = -13
        self.last_callLock = self.callLock
        self.get_logger().info(f"resquest的mode为{self.mode}")
        self.get_logger().info(f"resquest的val为{request.val}")
        self.get_logger().info(f"服务端的应答为{resp}")
        response.resp = resp
        return response

    def SververCallbackBlock(self, req, resp):
        '''
        服务端回调函数，阻塞式，等待电机到达目标位置才会反馈
        '''
        self.input_callback_time = time.time()
        self.command_send = 1
        
        self.mode = req.mode
        resp.state = -1
        response = -1
        self.timeoutFlag = False
        init_state_lock = True
        
        if(self.callLock == self.last_callLock):
            while response == -1:
                self.callLock = not self.last_callLock
                # 初始化模式
                if(req.mode == 1):
                    if(init_state_lock):
                        self.init_state = False
                        init_state_lock = False
                    if(self.init_state): response = 1
                    else: response = -1
                # 位置模式
                elif(req.mode == 0 and self.init_state):
                    self.target_height = self.ctrl.SetTargetHeight(req.val)
                    # self.get_logger().info(f"target height: {self.target_height}")
                    # 电机是否到达了目标位置，误差小于1认为到达了，反馈1为到达，反馈-1为未到达
                    if(abs(self.target_height - self.back_height) < 2): 
                        response = 1
                    else: response = -1
                # 急停
                elif(req.mode == -2 and self.init_state):
                    if isinstance(self.motor_states, C_LiftingMotorCtrl.motor_msg):
                        if(self.motor_states.back_speed == 0):
                            response = 1
                    else: response = -1
                # -3 指令用来初始化电机，并清空错误帧率
                elif(req.mode == -3):
                    response = 1
                # 恒定速度
                elif(req.mode == -4 and self.init_state):
                    self.ctrl.SetTargetHeight(self.back_height)
                    self.target_speed = req.val
                    print(f"----------操作电机 {self.motor_id} 目标速度为{self.target_speed}")
                    if(self.target_speed > abs(self.liftTargetSpd)): self.target_speed=abs(self.liftTargetSpd)
                    if(self.target_speed < -abs(self.liftTargetSpd)): self.target_speed=-abs(self.liftTargetSpd)
                    print(f"----------操作电机 {self.motor_id} 最终发送速度为{self.target_speed}")
                    if isinstance(self.motor_states, C_LiftingMotorCtrl.motor_msg):
                        print(self.motor_states.back_speed)
                        if(abs(round((self.motor_states.back_speed*10000) / (self.ctrl.reductionRatio * 60)) - self.target_speed)<5):
                            response = 1
                    else: response = -1
                elif(req.mode == -5 and self.init_state):
                    self.target_speed = req.val
                    if(self.target_speed > self.liftTargetSpd): self.target_speed=self.liftTargetSpd
                    if(self.target_speed < -self.liftTargetSpd): self.target_speed=-self.liftTargetSpd
                    self.target_speed = round((self.target_speed * self.ctrl.reductionRatio * 60)/10000)
                    self.ctrl.MotorSetMaxSpd(self.target_speed)
                    response = 1
                elif(req.mode == -6):
                    if(self.target_speed ==0 ):
                        self.ctrl.MotorClearPosZero()
                    else:
                        response = -15
                    if(abs(self.motor_states.back_pos) <= 1):
                        response = 1
                    else:
                        response = -1
                # 恒定速度
                elif(req.mode == -7 and self.init_state):
                    # print(f"the max speed is: {self.liftTargetSpd}")
                    self.ctrl.SetTargetHeight(self.back_height)
                    self.target_speed = req.val
                    max_spd = self.ctrl.GetMotorMaxSpd()
                    print(f"----------操作电机 {self.motor_id} 目标速度为{self.target_speed}")
                    if(self.target_speed > max_spd): self.target_speed=max_spd
                    if(self.target_speed < -max_spd): self.target_speed=-max_spd
                    print(f"----------操作电机 {self.motor_id} 最终发送速度为{self.target_speed}")
                    
                    if isinstance(self.motor_states, C_LiftingMotorCtrl.motor_msg):
                        # print(f"{self.motor_states.back_speed}")
                        if(abs(self.motor_states.back_speed - self.target_speed)<5):
                            response = 1
                    else: response = -1
                else:
                    response = 0
                    if(not self.init_state):
                        response = -14
                
                if(response == -1):
                    if(self.init_state == True):
                        if isinstance(self.motor_states, C_LiftingMotorCtrl.motor_msg):
                            if(self.motor_states.upLimit):
                                if(self.mode == 0 and self.JudgeMotorDirection()==-1):
                                    response = -1
                                elif(self.mode == -4 and self.JudgeMotorDirectionWithSpeed()==-1):
                                    response = -1
                                else: response = -2
                            elif(self.motor_states.downLimit):
                                if(self.mode == 0 and self.JudgeMotorDirection()==1):
                                    response = -1
                                elif(self.mode == -4 and self.JudgeMotorDirectionWithSpeed()==1):
                                    response = -1
                                else: response = 2
                    else: 
                        response = -1
                # 如果错误帧率达到4以上
                if(self.fps_error > 4):
                    response = -9
                
                if(time.time()  - self.input_callback_time > self.response_timeout):
                    self.mode == -2
                    self.timeoutFlag = True 
                    # self.get_logger().error("lifting_motor timeout!!!!")
                    response = -12
                    if(not self.init_state):
                        response = -13
                    break
                time.sleep(0.5)
        
        resp.state = response
        self.last_callLock = self.callLock
        print(f"-----resquest的mode为{self.mode}")
        print(f"-----resquest的val为{req.val}")
        print(f"-----服务端的应答为{resp}")
        return resp
    
    def ModeCtrl(self):
        '''
        电机模式控制
        '''
        # if(self.timeoutFlag):
        #     if(self.init_state == False):
        #         self.get_logger().info(f"{self.motor_id} 号电机初始化超时,直接将当前位置作为初始化位置")
        #         self.init_state = True
        #         self.ctrl.LiftTimeoutInit(True, self.motor_states)
        #     self.mode == -2
        
        # 判断初始化标志位，进行电机初始化
        # if(not self.init_state): self.mode = 1
        # print("current_mode: ",self.mode)
        
        if(self.mode == 1 and self.fps_error <= 3):
            # print("初始化ing...速度为：",self.initSpd)
            self.ctrl.SetStopFlag(False)
            if(not self.overload_flag):
                if(self.print_flag_init):
                    print("开始初始化...")
                    # self.get_logger().info("开始下限位初始化...")
                self.init_state = self.ctrl.LiftLimitInit(self.initSpd, self.motor_states)
            
            if(self.init_state): 
                self.mode = 0xF1
                self.target_height = self.__initPos
                print("初始化完成,over")
                time.sleep(1)
            else:
                if(self.print_flag_init):
                    print("未完成初始化...")
                if(self.motor_msgs.over_load or self.motor_msgs.back_current > self.ctrl.GetOverflowILimit()):
                    self.overload_flag = True
                    print("检测到电机过载，可能是电机卡住或者下限位失效，在当前位置进行初始化")
                    self.init_state = True
                    self.ctrl.LiftTimeoutInit(True, self.motor_states)
                    self.mode = 0xF2
                if(self.timeoutFlag):
                    print(f"{self.motor_id} 号电机初始化超时,直接将当前位置作为初始化位置")
                    self.init_state = True
                    self.ctrl.LiftTimeoutInit(True, self.motor_states)
                    self.mode = 0xF2
            self.print_flag_init = False
        
        if(self.mode == 0xF1):
            if(self.__initPos > 0):
                self.ctrl.MotorStop(self.JudgeMotorDirection(), self.motor_states, force=False)
                # 运动电机
                self.ctrl.LiftMovePos(self.target_height)
        
        if(self.mode == 0xF2):
            self.ctrl.LiftMoveSpd(0)
            self.ctrl.MotorStop(self.JudgeMotorDirectionWithSpeed(), self.motor_states, force=False)
        
        if(self.mode == -2):# -2 指令用来紧急停止电机
            self.target_speed = 0
            self.ctrl.LiftMoveSpd(self.target_speed)
            self.ctrl.MotorStop(self.JudgeMotorDirectionWithSpeed(), self.motor_states, force=True)
        
        if(self.mode == -3):# -3 指令用来初始化电机，并清空错误帧率
            self.ctrl.MotorClearBug()
            self.fps_error = 0
        
        if(self.mode == -4 and self.fps_error <= 1):# -4指令用来运动电机指定速度
            self.ctrl.SetTargetHeight(self.back_height)
            self.ctrl.LiftMoveSpd(self.target_speed)
            self.ctrl.MotorStop(self.JudgeMotorDirectionWithSpeed(), self.motor_states, force=False)
        
        if(self.mode == -5 and self.fps_error <= 1):# -5指令用来设置位置模式下电机最大速度
            self.ctrl.MotorSetMaxSpd(self.target_speed)
        
        if(self.mode == -6 and self.fps_error <= 1):# -6指令用来电机位置清零
            self.ctrl.MotorClearPosZero()
        
        if(self.mode == -7 and self.fps_error <= 1):# -7指令用来运动i电机指定速度
            self.ctrl.SetTargetHeight(self.back_height)
            self.ctrl.MotorMoveSpd(self.target_speed)
            self.ctrl.MotorStop(self.JudgeMotorDirectionWithSpeed(), self.motor_states, force=False)
        
        if(self.mode == 0 and self.fps_error <= 1):
            self.ctrl.MotorStop(self.JudgeMotorDirection(), self.motor_states, force=False)
            # 运动电机
            self.ctrl.LiftMovePos(self.target_height)
        
        # 过流保护
        if(self.ctrl.OverflowIProtect()):
            self.ctrl.MotorStop(self.JudgeMotorDirection(), self.motor_states, force=True)
            # self.get_logger().warning("电机电流超过11A并且反馈速度为0，判定是卡住")
            print("电机电流超过11A并且反馈速度为0，判定是卡住")
            self.target_speed = 0
            self.ctrl.LiftMoveSpd(self.target_speed)
            self.ctrl.MotorStop(self.JudgeMotorDirectionWithSpeed(), self.motor_states, force=True)
            self.ctrl.MotorClearBug()
    
    # 更新电机信息
    def UpdateDataAndCtrlThread(self, state) -> None:
        success_fps = 0
        while state == 1:
            self.motor_states = self.ctrl.ReadMotorData() #读取电机状态消息
            # ## 返回的字符串消息如果非空，publish电机消息并赋值当前升降柱高度
            # print(self.motor_states)
            if isinstance(self.motor_states, C_LiftingMotorCtrl.motor_msg):
                self.back_height = self.motor_states.back_lift_height
                success_fps = success_fps + 1
                if(success_fps > 5):  self.fps_error = 0
            # ## 如果反馈消息为空，计算错误帧率
            else: 
                success_fps = 0
                self.fps_error = self.fps_error + 1
            # 发布消息
            if(self.topicHz <= 0):
                self.PublishMotorMsgs()
            # ## 模式控制
            self.ModeCtrl()
    
    # 发布电机各种信息线程
    def PubThread(self, state) -> None:
        while rclpy.ok():
            # 发布消息
            self.PublishMotorMsgs()
            time.sleep(1/self.topicHz)

    # 发布电机各种信息线程
    def Thread(self, state) -> None:
        success_fps = 0
        while state == 1:
            self.motor_states = self.ctrl.ReadMotorData() #读取电机状态消息
            
            if isinstance(self.motor_states, C_LiftingMotorCtrl.motor_msg):
                self.back_height = self.motor_states.back_lift_height
                success_fps = success_fps + 1
                if(success_fps > 5):  self.fps_error = 0
            else: 
                success_fps = 0
                self.fps_error = self.fps_error + 1
            
            self.PublishMotorMsgs()
            self.ModeCtrl()
            time.sleep(0.02)
    
    # def StartThread(self, state) -> None:
    #     _thread.start_new_thread(self.Thread, (state,))
    
    def StartThread(self, state) -> None:
        # _thread.start_new_thread(self.Thread, (state,))
        _thread.start_new_thread(self.UpdateDataAndCtrlThread, (state,))
        if(self.topicHz > 0):
            _thread.start_new_thread(self.PubThread, (state,))

# def main(args=None):
#     rclpy.init(args=args)
#     parser = argparse.ArgumentParser()
#     parser.add_argument('--id', default=1)     # add_argument()指定程序可以接受的命令行选项
#     args_output = parser.parse_args()      # parse_args()从指定的选项中返回一些数据
#     server = C_ROS_Server(int(args_output.id))
#     server.StartThread(1)
#     rclpy.spin(server)
#     rclpy.shutdown()

# if __name__ == '__main__':
#     main()
