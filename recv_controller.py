import socket
import threading
import controller
import ctypes
from interfaces.srv import IK 
import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32MultiArray, Float32MultiArray
import numpy as np
import time

sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
sock.bind(('192.168.0.10',3001))
sock.listen(5)

cont_buttons = controller.ButtonData()

TICKS_PER_REV = 1600


L1 = 44.7
L2 = 31.5
L3 = 8.8
L4 = 13.7

start_pos = [0,L1+L2+L3+L4,0]
rot = [0,0,0]

stick_thresh = 0.1
moving_to_start = False
target = None
rot = None
moving = False
az = None

a_pressed = False
last_buttons = cont_buttons

angles = [0,0,0,0,0]
flip_side = False
class MinimalClientAsync(Node):
    def __init__(self):
        super().__init__('arm_client')
        self.get_logger().set_level(rclpy.logging.LoggingSeverity.ERROR)
        self.cli = self.create_client(IK, 'arm_position')
        while not self.cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('service not available, waiting again...')
        self.req = IK.Request()
        self.pos_publisher = self.create_publisher(Int32MultiArray, 'arm_target_motor_positions', 10)
        self.current_arm_pos = [0,0,0]
        self.current_arm_rot = [0,0,0]
        self.angles = [0,0,0,0,0,0]
        self.get_pos = self.create_subscription(Float32MultiArray, 'current_cart', self.callback, 10)
        self.get_rot = self.create_subscription(Float32MultiArray, 'current_rot', self.rot_callback, 10)
        self.get_angles = self.create_subscription(Int32MultiArray, 'get_angles', self.angle_callback, 10)
    def callback(self,msg):
        self.current_arm_pos = list(msg.data)
            
    def rot_callback(self,msg):
        self.current_arm_rot = list(msg.data)
    
    def angle_callback(self,msg):
        self.angles = list(msg.data)
            
    def send_request(self,pos,rot):
        self.req.x = float(pos[0])
        self.req.y = float(pos[1])
        self.req.z = float(pos[2])
        self.req.rx = float(rot[0])
        self.req.ry = float(rot[1])
        self.req.rz = float(rot[2])
        print(self.req)
        self.future = self.cli.call_async(self.req)

rclpy.init(args=None)
ik_client = MinimalClientAsync()
gripper_pos = 0
azimuth_pos = 0

def client(sock):
    global cont_buttons, target, rot, az, moving, ik_client, moving_to_start, flip_side
    
    def rumble(num_pulses=1,pulse_length=0x80,forces=[0.1,0.1,0.1,0.1]):
        start_rumble = b'\x09\x08\x00\x08\x00'
        bitmask = 0
        for f in range(4):
            if forces[f] > 0:
                bitmask |= 1 << f
        start_rumble += bytes([bitmask])
        
        forces = [(int(i*256))>>2 for i in forces]
        start_rumble += bytes(forces)
        
        start_rumble += bytes([pulse_length])
        start_rumble += b'\x00'
        start_rumble += bytes([num_pulses])
        sock.send(start_rumble)
    def move_to(pos,rot,az,grip):
        ik_client.send_request(pos,rot)
        while not ik_client.future.done():
            rclpy.spin_once(ik_client)
        result = ik_client.future.result()
        if result.valid != 0:
            print(result)
            print("Invalid position - reason:",result.valid)
            rumble(pulse_length = 10,forces = [1,1,1,1])
        else:
            send = []
            send.append(az)
            #send.append(result.out.joint1/(2*np.pi)*TICKS_PER_REV)
            send.append(result.out.joint2/(2*np.pi)*TICKS_PER_REV*100)
            send.append(result.out.joint3/(2*np.pi)*TICKS_PER_REV*100)
            send.append(result.out.joint4/(2*np.pi)*TICKS_PER_REV*50)
            send.append(-(result.out.joint5/(2*np.pi)*TICKS_PER_REV*50*22/9))
            send.append(grip)
            send = [int(i) for i in send]
            send_angles(send)
        return result.valid

    while True:
        b = ctypes.create_string_buffer(sock.recv(1024))
        cont_buttons = controller.ButtonData.from_buffer(b)
        if not b:
            sock.close()
            return
        
        rclpy.spin_once(ik_client)
        buttons = cont_buttons
        #get distances of sticks from center
        l_stick_d = np.linalg.norm([buttons.stick_left_x/32768,buttons.stick_left_y/32768])
        r_stick_d = np.linalg.norm([buttons.stick_right_x/32768,buttons.stick_right_y/32768])
        
        norm_triggers = [buttons.trigger_left/1024,buttons.trigger_right/1024]
        
        #get current arm position and rotation
        arm_rot = np.array(ik_client.current_arm_rot)
        arm_pos = np.array(ik_client.current_arm_pos)
        gripper_pos = ik_client.angles[5]
        azimuth_pos = ik_client.angles[0]
        print(gripper_pos)
        
        if target is None:
            target = arm_pos
        if rot is None:
            rot = arm_rot
        if az is None:
            az = azimuth_pos
        
        #only do this if not currently using the moving to start macro
        if not moving_to_start:
            #if holding A button, rotate arm
            
            if buttons.a:
                if l_stick_d > stick_thresh:
                    rot = arm_rot + np.array([buttons.stick_left_x/32768, 0, 0])*0.2
                    moving = True
                elif l_stick_d < stick_thresh and moving:
                    rot = arm_rot
                    moving = False
            else:
                #otherwise, translate arm
                if l_stick_d > stick_thresh:
                    target = arm_pos + np.array([0,-buttons.stick_left_x/32768, buttons.stick_left_y/32768])*5
                    moving = True
                elif l_stick_d < stick_thresh and moving:
                    moving = False
                    target = arm_pos
                if r_stick_d > stick_thresh:
                    az = azimuth_pos + buttons.stick_right_x/32768*1000
        if norm_triggers[0] > stick_thresh:
            gripper_pos += norm_triggers[0]*2000
        if norm_triggers[1] > stick_thresh:
            gripper_pos -= norm_triggers[1]*2000
    
        a_pressed = buttons.a and not last_buttons.a
        b_pressed = buttons.b and not last_buttons.b
        x_pressed = buttons.x and not last_buttons.x
        if not moving_to_start:
            if move_to(target,rot,az,gripper_pos) != 0:
                target = arm_pos
                rot = arm_rot
        if b_pressed:
            flip_side = not flip_side
        if x_pressed:
            moving_to_start = True
            if move_to(start_pos,[0,0,0],az,gripper_pos) != 0:
                moving_to_start = False
            else:
                target = start_pos
                rot = [0,0,0]
        if moving_to_start:
            if np.linalg.norm(arm_pos-np.array(start_pos)) < 0.5:
                moving_to_start = False
        
        last_buttons = buttons
        time.sleep(1/60)

def send_angles(angles):
    global ik_client
    ik_client.pos_publisher.publish(Int32MultiArray(data=angles))
 
while True:
    c,add = sock.accept()
    print(c,"!!!")
    client_thread = threading.Thread(target=client,args=[c])
    client_thread.start()