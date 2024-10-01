#!/usr/bin/env python3

import numpy as np
import os
import rospy
import sys
from threading import Lock
from std_msgs.msg import Float32MultiArray
from stewart_end_effector.srv import StewartControl, ComputeMotorAngles, ComputeMotorAnglesResponse
from compute_motor_angles import normalize_motor_angles_zero_twopi, normalize_motor_angles_pi_minus_pi
from stewart_end_effector.srv import StewartControl, StewartControlRequest, StewartControlResponse


if os.name == 'nt':
    import msvcrt
    def getch():
        return msvcrt.getch().decode()
else:
    import sys, tty, termios
    fd = sys.stdin.fileno()
    old_settings = termios.tcgetattr(fd)
    def getch():
        try:
            tty.setraw(sys.stdin.fileno())
            ch = sys.stdin.read(1)
        finally:
            termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)
        return ch


from dynamixel_sdk.port_handler import PortHandler # goes into the port_handler.py file and imports the class PortHandler
from dynamixel_sdk.packet_handler import PacketHandler # does the same as the previous line but PacketHandler is a function
from dynamixel_sdk.group_sync_write import GroupSyncWrite
from dynamixel_sdk.group_sync_read import GroupSyncRead
from dynamixel_sdk.robotis_def import *
import dynamixel_sdk as dxl

#********* DYNAMIXEL Model definition *********
#***** (Use only one definition at a time) *****
MY_DXL = 'X_SERIES'       # X330 (5.0 V recommended), X430, X540, 2X430
# Control table address
if MY_DXL == 'X_SERIES' or MY_DXL == 'MX_SERIES':
    ADDR_TORQUE_ENABLE          = 64
    ADDR_GOAL_POSITION          = 116
    LEN_GOAL_POSITION           = 4         # Data Byte Length
    ADDR_PRESENT_POSITION       = 132
    LEN_PRESENT_POSITION        = 4         # Data Byte Length
    ADDR_HARDWARE_ERROR              = 70
    LEN_HARDWARE_ERROR          = 1
    DXL_MINIMUM_POSITION_VALUE  = 0         # Refer to the Minimum Position Limit of product eManual
    DXL_MAXIMUM_POSITION_VALUE  = 4095      # Refer to the Maximum Position Limit of product eManual
    BAUDRATE                    = 57600

# DYNAMIXEL Protocol Version (1.0 / 2.0)
# https://emanual.robotis.com/docs/en/dxl/protocol2/
PROTOCOL_VERSION            = 2.0

# Make sure that each DYNAMIXEL ID should have unique ID.
DXL1_ID                     = 1                 # Dynamixel#1 ID : 1
DXL2_ID                     = 2                 # Dynamixel#1 ID : 2
DXL3_ID                     = 3                 # Dynamixel#1 ID : 3
DXL4_ID                     = 4                 # Dynamixel#1 ID : 4
DXL5_ID                     = 5                 # Dynamixel#1 ID : 5
DXL6_ID                     = 6                 # Dynamixel#1 ID : 6

DXL_IDS = [DXL1_ID, DXL2_ID, DXL3_ID, DXL4_ID, DXL5_ID, DXL6_ID]

# Use the actual port assigned to the U2D2.
# ex) Windows: "COM*", Linux: "/dev/ttyUSB*", Mac: "/dev/tty.usbserial-*"
DEVICENAME                  = '/dev/output/stewart-motor'

TORQUE_ENABLE               = 1                 # Value for enabling the torque
TORQUE_DISABLE              = 0                 # Value for disabling the torque
DXL_MOVING_STATUS_THRESHOLD = 20                # Dynamixel moving status threshold

DEGREE_TO_POSITION_VALUE = (DXL_MAXIMUM_POSITION_VALUE - DXL_MINIMUM_POSITION_VALUE)/360.0


class MoveMotorAngles():
    def __init__(self):
        # Initialize PortHandler instance
        # Set the port path
        # Get methods and members of PortHandlerLinux or PortHandlerWindows
        self.portHandler = PortHandler(DEVICENAME)

        # Initialize PacketHandler instance
        # Set the protocol version
        # Get methods and members of Protocol1PacketHandler or Protocol2PacketHandler
        self.packetHandler = PacketHandler(PROTOCOL_VERSION)

        # Initialize GroupSyncWrite instance
        self.groupSyncWrite = GroupSyncWrite(self.portHandler, self.packetHandler, ADDR_GOAL_POSITION, LEN_GOAL_POSITION)

        # Initialize GroupSyncRead instance for Present Position
        self.groupSyncRead = GroupSyncRead(self.portHandler, self.packetHandler, ADDR_PRESENT_POSITION, LEN_PRESENT_POSITION)
        self.groupSyncReadAlarm = GroupSyncRead(self.portHandler, self.packetHandler, ADDR_HARDWARE_ERROR, LEN_HARDWARE_ERROR)

        self.pub = rospy.Publisher('actual_motor_positions', Float32MultiArray, queue_size=1)
        self.pub_srv_lock = Lock()

    
    def initialize(self):
        # Open port
        if self.portHandler.openPort():
            print("Succeeded to open the port")
        else:
            print("Failed to open the port")
            print("Press any key to terminate...")
            getch()
            quit()

        # Set port baudrate
        if self.portHandler.setBaudRate(BAUDRATE):
            print("Succeeded to change the baudrate")
        else:
            print("Failed to change the baudrate")
            print("Press any key to terminate...")
            getch()
            quit()

    
    def zero_platform(self):
        '''
        Zero the platform to start
        '''
        rospy.sleep(3)
        cmd = StewartControl()
        cmd.x, cmd.y, cmd.z, cmd.psi, cmd.theta, cmd.phi = 0, 0, 100, 0, 0, 0
        self.compute_motor_angles_client(cmd)

    # Initialize each of the servos with certain parameters
    def boot_servo(self, dxl_id):
        # TODO: ensure that torque does not need to be enabled when booting servos since auto-enable was set in the Dynamixel Wizard
        # Enable torques for each of the Dynamixels
        dxl_comm_result, dxl_error = self.packetHandler.write1ByteTxRx(self.portHandler, dxl_id, ADDR_TORQUE_ENABLE, TORQUE_ENABLE)
        if dxl_comm_result != COMM_SUCCESS:
            print("NO COMM SUCCESS")
            print(dxl_comm_result)
            print("%s" % self.packetHandler.getTxRxResult(dxl_comm_result))
        elif dxl_error != 0:
            print("DXL ERROR")
            print("%s" % self.packetHandler.getRxPacketError(dxl_error))
        else:
            print("Dynamixel#%d has been successfully connected" % dxl_id)

        ADDR_PROFILE_VELOCITY = 112
        DESIRED_MAX_VELOCITY = 30
        self.set_servo_param(dxl_id, 'profile velocity', 4, ADDR_PROFILE_VELOCITY, DESIRED_MAX_VELOCITY)

        ADDR_POSITION_D_GAIN = 80
        DESIRED_POSITION_D_GAIN = 0
        self.set_servo_param(dxl_id, 'position D gain', 2, ADDR_POSITION_D_GAIN, DESIRED_POSITION_D_GAIN)

        ADDR_POSITION_I_GAIN = 82
        DESIRED_POSITION_I_GAIN = 250
        self.set_servo_param(dxl_id, 'position I gain', 2, ADDR_POSITION_I_GAIN, DESIRED_POSITION_I_GAIN)

        ADDR_POSITION_P_GAIN = 84
        DESIRED_POSITION_P_GAIN = 500
        self.set_servo_param(dxl_id, 'position P gain', 2, ADDR_POSITION_P_GAIN, DESIRED_POSITION_P_GAIN)

        # ADDR_SHUTDOWN = 63
        # DESIRED_SHUTDOWN = 52
        # self.packetHandler.write1ByteTxRx(self.portHandler, dxl_id, ADDR_SHUTDOWN, DESIRED_SHUTDOWN)
        # self.set_servo_param(dxl_id, 'shutdown', 1, ADDR_SHUTDOWN, DESIRED_SHUTDOWN)


    # For servo dxl_id, set the servo param at param_address to the desired_value 
    def set_servo_param(self, dxl_id, param_string, num_bytes, param_address, desired_value):
        num_bytes_to_write_method = {1: self.packetHandler.write1ByteTxRx, 2: self.packetHandler.write2ByteTxRx, 4: self.packetHandler.write4ByteTxRx}

        try:
            write_method = num_bytes_to_write_method[num_bytes]
            dxl_comm_result, dxl_error = write_method(self.portHandler, dxl_id, param_address, desired_value)
        except KeyError:
            raise RuntimeError(f'Trying to set the value of {param_string} with {num_bytes} bytes. The allowed number of bytes are {list(num_bytes_to_write_method.keys())}')
        
        if dxl_comm_result != COMM_SUCCESS:
            print("%s" % self.packetHandler.getTxRxResult(dxl_comm_result))
        elif dxl_error != 0:
            print("%s" % self.packetHandler.getRxPacketError(dxl_error))
        else:
            print(f"Dynamixel#{dxl_id} has successfully set {param_string} to {desired_value}")


    # Synchronously set the positions of all motors in DXL_IDS to the corresponding position in motor_positions
    # List[Int] -> void
    def sync_write_motor_positions(self, motor_positions):
        self.read_ol_status()
        for i in range(len(motor_positions)-1,-1,-1):
            goal_position = motor_positions[i]
            dxl_id = DXL_IDS[i]
            param_goal_position = [DXL_LOBYTE(DXL_LOWORD(goal_position)), DXL_HIBYTE(DXL_LOWORD(goal_position)), DXL_LOBYTE(DXL_HIWORD(goal_position)), DXL_HIBYTE(DXL_HIWORD(goal_position))]
            dxl_addparam_result = self.groupSyncWrite.addParam(dxl_id, param_goal_position)
            if dxl_addparam_result != True:
                print("[ID:%03d] groupSyncWrite addparam failed" % dxl_id)
                quit()
        # Syncwrite goal position
        dxl_comm_result = self.groupSyncWrite.txPacket()
        if dxl_comm_result != COMM_SUCCESS:
            print("%s" % self.packetHandler.getTxRxResult(dxl_comm_result))
        # Clear syncwrite parameter storage
        self.groupSyncWrite.clearParam()

    
    # Set the positions of the motors to achieve a given desired pose
    # Float Float Float Float Float Float -> void
    def compute_motor_angles_client(self, req: StewartControl) -> StewartControl:
        with self.pub_srv_lock:
            x,y,z,psi,theta,phi = req.x,req.y,req.z,req.psi,req.theta,req.phi

            try:
                compute_motor_angles = rospy.ServiceProxy('compute_motor_angles', ComputeMotorAngles)
                motor_angles_resp = compute_motor_angles(x, y, z, psi, theta, phi)
                motor_angles = motor_angles_resp.motor_angles
                print(f'motor_angles: {motor_angles}')
                
                rescaled_motor_angles = normalize_motor_angles_zero_twopi(rescale_motor_angles(motor_angles))
                print(f'rescaled_motor_angles (in degrees): {" ".join(list(map(lambda rad: str(np.degrees(rad)), rescaled_motor_angles)))}')
                
                motor_positions = [angle_radian_to_motor_position(angle) for angle in rescaled_motor_angles]
                print(f'motor_positions: {motor_positions}')
                
                self.sync_write_motor_positions(motor_positions)
                return True , motor_positions
            except:
                # print("Service call failed: %s"%e)
                return False, []
    

    # Publish the current motor positions (according to the Dynamixel servos' encoders)
    def output_motor_positions(self):
        with self.pub_srv_lock:
            # Syncread present position
            for dxl_id in DXL_IDS:
                self.groupSyncRead.addParam(dxl_id)

            dxl_comm_result = self.groupSyncRead.txRxPacket()
            if dxl_comm_result != COMM_SUCCESS:
                print("%s" % self.packetHandler.getTxRxResult(dxl_comm_result))
            
            positions = []
            for dxl_id in DXL_IDS:
                # Check if groupsyncread data of Dynamixel#1 is available     
                dxl1_present_position = self.groupSyncRead.getData(dxl_id, ADDR_PRESENT_POSITION, LEN_PRESENT_POSITION)
                positions.append(dxl1_present_position)

            msg = Float32MultiArray()
            msg.data = positions
            self.pub.publish(msg)
        self.groupSyncRead.clearParam()

    
    def check_and_fix_ol(self, dxl_id):
        '''
        Identify servos that are in overload mode and reset them so they can continue following a desired trajectory
        '''
        print(f'Clearing alarm status for servo: {dxl_id}')
        # Send the instruction packet
        result, error = self.packetHandler.reboot(self.portHandler, dxl_id)
        rospy.sleep(0.25)
        if result != dxl.COMM_SUCCESS:
            print(f'Failed to send instruction packet. Error code: {self.packetHandler.getTxRxResult(result)}')
        # Reboot servo
        self.boot_servo(dxl_id)
        print(f'Alarm shutdown cleared for servo {dxl_id}')
    

    def read_ol_status(self):
        '''
        Identify which servos are in OL status
        '''
        for dxl_id in DXL_IDS:
            self.groupSyncReadAlarm.addParam(dxl_id)

        dxl_comm_result = self.groupSyncReadAlarm.txRxPacket()
        if dxl_comm_result != COMM_SUCCESS:
            print("%s" % self.packetHandler.getTxRxResult(dxl_comm_result))
        
        alarms = []
        for dxl_id in DXL_IDS: 
            alarm_status = self.groupSyncReadAlarm.getData(dxl_id, ADDR_HARDWARE_ERROR, LEN_HARDWARE_ERROR)
            alarms.append(alarm_status != 0)
            if alarm_status != 0:
                self.check_and_fix_ol(dxl_id)
        self.groupSyncReadAlarm.clearParam()
        print(f'Alarm status: {alarms}')


    def run(self):
        self.initialize()
        for dxl_id in DXL_IDS:
            self.boot_servo(dxl_id)
        self.read_ol_status()
        self.zero_platform()
        # Register a service with the callback to compute_motor_angles
        stewart_control_server = rospy.Service('stewart_control', StewartControl, self.compute_motor_angles_client)
        while not rospy.is_shutdown():
            self.output_motor_positions()
            rospy.sleep(0.01)


# Convert an angle in radians into a Dynamixel servo position
# Float -> Int
def angle_radian_to_motor_position(angle):
    return int(DEGREE_TO_POSITION_VALUE * np.degrees(angle))


# rescale the motor angles to be between the min and max bounds for the even/odd servos
# List[float] -> List[float]
def rescale_motor_angles(motor_angles):
    # TODO come up with a better naming for these
    MIN_ODD_ANGLE = np.radians(90)
    MAX_ODD_ANGLE = np.radians(230)
    MAX_EVEN_ANGLE = np.radians(270)
    MIN_EVEN_ANGLE = np.radians(130)
    NEUTRAL = 180
    rescaled = []
    for i in range(len(motor_angles)):
        angle_rad = motor_angles[i]
        angle_deg = np.degrees(angle_rad)
        if i % 2 != 0: # Because we index from 0, this is the handling for ODD servos 
            rescaled_angle_rad = np.radians(NEUTRAL + angle_deg)
            assert MIN_EVEN_ANGLE <= rescaled_angle_rad <= MAX_EVEN_ANGLE
            rescaled.append(rescaled_angle_rad)
        else:
            rescaled_angle_rad = np.radians(NEUTRAL - angle_deg)
            assert MIN_ODD_ANGLE <= rescaled_angle_rad <= MAX_ODD_ANGLE
            rescaled.append(rescaled_angle_rad)
    return rescaled


def usage():
    return "%s [x y z psi theta phi]"%sys.argv[0]

if __name__ == "__main__":
    rospy.init_node('MoveMotor', anonymous = True)
    move_motor_angles = MoveMotorAngles()
    move_motor_angles.run()
