#!/usr/bin/python3
import re
import rospy
import serial
import traceback
import numpy as np
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Int32
from stewart_end_effector.msg import CoordinateData
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
from stewart_end_effector.srv import StewartControl, StewartControlRequest, StewartControlResponse, Light


class ToFDriver():
    def __init__(self):
        # Initialize empty store to aggregate spectral readings
        # Port name
        self.port_path = rospy.get_param('port', '/dev/ttyACM0')

        # Initialize the spectrometer the Baudrate must equal 115200
        self.tof = serial.Serial(self.port_path, baudrate=9600)
        self.tof.flushInput()
        self.tof.flushOutput()
        self.stewart_height = 100
        
        # Initialize publishers
        # Publish the distance to object in millimeters
        self.pub = rospy.Publisher('/tof/coords', CoordinateData, queue_size=1)
        self.measurements = []
        
        # Define shutdown behavior
        rospy.on_shutdown(self.shutdown)
        # Connect to planning service
        # self.stewart_command = rospy.ServiceProxy('stewart_control', StewartControl)

        #Initialize service utilities
        self.service_light = rospy.Service("/stewart/light_power", Light, self.set_light_power)
        
        # Buffer of commands to write to the device
        self.command_buffer = []

    def plane(self, vals):
        '''
        (mm)
        ToF 1: (0, -22, Distance 1)  -> A
        ToF 2: (20, 10, Distance 2)  -> B
        ToF 3: (-18, 14, Distance 3) -> C
        '''
        # print(vals)
        x1 = 0
        y1 = -22
        z1 = int(vals[0])
        p1 = np.array([x1,y1,z1])
        x2 = 20
        y2 = 10
        z2 = int(vals[1])
        p2 = np.array([x2,y2,z2])
        x3 = -18
        y3 = 14
        z3 = int(vals[2])
        p3 = np.array([x3,y3,z3])

        a1 = x2 - x1
        b1 = y2 - y1
        c1 = z2 - z1
        a2 = x3 - x1
        b2 = y3 - y1
        c2 = z3 - z1
        a = b1 * c2 - b2 * c1
        b = a2 * c1 - a1 * c2
        c = a1 * b2 - b1 * a2
        k = (- a * x1 - b * y1 - c * z1)

        self.calculate_normal(a, b, c, k, p1, p2, p3)

    def calculate_normal(self, a,b,c,d, p1, p2, p3):
        '''
        Find the normal vector from the central point on the Stewart platform (0,0)
        '''
        z  = (-a * 0 - b * 0 - d) / c
        # print('Z at center {}'.format(z))
        print('Center offset {} mm'.format(z))
        center = np.array((0,0,z))
        # We have three points, and the center coordinate
        # We need to calculate the vectors to this point
        P1_Center = p1 - center
        P2_Center = p2 - center
        # Calculate the normal vector
        normal_vector = np.cross(P1_Center, P2_Center)
        normal_vector = normal_vector / np.linalg.norm(normal_vector)
        self.measurements.append(normal_vector)
        if len(self.measurements) > 10:
            self.measurements.pop(0)
        
        # print('Normal vector: {}'.format(normal_vector))
        data = CoordinateData()
        data.coeff = [a, b, c, d]
        data.z = self.z_data
        data.normal = normal_vector
        self.pub.publish(data)

        self.calculate_angle_about_axis(np.mean(np.array(self.measurements),axis=0))

        self.adjust_height(z)

    def adjust_height(self, z):
        '''
        Given the current ToF distance, appropriately offset the distance of the platform
        '''
        
        # Assume optimal distance is 75 mm
        
        # Handle very bad cases
        if z < 30:
            print('Bad values detected!')
            prop_z = 80
        elif z > 110:
            # Use the maximum throw
            prop_z = 125
        else:
            # Value is within the range. Stewart platform is nominally 20 mm above the surface.
            adjust = 75 - z
            # If z = 60, we are too close and new to move away
            prop_z = self.stewart_height - adjust
        # If z = 85 we are too far and need to come closer
        print('Proposed Stewart height! :{}'.format(prop_z))
        clip_z = np.clip(prop_z,80,125)
        print('Clipped stewart height {}'.format(clip_z))
        # Update the values of stewart height
        self.stewart_height = clip_z
        # self.stewart_command(0, 0, self.stewart_height, 0, 0, 0)


    def calculate_angle_about_axis(self, vector):
        yaw = np.rad2deg(np.arccos((np.dot([1,0,0],vector)) / (np.linalg.norm([1,0,0]) * np.linalg.norm(vector))))
        pitch = np.rad2deg(np.arccos((np.dot([0,1,0],vector)) / (np.linalg.norm([0,1,0]) * np.linalg.norm(vector))))
        roll = np.rad2deg(np.arccos((np.dot([0,0,1],vector)) / (np.linalg.norm([0,0,1]) * np.linalg.norm(vector))))
        # print(f'Roll {roll}, Pitch: {pitch-90}, Yaw: {yaw}')

        
    def process_data(self, data):
        '''
        Take raw data from serial output and parse it into correct format
        '''

        vals = []

        # Remove ANSI escape charters \x1b[36m (cyan)
        ansi_escape =re.compile(r'(\x9B|\x1B\[)[0-?]*[ -\/]*[@-~]')

        # Remove ANSI escape characters
        ansi_removed = ansi_escape.sub('', data)
        # Process into parts
        record = ansi_removed.split(',')
        
        for value in record:
            try: 
                key, val = value.split(':')
                vals.append(int(val))
            except:
                continue

        if len(vals) == 3:
            self.z_data = vals
            self.plane(vals)

    def set_light_power(self, power: Light):
        '''
        Sets the light power to the selected value
        '''
        powerToSend = ''
        if power.data.lower() == 'on':
            powerToSend = 'light_power OFF' # THIS IS BACKWARDS, NO IDEA WHY # TODO!
        else:
            powerToSend = 'light_power ON' # THIS IS BACKWARDS, NO IDEA WHY # TODO!
        
        # Add command to the command buffer
        self.command_buffer.append(powerToSend)
        return True

    def write_commands(self):
        '''
        Write serial commands to the Arduino device
        '''
        if len(self.command_buffer) > 0:
            self.tof.write(self.command_buffer.pop(0).encode())

    def run(self):
        '''
        Main operating loop used to read and send data from the spectrometer
        '''
        while not rospy.is_shutdown():
            try:
                # Grab the raw data
                raw_data = self.tof.readline()
                # Decode the spectral data
                spectra_data = raw_data.decode('utf-8').strip()
                # Process and publish the data
                self.process_data(spectra_data)
           
                #write the latest command
                self.write_commands()
            except Exception as e:
                rospy.logerr('Error in main ToF loop: {}'.format(str(e)))
                rospy.logerr(traceback.print_exc())
            
            rospy.sleep(0.1)

    def shutdown(self):
        '''
        Custom shutdown behavior
        '''
        #turn off the light 
        self.tof.write('light_power OFF'.encode())
        # Close the serial transmission
        self.tof.close()

# Main functionality
if __name__ == '__main__':
    # Initialize the node and name it.
    rospy.init_node('tof_driver', anonymous=True)
    try:
        controller = ToFDriver()
        controller.run()
    except rospy.ROSInterruptException:
        controller.shutdown()