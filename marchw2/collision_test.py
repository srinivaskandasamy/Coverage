# Collision detection and data streaming
# Recalibration of Sphero after collision
# Recording collision data and abstracting correlations

import sphero_driver  # Main class file
import math
import numpy as np
import matplotlib.pyplot as plt

REQ=dict(CMD_ROLL = [0x02, 0x30])

class SpheroNode(object):
    
    def __init__(self, default_update_rate=50.0):

        self.update_rate = default_update_rate
        self.sampling_divisor = int(400 / self.update_rate)

        self.is_connected = False
        self.robot = sphero_driver.Sphero()
        self.cmd_heading = 0
        self.cmd_speed = 0
        self.power_state_msg = "No Battery Info"
        self.power_state = 0
        self.collipy = np.matrix([0, 0, 0, 0, 0, 0])
        self.t = np.matrix([0])
        
    def start(self):
        try:
            self.is_connected = self.robot.connect()
        except:
            print "Failed to connect Sphero"
        # setup streaming
#        self.robot.set_filtered_data_strm(self.sampling_divisor, 1 , 0, True)
#        self.robot.add_async_callback(sphero_driver.IDCODE['DATA_STRM'], self.parse_data_strm)
        # setup power notification
#       self.robot.set_power_notify(True, False)
#       self.robot.add_async_callback(sphero_driver.IDCODE['PWR_NOTIFY'], self.parse_power_notify)
#       # setup collision detection
        self.robot.config_collision_detect(1, 45, 110, 45, 110, 100, False)
        self.robot.add_async_callback(sphero_driver.IDCODE['COLLISION'], self.parse_collision)
#       # set the ball to connection color
#       self.robot.set_rgb_led(self.connect_color_red, self.connect_color_green, self.connect_color_blue, 0, False)
#       # now start receiving packets
        self.robot.start()
        
    def parse_data_strm(self, data):
        if self.is_connected:
            self.ox = data["QUATERNION_Q0"]
            self.oy = data["QUATERNION_Q1"]
            self.oz = data["QUATERNION_Q2"]
            self.ow = data["QUATERNION_Q3"]
            self.lx = data["ACCEL_X_FILTERED"]/4096.0*9.8
            self.ly = data["ACCEL_Y_FILTERED"]/4096.0*9.8
            self.lz = data["ACCEL_Z_FILTERED"]/4096.0*9.8
            self.ax = data["GYRO_X_FILTERED"]*10*math.pi/180
            self.ay = data["GYRO_Y_FILTERED"]*10*math.pi/180
            self.az = data["GYRO_Z_FILTERED"]*10*math.pi/180
            
            print '[x:{0}],[y:{1}]'.format(self.ox, self.oy)
           
    def stop(self):    
        # tell the ball to stop moving before quiting
        self.robot.roll(int(0), int(0), 1, False)
        self.robot.shutdown = True
        self.is_connected = self.robot.disconnect()
        self.robot.join()
        
    def parse_collision(self, data):
        if self.is_connected:
            self.cx = data["X"]
            self.cy = data["Y"]
            self.cz = data["Z"]
            self.caxis = int(data["Axis"])
            self.cx_magnitude = data["xMagnitude"]
            self.cy_magnitude = data["yMagnitude"]
            self.cspeed = data["Speed"]
            self.ctimestamp = data["Timestamp"]
            
            
            
            c = math.sqrt(self.cx**2+self.cy**2)
            self.cx = self.cx / c
            self.cy = self.cy / c
             
            self.angle = math.atan2(self.cy, self.cx)*180/math.pi
#             self.angle = self.angle + 90
            
            print '[cx:{0}],[cy:{1}],[ax:{2}]'.format(self.cx, self.cy,self.angle, self)
            
            self.collipy = np.vstack([self.collipy, np.matrix([self.cx, self.cy, self.cx_magnitude, self.cy_magnitude, self.angle, self.cspeed])])
            
            self.t = np.vstack([self.t,self.t[-1]+1])
            
    def cmd_vel(self, msg):
        if self.is_connected:
            self.cmd_heading = self.normalize_angle_positive(math.atan2(msg[0], msg[1])) * 180 / math.pi
            self.cmd_speed = math.sqrt(math.pow(msg[0], 2) + math.pow(msg[1], 2))
            self.robot.roll(int(self.cmd_speed), int(self.cmd_heading), 1, False)
    
    def set_color(self, msg):
        if self.is_connected:
            self.robot.set_rgb_led(int(msg[0]), int(msg[1]), int(msg[2]), 0, False)
    
    def roll(self, speed, heading, state, response):
            self.send(self.pack_cmd(REQ['CMD_ROLL'], [self.clamp(speed, 0, 255), (heading >> 8), (heading & 0xff), state]), response)
            
    def set_stabilization(self, msg):
        if self.is_connected:
            if not msg.data:
                self.robot.set_stablization(1, False)
            else:
                self.robot.set_stablization(0, False)
                
    def set_angular_velocity(self, msg):
        if self.is_connected:
            rate = int((msg.data * 180 / math.pi) / 0.784)
            self.robot.set_rotation_rate(rate, False)

    def configure_collision_detect(self, msg):
        pass

    def reconfigure(self, config, level):
        if self.is_connected:
            self.robot.set_rgb_led(int(config['red'] * 255), int(config['green'] * 255), int(config['blue'] * 255), 0, False)
        return config
    
# Main function
    
if __name__ == '__main__':
    plt.ion()
    fig = plt.figure()
    ax = fig.add_subplot(111)
    print "Starting..."
    s = SpheroNode()
    s.start()
    s.set_color([0, 255, 0])
    print "Junga"
    s.robot.roll(80, 0, 1, False)
    print "Ready"    
    a=1
    line, = ax.plot([],[],'r-')
    ax.set_xlim([0,50])
    ax.set_ylim([-180,180])
    
    while a==1:
        # Plotting collision data
        line.set_xdata(s.t)
        line.set_ydata(s.collipy[:,4])
        fig.canvas.draw()
    print "Over"