from math import cos, sin, pi
from numpy import arange
from Tkinter import *

WB = 2                            # RHP zero, wheel base

class Sphero(object):
    """ Creating a Sphero object for the sample maze construction. The steering angle is assumed to be small to make the 
assumption in calculations. The numerical calculations, as a result, are stable. In future, two other models like point mass 
and SO(3) will also be added if needed.
1. In the first model, we assumed a segway model which has the same inputs as the actual robot but the measurements are not 
available as position or orientation
2. In the second model, we assume a point mass model where the acceleration is the input to the robot as well as the measurements
in the form of change in acceleration.
3. The third model is a mix of both the above models, that represents an input-output model of the robot. The input to the 
system is the heading and velocity (segway) and the output is the collision in the form of change in acceleration. The segway 
model is simulated and the states are differentiatied to obtain V and A. With added noise in A, we integrate back to get the 
position of the robot. If collision is detected, we add this to the integrated data.

Created by Srinivas K."""
    def __init__(self):
        #self.pose = [0, 0, 0]         # Pose of the robot 
        #self.v = 0                    # Velocity input to the robot
        #self.g =0                     # Steering input to the robot
        pass
    
    def motion(self, pose, control, dt):  # Unicycle mkotion model
        return ([x+y for x,y in zip(pose,[control[0]*dt*cos(control[1]+pose[2]), control[0]*dt*sin(control[1]+pose[2]), 
                                               control[0]*dt*sin(control[1])/WB])] )
    
    def draw(self, pose):     # Draw the Sphero robot in the simulated scene
        r = 10     # Radius of the sphero robot
        xsp, ysp =[], []
        xsp.append(pose[0])
        ysp.append(pose[1])
        for i in arange(0,360,1):      # List of spherical points
            xsp.append(pose[0] + r * cos(i*pi/180))
            ysp.append(pose[1] + r * sin(i*pi/180))
            
        return (xsp,ysp)
    
    def callme(self, pose, control, w):
        
        pose = self.motion(pose,control,0.1)
        print pose
        f = self.draw(pose)
    
        for i in range(len(f[0])):
            w.create_line(f[0][(i)%len(f[0])],f[1][(i)%len(f[0])],f[0][(i+1)%len(f[0])],f[1][(i+1)%len(f[0])])
            
        self.after(1000,self.callme(pose,control,w))
        
if __name__=="__main__":
    s = Sphero()
# pose = [0,0,0]
# control = [1,1]
# pose = s.motion(pose,control,0.1)
# print pose

