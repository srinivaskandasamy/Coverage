from Tkinter import *
from math import sin, cos, pi, fabs
from numpy import sign
import sys

class walls(object):
    def __init__(self,wall_width,location):
        self.warray=[]
        self.wall_width = wall_width # necessary to create variable width blocks
        
    def draw(self,ww,loc,w):
        
        for i in range(len(loc)):
            m = self.slope(loc[i])
            w.create_line(loc[i][0]-m*ww/2,loc[i][1]-(1-m)*ww/2,loc[i][0]+m*ww/2,loc[i][1]+(1-m)*ww/2)
            w.create_line(loc[i][2]-m*ww/2,loc[i][3]-(1-m)*ww/2,loc[i][2]+m*ww/2,loc[i][3]+(1-m)*ww/2)
            w.create_line(loc[i][0]-m*ww/2,loc[i][1]-(1-m)*ww/2,loc[i][2]-m*ww/2,loc[i][3]-(1-m)*ww/2)
            w.create_line(loc[i][0]+m*ww/2,loc[i][1]+(1-m)*ww/2,loc[i][2]+m*ww/2,loc[i][3]+(1-m)*ww/2)
            
    def slope(self,wall):
        ws = wall[:2]
        we = wall[2:]
        if fabs(ws[1]-we[1]) < 1e-3:
            m = 0
        elif fabs(ws[0]-we[0]) < 1e-3:
            m = 1
        else:
            m = -1
            print 'Error: Rectilinear walls are only allowed, check this wall {0}'.format(wall)
            sys.exit()
        return m
    
    def wall_grid(self,ww,loc,w):
        
        d = 100  # Discretization of the grid
        
        for i in range(len(loc)):
            m = self.slope(loc[i])
            ws = loc[i][:2]
            we = loc[i][2:]
            for i in range(d):
                self.warray.append([ws[0]+sign(we[0]-ws[0])*(we[0]-ws[0])*i/d,
                                   ws[1]+sign(we[1]-ws[1])*(we[1]-ws[1])*i/d])
                w.create_line(self.warray[-1][0]+sign(we[1]-ws[1])*ww/2,self.warray[-1][1]+sign(we[0]-ws[0])*ww/2,
                              self.warray[-1][0]-sign(we[1]-ws[1])*ww/2,self.warray[-1][1]-sign(we[0]-ws[0])*ww/2)
                
        return self.warray

if __name__=="__main__":
    mw = walls(wall_width,wall_location)