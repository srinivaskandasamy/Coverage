from Tkinter import *
from math import sin, cos, pi, fabs
from numpy import sign
import sys

class boundaries(object):
    
    def __init__(self,world_size,wall_width):
        self.world_size = world_size
        self.wall_width = wall_width
        self.parray = []   # Consider preallocation for faster memory access
        
    def draw(self,origin,w):
        a = origin[0] 
        b = origin[1]    # to be added in the main loop
        # Outer boundary
        w.create_line(a,b,a+self.world_size[0],b)
        w.create_line(a+self.world_size[0],b, a+self.world_size[0],b+self.world_size[1])
        w.create_line(a,b,a,b+self.world_size[1])
        w.create_line(a,b+self.world_size[1],a+self.world_size[0],b+self.world_size[1])
        
        # Inner boundary
        w.create_line(a+self.wall_width,b+self.wall_width,
                      a-self.wall_width+self.world_size[0],b+self.wall_width)
        w.create_line(a-self.wall_width+self.world_size[0],b+self.wall_width, 
                      a-self.wall_width+self.world_size[0],b-self.wall_width+self.world_size[1])
        w.create_line(a+self.wall_width,b+self.wall_width,
                      a+self.wall_width,b-self.wall_width+self.world_size[1])
        w.create_line(a+self.wall_width,b-self.wall_width+self.world_size[1],
                      a-self.wall_width+self.world_size[0],b-self.wall_width+self.world_size[1])
        
    def boundary_grid(self,origin,w): # Defining the probability grid for the boundary by taking input as the corners
        # Assuming corners are available in the form of a list (given in an sequential fashion)
        a = origin[0]
        b = origin[1]
        d = 100     # Extent or level of discretization 
        
        # Addition of corners in a clockwise fashion
        corner = []
        corner.append([a + self.wall_width, b + self.wall_width])
        corner.append([a-self.wall_width+self.world_size[0],b+self.wall_width])
        corner.append([a-self.wall_width+self.world_size[0],b-self.wall_width+self.world_size[1]])
        corner.append([a+self.wall_width,b-self.wall_width+self.world_size[1]])

        for j in range(len(corner)):
            length = self.length_boundary(corner[j%len(corner)],corner[(j+1)%len(corner)])
            if length[1] == -1:
                print "Error in computing slope - Not a rectilinear world"
                sys.exit()

            for i in range(d):
                self.parray.append([corner[j][0]+sign(corner[(j+1)%len(corner)][0]-corner[j%len(corner)][0])*sin(length[1])*length[0]*i/d,
                                    corner[j][1]+sign(corner[(j+1)%len(corner)][1]-corner[j%len(corner)][1])*cos(length[1])*length[0]*i/d,0])
                w.create_line(self.parray[-1][0],self.parray[-1][1],
                              self.parray[-1][0]+sign(corner[(j+1)%len(corner)][1]-corner[j%len(corner)][1])*self.wall_width,
                              self.parray[-1][1]-sign(corner[(j+1)%len(corner)][0]-corner[j%len(corner)][0])*self.wall_width)
#                 w.create_rectangle(self.parray[-1][0],self.parray[-1][1],
#                                    self.parray[-2][0]+sign(corner[(j+1)%len(corner)][1]-corner[j%len(corner)][1])*self.wall_width,
#                                    self.parray[-2][1]-sign(corner[(j+1)%len(corner)][0]-corner[j%len(corner)][0])*self.wall_width)
            w.create_line(corner[j%len(corner)][0],corner[j%len(corner)][1],
                          corner[j%len(corner)][0]+sign(corner[j%len(corner)][1]-corner[(j-1)%len(corner)][1])*self.wall_width,
                          corner[j%len(corner)][1]-sign(corner[j%len(corner)][0]-corner[(j-1)%len(corner)][0])*self.wall_width)
        return self.parray
    
#     def histogram_distribution(self,collision,parray):
        
        
    def length_boundary(self,cornerA,cornerB):
        
        a = max(fabs(cornerA[0]-cornerB[0]),fabs(cornerA[1]-cornerB[1])) # length of the line
        # To compute the slope of the line
        if cornerA[0]-cornerB[0]==0:
            b = 0
        elif cornerA[1]-cornerB[1]==0:
            b = pi/2
        else: # It is already in clockwise fashion; to report if something goes false
            b = -1
            
        return (a,b)
    
if __name__=="__main__":
    sri = boundaries(world_size,wall_width)