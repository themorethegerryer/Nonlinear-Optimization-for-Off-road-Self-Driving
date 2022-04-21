import numpy as np

class DoubleTrackModel():
    #All vectors are in order of 0:FL, 1:FR, 2:RL, 3:RR
    def __init__(self, ax=0.0, ay=0.0, r=0.0, Iw=0.0, w=[0.0,0.0,0.0,0.0], 
                 Td=0.0, Tb=[0.0,0.0,0.0,0.0], Fx=[0.0,0.0,0.0,0.0], lw1=0.0,
                 lw2=0.0, lr=0.0, lf=0.0, Rw=0.0, Fr=[0.0,0.0,0.0,0.0], 
                 Vxwheel=[0.0,0.0,0.0,0.0], Vw=[0.0,0.0,0.0,0.0], 
                 alpha=[0.0,0.0,0.0,0.0], beta=0.0, Vx=0.0, Vy=0.0,
                 lamb=[0.0,0.0,0.0,0.0]):
        
        self.parameters = {
            "ax"     : ax,
            "ay"     : ay,
            "r"      : r,
            "Iw"     : Iw,
            "w"      : w, 
            "Td"     : Td,
            "Tb"     : Tb,
            "Fx"     : Fx,
            "lw1"    : lw1,
            "lw2"    : lw2,
            "lr"     : lr,
            "lf"     : lf,
            "Rw"     : Rw, 
            "Fr"     : Fr,
            "Vxwheel": Vxwheel,
            "Vw"     : Vw,
            "alpha"  : alpha,
            "beta"   : beta,
            "Vx"     : Vx,
            "Vy"     : Vy,
            "lambda" : lamb}
        
    #Idealized turn that ackermann steering is based on
    def ackermann_turn(self,steering_angle):
        length = self.parameters["lr"]+self.parameters["lf"]
        radius = length*np.tan(steering_angle)
        #Check which turn direction
        if steering_angle < 0: #Right turn
            delta_right = np.arctan2(length,radius-(self.parameters["lw1"]/2))
            delta_left = np.arctan2(length,radius+(self.parameters["lw1"]/2))
        else: #Left turn
            delta_left = np.arctan2(length,radius-(self.parameters["lw1"]/2))
            delta_right = np.arctan2(length,radius+(self.parameters["lw1"]/2))
        return delta_left, delta_right