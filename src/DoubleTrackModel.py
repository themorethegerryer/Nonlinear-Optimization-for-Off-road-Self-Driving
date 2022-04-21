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
        
        self.update_parameters([0.0,0.0,0.0])
        
    def update_parameters(self, U):
        theta_left, theta_right = self.ackermann_turn(U[0])
        self.parameters["alpha"][0] = theta_left - np.arctan2(self.parameters["Vy"]+self.parameters["lf"]*self.parameters["r"], self.parameters["Vx"]-(self.parameters["lw1"]/2)*self.parameters["r"])
        self.parameters["alpha"][1] = theta_right - np.arctan2(self.parameters["Vy"]+self.parameters["lf"]*self.parameters["r"], self.parameters["Vx"]+(self.parameters["lw1"]/2)*self.parameters["r"])
        self.parameters["alpha"][2] = -np.arctan2(self.parameters["Vy"]-self.parameters["lr"]*self.parameters["r"], self.parameters["Vx"]-(self.parameters["lw2"]/2)*self.parameters["r"])
        self.parameters["alpha"][3] = -np.arctan2(self.parameters["Vy"]-self.parameters["lr"]*self.parameters["r"], self.parameters["Vx"]+(self.parameters["lw2"]/2)*self.parameters["r"])
        
        for i in range(len(self.parameters["lambda"])):
            lambda_check = self.parameters["w"][i]*self.parameters["Rw"]-self.parameters["Vxwheel"][i]
            if lambda_check < 0:
                self.parameters["lambda"][i] = lambda_check/self.parameters["Vxwheel"][i]
            else:
                self.parameters["lambda"][i] = lambda_check/(self.parameters["w"][i]*self.parameters["Rw"])
        
        
        
    #Idealized turn that ackermann steering is based on
    def ackermann_turn(self,steering_angle):
        length = self.parameters["lr"]+self.parameters["lf"]
        radius = length*np.tan(steering_angle)
        #Check which turn direction
        if steering_angle < 0: #Right turn
            theta_right = np.arctan2(length,radius-(self.parameters["lw1"]/2))
            theta_left = np.arctan2(length,radius+(self.parameters["lw1"]/2))
        else: #Left turn
            theta_left = np.arctan2(length,radius-(self.parameters["lw1"]/2))
            theta_right = np.arctan2(length,radius+(self.parameters["lw1"]/2))
        return theta_left, theta_right