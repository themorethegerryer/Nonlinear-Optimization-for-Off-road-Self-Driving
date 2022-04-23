import numpy as np

class DoubleTrackModel():
    #All vectors are in order of 0:FL, 1:FR, 2:RL, 3:RR
    def __init__(self, U0=[0.0,0.0], ax=0.0, ay=0.0, r=0.0, m=2000.0, w=[0.0,0.0,0.0,0.0], 
                 lw1=1.8,
                 lw2=1.8, lr=2.25, lf=2.25, Rw=0.23, c=0.04, 
                 Vxwheel=[0.0,0.0,0.0,0.0], Vw=[0.0,0.0,0.0,0.0], 
                 beta=0.0, Vx=0.0, Vy=0.0):
        
        self.parameters = {
            "ax"     : ax,
            "ay"     : ay,
            "r"      : r,
            "m"      : m,
            "Iw"     : 0.0,
            "Iz"     : 0.0,
            "w"      : w, 
            # "Td"     : Td, # embedded in torque input
            # "Tb"     : Tb, # 4x1 embedded in torque input (uniform braking as negative torque)
            "Fx"     : [0.0,0.0,0.0,0.0],
            "lw1"    : lw1,
            "lw2"    : lw2,
            "lr"     : lr,
            "lf"     : lf,
            "Rw"     : Rw, 
            "c"      : c, #Rolling Friction Coefficient - car tire on solid sand, gravel loose worn, soil medium hard
            "Fr"     : [0.0,0.0,0.0,0.0],
            "Vxwheel": Vxwheel,
            "Vw"     : Vw,
            "alpha"  : [0.0,0.0,0.0,0.0],
            "beta"   : beta,
            "Vx"     : Vx,
            "Vy"     : Vy,
            "lambda" : [0.0,0.0,0.0,0.0]}
        
        self.state = np.zeros(13)
        self.state[3] = 1 #Initialize unit quaternion
        self.state[7] = self.parameters["Vx"]
        self.state[8] = self.parameters["Vy"]
        
        self.dynamics(self.state, U0)
        
    def dynamics(self, X, U):
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
                
        #Iw
        #Iz
        #Fx
        #Fr
        
    def cost(x,u,dt):
        pass
        
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
    