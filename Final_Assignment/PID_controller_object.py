import numpy as np
import cart_pole_angle_detector as ad
import pandas as pd

class KalmanFilter:

    def __init__(self) -> None:
        #time of init is t
        self.dt = 0.005
        self.dx_curr = 0    #theta_dot @t
        self.x_curr = 0  #theta @t
        self.X_pred = np.matrix([[self.x_curr],[self.dx_curr]])
        self.X_curr = np.matrix([[self.x_curr],[self.dx_curr]])
        self.A = np.matrix([[1,self.dx_curr],[(9.81*self.dt)*0.5,1]])
        self.B = np.matrix([[0],[(self.dt)*2]])
        #self.Q = np.matrix([[(0.25*self.dt**4),0.5*(self.dt**3)],[0.5*(self.dt**3), self.dt**2]])
        self.Q = np.matrix([[100000,100000],[100000, 100000]])
        self.alpha = 1 # adjusting scalar for P
        self.beta = 1 #adjustment scalar for R
        self.P = np.eye(2)*self.alpha
        self.P_pred = np.matrix([[0,0],[0,0]])
        self.R = np.eye(2)*self.beta
        self.H = np.eye(2)

    def set_values(self,theta,theta_dot):
        self.X_curr = np.matrix([[theta],[theta_dot]])
        
    def update(self,action):
        #state = [theta, theta_dot], which were just observed
        self.X_pred = np.add(np.matmul(self.A,self.X_curr), self.B*action)
        self.P_pred = np.matmul(np.matmul(self.A,self.P),np.transpose(self.A)) + self.Q

        #now update the filter

        r = np.subtract(self.X_curr,self.X_pred)
        S = np.add(np.matmul(np.matmul(self.H,self.P_pred),np.transpose(self.H)),self.R)
        K = np.matmul(np.dot(self.P_pred,np.transpose(self.H)),np.linalg.inv(S))
        self.X_pred = np.add(self.X_pred,(K*r))
        self.P_pred = np.subtract(self.P_pred,np.matmul(np.matmul(K,self.H),self.P_pred))
        
    
class PID_controller:
    def __init__(self):
        self.prev_error = 0 #action is in torq
        self.integral_term = 0
        self.prev_theta = 0
        self.prev_theta_dot = 0


    def get_angle(self, timestep, image_state, random_controller=False):
        #get the angle from the image:
        #Left Quadrant is -(theta_radians)
        #Right Quadrant is (theta_radians)
        #set point is 0 radians (upright)
        theta = ad.find_pole(image_state)
        theta_dot = (theta+self.prev_theta)*0.005
        #print("Current angle is: {a} \n Change in angle is {b}".format(a=theta,b=theta_dot))
        self.prev_theta = theta
        return theta,theta_dot


    def get_action(self, theta, theta_dot):
        alpha = 1
        new_error = (self.prev_error * alpha) + (theta*(1-alpha))
        print("Theta is: ", theta)

        #the different controllers
        
        P_cntrl = 4*new_error
        PI_cntrl = P_cntrl + ((self.integral_term/0.005)*0.001)
        DI_cntrl = (theta_dot*200) + (self.integral_term*0.005*0.001)
        PD_cntrl = P_cntrl + 50*theta_dot
        PID_cntrl = P_cntrl + ((self.integral_term*0.005)) + ((theta_dot/0.005))

        """print("theta: {theta}".format(theta=theta)) 
        print("theta dot {thetad}".format(thetad=theta_dot))
        print("Error is: {e}".format(e=theta))
        print("previous action: {a}".format(a=self.prev_error))
        print("Integral is: {i}".format(i=self.integral_term))"""

        self.prev_error = new_error
        self.integral_term += new_error
        #replace return value with whatever controller is to be tested.
        return PID_cntrl

