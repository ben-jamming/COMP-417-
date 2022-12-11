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
        self.A = np.matrix([[1,self.dt],[(9.81*self.dt)/0.5,1]])
        self.B = np.matrix([[0],[3*self.dt/0.5]])
        #self.Q = np.matrix([[(0.25*self.dt**4),0.5*(self.dt**3)],[0.5*(self.dt**3), self.dt**2]])
        self.Q = np.eye(2)*0.3
        self.G = np.eye(2)
        self.alpha = 0.00000025 # adjusting scalar for P
        self.beta = 0.0000000000000003 #adjustment scalar for R
        self.P_pred = np.eye(2)*self.alpha
        self.R = np.eye(2)*self.beta
        self.H = np.eye(2)*0.3

    def set_values(self,theta,theta_dot):
        self.X_curr = np.matrix([[theta],[theta_dot]])
        
    def update(self,action):
        #state = [theta, theta_dot], which were just observed
        self.X_pred = np.add(np.matmul(self.A,self.X_curr), self.B*action)
        self.P_pred = np.matmul(np.matmul(self.A,self.P_pred),np.transpose(self.A)) + np.matmul(np.matmul(self.G,self.Q),np.transpose(self.G))
        
        #now update the filter
        self.G *= self.X_pred.item(0)
        r = np.subtract(self.X_curr,self.X_pred)
        S = np.add(np.matmul(np.matmul(self.H,self.P_pred),np.transpose(self.H)),self.R)
        K = np.matmul(np.dot(self.P_pred,np.transpose(self.H)),np.linalg.inv(S))
        self.X_pred = np.add(self.X_pred,(K*r))
        self.P_pred = np.subtract(self.P_pred,np.matmul(np.matmul(K,self.H),self.P_pred))
        
    
class PID_controller:
    def __init__(self):
        self.prev_error = 0 #action is in torq
        self.integral_term = 0
        self.dt = 0.005
        self.prev_theta = 0


    def get_angle(self, timestep, image_state, random_controller=False):
        #get the angle from the image:
        #Left Quadrant is -(theta_radians)
        #Right Quadrant is (theta_radians)
        #set point is 0 radians (upright)
        theta = ad.find_pole(image_state)
        theta_dot = (theta - self.prev_theta)/self.dt
        self.prev_theta = theta
        #print("Current angle is: {a} \n Change in angle is {b}".format(a=theta,b=theta_dot))
        return theta,theta_dot


    def get_action(self, theta, theta_dot):
        #alpha = 0.9
        #new_error = (self.prev_error * alpha) + (theta*(1-alpha))
        
        #the different controllers
        freq = 1/520
        P_cntrl = 1.2*theta
        PID_cntrl = P_cntrl + ((self.integral_term*self.dt)*0.001) + ((theta_dot/self.dt))*0.0005

        """print("theta: {theta}".format(theta=theta)) 
        print("theta dot {thetad}".format(thetad=theta_dot))
        print("Error is: {e}".format(e=theta))
        print("previous action: {a}".format(a=self.prev_error))
        print("Integral is: {i}".format(i=self.integral_term))"""
        print(PID_cntrl)
        #self.prev_error = new_error
        self.integral_term += theta
        #replace return value with whatever controller is to be tested.
        return PID_cntrl

