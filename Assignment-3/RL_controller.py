
import numpy as np
import seaborn as sns
import matplotlib.pylab as plt


class RL_controller:
    def __init__(self, args):
        self.gamma = args.gamma
        self.lr = args.lr
        self.Q_value = np.zeros((args.theta_discrete_steps, args.theta_dot_discrete_steps, 3)) # state-action values
        self.V_values = np.zeros((args.theta_discrete_steps, args.theta_dot_discrete_steps)) # state values
        self.prev_a = 0 # previous action
        # Use a previous_state = None to detect the beginning of the new round e.g. if not(self.prev_s is None): ...
        self.prev_s = None # Previous state

    def reset(self):
        #You need to reset sth
        print("TBD")

    def get_action(self, state, image_state, random_controller=False, episode=0):

        terminal, timestep, theta, theta_dot, reward = state

        if random_controller:
            action = np.random.randint(0, 3) # you have three possible actions (0,1,2)

        else:
            if np.random.rand() > 0.8:
                action = np.random.randint(0, 3)                
            else:
                action = np.argmax(self.Q_value[theta][theta_dot])



        if not(self.prev_s is None or self.prev_s == [theta, theta_dot]):
            # Calculate Q values here
            #alpha = 0
            #self.Q_value[theta]

            
            self.Q_value[self.prev_s[0]][self.prev_s[1]][self.prev_a] = self.Q_value[self.prev_s[0]][self.prev_s[1]][self.prev_a] \
                 + self.lr * (reward + self.gamma * np.max(self.Q_value[theta][theta_dot]))

            self.V_values[self.prev_s[0]][self.prev_s[1]] = self.Q_value[theta][theta_dot][action]
            print(reward)
                

            #new_theta = 
            #new_theta_dot = 

            #self.Q_value = self.Q_value + alpha*(reward + self.gamma*np.argmax((self.prev_a,self.prev_s)-(self.Q_value)))

        #############################################################
        #    
        self.prev_s = [theta, theta_dot]
        self.prev_a = action
        return action

    def save_state_matrix(self,round):
        data_set = self.V_values
        ax = sns.heatmap( data_set , cmap = 'coolwarm',robust=True, annot=True,fmt=".1f" )
        plt.xlabel('Theta')
        plt.ylabel('ThetaDot')
        plt.title( "2-D Heat Map" )
        plt.savefig("State_Values_" +str(round) +".png")
        plt.close()  


