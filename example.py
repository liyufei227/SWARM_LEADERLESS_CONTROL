'''
N - agents

Install numpy: `pip install numpy`
Install matplotlib: `pip install matplotlib`
'''

import matplotlib.pyplot as plt
import numpy as np


class Drone(object):
    '''Drone

    State of drones is `orinetation`
    '''

    def __init__(self, identifier, initial_state, neighbors=None, time_step=0.01):
        '''Constructor
        '''
        self.identifier = identifier
        self.initial_state = initial_state
        self.current_state = self.initial_state
        self.trajectory = [self.initial_state]
        self.neighbors = neighbors
        self.time_step = time_step
        self.formation = []

    def move(self):
        '''
        '''
        u = self.compute_controls()
        self.current_state = self.current_state + u
        self.trajectory.append(self.current_state)

    def get_neighbors(self):
        '''
        '''
        return self.neighbors

    def get_neighbor_relative_position(self, other_drone):
        '''
        '''
        return other_drone.current_state - self.current_state

    def get_formation_relative_position(self, other_drone):
        '''
        '''
        for k, neighbor in enumerate(self.neighbors):
            if other_drone.identifier == neighbor.identifier:
                return self.formation[k]

    def compute_controls(self):
        '''
        Agent i:
        u = \sum_{j in N_i} (x_{ij} - r_{ij})
        '''
        command = 0.0
        for j in self.get_neighbors():
            command += (self.get_neighbor_relative_position(j)
                        - self.get_formation_relative_position(j))
        return command * self.time_step

if __name__ == '__main__':

    np.random.seed(1) # set seed for the random number generator

    T = 10 # simulation time in seconds
    dt = 0.1 # time step
    number_steps = int(round(T / dt))

    N = 9 # number of drones
    # initial positions of the drones
    initial_states = np.random.uniform(0, 1, size=(N, 2))
    # drone instances
    drones = [Drone(identifier, init_state, time_step=dt)
              for identifier, init_state in enumerate(initial_states)]

    # set neighbors for all drones
    drones[0].neighbors = [drones[1], drones[3]] # drone 0
    drones[1].neighbors = [drones[0], drones[2], drones[3], drones[4]] # drone 1
    drones[2].neighbors = [drones[1]] # drone 2
    drones[3].neighbors = [drones[0], drones[1]] # drone 3
    drones[4].neighbors = [drones[1], drones[5], drones[6], drones[8]] # drone 4
    drones[5].neighbors = [drones[4]] # drone 5
    drones[6].neighbors = [drones[4], drones[7]] # drone 6
    drones[7].neighbors = [drones[6], drones[8]] # drone 7
    drones[8].neighbors = [drones[4], drones[7]] # drone 8

    drones[0].formation = [np.array([1,0]), np.array([0, 1])] # r_01, r_03
    drones[1].formation = [np.array([-1, 0]), np.array([1, 0]), np.array([-1, 1]), np.array([0, 1])]# r_10, r_12, r_13, r_14
    drones[2].formation = [np.array([-1, 0])]
    drones[3].formation = [np.array([0, -1]), np.array([1, -1])]
    drones[4].formation = [np.array([0, -1]), np.array([1, 0]), np.array([-1, 1]), np.array([1, 1])]
    drones[5].formation = [np.array([-1, 0])]
    drones[6].formation = [np.array([1, -1]), np.array([1, -0])]
    drones[7].formation = [np.array([-1, 0]), np.array([1, 0])]
    drones[8].formation = [np.array([-1, -1]), np.array([-1, 0])]
    
    # simulate moving in the environment
    for k in range(number_steps):
        for d in drones:
            d.move()

    # plot orinetation of each drone in time
    plt.figure(1)
    time  = np.arange(0, T + dt, dt)
    for d in drones:
        x, y = zip(*d.trajectory)
        plt.plot(x, y, '-o')
        plt.plot(x[-1], y[-1], 'D') # final position represented with diamond marker
    plt.show()

# for k in range(number_steps)
#   x1, y1 = zip(*drones[0].trajectory[k])
#   f1 = client.moveToPositionAsync(x1, y1, -10, 5, vehicle_name="Drone1")
