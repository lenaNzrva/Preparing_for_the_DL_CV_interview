import numpy as np
import random

class Robot():
    def __init__(self, x=0, y=0, theta=0):
        # distance btw forward and backward wheels
        self.L = 2
        
        # init for the move
        self.x, self.y, self.theta = x, y, theta
        
        # noise
        self.motion_noise = (0.1, 1)  # Motion noise (std dev of steering angle and distance)
        self.measurement_noise = 1
        
        # init for the bearings
        self.landmarks = [(50, 50), (-50, 50), (-50, -50), (50, -50)]
        
    def move(self, alpha=0, dist=0):
        """
        alpha in degrees
        """
        alpha = (random.gauss(alpha, self.motion_noise[0]))%360
        dist = random.gauss(dist, self.motion_noise[1])
        
        # move
        x_next, y_next, theta_next = self.kinematics(np.radians(alpha), dist, self.x, self.y, self.theta)
        self.x, self.y, self.theta = x_next, y_next, theta_next
        self.current_pose = (self.x, self.y, self.theta)

        # print outputs
        print("Robot position: x:", np.round(self.x, 3), "y:", np.round(self.y, 3), "theta:", np.round(np.degrees(self.theta),3))
    
    def kinematics(self, alpha, dist, x, y, theta):
        
        if alpha:
        
            turn_radius = self.L/np.tan(alpha)
            turn_angle = dist/turn_radius

            # next x calculation
            x_c = x - turn_radius*np.sin(theta)
            x_next = x_c + turn_radius*np.sin(theta + turn_angle)

            # next y calculation
            y_c = y + turn_radius*np.cos(theta)
            y_next = y_c - turn_radius*np.cos(theta + turn_angle)

            # next theta calculation
            theta_next = theta + turn_angle

            return x_next, y_next, theta_next%(2*np.pi)
        
        x_next = x + dist*np.cos(theta)
        y_next = y + dist*np.sin(theta)
        
        return x_next, y_next, theta%(2*np.pi)
    
    
    def sense(self):
        bearings = []

        for landmark in self.landmarks:
            dx = landmark[0] - self.x
            dy = landmark[1] - self.y
            bearing = np.arctan2(dy, dx) - self.theta
            bearing = random.gauss(bearing, self.measurement_noise)
            bearing %= 2 * np.pi
            bearings.append(bearing)

        return bearings
    
    @staticmethod
    def _calculate_gaussian(x, mu, sigma):
        exp = -0.5 * ((x - mu) ** 2) / (sigma ** 2)
        return np.exp(exp) / np.sqrt(2 * np.pi * (sigma ** 2))