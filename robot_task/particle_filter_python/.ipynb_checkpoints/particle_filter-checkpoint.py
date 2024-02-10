import numpy as np
import random
from robot import Robot

class ParticleFilter(Robot):
    def __init__(self, N=1000, x=0, y=0, theta=0):
        super().__init__()
        self.N = N
        # generate_particles
        self.particles = self.generate_particles(N, x, y, theta)
    
    def generate_particles(self, N, x, y, theta):
        particles, self.weights = [], []
        
        for i in range(N):
            # Примерно знаем где находится робот
            X = random.gauss(x, 25)
            Y = random.gauss(y, 25)
            theta = 0
            # theta = random.gauss(theta, 0.01)%(2*np.pi)

            particle = (X, Y, theta)
            particles.append(particle)
            self.weights.append(1.0 / N)

        return particles
    
    def calculate(self, alpha, dist):
        #move
        self.move_particles(alpha, dist)
        
        #find sense
        robot_sense = self.sense()
        sense_particles = self.sense_particles()
        
        #weights
        self.find_weights(robot_sense,sense_particles)
        
        # filter particles
        self.filtration()
    
    
    def move_particles(self, alpha, dist):
        """
        alpha in degrees
        """
        alpha = (random.gauss(alpha, self.motion_noise[0]))%360
        alpha = np.radians(alpha)
        new_particles = [self.kinematics(alpha, 
                                         random.gauss(dist, self.motion_noise[1]), particle[0], particle[1], particle[2]) for particle in self.particles]
        
        self.particles = new_particles
    
    
    def sense_particles(self):
        
        return [self.__calculate_sense_particles(particle) for particle in self.particles]

    
    def __calculate_sense_particles(self, data):
        x, y, theta = data
        bearings = []

        for landmark in self.landmarks:
            dx = landmark[0] - x
            dy = landmark[1] - y
            bearing = np.arctan2(dy, dx) - theta
            bearing = random.gauss(bearing, self.measurement_noise)
            bearing %= 2 * np.pi
            bearings.append(bearing)

        return bearings
    
    def find_weights(self, robot_sense, sense_particles):
        # находим веса
        self.weights = self._calculate_gaussian(np.array(sense_particles), np.array(robot_sense), self.measurement_noise)
        self.weights = np.prod(self.weights, axis=1)
        # нормализируем
        S = np.sum(self.weights)
        self.weights /= S
        
    def filtration(self):
        index = random.randint(0, self.N-1)
        betta = 0
        newParticleList, new_weights = [], []
        for i in range(self.N):
            betta = betta + random.uniform(0, 2*np.max(self.weights))
            while betta > self.weights[index]:
                betta = betta - self.weights[index]
                index = (index + 1)%self.N # индекс изменяется в цикле от 0 до N
                
            newParticleList.append(self.particles[index])
            new_weights.append(self.weights[index])

        self.particles = newParticleList

        S = np.sum(new_weights)
        self.weights = new_weights/S
        
        
    def calculate_estimation(self):
        estimateX = 0
        estimateY = 0
        estimateT = 0
        for i in range(self.N):
            estimateX = estimateX + self.particles[i][0]*self.weights[i]
            estimateY = estimateY + self.particles[i][1]*self.weights[i]
            estimateT = estimateT + self.particles[i][2]*self.weights[i]
            
            
        return estimateX, estimateY, estimateT%(2*np.pi)