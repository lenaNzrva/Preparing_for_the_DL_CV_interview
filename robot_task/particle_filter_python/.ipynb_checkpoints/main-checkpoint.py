from robot import Robot
from particle_filter import ParticleFilter
import numpy as np

robot = Robot()
particle_filter = ParticleFilter()

points = [(10, 18),(-5, 21), (11, -14), (-5, 17), (-3, 11), (-8, 17), (-11, 9), (-8, 2), 
          (8, -24), (10, 5), (2, 20), (-23, -29),(-14, 34), (9, 20), (-2, -26), (1, -31),
          (7, -28), (7, 18), (0,0), (18, -28)]

for point in points:
    robot.move(point[0], point[1])
    particle_filter.calculate(point[0], point[1])
    result = particle_filter.calculate_estimation()
    print("Filter approximation: x:", np.round(result[0], 3), "y:", np.round(result[1], 3), 
          "theta:", np.round(np.degrees(result[2]),3))