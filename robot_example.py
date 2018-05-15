
from math import *
import random


# noise parameters

steering_noise = 0.1
distance_noise = 0.03
measurement_noise = 0.3


class robot:

    def __init__(self, length=0.5):
        self.x = 0.0
        self.y = 0.0
        self.orientation = 0.0
        self.length = length
        self.steering_noise = 0.0
        self.distance_noise = 0.0
        self.measurement_noise = 0.0
        self.num_collisions = 0
        self.num_steps = 0

    # set a robot coordinate
    def set(self, new_x, new_y, new_orientation):

        self.x = float(new_x)
        self.y = float(new_y)
        self.orientation = float(new_orientation) % (2.0*pi)

    # set the noise parameters

    def set_noise(self, new_s_noise, new_d_nois, new_m_noise):
        self.steering_noise = float(new_s_noise)
        self.distance_noise = float(new_d_nois)
        self.measurement_noise = float(new_m_noise)

    # checks of the robot pose collides with an obstacle, or is
    # too far outside the plane

    def check_collision(self, grid):

        for i in range(len(grid)):
            for j in range(len(grid[0])):
                if grid[i][j] == 1:
                    dist = sqrt((self.x - float(i))**2 +
                                (self.y - float(j))**2)
                    if dist < 0.5:
                        self.num_collisions += 1
                        return False
        return True

    def check_goal(self, goal, threshold=1.0):
        dist = sqrt((float(goal[0]) - self.x)**2 +
                    (float(goal[1]) - self.y)**2)

    # move
    # steering = front wheel steering angle, limited by max_steering_angle
    # distance = total distance driven, must be non-negative

    def move(self, grid, steering, distance, tolerance=0.001, max_steering_angle=pi/4.0):

        if steering > max_steering_angle:
            steering = max_steering_angle
        if steering < -max_steering_angle:
            steering = -max_steering_angle
        if distance < 0.0:
            distance = 0.0

        # make a new copy
        res = robot()
        res.length = self.length
        res.steering_noise = self.steering_noise
        res.distance_noise = self.distance_noise
        res.measurement_noise = self.measurement_noise
        res.num_collisions = self.num_collisions
        res.num_steps = self.num_steps + 1

        # apply noise
        steering2 = random.gauss(steering, self.steering_noise)
        distance2 = random.gauss(distance, self.distance_noise)

        # Execute motion
        turn = tan(steering2) * distance2 / res.length

        if abs(turn) < tolerance:

            # approximate by straight line motion
            res.x = self.x + (distance2*cos(self.orientation))
            res.y = self.y + (distance2*sin(self.orientation))
            res.orientation = (self.orientation +turn) % (2.0*pi)

        else:

            # approximate bicycle model for motion
            radius = distance2/turn
            cx = self.x - (sin(self.orientation)*radius)
            cy = self.y + (cos(self.orientation)*radius)
            res.orientation = (self.orientation + turn) % (2.0*pi)
            res.x = cx + (sin(res.orientation) * radius)
            res.y = cy - (cos(res.orientation) * radius)

        # check for collision
        # res.check_collision(grid)
        return res

    # sense
    def sense(self):
        return [random.gauss(self.x, self.measurement_noise),
                random.gauss(self.y, self.measurement_noise)]

    # computes the probability of a measurement
    def measurement_prob(self, measurement):

        # compute errors
        error_x = measurement[0] - self.x
        error_y = measurement[1] - self.y

        # calculate Gaussian
        error = exp(-(error_x**2)/(self.measurement_noise**2)/2.0)\
            / sqrt(2.0 * pi *(self.measurement_noise ** 2))

        error *= exp(-(error_y**2)/(self.measurement_noise**2)/2.0)\
            / sqrt(2.0 * pi * (self.measurement_noise ** 2))

        return error

    def __repr__(self):
        return '{}, {}'.format(self.x, self.y)
        # return '[%.5f, %.5f]' % (self.x, self.y)


grid = [[0, 1, 0, 0, 0, 0],
        [0, 1, 0, 1, 1, 0],
        [0, 1, 0, 1, 0, 0],
        [0, 0, 0, 1, 0, 1],
        [0, 1, 0, 1, 0, 0]]

init = [0, 0]
goal = [len(grid)-1, len(grid[0])-1]


myrobot = robot()
myrobot.set_noise(steering_noise, distance_noise, measurement_noise)

while not myrobot.check_collision(grid):
    theta = atan2(goal[1] - myrobot.y, goal[0]-myrobot.x) - myrobot.orientation
    myrobot = myrobot.move(grid, theta, 0.1)
    if not myrobot.check_collison(grid):
        print("!!!!!Collision!!!!!!!!!")
    print(myrobot)
