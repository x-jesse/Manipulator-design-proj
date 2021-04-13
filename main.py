import math

######################################
# MANIPULATOR DESIGN PROJECT MTE 119 #
######################################


def distance(x, y):
    d = math.sqrt(0.75**2 + 0.1**2)
    return d

def torque(length, force, angle):
    t = 0.5*length*force*math.sin(angle)
    return t


# TORQUE CALCULATIONS

# CASE X = 0.75M, Y = 0.1M, THETA = -60 WRT X

# CASE X = 0.5M, Y = 0.5M, THETA = 0 WRT X

# CASE X = 0.2M, Y = 0.6M, THETA = 45 WRT X




