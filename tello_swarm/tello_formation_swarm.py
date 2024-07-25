from djitellopy import TelloSwarm
from time import sleep

# List of drone IP addresses
tellos = [
    "192.168.66.228",  # Drone 1
    "192.168.66.40",   # Drone 2
    "192.168.66.187"   # Drone 3
]

# Create a TelloSwarm object
swarm = TelloSwarm.fromIps(tellos)

# Define custom commands for each drone
def initial_position(i, tello):
    tello.move_up(50)
    if i == 0:
        tello.move_left(30)
    elif i == 1:
        tello.move_forward(50)
    elif i == 2:
        tello.move_right(30)

def formation1(i, tello):
    if i == 0:
        tello.move_forward(50)
    elif i == 1:
        tello.move_left(30)
        tello.move_back(50)
    elif i == 2:
        tello.move_right(30)
        tello.move_back(50)

def formation2(i, tello):
    if i == 0:
        tello.move_left(30)
    elif i == 1:
        tello.move_forward(30)
    elif i == 2:
        tello.move_right(30)

def formation3(i, tello):
    if i == 0:
        tello.curve_xyz_speed(50, 50, 0, 100, 0, 0, 20)
    elif i == 1:
        tello.curve_xyz_speed(-50, 50, 0, -100, 0, 0, 20)
    elif i == 2:
        tello.curve_xyz_speed(50, -50, 0, 100, -100, 0, 20)

def formation4(i, tello):
    if i == 0:
        tello.move_left(30)
        tello.move_forward(50)
    elif i == 1:
        tello.move_forward(50)
    elif i == 2:
        tello.move_right(30)
        tello.move_forward(50)

def end(i, tello):
    tello.move_back(50)
    if i == 0:
        tello.move_right(30)
    elif i == 2:
        tello.move_left(30)

# Connect to the swarm and execute commands
swarm.connect()

# Take off all drones
swarm.takeoff()


swarm.parallel(formation1)

swarm.parallel(formation2)

swarm.parallel(formation3)
               
swarm.parallel(formation4)

swarm.parallel(end)

# Land all drones
swarm.land()

# End connection
swarm.end()
