from djitellopy import TelloSwarm

# List of drone IP addresses
tellos = [
    "192.168.179.228",  # Drone 1
    "192.168.179.40",   # Drone 2 left
    "192.168.179.187"   # Drone 3 right
]

# Create a TelloSwarm object
swarm = TelloSwarm.fromIps(tellos)

# Define custom commands for each drone
def triangular_formation(i, tello):
    if i == 0:  # Drone 1 (base of the triangle)
        tello.move_up(50)
        tello.move_forward(100)
    elif i == 1:  # Drone 2 (left side of the triangle)
        tello.move_up(50)
        tello.move_left(50)
        tello.move_forward(100)
    elif i == 2:  # Drone 3 (right side of the triangle)
        tello.move_up(50)
        tello.move_right(50)
        tello.move_forward(100)

def create_letter_A(i, tello):
    if i == 0:  # Drone 1 (left leg of A)
        tello.move_up(100)
        tello.move_forward(50)
        tello.move_down(100)
    elif i == 1:  # Drone 2 (right leg of A)
        tello.move_up(100)
        tello.move_forward(50)
        tello.move_down(100)
    elif i == 2:  # Drone 3 (cross of A)
        tello.move_up(50)
        tello.move_left(25)
        tello.move_right(50)

# Connect to the swarm and execute commands
swarm.connect()

# Take off all drones
swarm.takeoff()

# Perform triangular formation
swarm.parallel(triangular_formation)

# Create letter A
swarm.parallel(create_letter_A)

# Land all drones
swarm.land()

# End connection
swarm.end()
