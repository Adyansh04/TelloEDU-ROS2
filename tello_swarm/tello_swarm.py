from djitellopy import TelloSwarm
from djitellopy import Tello

# List of drone IP addresses
tellos = [
           "192.168.60.228", # Drone 1
           "192.168.66.40", # Drone 2 left
           "192.168.66.187" # Drone 3 right
        ]

swarm = TelloSwarm.fromIps(tellos)

# swarm.parallel(lambda i, tello: tello.land())


# swarm.parallel(lambda i, tello: tello.takeoff())


swarm.connect()

swarm.takeoff()

swarm.move_up(100)

swarm.sequential(lambda i, tello: tello.move_forward(i * 30 + 20))

swarm.parallel(lambda i, tello: tello.move_left(i * 100 + 20))

swarm.parallel(lambda i, tello: tello.flip_forward())

swarm.land()


