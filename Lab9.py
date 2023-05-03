import networkx as nx
import matplotlib
import sys
from math import *
import time
import numpy as np
from NatNetClient import NatNetClient
from util import quaternion_to_euler_angle_vectorized1
import socket
import time

IP_ADDRESS = '192.168.0.209'

# Connect to the robot
s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
s.connect((IP_ADDRESS, 5000))
print('Connected')

positions = {}
rotations = {}

# Starting Pos: 2.8, 1.6
# Close to Box: 2.4, 1.6
# To the right: 2.4, 2.2
# Forward: 0.9, 2.4
# Get Duck and Go: 0.9, 1.3
# Back to Other Side: 2.5, 1.1
# Starting Pos: 2.8, 1.6

# This is a callback function that gets connected to the NatNet client. It is called once per rigid body per frame
def receive_rigid_body_frame(robot_id, position, rotation_quaternion):
    # Position and rotation received
    positions[robot_id] = position
    # The rotation is in quaternion. We need to convert it to euler angles

    rotx, roty, rotz = quaternion_to_euler_angle_vectorized1(rotation_quaternion)

    rotations[robot_id] = rotz

# Function to calculate weight of edges
def sqdist(p1, p2):
    return sqrt( ((p1["x"] - p2["x"])**2) + ((p1["y"] - p2["y"])**2) )

# Create graph
G = nx.path_graph(10)

# Add coordinates to graph points
pos = {
    0: {"x": -4.199, "y": 3.426},  # done
    1: {"x": -4.212, "y": 1.522},  # done
    2: {"x": -2.048, "y": 3.301},  # done
    3: {"x": -1.989, "y": 1.382},  # done
    4: {"x": -3.138, "y": 0.069},  # done
    5: {"x": -1.331, "y": -0.756}, # done
    6: {"x": -0.035, "y": 0.022},  # done
    7: {"x": -0.279, "y": 2.288},  # done
    8: {"x": 1.045, "y": 1.529},   # done
    9: {"x": 1.310, "y": 0.098},   # done
}
nx.set_node_attributes(G, pos)

# Add edges between points with weights
G.add_edge(0, 1, weight = sqdist(G.nodes[0], G.nodes[1]))
G.add_edge(0, 2, weight = sqdist(G.nodes[0], G.nodes[2]))
G.add_edge(1, 3, weight = sqdist(G.nodes[1], G.nodes[3]))
G.add_edge(2, 3, weight = sqdist(G.nodes[2], G.nodes[3]))
G.add_edge(4, 1, weight = sqdist(G.nodes[4], G.nodes[1]))
G.add_edge(4, 3, weight = sqdist(G.nodes[4], G.nodes[3]))
G.add_edge(5, 4, weight = sqdist(G.nodes[5], G.nodes[4]))
G.add_edge(5, 3, weight = sqdist(G.nodes[5], G.nodes[3]))
G.add_edge(3, 6, weight = sqdist(G.nodes[0], G.nodes[1]))
G.add_edge(5, 6, weight = sqdist(G.nodes[5], G.nodes[6]))
G.add_edge(6, 7, weight = sqdist(G.nodes[6], G.nodes[7]))
G.add_edge(3, 7, weight = sqdist(G.nodes[3], G.nodes[7]))
G.add_edge(8, 3, weight = sqdist(G.nodes[8], G.nodes[3]))
G.add_edge(8, 7, weight = sqdist(G.nodes[8], G.nodes[7]))
G.add_edge(8, 9, weight = sqdist(G.nodes[8], G.nodes[9]))
G.add_edge(9, 6, weight = sqdist(G.nodes[9], G.nodes[6]))
G.add_edge(2, 8, weight = sqdist(G.nodes[2], G.nodes[8]))

# Shortest Path to Goal
path = nx.astar_path(G, 2, 9, heuristic=None, weight="weight")

if __name__ == "__main__":
    import socket
    ## getting the hostname by socket.gethostname() method
    hostname = socket.gethostname()
    ## getting the IP address using socket.gethostbyname() method
    ip_address = socket.gethostbyname(hostname)
    print(ip_address)
    
    clientAddress = "192.168.0.196"
    optitrackServerAddress = "192.168.0.172"
    robot_id = 209

    # This will create a new NatNet client
    streaming_client = NatNetClient()
    streaming_client.set_client_address(clientAddress)
    streaming_client.set_server_address(optitrackServerAddress)
    streaming_client.set_use_multicast(True)
    # Configure the streaming client to call our rigid body handler on the emulator to send data out.
    streaming_client.rigid_body_listener = receive_rigid_body_frame

    # Start up the streaming client now that the callbacks are set up.
    # This will run perpetually, and operate on a separate thread.
    is_running = streaming_client.run()

    # Point index
    p = 0

    # P-Controller gain
    k_v  = 1700
    k_pr = 300

    t = 0.
    while is_running:
        try:
            if robot_id in positions:
                if p > len(path) - 1:
                    break
                print('Current Node: %f', path[p])

                theta = rotations[robot_id] * pi/180

                x = G.nodes[path[p]]["x"] 
                y = G.nodes[path[p]]["y"]

                # P control for x
                errx = x - positions[robot_id][0]

                # P control for y
                erry = y - positions[robot_id][1]

                print('Robot Position: (%f,%f)'%(positions[robot_id][0],positions[robot_id][1]))
                print('Distance to Next Node: (%f,%f)'%(errx,erry))

                # P control for rotation
                alpha = atan2(erry, errx)
                errw  = degrees(atan2(sin(alpha-theta), cos(alpha-theta)))
                print(errw)
                omega = k_pr*errw

                v = k_v*(sqrt(errx**2 + erry**2))
                u = np.array([v-omega, v+omega])
                # set bound of motor input
                u[u > 1500] = 1500
                u[u < -1500] = -1500

                # Send control input to the motors
                command = 'CMD_MOTOR#%d#%d#%d#%d\n'%(u[0], u[0], u[1], u[1])
                print(command)
                s.send(command.encode('utf-8'))

                if sqrt(errx**2 + erry**2) < 0.15:
                    p += 1

                time.sleep(.1)
                t += .02
                
        except KeyboardInterrupt:
            # STOP
            command = 'CMD_MOTOR#00#00#00#00\n'
            s.send(command.encode('utf-8'))

            # Close the connection
            s.shutdown(2)
            s.close()
            streaming_client.shutdown()

    # STOP
    command = 'CMD_MOTOR#00#00#00#00\n'
    s.send(command.encode('utf-8'))

    # Close the connection
    s.shutdown(2)
    s.close()
    streaming_client.shutdown()