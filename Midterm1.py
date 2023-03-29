import sys
import time
from NatNetClient import NatNetClient
from util import quaternion_to_euler_angle_vectorized1
import math 
import numpy as np
import copy
import socket 


positions = {}
rotations = {}


# This is a callback function that gets connected to the NatNet client. It is called once per rigid body per frame
def receive_rigid_body_frame(robot_id, position, rotation_quaternion):
    # Position and rotation received
    positions[robot_id] = position
    # The rotation is in quaternion. We need to convert it to euler angles

    rotx, roty, rotz = quaternion_to_euler_angle_vectorized1(rotation_quaternion)

    rotations[robot_id] = rotz

def lin_control(x,y,x1,y1):
    Kv = 1700
    x_speed = (x-x1)**2
    
    y_speed = (y-y1)**2
    distance = Kv * math.sqrt(x_speed + y_speed)
    return distance #create a 2D array for the x and y coordinates

def ang_control(theta, x, y, x1, y1):
    Kw = 100
    newOrient = math.atan2((y-y1), (x - x1))
    print('2a: '+ str(newOrient) )
    orient_error = (newOrient-theta)
    print('2b: ' + str(orient_error))
    return Kw * math.degrees(math.atan2((math.sin(orient_error)), (math.cos(orient_error))))

if __name__ == "__main__":
    IP_ADDRESS = '192.168.0.209'

    # Connect to the robot
    s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    s.connect((IP_ADDRESS, 5000))
    print('Connected')
    clientAddress = "192.168.0.35"
    optitrackServerAddress = "192.168.0.4"
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

    
    #def ang_simulate(Δt, x, u):
    #    x += Δt * u # Euler integration
    #    return x


    #def lin_simulate(Δt, x, y, u1, u2):
    #    x += Δt * u1 # Euler integration
    #    y += Δt * u2
    #    return np.array([x,y])

    

    #for t in time:
    #    u = ang_control(coor[0],coor[1])
    #    sim = simulate(Δt, x, y, u[0], u[1])
    #    x, y = sim[0], sim[1]
    #    error_log.append(copy.copy(np.array([x-2,y-10])))
    #    coor_log.append(copy.copy(sim))

    try:
        pos = [[-1.8, 1.3], [-1.75,1.98], [-3.12, 1.96], [-3.12, 0.71], [-1.65, 0.71], [-1.2, 1.3]] # some coordinate where you want to be
        count = 0
        while True:
            print(is_running)
            while is_running:
                #print(positions)
                if robot_id in positions:
                    # last position
                    print('Last position', positions[robot_id], ' rotation', rotations[robot_id])
                    
                    x = pos[count][0]
                    y = pos[count][1]
                    #orientation_log = numpy.array([x])
                    #distance_log = numpy.array([x,y])
                    x1 = positions[robot_id][0] # where you are
                    y1 = positions[robot_id][1]

                    theta = rotations[robot_id] * (math.pi/180)
                    print('Actual Angle: ' + str(theta))
                    #coor_log = [copy.copy(math.array([x,y]))] # shallow copy of array
                    #v = 0
                    v = lin_control(x, y, x1, y1)
                    print('Distance Error: ' + str(v))

                    omega = ang_control(theta,x,y,x1,y1)
                    print('Orientation Error: ' + str(omega))

                    u = np.array([v - omega, v + omega])
                    print('Raw Motor Speed Gains: '+ str(u))
                    u[u > 1500] = 1500
                    u[u < -1500] = -1500
                    # Send control input to the motors
                    command = 'CMD_MOTOR#%d#%d#%d#%d\n'%(u[0], u[0], u[1], u[1])
                    # Send control input to the motors
                    s.send(command.encode('utf-8'))
                    print('Motor speed: ' + str(command))

                    time.sleep(.1)
                    #orientation_log.append(x)
                    #distance_log.append(y)

                    if abs(x-x1) < 0.2 and abs(y-y1) < 0.2:
                        count += 1
                        if count > 5:
                            command = 'CMD_MOTOR#00#00#00#00\n'
                            s.send(command.encode('utf-8'))
                            streaming_client.shutdown()
                            s.shutdown(2)
                            s.close()


    except KeyboardInterrupt:
        # STOP
        command = 'CMD_MOTOR#00#00#00#00\n'
        s.send(command.encode('utf-8'))
        streaming_client.shutdown()

    # Close the connection
    s.shutdown(2)
    s.close()

    