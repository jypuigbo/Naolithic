# -*- encoding: UTF-8 -*-

import time
import argparse
from naoqi import ALProxy
from OSC import OSCServer, OSCClient
import sys
import motion
import almath
import random

motionProxy = 0
jointNames = ["LHipYawPitch","LShoulderRoll","LShoulderPitch","LWristYaw","LElbowYaw","LElbowRoll"]

p0 = [-0.210, 0.01, 0.15, -1.354, -0.177, -0.036]
p1 = [-0.207, -0.314, 0.279, -1.172, -0.219, -0.397]
p2 = [-0.205, -0.314, 0.395, -1.176, -0.099, -1.119]
p3 = [-0.170, 0.800, 0.426, -1.405, -0.116, -1.544]
p = [p3]#[p0,p1,p2,p3]


# Global variable to store the ReactToTouch module instance
ReactToTouch = None
memory = None
go_to_center = False
class ReactToTouch(ALModule):
    """ A simple module able to react
        to touch events.
    """
    def __init__(self, name):
        ALModule.__init__(self, name)
        # No need for IP and port here because
        # we have our Python broker connected to NAOqi broker

        # Subscribe to TouchChanged event:
        global memory
        memory = ALProxy("ALMemory")
        memory.subscribeToEvent("TouchChanged",
            "ReactToTouch",
            "onTouched")

    def onTouched(self, strVarName, value):
        """ This will be called each time a touch
        is detected.

        """
        # Unsubscribe to the event when talking,
        # to avoid repetitions
        memory.unsubscribeToEvent("TouchChanged",
            "ReactToTouch")

        touched_bodies = []
        for p in value:
            if p[1]:
                touched_bodies.append(p[0])
        global go_to_center
        go_to_center = True

        # Subscribe again to the event
        memory.subscribeToEvent("TouchChanged",
            "ReactToTouch",
            "onTouched")

def move(addr, tags, data, source):
    global motionProxy,jointNames

    print addr
    maxSpeedFraction  = 0.15
    x = data[0]
    #y = data[1]
    motionProxy.angleInterpolationWithSpeed(jointNames, p[x], maxSpeedFraction)



def main(robotIP, PORT = 9559):
    global motionProxy, jointNames
    
    motionProxy = ALProxy("ALMotion", robotIP, PORT)
    postureProxy = ALProxy("ALRobotPosture", robotIP, PORT)

    motionProxy.setStiffnesses("Body",0)
    motionProxy.setStiffnesses(jointNames,1)

    motionProxy.openHand("LHand")
    motionProxy.closeHand("LHand")
    maxSpeedFraction  = 0.3
    for i in range(1):
        motionProxy.angleInterpolationWithSpeed(jointNames, p[i], maxSpeedFraction)
        time.sleep(1.0)

    # Copy from inverse kinematics
    effector   = "LArm"
    space      = motion.FRAME_ROBOT
    axisMask   = almath.AXIS_MASK_VEL    # just control position
    isAbsolute = False

    # Since we are in relative, the current position is zero
    currentPos = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
    center = currentPos
    # Define the changes relative to the current position
    dx         =  0.03      # translation axis X (meters)
    dy         =  -0.04     # translation axis Y (meters)
    dz         =  0.02      # translation axis Z (meters)
    dwx        =  0.00      # rotation axis X (radians)
    dwy        =  0.00      # rotation axis Y (radians)
    dwz        =  0.00      # rotation axis Z (radians)
    targetPos  = [dx, dy, dz, dwx, dwy, dwz]

    # Go to the target and back again
    path       = [targetPos]
    times      = [2.0] # seconds

    motionProxy.positionInterpolation(effector, space, path,
                                      axisMask, times, isAbsolute)
    sz=0.00
    n = input('# of trials?')
    k=1.6
    L=10
    for i in range(n):
        if random.random()<0.3:
            k=max(1,k-0.15)
        dx = k*random.random()*L-L/2 + center[0]
        dy = k*random.random()*L-L/2 + center[1]
        
        targetPos  = [dx, dy, dz, dwx, dwy, dwz]
        path       = [targetPos]
        center[0] -= targetPos[0]
        center[1] -= targetPos[1]
        motionProxy.positionInterpolation(effector, space, path,
                                      axisMask, times, isAbsolute)
    print 'Done!'
    
    

    server = OSCServer( ("" , 2222) )
    server.addMsgHandler("/move", move)
    try:
        while 1:
            server.handle_request()
            #print "after handle request"
    except KeyboardInterrupt:
        print "closing all OSC connections... and exit"
        server.close() 
        motionProxy.rest()


    motionProxy.rest()





if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument("--ip", type=str, default="127.0.0.1",
                        help="Robot ip address")
    parser.add_argument("--port", type=int, default=9559,
                        help="Robot port number")

    args = parser.parse_args()
    main(args.ip, args.port)
