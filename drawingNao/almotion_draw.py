# -*- encoding: UTF-8 -*-

import time
import argparse
from naoqi import ALProxy, ALModule, ALBroker
import argparse
from OSC import OSCServer, OSCClient, OSCMessage
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

client = 0
server = 0

# Global variable to store the ReactToTouch module instance
ReactToTouch = None
memory = None
go_to_center = False
global_time = 0.0
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
        self.tts = ALProxy("ALTextToSpeech")

        self.tts.say("Hi! Can you help me paint?")

    def onTouched(self, strVarName, value):
        """ This will be called each time a touch
        is detected.

        """
        # Unsubscribe to the event when talking,
        # to avoid repetitions
        memory.unsubscribeToEvent("TouchChanged",
            "ReactToTouch")
        global go_to_center
        global global_time
        
        touched_bodies = []
        for p in value:
            if p[1]:
                if 'ront' in p[0] or 'iddle' in p[0] or 'ear' in p[0] or 'actil' in p[0]:
                    print 'someone touched my head!'
        
                    if time.time()-global_time>1.25:
                        go_to_center = True
                        sentences = ["Ouch!", "I'm sorry", "mmm..."]
                        self.tts.say(random.sample(sentences,1)[0])
        # Subscribe again to the event
        memory.subscribeToEvent("TouchChanged",
            "ReactToTouch",
            "onTouched")

def sendOSC( address='/xy', data=[] ):
    global client
    m = OSCMessage()
    m.setAddress(address)
    for d in data: m.append(d)
    client.send(m)


def move(addr, tags, data, source):
    global motionProxy,jointNames

    print addr
    maxSpeedFraction  = 0.15
    x = data[0]
    #y = data[1]
    motionProxy.angleInterpolationWithSpeed(jointNames, p[x], maxSpeedFraction)



def main(robotIP, PORT = 9559):
    myBroker = ALBroker("myBroker", #needed for subscribers and to construct naoqi modules
        "0.0.0.0",
        0,
        robotIP,
        PORT)

    meanx = 0.21835
    meany = 0.035625

    global ReactToTouch, go_to_center, global_time
    global server, client
    client = OSCClient()
    client.connect(("192.168.0.5",1234))

    global motionProxy, jointNames
    
    motionProxy = ALProxy("ALMotion", robotIP, PORT)
    postureProxy = ALProxy("ALRobotPosture", robotIP, PORT)

    motionProxy.setStiffnesses("Body",0)
    motionProxy.setStiffnesses(jointNames,1)

    #motionProxy.openHand("LHand")
    #motionProxy.closeHand("LHand")
    maxSpeedFraction  = 0.3
    for i in range(1):
        motionProxy.angleInterpolationWithSpeed(jointNames, p[i], maxSpeedFraction)
        time.sleep(1.0)    

    minx = -999
    maxx = 999
    miny = -999
    maxy = 999
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
    dz         =  0.03      # translation axis Z (meters)
    dwx        =  0.00      # rotation axis X (radians)
    dwy        =  0.00      # rotation axis Y (radians)
    dwz        =  0.00      # rotation axis Z (radians)
    targetPos  = [dx, dy, dz, dwx, dwy, dwz]

    # Go to the target and back again
    path       = [targetPos]
    times      = [1.5] # seconds

    motionProxy.positionInterpolation(effector, space, path,
                                      axisMask, times, isAbsolute)
    dz=0.00

    currentPos = motionProxy.getPosition(effector, space, True)
    offx = 0.034
    offy = 0.02
    xy=[1345+103-(currentPos[0]+offx)*400/(0.07), (currentPos[1]+offy)*400/(0.07)+11-87-45]
    #print xy

    input('Press 1 to begin: ')
    k=2.5
    L=0.5
    n=10
    speed = 0.15
    #Initialize listener
    ReactToTouch = ReactToTouch("ReactToTouch")

    center = motionProxy.getPosition(effector, space, True)
    print "center: " + str(center[0]) + ", " + str(center[1])
    
    try:
      while 1:#for i in range(n):

        if go_to_center:
            if random.random()<0.95:
                k=max(0.9,k-0.15)
            speed = 0.4
            go_to_center = False
            #print 'going to center!'
            # Get current Position & define target position
            if random.random()<0.8:
                currentPos = motionProxy.getPosition(effector, space, True)
                maxx = min(maxx,abs(currentPos[0]-meanx))
                maxy = min(maxy,abs(currentPos[1]-meany))
                
            #targetPos = currentPos
            targetPos[0] = center[0]
            targetPos[1] = center[1]
            targetPos[2] = center[2]
            #print 'new target'
            #print targetPos
            # Move to position, being able to interrupt
            motionProxy.setPosition(effector, space, targetPos, speed, axisMask)
            #Try it twice, because it doesn't seem to get there
            '''currentPos = motionProxy.getPosition(effector, space, True)
            motionProxy.positionInterpolation(effector, space, [currentPos],
                                      axisMask, [0.51], isAbsolute)
            targetPos  = [currentPos[0]-center[0], currentPos[1]-center[1], currentPos[2]-center[2], 0.0, 0.0, 0.0]
            path       = [targetPos]
            print targetPos
            motionProxy.positionInterpolation(effector, space, path,
                                      axisMask, [0.51], isAbsolute)
            print "sleeping 2 seconds"'''
            
            global_time = time.time()
        else:
            speed = 0.18
            # False learning
            if random.random()<0.15:
                k=max(0.85,k-0.15)

            # Generate next target x,y
            x = k*random.random()*L-L/2 + center[0]
            y = k*random.random()*L-L/2 + center[1]
            # Get current Position & define target position
            #currentPos = motionProxy.getPosition(effector, space, True)
            #targetPos = currentPos
            
            targetPos[0] = x#min(max(x,meanx-maxx),meanx+maxx)
            targetPos[1] = y#min(max(y,meany-maxy),meany+maxy)
            targetPos[2] = center[2]
            #print 'new target'
            #print targetPos
            # Move to position, being able to interrupt
            motionProxy.setPosition(effector, space, targetPos, speed, axisMask)
        now = time.time()
        go_to_center = False
        while time.time()-now < times[0] and not go_to_center:
            #time.sleep(0.1)
            currentPos = motionProxy.getPosition(effector, space, True)
            offx = 0.034
            offy = 0.04
            xy=[1345+97-(currentPos[0]+offx)*400/(0.07), (currentPos[1]+offy)*400/(0.07)+107-87-45]
            #print xy
            sendOSC('/xy',xy)
            time.sleep(0.01)

        print 'out of the loop'
        """else:
            #currentPos = motionProxy.getPosition(effector, space, True)
            #center[0] -= targetPos[0]
            #center[1] -= targetPos[1]
        """
      print 'Done!'

    
      

      server = OSCServer( ("" , 2222) )
      server.addMsgHandler("/move", move)
    
      while 1:
            server.handle_request()
    except KeyboardInterrupt:
        print "closing all OSC connections... and exit"
        
        motionProxy.rest()
        myBroker.shutdown()
        server.close() 


    motionProxy.rest()





if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument("--ip", type=str, default="127.0.0.1",
                        help="Robot ip address")
    parser.add_argument("--port", type=int, default=9559,
                        help="Robot port number")

    args = parser.parse_args()
    main(args.ip, args.port)
