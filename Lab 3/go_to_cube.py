import asyncio
import cozmo
import time
from statemachine import StateMachine, State
from enum import Enum

# Define the custom type that will be used
CustomType = cozmo.objects.CustomObjectTypes.CustomType00
# Define the custom markers that will be used
Circles2 = cozmo.objects.CustomObjectMarkers.Circles2
Circles3 = cozmo.objects.CustomObjectMarkers.Circles3
Circles4 = cozmo.objects.CustomObjectMarkers.Circles4
Triangles2 = cozmo.objects.CustomObjectMarkers.Triangles2
Triangles3 = cozmo.objects.CustomObjectMarkers.Triangles3
Triangles4 = cozmo.objects.CustomObjectMarkers.Triangles4
Diamonds2 = cozmo.objects.CustomObjectMarkers.Diamonds2
Diamonds3 = cozmo.objects.CustomObjectMarkers.Diamonds3
Diamonds4 = cozmo.objects.CustomObjectMarkers.Diamonds4
Hexagons2 = cozmo.objects.CustomObjectMarkers.Hexagons2
Hexagons3 = cozmo.objects.CustomObjectMarkers.Hexagons3
Hexagons4 = cozmo.objects.CustomObjectMarkers.Hexagons4

# Define cube and marker dimensions (in mms)
CubeSide = 38
MarkerSide = 25

# Define direction enumeration
Direction = Enum("Direction", ["Left", "Right"])

# Define the two cubes as custom objects
cube1 = cozmo.world.World.define_custom_box(CustomType, Circles2, Circles3, Circles4, 
                                           Triangles4, Triangles3, Triangles2, CubeSide, 
                                           CubeSide, CubeSide, MarkerSide, MarkerSide)
cube2 = cozmo.world.World.define_custom_box(CustomType, Diamonds2, Diamonds3, Diamonds4, 
                                           Hexagons4, Hexagons3, Hexagons2, CubeSide, 
                                           CubeSide, CubeSide, MarkerSide, MarkerSide)

def getSide():
    while(True):
        side = input("What side should the robot find? ")
        if(side == "C2"):
            return Circles2
        elif(side == "C3"):
            return Circles3
        elif(side == "C4"):
            return Circles4
        elif(side == "T2"):
            return Triangles2
        elif(side == "T3"):
            return Triangles3
        elif(side == "T4"):
            return Triangles4
        elif(side == "D2"):
            return Diamonds2
        elif(side == "D3"):
            return Diamonds3
        elif(side == "D4"):
            return Diamonds4
        elif(side == "H2"):
            return Hexagons2
        elif(side == "H3"):
            return Hexagons3
        elif(side == "H4"):
            return Hexagons4
        print("Bad input")

def getCube(side):
    if(side in [Circles2, Circles3, Circles4, Triangles2, Triangles3, Triangles4]):
        return cube1
    else:
        return cube2

class RobotMachine(StateMachine):
    "State Machine for Cozmo"
    waiting = State(initial=True)
    searching = State()
    moving = State()

    cancel = (
        searching.to(waiting)
        | moving.to(waiting)
    )   
    receive_side = waiting.to(searching)
    cube_found = searching.to(moving)
    cube_lost = moving.to(searching)
    at_cube = moving.to(waiting)

    def __init__(self):
        self.destMarker = None
        self.destCube = None
        self.lastDir = Direction.Left

    def on_enter_waiting(self):
        print("Starting waiting")
        
    def on_exit_waiting(self):
        print("Stopping waiting")
    def on_enter_searching(self):
        print("Starting searching")
        if(self.lastDir == Direction.Right):
            cozmo.robot.Robot.drive_wheels(10, 0)
        else:
            cozmo.robot.Robot.drive_wheels(0, 10)
    def on_exit_searching(self):
        print("Stopping searching")
        cozmo.robot.Robot.drive_wheels(0, 0)
    def on_enter_moving(self):
        print("Starting moving")
    def on_exit_moving(self):
        print("Stopping moving")

    def before_cancel(self):
        print("Cancelling actions")

    def before_receive_side(self, marker):
        self.destMarker = marker
        self.destCube = getCube(marker)

RobotSM = RobotMachine()
async def run(robot: cozmo.robot.Robot):
    while(True):
        if(RobotSM.current_state == RobotMachine.waiting):
            side = getSide()
            RobotSM.receive_side(side)
        elif(RobotSM.current_state == RobotMachine.searching):
            event = await robot.wait_for(cozmo.objects.EvtObjectObserved, timeout = None)
            if(event.obj == RobotSM.destCube):
                RobotSM.cube_found
                cubePose = event.pose
        elif(RobotSM.current_state == RobotMachine.moving):
            # move to cube
            print("bruh")