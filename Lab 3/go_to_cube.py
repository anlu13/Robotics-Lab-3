import asyncio
import cozmo
import time
import math
from cozmo.objects import CustomObjectTypes, CustomObjectMarkers
from statemachine import StateMachine, State
from enum import Enum

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

# Get input for sides
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

async def run(robot: cozmo.robot.Robot):
    # Define the two cubes as custom objects
    cube1 = await robot.world.define_custom_box(CustomObjectTypes.CustomType00, Circles2, Circles3,
                                                Circles4, Triangles2, Triangles3, Triangles4, CubeSide, 
                                                CubeSide, CubeSide, MarkerSide, MarkerSide, True)
    cube2 = await robot.world.define_custom_box(CustomObjectTypes.CustomType01, Diamonds2, Diamonds3, 
                                                Diamonds4, Hexagons4, Hexagons3, Hexagons2, CubeSide, 
                                                CubeSide, CubeSide, MarkerSide, MarkerSide, True)
    if(cube1 is None or cube2 is None):
        print("Failed cubes")
        return

    # Get corresponding cube from side
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
        driving_around = State()

        receive_side = waiting.to(searching)
        cube_found = searching.to(moving)
        cube_lost = (
            moving.to(searching)
            | driving_around.to(searching)
        )
        at_cube = moving.to(driving_around)
        at_side = driving_around.to(searching)

        def __init__(self):
            self.destMarker = None
            self.destCube = None
            self.secondaryMarker = None
            self.cubePose = None
            self.lastDir = Direction.Left
            super(RobotMachine, self).__init__()

        def on_enter_waiting(self):
            print("Starting waiting")
        def on_exit_waiting(self):
            print("Stopping waiting")
        def on_enter_searching(self):
            print("Starting searching")
        def on_exit_searching(self):
            print("Stopping searching")
        def on_enter_moving(self):
            print("Starting moving")
        def on_exit_moving(self):
            print("Stopping moving")

        # set destination cube
        def before_receive_side(self, marker1, marker2):
            self.destMarker = marker1
            self.destCube = getCube(marker1)
            self.secondaryMarker = marker2

        # set secondary destination as primary
        def before_at_side(self):
            temp = self.destMarker
            self.destMarker = self.secondaryMarker
            self.secondaryMarker = temp
            self.destCube = getCube(self.destMarker)

    RobotSM = RobotMachine()
    while(True):
        if(RobotSM.waiting.is_active):
            # get side selection from user
            side1 = getSide()
            side2 = getSide()
            RobotSM.receive_side(side1, side2)
        # search for cube
        elif(RobotSM.searching.is_active):
            if(RobotSM.lastDir == Direction.Right):
                await robot.drive_wheels(10, -10, duration = 10)
            else:
                await robot.drive_wheels(-10, 10, duration = 10)

            # wait until object found
            event = await robot.wait_for(cozmo.objects.EvtObjectObserved, timeout=None)
            # check if object is desired cube
            if(event.obj.object_type == RobotSM.destCube.object_type):
                RobotSM.cube_found()
        elif(RobotSM.moving.is_active):
            try:
                # ensure cube is still visible
                event = await robot.wait_for(cozmo.objects.EvtObjectObserved, timeout=100)
                # grab cube pose
                RobotSM.cubePose = event.pose
                # get pose w.r.t robot
                # TODO: Change pose so that desired face is considered front
                RobotSM.cubePose = robot.pose.define_pose_relative_this(RobotSM.cubePose)
                # Positions: x-axis is directly in front of bot, y-axis is to left, z-axis is up
                x = RobotSM.cubePose.position.x
                y = RobotSM.cubePose.position.y
                angle = math.atan(y / x)
                dist = math.hypot((x, y))
                # set search direction
                if(y < 0):
                    RobotSM.lastDir = Direction.Right
                else:
                    RobotSM.lastDir = Direction.Left
                # determine if robot is at cube
                if(dist < 40):
                    # change states
                    RobotSM.at_cube()
                else:
                    # move to cube
                    await robot.drive_wheels(dist - angle * 5, dist + angle * 5, duration = 10)
            except TimeoutError:
                RobotSM.cube_lost()
        elif(RobotSM.driving_around.is_active):
            print("Somehow rotate around the cube")
            # calculate angle needed to travel to specified face
            # turn to face
            #   turning radius = ((dist between wheels) / 2) * (vel outer + vel inner) / (vel outer - vel inner)
            #   needs to either somehow keep pose in memory and continue to update it or create desired end pose
            # once at face RobotSM.at_side() (will swap to second cube)
            

if __name__ == '__main__':
    cozmo.run_program(run)