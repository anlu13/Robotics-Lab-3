import asyncio
import cozmo
import cozmo.audio
import time as time
import math
from cozmo.objects import CustomObjectTypes, CustomObjectMarkers
from statemachine import StateMachine, State
from enum import Enum
from PIL import Image
from cozmo.util import degrees

# Images that will be displayed on the robot's face
waitingImg = Image.open("Cozmo_Faces/Waiting.png")
waitingImg = waitingImg.resize(cozmo.oled_face.dimensions(), Image.NEAREST)
waitingImg = cozmo.oled_face.convert_image_to_screen_data(waitingImg, invert_image=True)

searchingImg = Image.open("Cozmo_Faces/Searching.png")
searchingImg = searchingImg.resize(cozmo.oled_face.dimensions(), Image.NEAREST)
searchingImg = cozmo.oled_face.convert_image_to_screen_data(searchingImg, invert_image=True)

movingImg = Image.open("Cozmo_Faces/Moving.png")
movingImg = movingImg.resize(cozmo.oled_face.dimensions(), Image.NEAREST)
movingImg = cozmo.oled_face.convert_image_to_screen_data(movingImg, invert_image=True)

aroundImg = Image.open("Cozmo_Faces/Around.png")
aroundImg = aroundImg.resize(cozmo.oled_face.dimensions(), Image.NEAREST)
aroundImg = cozmo.oled_face.convert_image_to_screen_data(aroundImg, invert_image=True)

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

# Calculate pose w.r.t other pose
def adjustPose(pose: cozmo.util.Pose, origin: cozmo.util.Pose):
    currentX = pose.position.x
    currentY = pose.position.y
    originX = origin.position.x
    originY = origin.position.y
    diffX = currentX - originX
    diffY = currentY - originY
    originAngle = origin.rotation.angle_z.radians
    x = math.cos(originAngle) * diffX + math.sin(originAngle) * diffY
    y = math.cos(originAngle) * diffY - math.sin(originAngle) * diffX
    angle_z = pose.rotation.angle_z - origin.rotation.angle_z

    return cozmo.util.Pose(x, y, 0, angle_z=angle_z)

def poseByFace(pose: cozmo.util.Pose, face: cozmo.objects._CustomObjectMarker):
    if(face == Circles2 or face == Diamonds2):
        return pose
    if(face == Circles3 or face == Hexagons3):
        newAngle = pose.rotation.angle_z + cozmo.util.degrees(180)
        return cozmo.util.Pose(pose.position.x, pose.position.y, pose.position.z,
                               angle_z=newAngle)
    if(face == Triangles2 or face == Hexagons4):
        newAngle = pose.rotation.angle_z + cozmo.util.degrees(90)
        return cozmo.util.Pose(pose.position.x, pose.position.y, pose.position.z,
                               angle_z=newAngle)
    if(face == Triangles3 or face == Hexagons2):
        newAngle = pose.rotation.angle_z + cozmo.util.degrees(270)
        return cozmo.util.Pose(pose.position.x, pose.position.y, pose.position.z,
                               angle_z=newAngle)
    if(face == Circles4 or face == Diamonds2):
        if(pose.rotation.euler_angles[0] == 0):
            newAngle = pose.rotation.euler_angles[1]
        else:
            newAngle = pose.rotation.euler_angles[0]
        return cozmo.util.Pose(pose.position.x, pose.position.y, pose.position.z,
                               angle_z=cozmo.util.radians(newAngle))
    else:
        if(pose.rotation.euler_angles[0] == 0):
            newAngle = pose.rotation.euler_angles[1]
        else:
            newAngle = pose.rotation.euler_angles[0]
        return cozmo.util.Pose(pose.position.x, pose.position.y, pose.position.z,
                               angle_z=cozmo.util.radians(newAngle))

async def run(robot: cozmo.robot.Robot):
    # Define the two cubes as custom objects
    cube1 = await robot.world.define_custom_box(CustomObjectTypes.CustomType00, Circles2, Circles3,
                                                Circles4, Triangles4, Triangles3, Triangles2, CubeSide, 
                                                CubeSide, CubeSide, MarkerSide, MarkerSide, True)
    cube2 = await robot.world.define_custom_box(CustomObjectTypes.CustomType01, Diamonds2, Hexagons3, 
                                                Diamonds3, Diamonds4, Hexagons2, Hexagons4, CubeSide, 
                                                CubeSide, CubeSide, MarkerSide, MarkerSide, True)
    if(cube1 is None or cube2 is None):
        print("Failed cubes")
        return

    # Get corresponding cube from side
    def getCube(side):
        if(side in [Circles2, Circles3, Circles4, Triangles4, Triangles3, Triangles2]):
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
            robot.display_oled_face_image(waitingImg, 1000, in_parallel=True)
            time.sleep(1)
            print("Starting waiting")
        def on_exit_waiting(self):
            print("Stopping waiting")
        def on_enter_searching(self):
            print("Starting searching")
        def on_exit_searching(self):
            print("Stopping searching")
        def on_enter_moving(self):
            robot.display_oled_face_image(movingImg, 1000, in_parallel=True)
            time.sleep(1)
            print("Starting moving")
        def on_exit_moving(self):
            print("Stopping moving")
        def on_enter_driving_around(self):
            robot.display_oled_face_image(aroundImg, 1000, in_parallel=True)
            time.sleep(1)
            print("Starting Driving Around")
        def on_exit_driving_around(self):
            print("Stopping Driving Around")

        # set destination cube
        def before_receive_side(self, marker1, marker2):
            self.destMarker = marker1
            self.destCube = getCube(marker1)
            self.secondaryMarker = marker2

        def before_cube_found(self):
            print("Starting moving")
        
        # set secondary destination as primary
        def before_at_side(self):
            temp = self.destMarker
            self.destMarker = self.secondaryMarker
            self.secondaryMarker = temp
            self.destCube = getCube(self.destMarker)
            print("side found!")
    

    await robot.say_text("Waiting", in_parallel=True).wait_for_completed()
    await robot.display_oled_face_image(waitingImg, 1000, in_parallel=True).wait_for_completed()
    
    # reset head position
    await robot.set_head_angle(degrees(-10)).wait_for_completed()
    await robot.set_lift_height(0).wait_for_completed()

    yLast = 0
    RobotSM = RobotMachine()

    while(True):
        if(RobotSM.waiting.is_active):
            # get side selection from user
            side1 = getSide()
            side2 = getSide()
            await robot.say_text("Searching", in_parallel=True).wait_for_completed()
            robot.display_oled_face_image(searchingImg, 1000, in_parallel=True)
            time.sleep(1)
            RobotSM.receive_side(side1, side2)
        # search for cube
        elif(RobotSM.searching.is_active):
            if(RobotSM.lastDir == Direction.Right):
                await robot.drive_wheels(7.5, -7.5)
            else:
                await robot.drive_wheels(-7.5, 7.5)

            # wait until object found
            event = await robot.wait_for(cozmo.objects.EvtObjectObserved, timeout=None)
            robot.stop_all_motors()
            # check if object is desired cube
            if(event.obj.object_type == RobotSM.destCube.object_type):
                RobotSM.cube_found()
                await robot.say_text("Moving", in_parallel=True).wait_for_completed()
                await robot.display_oled_face_image(movingImg, 1000, in_parallel=True).wait_for_completed()
                print("Starting moving")
        elif(RobotSM.moving.is_active):
            try:
                # ensure cube is still visible
                event = await robot.wait_for(cozmo.objects.EvtObjectObserved, timeout=2)
                # grab cube pose
                RobotSM.cubePose = event.pose
                # get pose w.r.t robot
                RobotSM.cubePose = adjustPose(RobotSM.cubePose, robot.pose)
                RobotSM.cubePose = poseByFace(RobotSM.cubePose, RobotSM.destMarker)
                # Positions: x-axis is directly in front of bot, y-axis is to left, z-axis is up
                x = RobotSM.cubePose.position.x
                y = RobotSM.cubePose.position.y
                print(x, y)
                angle = math.atan(y / x)
                dist = math.hypot(x, y)
                print(angle)
                # set search direction
                if(y > yLast):
                    RobotSM.lastDir = Direction.Right
                else:
                    RobotSM.lastDir = Direction.Left
                yLast = y
                # determine if robot is at cube
                if(dist < 80):
                    # change states
                    await robot.drive_wheels(0, 0, duration=1)
                    RobotSM.at_cube()
                    await robot.say_text("Finding Side", in_parallel=True).wait_for_completed()
                    await robot.display_oled_face_image(aroundImg, 1000, in_parallel=True).wait_for_completed()
                else:
                    # move to cube
                    speedL = dist / 7 - angle * 50
                    speedR = dist / 7 + angle * 50
                    await robot.drive_wheels(speedL, speedR)
            except asyncio.exceptions.TimeoutError:
                await robot.drive_wheels(0, 0, duration=1)
                RobotSM.cube_lost()
                await robot.say_text("Searching", in_parallel=True).wait_for_completed()
                robot.display_oled_face_image(searchingImg, 1000, in_parallel=True)
                time.sleep(1)
        elif(RobotSM.driving_around.is_active):
            try:
                event = await robot.wait_for(cozmo.objects.EvtObjectObserved, timeout=1)
                RobotSM.cubePose = event.pose
                # adjust pose
                RobotSM.cubePose = poseByFace(RobotSM.cubePose, RobotSM.destMarker)
                RobotSM.cubePose = adjustPose(RobotSM.cubePose, robot.pose)
                # get the robot to face the cube
                x = RobotSM.cubePose.position.x
                y = RobotSM.cubePose.position.y
                angle = math.atan(y / x)
                dist = math.hypot(x, y)
                if(angle < 0):
                    RobotSM.lastDir = Direction.Right
                else:
                    RobotSM.lastDir = Direction.Left
                cubeAngle = RobotSM.cubePose.rotation.angle_z.degrees
                if(cubeAngle < 25 and cubeAngle > -25):
                    RobotSM.at_side()
                    await robot.say_text("Searching", in_parallel=True).wait_for_completed()
                    robot.display_oled_face_image(searchingImg, 1000, in_parallel=True)
                    time.sleep(1)
                elif(cubeAngle < 0):
                    await robot.drive_wheels(40, -40, duration=1.5)
                    t = -1 * cubeAngle / 23
                    await robot.drive_wheels(27.5, 55, duration=t)
                    await robot.drive_wheels(-20, 20, duration=1.5)
                    RobotSM.lastDir = Direction.Left
                    RobotSM.cube_lost()
                    await robot.say_text("Searching", in_parallel=True).wait_for_completed()
                    robot.display_oled_face_image(searchingImg, 1000, in_parallel=True)
                    time.sleep(1)
                else:
                    await robot.drive_wheels(-40, 40, duration=1.5)
                    t = cubeAngle / 23
                    await robot.drive_wheels(55, 27.5, duration=t)
                    await robot.drive_wheels(20, -20, duration=1.5)
                    RobotSM.lastDir = Direction.Right
                    RobotSM.cube_lost()
                    await robot.say_text("Searching", in_parallel=True).wait_for_completed()
                    robot.display_oled_face_image(searchingImg, 1000, in_parallel=True)
                    time.sleep(1)
            except asyncio.exceptions.TimeoutError:
                RobotSM.cube_lost()
                await robot.say_text("Searching", in_parallel=True).wait_for_completed()
                robot.display_oled_face_image(searchingImg, 1000, in_parallel=True)
                time.sleep(1)
            

if __name__ == '__main__':
    cozmo.run_program(run)