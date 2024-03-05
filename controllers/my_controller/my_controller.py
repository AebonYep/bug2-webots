from robot import MoveState, Navigator
from pioneer_proxsensors1 import PioneerProxSensors
from controller import Supervisor
from util import get_pose, angle_to_pose

# Navigator parameters
DISTANCE_THRESHOLD = 0.3
TURN_SPEED = 0.25
FOLLOW_WALL_SPEED = 0.3
FORWARD_SPEED = 5
SET_POINT = DISTANCE_THRESHOLD - 0.1


# Initial setup
robot = Supervisor()
TIMESTEP = int(robot.getBasicTimeStep())

# get target position and remove cone
targetNode = robot.getFromDef('target')
targetPose = get_pose(targetNode)
targetRadius = 0.4
targetRangeX = range(int((targetPose.x - targetRadius) * 100), int((targetPose.x + targetRadius) * 100))
targetRangeY = range(int((targetPose.y - targetRadius) * 100), int((targetPose.y + targetRadius) * 100))
print('Target Position: x=%f y=%f' % (targetPose.x, targetPose.y))

# navigation and sensor setup
proxSensors = PioneerProxSensors(robot, 'sensorDisplay', get_pose(robot.getSelf()))
robotNav = Navigator(robot, proxSensors, TIMESTEP)

# Calculate the initial angle to the target (will be used to find out if the robot is on the m-line)
initalAngle = angle_to_pose(robotNav.pose, targetPose)
angleToTarget = initalAngle


# Run until we are at the target (technically not precisely at the target but close enough)
while not(robotNav.pose.x*100 in targetRangeX and robotNav.pose.y*100 in targetRangeY):
    elapsedTime = 0

    # Turn towards the target
    turnTime = robotNav.turn(angleToTarget, TURN_SPEED)
    robotNav.state = MoveState.FACE_TARGET

    # Needed to make sure robot doesnt assume its on the m-line straight away (technically it will be but on the wrong section of the obstacle)
    followWallStep = 0

    while robot.step(TIMESTEP) != -1:

        # Update robots pose and display
        robotNav.update_pose()
        robotNav.update_display()
        proxSensors.set_pose(robotNav.pose)
        proxSensors.paint()

        # Check if we have reached the target
        if robotNav.pose.x*100 in targetRangeX and robotNav.pose.y*100 in targetRangeY:
            robotNav.stop()
            break

        # Check the robots state and act accordingly
        if robotNav.state == MoveState.STOP:
            robotNav.stop()
            if not robotNav.detect_obstacles_front():
                robotNav.state = MoveState.FORWARD

        elif robotNav.state == MoveState.FORWARD:
            robotNav.forward(FORWARD_SPEED)
            # If an obstacle has been detected change state to FOLLOW_WALL
            if robotNav.detect_obstacles_front(DISTANCE_THRESHOLD):
                robotNav.state = MoveState.FOLLOW_WALL

        elif robotNav.state == MoveState.FOLLOW_WALL:
            # Follow the wall left
            robotNav.follow_wall(FOLLOW_WALL_SPEED, SET_POINT, False)
            followWallStep += 1
            # Check if robot is on the m-line and make sure we let the robot move around the wall for a little bit first
            if angle_to_pose(robotNav.pose, targetPose, robotNav.initPose.theta) == initalAngle and followWallStep > 200:
                break

        # Keep performing an action until elapsed time is past the time that action takes
        elif robotNav.state == MoveState.FACE_TARGET:
            if elapsedTime > turnTime:
                elapsedTime = 0
                robotNav.state = MoveState.STOP
            else:
                elapsedTime += TIMESTEP


    angleToTarget = angle_to_pose(robotNav.pose, targetPose)

robotNav.finished = True
robotNav.stop()
robotNav.update_display()
