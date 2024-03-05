import math
import pose

# Get the pose of an object
def get_pose(node):
    position = node.getPosition()
    orientation = node.getOrientation()
    theta = math.atan2(-orientation[0], orientation[3])
    theta2 = theta + (math.pi / 2)
    if theta > (math.pi / 2):
        theta2 = -(3 * (math.pi/2)) + theta

    return pose.Pose(round(position[0], 2), round(position[1], 2), theta2)


# Simple function to calculate the angle to a pose from our robots pose
def angle_to_pose(ourPose, targetPose, initTheta=None):
    # Calculate angle to target 
    yDif = targetPose.y - ourPose.y
    xDif = targetPose.x - ourPose.x
    
    # Used to check if we are on the m-line (using the initial angle but most current position can allow us to check if we are on the m-line)
    if initTheta != None:
        angleToTarget = round(math.atan2(yDif, xDif) - initTheta, 2)
    else:
        angleToTarget = round(math.atan2(yDif, xDif) - ourPose.theta, 2)

    return angleToTarget
