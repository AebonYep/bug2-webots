import util
import math
from enum import Enum


class MoveState(Enum):
    STOP = 0
    FORWARD = 1
    FACE_TARGET = 2
    FOLLOW_WALL = 3

class Navigator:

    WHEEL_RADIUS = 0.0957 # Taken from labs
    AXEL_LENGTH = 0.323 # Taken from labs

    def __init__(self, robot, proxSensors, timestep):
        self.robot = robot
        self.robotNode = robot.getSelf()
        self.proxSensors = proxSensors
        self.timestep = timestep

        self.state = MoveState.STOP

        # Report data
        self.reportDisplay = robot.getDevice('reportDisplay')
        self.reportDisplay.setFont('Arial', 20, True)
        self.distanceTravelled = 0
        self.lWheelSpeed = 0
        self.rWheelSpeed = 0
        self.finished = False

        # Pose data
        self.pose = util.get_pose(self.robotNode)
        self.initPose = self.pose
        print('Robot Inital Pose: x=%.2f y=%.2f theta=%.2f' % (self.pose.x, self.pose.y, self.pose.theta))
        
        # setup motors
        self.leftWheel = robot.getDevice('left wheel') 
        self.rightWheel = robot.getDevice('right wheel') 
        self.leftWheel.setPosition(float('inf'))
        self.rightWheel.setPosition(float('inf'))
        self.leftWheel.setVelocity(0)
        self.rightWheel.setVelocity(0)

        # PID data
        self.maxVel = self.leftWheel.getMaxVelocity() - 0.1 # Fudge: just under max vel
        self.prevError = 0
        self.totalError = 0

    # ---------------------------- STATE RELATED FUNCTIONS ----------------------------
    # Simple function to update the display with the most current information
    def update_display(self):
        self.reportDisplay.setColor(0xF0F0F0)
        self.reportDisplay.fillRectangle(0, 0, 500, 500)
        self.reportDisplay.setColor(0x000000)
        self.reportDisplay.drawText('Distance Travelled: %.2f' % self.distanceTravelled,0,0)
        self.reportDisplay.drawText('Current State: %s' % self.state.name,0,25)
        self.reportDisplay.drawText('Left Wheel Velocity: %.2f' % self.lWheelSpeed,0,50)
        self.reportDisplay.drawText('Right Wheel Velocity: %.2f' % self.rWheelSpeed,0,75)
        self.reportDisplay.drawText('Pose: %s' % str(self.pose) ,0 , 100)
        self.reportDisplay.drawText('Finished: %s' % str(self.finished) ,0 , 125)

    # Simple function to update the pose and distance travelled
    def update_pose(self):
        lastX, lastY = self.pose.x, self.pose.y
        self.pose = util.get_pose(self.robotNode)
        
        # Update the distance travelled using pythag
        currentX, currentY = self.pose.x, self.pose.y
        a = currentX - lastX
        b = currentY - lastY

        self.distanceTravelled += math.sqrt(math.pow(a,2) + math.pow(b,2)) 

    # ---------------------------- SENSOR RELATED FUNCTIONS ----------------------------
    # Simple function to detect if we are close to an obstacle
    def detect_obstacles_front(self, distance=0.5):
        # Check for obstacles whilst moving forwards
        closestFrontReading = min(self.proxSensors.get_value(1), 
                                  self.proxSensors.get_value(2),
                                  self.proxSensors.get_value(3),
                                  self.proxSensors.get_value(4),
                                  self.proxSensors.get_value(5),
                                  self.proxSensors.get_value(6)) 

        return closestFrontReading < distance


    # ---------------------------- MOVEMENT RELATED FUNCTIONS ----------------------------
    # Move the robot in a straight line
    def forward(self, velocity):
        self.leftWheel.setVelocity(velocity)
        self.rightWheel.setVelocity(velocity)

        self.lWheelSpeed = velocity
        self.rWheelSpeed = velocity

    # Stop the robot from moving
    def stop(self):
        self.leftWheel.setVelocity(0)
        self.rightWheel.setVelocity(0)
        self.lWheelSpeed = 0
        self.rWheelSpeed = 0 

    # Turn on the spot a given amount
    def turn(self, amount, speed):
        target_time = abs(amount / speed)

        # Calculate each wheel velocity around ICR
        velocity = speed * (0 - (self.AXEL_LENGTH / 2))
        velocity = velocity/self.WHEEL_RADIUS
        
        if amount < 0:
            self.leftWheel.setVelocity(-velocity)
            self.rightWheel.setVelocity(velocity)
            self.lWheelSpeed = -velocity
            self.rWheelSpeed = velocity
        else:
            self.leftWheel.setVelocity(velocity)
            self.rightWheel.setVelocity(-velocity)
            self.lWheelSpeed = velocity
            self.rWheelSpeed = -velocity


        # return target_time as millisecs
        return 1000.0*target_time

    # Taken from lab 4
    def set_velocity(self, base, control=0):
        # base gives the velocity of the wheels in m/s
        # control is an adjustment on the main velocity
        base_av = (base/self.WHEEL_RADIUS)
    
        if (control != 0):
            control_av = (control/self.WHEEL_RADIUS)
            # Check if we exceed max velocity and compensate
            correction = 1
            lv = base_av - control_av
            rv = base_av + control_av
        
            if (lv > self.maxVel):
                correction = self.maxVel / lv
                lv = lv * correction
                rv = rv * correction
            
            if (rv > self.maxVel):
                correction = self.maxVel / rv
                lv = lv * correction
                rv = rv * correction
                                        
        else:
            lv = rv = base_av
                
        self.leftWheel.setVelocity(lv)
        self.rightWheel.setVelocity(rv)
        self.lWheelSpeed = lv
        self.rWheelSpeed = rv

    # Taken from lab 4 (some adjustments to weights to improve the movement)
    def pid(self, error):
        # kp = 0.6 # proportional weight (may need tuning)
        # kd = 3.0 # differential weight (may need tuning)
        # ki = 0.0 # integral weight (may need tuning)

        kp = 0.8 # smaller values seem to keep the robot closer to the wall
        kd = 1.0 # seems to effect the smoothness of travelling
        ki = 0.0 # changing this from 0 just seems to cause issues
        
        prop = error
        diff = error - self.prevError
        self.totalError += error
        control = (kp * prop) + (ki * self.totalError) + (kd * diff)
        self.prevError = error
    
        return control

    # Taken from lab 4
    def follow_wall(self, linearVelocity, setPoint, right=False):
        if right:
            direction_coeff = -1
        else:
            direction_coeff = 1
        
        # Approaching a wall, turn
        if(min(self.proxSensors.get_value(1), 
               self.proxSensors.get_value(2),
               self.proxSensors.get_value(3),
               self.proxSensors.get_value(4),
               self.proxSensors.get_value(5),
               self.proxSensors.get_value(6)) < setPoint):
            self.set_velocity(linearVelocity/3, -0.2*direction_coeff)
        else:
            if not right: 
                wall_dist = min(self.proxSensors.get_value(1),
                                self.proxSensors.get_value(0))
            else:
                wall_dist = min(self.proxSensors.get_value(7),
                                self.proxSensors.get_value(8))
  
            # Running aproximately parallel to the wall
            if (wall_dist < self.proxSensors.max_range):
                error = wall_dist - setPoint
                control = self.pid(error)
                # adjust for right wall
                self.set_velocity(linearVelocity, control*direction_coeff)
            else:
                # No wall, so turn
                self.set_velocity(linearVelocity, 0.08*direction_coeff)
  



