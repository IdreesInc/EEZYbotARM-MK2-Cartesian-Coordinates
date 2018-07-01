#!/usr/bin/python

import Adafruit_PCA9685
from threading import Timer
import math
import time

# Hardware initialization
pwm = Adafruit_PCA9685.PCA9685()
pwm.set_pwm_freq(60)

class CColors:
    HEADER = '\033[95m'
    BLUE = '\033[94m'
    GREEN = '\033[92m'
    WARNING = '\033[93m'
    ERROR = '\033[91m'
    ENDC = '\033[0m'
    BOLD = '\033[1m'
    UNDERLINE = '\033[4m'

class LogType:
    DEBUG = 0
    NORMAL = 1
    WARNING = 2
    ERROR = 3
    NONE = 4

log_level = LogType.NORMAL

class Servo:
    def __init__(self, channel, name, min, max):
        self.channel = channel
        self.name = name
        self.min = min
        self.max = max
        self.current_angle = -1337
        self.servoMin = 150  # Min pulse length out of 4096
        self.servoMax = 600  # Max pulse length out of 4096

    def set_angle(self, angle, callback=None):
        # log("Moving %s to angle %s" % (self.name, angle))
        delta = 170
        if self.current_angle != -1337:
            delta = math.fabs(self.current_angle - angle)
            if not self.is_within_bounds(angle):
                log("Angle outside of [%s]'s bounds: %s <= %s <= %s" % (self.name, self.min, angle, self.max), LogType.ERROR)
                return -1
        self.current_angle = angle
        delay = max(delta * 0.01, 0.01)  # calculate delay
        zero_pulse = (self.servoMin + self.servoMax) / 2  # half-way == 0 degrees
        pulse_width = zero_pulse - self.servoMin  # maximum pulse to either side
        pulse = zero_pulse + (pulse_width * angle / 80)
        pwm.set_pwm(self.channel, 0, int(pulse))
        log("[%s] channel=%s angle=%s pulse=%s delta=%s" % (self.name, self.channel, angle, pulse, delta), LogType.DEBUG)
        if callback != None:
            Timer(delay, callback).start()
        return delay

    def is_within_bounds(self, angle):
        return angle >= self.min and angle <= self.max

    def rest(self):
        pwm.set_pwm(self.channel, 0, 0)

class Arm:
    def __init__(self):
        self.INNER_ARM_LENGTH = 134.5
        self.OUTER_ARM_LENGTH = 148
        self.A_VERTICAL = -63          # a servo angle when wing is vertical
        self.B_VERTICAL = -15          # b servo angle when wing is vertical
        self.A_MIN_FROM_VERTICAL = 7   # a servo minimum degrees from vertical [vert - min]
        self.B_MIN_FROM_VERTICAL = -10 # b servo minimum degrees from vertical [vert - min]
        self.A_MAX_FROM_VERTICAL = 78  # a servo minimum degrees from vertical [vert + max]
        self.B_MAX_FROM_VERTICAL = 97  # b servo minimum degrees from vertical [vert + max]

        self.rot_servo = Servo(0, "rot", -85, 85)
        self.b_servo = Servo(1, "b", self.B_VERTICAL - self.B_MIN_FROM_VERTICAL, self.B_VERTICAL + self.B_MAX_FROM_VERTICAL)
        self.a_servo = Servo(2, "a", self.A_VERTICAL - self.A_MIN_FROM_VERTICAL, self.A_VERTICAL + self.A_MAX_FROM_VERTICAL)
        self.claw_servo = Servo(3, "claw", -85, 85)

    def open_claw(self, callback=None):
        self.claw_servo.set_angle(70, callback)


    def close_claw(self):
        self.claw_servo.set_angle(-1)
        self.claw_servo.set_angle(0)


    def reset_servos(self):
        """ Reset servos to original location """
        self.rot_servo.set_angle(0)
        self.claw_servo.set_angle(0)
        self.a_servo.set_angle(0)
        self.b_servo.set_angle(0)


    def rest_servos(self):
        """ Temporarily disable the servos """
        self.rot_servo.rest()
        self.a_servo.rest()
        self.b_servo.rest()
        self.claw_servo.rest()

    def distance(self, start_point, end_point):
        """ Returns the distance from the first point to the second """
        return math.sqrt((start_point[0] - end_point[0])**2 + (start_point[1] - end_point[1])**2)

    def law_of_cosines(self, a, b, c):
        """ Performs the Law of Cosines on the given side lengths """
        return math.degrees(math.acos((c**2 - b**2 - a**2)/(-2.0 * a * b)))

    def move(self, x, y):
        """ Returns the angles necessary to reach the given coordinate point """
        # Made with love by Idrees @ https://github.com/IdreesInc
        distance_to_goal = self.distance((0, 0), (x, y))
        if distance_to_goal == 0:
            log("Cannot move to origin", LogType.ERROR)
            return False
        angle_from_horizontal = math.degrees(math.asin(y / distance_to_goal))
        try:
            triangle_a = self.law_of_cosines(self.INNER_ARM_LENGTH, distance_to_goal, self.OUTER_ARM_LENGTH)
            triangle_b = self.law_of_cosines(self.INNER_ARM_LENGTH, self.OUTER_ARM_LENGTH, distance_to_goal)
            log("[Triangle] a: %s, b: %s, angle from horiz: %s" % (triangle_a, triangle_b, angle_from_horizontal), LogType.DEBUG)
            servo_angle_a = 90 - triangle_a - angle_from_horizontal
            servo_angle_b = triangle_b - servo_angle_a + angle_from_horizontal
            if not self.a_servo.is_within_bounds(self.A_VERTICAL + servo_angle_a) or not self.b_servo.is_within_bounds(self.B_VERTICAL + servo_angle_b):
                log("Coordinates out of range due to servo constraints", LogType.ERROR)
                return False
            self.a_servo.set_angle(self.A_VERTICAL + servo_angle_a)
            self.b_servo.set_angle(self.B_VERTICAL + servo_angle_b)
            log("[Servos] a: %s, b: %s" % (servo_angle_a, servo_angle_b), LogType.DEBUG)
            return True
        except ValueError:
            log("Coordinates out of range due to the physical laws of the universe (given point is out of reach)", LogType.ERROR)
        return False


def log(message, type=LogType.NORMAL):
    if type >= log_level:
        if type == LogType.NORMAL:
            print(message)
        elif type == LogType.DEBUG:
            print("DEBUG: " + message)
        elif type == LogType.WARNING:
            print(CColors.WARNING + "WARNING: " + message + CColors.ENDC)
        elif type == LogType.ERROR:
            print(CColors.ERROR + "ERROR: " + message + CColors.ENDC)

def prompt(message):
    print(CColors.GREEN + message + CColors.ENDC)
    return raw_input()

robot_arm = Arm()

def start():
        robot_arm.reset_servos()
        robot_arm.open_claw()
        robot_arm.close_claw()
        update()

def update():
        running = True
        while running:
            input = prompt("Command?")
            command = input.split(" ")
            if len(command) == 2:
                if command[0] == "r" or command[0] == "rot" or command[0] == "rotation":
                    robot_arm.rot_servo.set_angle(float(command[1]))
                elif command[0] == "b":
                    robot_arm.b_servo.set_angle(float(command[1]))
                elif command[0] == "a":
                    robot_arm.a_servo.set_angle(float(command[1]))
                elif command[0] == "c" or command[0] == "claw":
                    robot_arm.claw_servo.set_angle(float(command[1]))
            elif len(command) == 3:
                if command[0] == "go" or command[0] == "move": # Use this to test out the coordinate system, just enter "go [INSERT X] [INSERT Y]"
                    robot_arm.move(float(command[1]), float(command[2]))
            else:
                if command[0] == "q" or command[0] == "quit":
                    running = False
                    log("Goodbye!")
                elif command[0] == "r" or command[0] == "reset":
                    robot_arm.reset_servos()
                    log("Servos reset")
                elif command[0] == "rest" or command[0] == "d" or command[0] == "disable":
                    robot_arm.rest_servos()
                    log("Servos set to rest")
                elif command[0] == "open":
                    robot_arm.open_claw()
                    log("Opening claw")
                elif command[0] == "close":
                    robot_arm.close_claw()
                    log("Closing claw")
                elif command[0] == "up":
                    robot_arm.b_servo.set_angle(80, robot_arm.b_servo.rest)
                elif command[0] == "down":
                    robot_arm.b_servo.set_angle(0, robot_arm.b_servo.rest)
                elif command[0] == "left":
                    robot_arm.rot_servo.set_angle(-80, robot_arm.b_servo.rest)
                elif command[0] == "right":
                    robot_arm.rot_servo.set_angle(80, robot_arm.b_servo.rest)
                elif command[0] == "back":
                    robot_arm.a_servo.set_angle(-70, robot_arm.b_servo.rest)
                elif command[0] == "forward":
                    robot_arm.a_servo.set_angle(0, robot_arm.b_servo.rest)
                elif command[0] == "hi" or command[0] == "hello" or command[0] == "greet":
                    log("Hello!")
                    robot_arm.open_claw(robot_arm.close_claw)
                else:
                    log("Command '%s' not recognized" % (input), LogType.ERROR)
        robot_arm.reset_servos()
        time.sleep(0.75)
        robot_arm.rest_servos()

start()