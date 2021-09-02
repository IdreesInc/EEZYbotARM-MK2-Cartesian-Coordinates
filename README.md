
# EEZYbotARM MK2 Cartesian Coordinate System
A cartesian coordinate system for the [EEZYbotARM MK2](https://www.thingiverse.com/thing:1454048) project that I used for my [Dave - The Game Playing Robotic Arm](https://github.com/IdreesInc/Dave) project (WIP). The python script requires the [Adafruit Servo Hat](https://www.adafruit.com/product/2327) and a Raspberry Pi, but the math is alone is sound and can be ported to any system that allows you to set the servo angles.
The coordinate system is two dimensional, with the base of the servos acting as the origin. The points are in millimeters for easy movement. In a sense the coordinate system is not fully cartesian, as the base's rotation is not yet factored in.

## Calibration
The code is calibrated for my arm in particular, and as such you need to set the constants to be respective to your arm. While the inner and outer arm lengths may be similar (measured from rotating axis to axis, not end to end), the servo angles will vary between implementations. 

The VERTICAL servo constants refer to the angle at which the WING of the servo is pointing up, which can be found by simply setting the angle of the servo until it reaches that point. 

The MINIMUM and MAXIMUM constants refer to the minimum and maximum degrees the servos can rotate **from the axis**. Note that these values will usually be limited not by the servo itself but by the arm's movement boundaries. Please determine accordingly.

## The Important Bits
``` python
    def move(self, x, y):
        """ Returns the angles necessary to reach the given coordinate point """
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
            log("Coordinates out of range due to the physical laws of the universe", LogType.ERROR)
        return False
```

## How the Math Works
1. Create a triangle with one vertex at the servo's base, another at the inner and outer arm's axis, and a third at the point we are trying to reach
2. The inner and outer arm's lengths are already defined which means two sides of the triangle are known. Determine the third sides length by getting the distance from the origin to the point, resulting in an SSS triangle (a triangle where all side lengths are known)
3. Use the Law of Cosines to determine the inner angles A and B (C, the angle from the claw to the origin, is not needed)
4. Determine how much the triangle is rotated from the horizontal
5. Determine the required angle for servo a, factoring in the angle from the horizontal. Remember that servo a rotates from the _vertical axis_, so we have to subtract it from 90
6. Determine the required angle for servo b (which is a lot easier, as it is only angle B minus servo a's angle, plus the angle from the horizontal)
7. Set the servo angles accordingly, but factor in the angle at which the servo is vertical to get an accurate setting (as it is unlikely your servos' vertical axes are at 0 exactly)

## FAQ
### What hardware do I need?
Besides the [EEZYbotARM MK2](https://www.thingiverse.com/thing:1454048), I use a Raspberry Pi and the [Adafruit Servo Hat](https://www.adafruit.com/product/2327). However, you could definitely get away with just porting the code over to whatever system you use as long as you have got a way to set the servo angles (or can do the frequency math on the fly).
### Why?
Because I want to be able to accurately move my robotic arm by millimeter values rather than trying to guess the necessary angles to set the arm. This is for my project [Dave](https://github.com/IdreesInc/Dave), which is a game playing robotic arm. As such, it needs super accurate movements otherwise it won't be able to do much of anything.
### Why can't I move the arm left or right via coordinates?
This coordinate system is 2D, which means that you can only move the arm itself and not the base. To move the base, just adjust the rot_servo's angle.
### Is this being updated?
Not really, as it is just a snippet of code from the aforementioned [Dave](https://github.com/IdreesInc/Dave) project. Except for bug fixes, don't expect much more complexity with this project than there already is.
### Can I use this in my code?
Yup! The MIT license is super permissive, which means you can basically use it however you like. I would like to know if you use it so I can feel all warm and fuzzy inside knowing someone is using my code (as well as find potential bugs), and you can even attribute me if you are feeling particularly generous, but you are free to do whatever you want. Unless you are creating a terminator. Don't do that.

<br/>
<p align="center"><a href="https://idreesinc.com?utm_source=github&utm_medium=readme&utm_campaign=dave"><img src="https://idreesinc.com/images/logo/logo-transparent.png" width=200 height=200></a></p>
