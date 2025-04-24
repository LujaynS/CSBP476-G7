# CSBP476-G7
1. Objective
This report outlines the process and results of the Line-Following Robot
Competition. The challenge involved building and programming a robot that
could follow a designated path from a start area to an end area and stop at the
finish. The robot was required to complete five milestones within a three-
minute time limit.
2. Introduction
Because of this, it is difficult to settle on one definition for any robot. In pop
culture, robots are often pictured as being super-intelligent, autonomous
machines. However, not all robots are fully autonomous. Many robots are
programmed to perform specific functions without being able to react to
different changes. Thus, it would be better to define robots as: "A machine
capable of interacting with its environment and executing tasks based on
programmed instructions."
Robots can be subdivided into two main types: the fixed-type and the mobile-
type. Mobile robots, as the name suggests, have locomotion capabilities and
hence are able to navigate through differing environments. The Line Follower
Robot is one case of mobile robots.
A Line Follower Robot is a pre-programmed robot that will follow a specific
path traced out for it, which is mostly a dark line on a light surface. The Line
Follower Robot uses infrared (IR) sensors to detect the path, keeping the robot
on course following prewritten logic uploaded into the microcontroller.
3. Robot Development Process
Building a line-following robot involves several essential steps:
• Designing the mechanical structure of the robot
• Programming the robot's movement logic using Arduino
• Wiring and connecting sensors and motor drivers
• Testing and calibrating the sensors for accurate line detection
4. Our Robot: 4WD Keyestudio Bluetooth Multi-Functional Car
For this competition, we used the 4WD Keyestudio Bluetooth Multi-Functional
Car. This robot kit is based on an ATmega-328 microcontroller (Arduino-
compatible) and supports various features, including:
• Line tracking (used in this project)
• Obstacle avoidance
• Infrared and Bluetooth remote control
• Ultrasonic distance sensing
It had a pre-programmed application into the Arduino IDE that could identify
and follow a black line across a white surface with the help of sensor input. This
means that it had no intelligence to make decisions beyond its programming
logic; thus, always behaving in a fixed pattern according to sensor readings.
The educational kit provides a good balance between education and practical
application, enabling us to understand the core concepts of electronics,
programming, and robotics better.
5. Line Following Sensor System
Three IR line-tracking sensors are fitted on the underside of the robot's front
chassis, which will emit infrared light and will rely on the surface reflection to
detect obstacles:
• White is a good reflector of infrared light.
• Black absorbs infrared light; hence, the strength of reflection will be low.
Based on the amount of reflected light, these sensors can tell whether the
robot is on the path or it is time to correct him. That information is sent to the
Arduino, which in turn carries out the related motor control commands
required to make the robot follow the line.
The logic is straightforward:
• If the center sensor detects the line, go straight.
• If the left sensor detects the line, turn left.
• If the right sensor detects the line, turn right.
This setup is ideal for simple and efficient line-following behavior.
6. Cost of the Robot
Approximately 285 AED was the price paid for the 4WD Keyestudio Bluetooth
Multi-Functional Car Kit, which was purchased through Amazon. The kit
consists of
- Four-wheel drive chassis with motors
- IR line tracking sensor module
- Ultrasonic distance sensor
- L298P motor driver board
- Arduino-compatible microcontroller
(ATmega328)
- Bluetooth module
- Battery holder and wiring accessories
Considering the price, it becomes a viable
option for beginners and students who want to learn the basic principles of
robotics and embedded systems.
7. Flowchart
  
