# CSBP476-G7
Line Following Robot
The robot autonomously follows the path from the Start area to the end area and stop
Prepared by:
Name ID
Lujayn Shehada 202050925
Salama Alkaabi 202211492
Amna Alameri 202015488
Sumaya Alketbi 202200100
Submitted to Dr. Munkhjargal Gochoo

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
![image](https://github.com/user-attachments/assets/c9b4297a-69d2-4d67-880a-af2fa81ff22e)
The logic is straightforward:
• If the center sensor detects the line, go straight.
• If the left sensor detects the line, turn left.
• If the right sensor detects the line, turn right.
This setup is ideal for simple and efficient line-following behavior.
![image](https://github.com/user-attachments/assets/8716db9a-e796-43e9-a848-fa5388301ad9)

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
![image](https://github.com/user-attachments/assets/ebdd1aee-e881-4242-8265-349dc502431a)
Considering the price, it becomes a viable
option for beginners and students who want to learn the basic principles of
robotics and embedded systems.

7. Flowchart
8. Code
//*************************************************************************
/*
 Keyestudio 4WD BT Car
 Smooth Line Following with Obstacle Avoidance
*/
unsigned char start01[] = {0x01, 0x02, 0x04, 0x08, 0x10, 0x20, 0x40, 0x80,
                           0x80, 0x40, 0x20, 0x10, 0x08, 0x04, 0x02, 0x01};

#define SDA_Pin  A4
#define SCL_Pin  A5
#define trigPin  12
#define echoPin  13

// Distance threshold in cm
const int stopDistance = 5;

// Motor control pins
const int left_ctrl = 2;
const int left_pwm  = 5;
const int right_ctrl = 4;
const int right_pwm  = 6;

// Line sensor pins
const int sensor_L = 11;
const int sensor_M = 7;
const int sensor_R = 8;

// Variables
int L_val, M_val, R_val;
int lastDirection = 0; // 0=straight, 1=left, 2=right
bool lineLost = false;
unsigned long lastLineTime = 0;

// PID Control Variables
float Kp = 0.8;
float Ki = 0.01;
float Kd = 0.3;
int previousError = 0;
int integral = 0;

// Speed settings
const int baseSpeed = 150;
const int maxSpeed = 200;
const int minSpeed = 100;
const int searchSpeed = 120;
const int sharpTurnSpeed = 180;
const int aggressiveTurnSpeed = 200; // Added for aggressive turns
const unsigned long maxLineLostTime = 1000; // 1 second

// Timing
unsigned long lastDistanceCheck = 0;
bool shouldStop = false;

// Function prototypes
void matrix_display(unsigned char matrix_value[]);
void IIC_start();
void IIC_end();
void IIC_send(unsigned char send_data);
void setMotors(int leftSpeed, int rightSpeed);
void front();
void smoothLeft();
void smoothRight();
void sharpLeft();
void sharpRight();
void searchLeft();
void searchRight();
void aggressiveLeft();
void aggressiveRight();
void tracking();
long getDistance();
void stopMoving();

//*************************************************************************
// Dot matrix display functions
void matrix_display(unsigned char matrix_value[]) {
  IIC_start();
  IIC_send(0xC0);
  for (int i = 0; i < 16; i++) {
    IIC_send(matrix_value[i]);
  }
  IIC_end();
  IIC_start();
  IIC_send(0x8A);
  IIC_end();
}

void IIC_start() {
  digitalWrite(SDA_Pin, HIGH);
  digitalWrite(SCL_Pin, HIGH);
  delayMicroseconds(3);
  digitalWrite(SDA_Pin, LOW);
  delayMicroseconds(3);
  digitalWrite(SCL_Pin, LOW);
}

void IIC_end() {
  digitalWrite(SCL_Pin, LOW);
  digitalWrite(SDA_Pin, LOW);
  delayMicroseconds(3);
  digitalWrite(SCL_Pin, HIGH);
  delayMicroseconds(3);
  digitalWrite(SDA_Pin, HIGH);
  delayMicroseconds(3);
}

void IIC_send(unsigned char send_data) {
  for (byte mask = 0x01; mask != 0; mask <<= 1) {
    if (send_data & mask) {
      digitalWrite(SDA_Pin, HIGH);
    } else {
      digitalWrite(SDA_Pin, LOW);
    }
    delayMicroseconds(3);
    digitalWrite(SCL_Pin, HIGH);
    delayMicroseconds(3);
    digitalWrite(SCL_Pin, LOW);
  }
}
//*************************************************************************
// Setup
void setup() {
  Serial.begin(9600);
  
  pinMode(left_ctrl, OUTPUT);
  pinMode(left_pwm, OUTPUT);
  pinMode(right_ctrl, OUTPUT);
  pinMode(right_pwm, OUTPUT);
  
  pinMode(sensor_L, INPUT);
  pinMode(sensor_M, INPUT);
  pinMode(sensor_R, INPUT);
  
  pinMode(trigPin, OUTPUT);
  pinMode(echoPin, INPUT);
  
  pinMode(SCL_Pin, OUTPUT);
  pinMode(SDA_Pin, OUTPUT);
  
  matrix_display(start01);
}

//*************************************************************************
// Main loop
void loop() {
  unsigned long now = millis();

  if (now - lastDistanceCheck >= 200) {
    long distance = getDistance();
    shouldStop = (distance > 0 && distance <= stopDistance);
    lastDistanceCheck = now;
  }

  if (shouldStop) {
    stopMoving();
  } else {
    tracking();
  }
}

//*************************************************************************
// Ultrasonic distance sensor
long getDistance() {
  digitalWrite(trigPin, LOW);
  delayMicroseconds(2);
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);

  long duration = pulseIn(echoPin, HIGH);
  return duration * 0.034 / 2; // Convert to centimeters
}

void stopMoving() {
  setMotors(0, 0);
}

//*************************************************************************
// Motor control
void setMotors(int leftSpeed, int rightSpeed) {
  leftSpeed = constrain(leftSpeed, -255, 255);
  rightSpeed = constrain(rightSpeed, -255, 255);

  if (leftSpeed > 0) {
    digitalWrite(left_ctrl, HIGH);
    analogWrite(left_pwm, leftSpeed);
  } else if (leftSpeed < 0) {
    digitalWrite(left_ctrl, LOW);
    analogWrite(left_pwm, -leftSpeed);
  } else {
    digitalWrite(left_ctrl, LOW);
    analogWrite(left_pwm, 0);
  }

  if (rightSpeed > 0) {
    digitalWrite(right_ctrl, HIGH);
    analogWrite(right_pwm, rightSpeed);
  } else if (rightSpeed < 0) {
    digitalWrite(right_ctrl, LOW);
    analogWrite(right_pwm, -rightSpeed);
  } else {
    digitalWrite(right_ctrl, LOW);
    analogWrite(right_pwm, 0);
  }
}

//*************************************************************************
// Line tracking logic
void tracking() {
  L_val = digitalRead(sensor_L);
  M_val = digitalRead(sensor_M);
  R_val = digitalRead(sensor_R);

  if (L_val == 1 || M_val == 1 || R_val == 1) {
    lineLost = false;
    lastLineTime = millis();

    int error = 0;
    if (L_val == 1 && M_val == 0 && R_val == 0) error = -2;
    else if (L_val == 1 && M_val == 1 && R_val == 0) error = -1;
    else if (L_val == 0 && M_val == 1 && R_val == 0) error = 0;
    else if (L_val == 0 && M_val == 1 && R_val == 1) error = 1;
    else if (L_val == 0 && M_val == 0 && R_val == 1) error = 2;

    integral += error;
    integral = constrain(integral, -50, 50);
    int derivative = error - previousError;
    previousError = error;

    int correction = Kp * error + Ki * integral + Kd * derivative;

    int leftSpeed = baseSpeed - correction;
    int rightSpeed = baseSpeed + correction;

    leftSpeed = constrain(leftSpeed, minSpeed, maxSpeed);
    rightSpeed = constrain(rightSpeed, minSpeed, maxSpeed);

    setMotors(leftSpeed, rightSpeed);

    if (error < 0) lastDirection = 1;
    else if (error > 0) lastDirection = 2;
    else lastDirection = 0;
  }
  else {
    if (!lineLost) {
      lineLost = true;
      lastLineTime = millis();
    }

    unsigned long lostDuration = millis() - lastLineTime;

    if (lostDuration < maxLineLostTime) {
      if (lastDirection == 1) {
        searchLeft();
      } else if (lastDirection == 2) {
        searchRight();
      } else {
        searchRight();
      }
    }
    else {
      if (lastDirection == 1) {
        aggressiveLeft();
      } else {
        aggressiveRight();
      }
    }
  }
}

//*************************************************************************
// Movement functions
void front() {
  setMotors(baseSpeed, baseSpeed);
}

void smoothLeft() {
  setMotors(baseSpeed - 50, baseSpeed + 50);
}

void smoothRight() {
  setMotors(baseSpeed + 50, baseSpeed - 50);
}

void sharpLeft() {
  setMotors(-sharpTurnSpeed, sharpTurnSpeed);
}

void sharpRight() {
  setMotors(sharpTurnSpeed, -sharpTurnSpeed);
}

void searchLeft() {
  setMotors(-searchSpeed, searchSpeed);
}

void searchRight() {
  setMotors(searchSpeed, -searchSpeed);
}

void aggressiveLeft() {
  setMotors(-aggressiveTurnSpeed, aggressiveTurnSpeed);
}

void aggressiveRight() {
  setMotors(aggressiveTurnSpeed, -aggressiveTurnSpeed);
}
//*************************************************************************
WHITE LINE DETECTING CODE :
//*************************************************************************
/*
 Keyestudio 4WD BT Car
 Smooth Line Following with Obstacle Avoidance
*/
unsigned char start01[] = {0x01, 0x02, 0x04, 0x08, 0x10, 0x20, 0x40, 0x80,
                           0x80, 0x40, 0x20, 0x10, 0x08, 0x04, 0x02, 0x01};

#define SDA_Pin  A4
#define SCL_Pin  A5
#define trigPin  12
#define echoPin  13

// Distance threshold in cm
const int stopDistance = 5;

// Motor control pins
const int left_ctrl = 2;
const int left_pwm  = 5;
const int right_ctrl = 4;
const int right_pwm  = 6;

// Line sensor pins
const int sensor_L = 11;
const int sensor_M = 7;
const int sensor_R = 8;

// Variables
int L_val, M_val, R_val;
int lastDirection = 0; // 0=straight, 1=left, 2=right
bool lineLost = false;
unsigned long lastLineTime = 0;

// PID Control Variables
float Kp = 0.8;
float Ki = 0.01;
float Kd = 0.3;
int previousError = 0;
int integral = 0;

// Speed settings
const int baseSpeed = 150;
const int maxSpeed = 200;
const int minSpeed = 100;
const int searchSpeed = 120;
const int sharpTurnSpeed = 180;
const int aggressiveTurnSpeed = 200; // Added for aggressive turns
const unsigned long maxLineLostTime = 1000; // 1 second

// Timing
unsigned long lastDistanceCheck = 0;
bool shouldStop = false;

// Function prototypes
void matrix_display(unsigned char matrix_value[]);
void IIC_start();
void IIC_end();
void IIC_send(unsigned char send_data);
void setMotors(int leftSpeed, int rightSpeed);
void front();
void smoothLeft();
void smoothRight();
void sharpLeft();
void sharpRight();
void searchLeft();
void searchRight();
void aggressiveLeft();
void aggressiveRight();
void tracking();
long getDistance();
void stopMoving();

//*************************************************************************
// Dot matrix display functions
void matrix_display(unsigned char matrix_value[]) {
  IIC_start();
  IIC_send(0xC0);
  for (int i = 0; i < 16; i++) {
    IIC_send(matrix_value[i]);
  }
  IIC_end();
  IIC_start();
  IIC_send(0x8A);
  IIC_end();
}

void IIC_start() {
  digitalWrite(SDA_Pin, HIGH);
  digitalWrite(SCL_Pin, HIGH);
  delayMicroseconds(3);
  digitalWrite(SDA_Pin, LOW);
  delayMicroseconds(3);
  digitalWrite(SCL_Pin, LOW);
}

void IIC_end() {
  digitalWrite(SCL_Pin, LOW);
  digitalWrite(SDA_Pin, LOW);
  delayMicroseconds(3);
  digitalWrite(SCL_Pin, HIGH);
  delayMicroseconds(3);
  digitalWrite(SDA_Pin, HIGH);
  delayMicroseconds(3);
}

void IIC_send(unsigned char send_data) {
  for (byte mask = 0x01; mask != 0; mask <<= 1) {
    if (send_data & mask) {
      digitalWrite(SDA_Pin, HIGH);
    } else {
      digitalWrite(SDA_Pin, LOW);
    }
    delayMicroseconds(3);
    digitalWrite(SCL_Pin, HIGH);
    delayMicroseconds(3);
    digitalWrite(SCL_Pin, LOW);
  }
}
//*************************************************************************
// Setup
void setup() {
  Serial.begin(9600);
  
  pinMode(left_ctrl, OUTPUT);
  pinMode(left_pwm, OUTPUT);
  pinMode(right_ctrl, OUTPUT);
  pinMode(right_pwm, OUTPUT);
  
  pinMode(sensor_L, INPUT);
  pinMode(sensor_M, INPUT);
  pinMode(sensor_R, INPUT);
  
  pinMode(trigPin, OUTPUT);
  pinMode(echoPin, INPUT);
  
  pinMode(SCL_Pin, OUTPUT);
  pinMode(SDA_Pin, OUTPUT);
  
  matrix_display(start01);
}

//*************************************************************************
// Main loop
void loop() {
  unsigned long now = millis();

  if (now - lastDistanceCheck >= 200) {
    long distance = getDistance();
    shouldStop = (distance > 0 && distance <= stopDistance);
    lastDistanceCheck = now;
  }

  if (shouldStop) {
    stopMoving();
  } else {
    tracking();
  }
}

//*************************************************************************
// Ultrasonic distance sensor
long getDistance() {
  digitalWrite(trigPin, LOW);
  delayMicroseconds(2);
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);

  long duration = pulseIn(echoPin, HIGH);
  return duration * 0.034 / 2; // Convert to centimeters
}

void stopMoving() {
  setMotors(0, 0);
}

//*************************************************************************
// Motor control
void setMotors(int leftSpeed, int rightSpeed) {
  leftSpeed = constrain(leftSpeed, -255, 255);
  rightSpeed = constrain(rightSpeed, -255, 255);

  if (leftSpeed > 0) {
    digitalWrite(left_ctrl, HIGH);
    analogWrite(left_pwm, leftSpeed);
  } else if (leftSpeed < 0) {
    digitalWrite(left_ctrl, LOW);
    analogWrite(left_pwm, -leftSpeed);
  } else {
    digitalWrite(left_ctrl, LOW);
    analogWrite(left_pwm, 0);
  }

  if (rightSpeed > 0) {
    digitalWrite(right_ctrl, HIGH);
    analogWrite(right_pwm, rightSpeed);
  } else if (rightSpeed < 0) {
    digitalWrite(right_ctrl, LOW);
    analogWrite(right_pwm, -rightSpeed);
  } else {
    digitalWrite(right_ctrl, LOW);
    analogWrite(right_pwm, 0);
  }
}

//*************************************************************************
// Line tracking logic
void tracking() {
  L_val = digitalRead(sensor_L);
  M_val = digitalRead(sensor_M);
  R_val = digitalRead(sensor_R);

  if (L_val == 1 || M_val == 1 || R_val == 1) {
    lineLost = false;
    lastLineTime = millis();

    int error = 0;
    if (L_val == 1 && M_val == 0 && R_val == 0) error = -2;
    else if (L_val == 1 && M_val == 1 && R_val == 0) error = -1;
    else if (L_val == 0 && M_val == 1 && R_val == 0) error = 0;
    else if (L_val == 0 && M_val == 1 && R_val == 1) error = 1;
    else if (L_val == 0 && M_val == 0 && R_val == 1) error = 2;

    integral += error;
    integral = constrain(integral, -50, 50);
    int derivative = error - previousError;
    previousError = error;

    int correction = Kp * error + Ki * integral + Kd * derivative;

    int leftSpeed = baseSpeed - correction;
    int rightSpeed = baseSpeed + correction;

    leftSpeed = constrain(leftSpeed, minSpeed, maxSpeed);
    rightSpeed = constrain(rightSpeed, minSpeed, maxSpeed);

    setMotors(leftSpeed, rightSpeed);

    if (error < 0) lastDirection = 1;
    else if (error > 0) lastDirection = 2;
    else lastDirection = 0;
  }
  else {
    if (!lineLost) {
      lineLost = true;
      lastLineTime = millis();
    }

    unsigned long lostDuration = millis() - lastLineTime;

    if (lostDuration < maxLineLostTime) {
      if (lastDirection == 1) {
        searchLeft();
      } else if (lastDirection == 2) {
        searchRight();
      } else {
        searchRight();
      }
    }
    else {
      if (lastDirection == 1) {
        aggressiveLeft();
      } else {
        aggressiveRight();
      }
    }
  }
}

//*************************************************************************
// Movement functions
void front() {
  setMotors(baseSpeed, baseSpeed);
}

void smoothLeft() {
  setMotors(baseSpeed - 50, baseSpeed + 50);
}

void smoothRight() {
  setMotors(baseSpeed + 50, baseSpeed - 50);
}

void sharpLeft() {
  setMotors(-sharpTurnSpeed, sharpTurnSpeed);
}

void sharpRight() {
  setMotors(sharpTurnSpeed, -sharpTurnSpeed);
}

void searchLeft() {
  setMotors(-searchSpeed, searchSpeed);
}

void searchRight() {
  setMotors(searchSpeed, -searchSpeed);
}

void aggressiveLeft() {
  setMotors(-aggressiveTurnSpeed, aggressiveTurnSpeed);
}

void aggressiveRight() {
  setMotors(aggressiveTurnSpeed, -aggressiveTurnSpeed);
}
//*************************************************************************
9- Conclusion
We successfully made the robot follow the line and finish in 37 seconds. The main code struggled with the white line, but a separate test code worked perfectly—stopping the robot at the white line as intended. Future improvements should focus on merging both codes for better performance.  





  
