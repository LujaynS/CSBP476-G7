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

