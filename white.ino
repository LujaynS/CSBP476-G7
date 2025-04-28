//*************************************************************************
/*
 keyestudio 4wd BT Car
 Never-Stop Line Following with U-Turn Handling
*/
unsigned char start01[] = {0x01, 0x02, 0x04, 0x08, 0x10, 0x20, 0x40, 0x80, 0x80, 0x40, 0x20, 0x10, 0x08, 0x04, 0x02, 0x01};
#define SDA_Pin  A4
#define SCL_Pin  A5

// Motor control pins
int left_ctrl = 2;
int left_pwm = 5;
int right_ctrl = 4;
int right_pwm = 6;

// Sensor pins
int sensor_L = 11;
int sensor_M = 7;
int sensor_R = 8;

// Variables
int L_val, M_val, R_val;
int lastDirection = 0; // 0=straight, 1=left, 2=right
bool lineLost = false;
unsigned long lastLineTime = 0;

// Parameters to tune
const int baseSpeed = 150;
const int turnSpeed = 200;
const int searchSpeed = 120;
const int sharpTurnSpeed = 180;
const unsigned long maxLineLostTime = 1000; // Max time to search before aggressive turn

// Function prototypes
void matrix_display(unsigned char matrix_value[]);
void IIC_start();
void IIC_end();
void IIC_send(unsigned char send_data);
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

// Dot matrix display functions
void matrix_display(unsigned char matrix_value[]) {
  IIC_start();
  IIC_send(0xc0);
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

void setup() {
  Serial.begin(9600);
  pinMode(left_ctrl, OUTPUT);
  pinMode(left_pwm, OUTPUT);
  pinMode(right_ctrl, OUTPUT);
  pinMode(right_pwm, OUTPUT);
  pinMode(sensor_L, INPUT);
  pinMode(sensor_M, INPUT);
  pinMode(sensor_R, INPUT);
  
  pinMode(SCL_Pin, OUTPUT);
  pinMode(SDA_Pin, OUTPUT);
  matrix_display(start01); // Now this will work
}

void loop() {
  tracking();
}

void tracking() {
  L_val = digitalRead(sensor_L);
  M_val = digitalRead(sensor_M);
  R_val = digitalRead(sensor_R);

  // Inverted logic: 0 means white (line), 1 means black (background)
  bool onLine_L = (L_val == 0);
  bool onLine_M = (M_val == 0);
  bool onLine_R = (R_val == 0);

  if (onLine_L || onLine_M || onLine_R) {
    lineLost = false;
    lastLineTime = millis();

    if (onLine_M) {
      if (onLine_L && !onLine_R) {
        smoothLeft();
        lastDirection = 1;
      }
      else if (!onLine_L && onLine_R) {
        smoothRight();
        lastDirection = 2;
      }
      else {
        front();
        lastDirection = 0;
      }
    }
    else if (onLine_L) {
      sharpLeft();
      lastDirection = 1;
    }
    else if (onLine_R) {
      sharpRight();
      lastDirection = 2;
    }
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
      } 
      else if (lastDirection == 2) {
        searchRight();
      }
      else {
        searchRight();
      }
    }
    else {
      if (lastDirection == 1) {
        aggressiveLeft();
      }
      else {
        aggressiveRight();
      }
    }
  }
}



// Movement functions
void front() {
  digitalWrite(left_ctrl, HIGH);
  analogWrite(left_pwm, baseSpeed);
  digitalWrite(right_ctrl, HIGH);
  analogWrite(right_pwm, baseSpeed);
}

void smoothLeft() {
  digitalWrite(left_ctrl, HIGH);
  analogWrite(left_pwm, baseSpeed - 50);  
  digitalWrite(right_ctrl, HIGH);
  analogWrite(right_pwm, baseSpeed + 50);
}

void smoothRight() {
  digitalWrite(left_ctrl, HIGH);
  analogWrite(left_pwm, baseSpeed + 50);
  digitalWrite(right_ctrl, HIGH);
  analogWrite(right_pwm, baseSpeed - 50);
}

void sharpLeft() {
  digitalWrite(left_ctrl, LOW);
  analogWrite(left_pwm, sharpTurnSpeed);  
  digitalWrite(right_ctrl, HIGH);
  analogWrite(right_pwm, sharpTurnSpeed);
}

void sharpRight() {
  digitalWrite(left_ctrl, HIGH);
  analogWrite(left_pwm, sharpTurnSpeed);
  digitalWrite(right_ctrl, LOW);
  analogWrite(right_pwm, sharpTurnSpeed);
}

void searchLeft() {
  digitalWrite(left_ctrl, LOW);
  analogWrite(left_pwm, searchSpeed);  
  digitalWrite(right_ctrl, HIGH);
  analogWrite(right_pwm, searchSpeed);
}

void searchRight() {
  digitalWrite(left_ctrl, HIGH);
  analogWrite(left_pwm, searchSpeed);
  digitalWrite(right_ctrl, LOW);
  analogWrite(right_pwm, searchSpeed);
}

void aggressiveLeft() {
  digitalWrite(left_ctrl, LOW);
  analogWrite(left_pwm, turnSpeed);  
  digitalWrite(right_ctrl, HIGH);
  analogWrite(right_pwm, turnSpeed);
}

void aggressiveRight() {
  digitalWrite(left_ctrl, HIGH);
  analogWrite(left_pwm, turnSpeed);
  digitalWrite(right_ctrl, LOW);
  analogWrite(right_pwm, turnSpeed);
}
//*************************************************************************
