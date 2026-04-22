
#include <Servo.h>  //Need for Servo pulse output


#include <Adafruit_BNO08x.h>  //Need for Gyroscope

//Gyroscope initialisation
Adafruit_BNO08x bno08x(-1);
sh2_SensorValue_t sensorValue;
float rad = 0.0;

//Default motor control pins
const byte left_front = 46;
const byte left_rear = 47;
const byte right_rear = 50;
const byte right_front = 51;


//Default ultrasonic ranging sensor pins, these pins are defined my the Shield
const int TRIG_PIN = 48;
const int ECHO_PIN = 49;

// Anything over 400 cm (23200 us pulse) is "out of range". Hit:If you decrease to this the ranging sensor but the timeout is short, you may not need to read up to 4meters.
const unsigned int MAX_DIST = 23200;

Servo left_front_motor;   // create servo object to control Vex Motor Controller 29
Servo left_rear_motor;   // create servo object to control Vex Motor Controller 29
Servo right_rear_motor;  // create servo object to control Vex Motor Controller 29
Servo right_front_motor;  // create servo object to control Vex Motor Controller 29
Servo turret_motor;

//Servo Setup
Servo sensor_servo;  

int speed_val = 100;
int speed_change;

// variables for IR and sonar sensors

int frontleftsensor = A6; //frontleftsensor is attached on pinA0
int backleftsensor = A7; //frontleftsensor is attached on pinA1
int frontrightsensor = A4; //frontleftsensor is attached on pinA2
int backrightsensor = A5; //frontleftsensor is attached on pinA3

byte serialRead = 0; //for control serial communication

int signalADC0 = 0; // the read out signal in 0-1023 corresponding to 0-5v
int signalADC1 = 0; // the read out signal in 0-1023 corresponding to 0-5v
int signalADC2 = 0; // the read out signal in 0-1023 corresponding to 0-5v
int signalADC3 = 0; // the read out signal in 0-1023 corresponding to 0-5v

bool wall = false;

float frontleftsensor_cm = 0; // the calculated distance in cm from the front left sensor
float backleftsensor_cm = 0; // the calculated distance in cm from the back left sensor
float frontrightsensor_cm = 0; // the calculated distance in cm from the front right sensor
float backrightsensor_cm = 0; // the calculated distance in cm from the back right sensor   
float sonarsensor_cm = 0; // the calculated distance in cm from the sonar sensor
float left_sonarsensor_cm = 0; 
float right_sonarsensor_cm = 0; 
float horizontal_distance_cm = 0; // the calculated horizontal distance from the front of the robot to the wall
float turn_angle = 0; // the calculated angle to turn to be parallel to the wall

float robot_width = 20.5; // the width of the robot in cm
float robot_length = 22.5; // the length of the robot in cm
float map_width = 121.5; // the width of the map in cm
float map_length = 199; // the length of the map in cm


//Serial Pointer
HardwareSerial *SerialCom;

// variable for home states
int home_side = 0;
int home_face = 0;
#define LEFT 0
#define RIGHT 1
#define FRONT 0
#define BACK 1

int direction = 1;
int y_dir = 1;
float y_distance = 10.0;
float x_distance = 14.0;
int x_dir = 1;
float past_error_y = 0.00;
float past_error_x = 0.00;

int pos = 0;

float wall_sensor_left = 0;
float wall_sensor_right = 0;

//----------------------------------- 
// inverse kinematics ones 
float vx = 0; // m/s 
float vy = 0; // m/s 
float wz = 0.2; // rad/s 
float Rw = 2.5; // wheel radius (m) 
float L = 11.25; // robot half-length 
float D = 10.5; // robot half-width 
float theta_dot[4] = {0, 0, 0, 0};

float gain = 50.0;        // change gain 
float past_error_wz = 0;


void setup(void) {
  Serial.begin(115200);
  turret_motor.attach(11);
  enable_motors();
  stop();
  pinMode(LED_BUILTIN, OUTPUT);

  // The Trigger pin will tell the sensor to range find
  pinMode(TRIG_PIN, OUTPUT);
  digitalWrite(TRIG_PIN, LOW);

  //Servo Setup for ultrasonic sensor
  sensor_servo.attach(10);
  sensor_servo.write(90);

   Serial.println("Enabling Gyroscope...");
   bno08x.begin_I2C();
   bno08x.enableReport(SH2_GYROSCOPE_UNCALIBRATED, 10000);
   Serial.println(" test");

  // Use USB Serial for debug output and reserve Serial1 for command input only.
  SerialCom = &Serial1;
  SerialCom->begin(115200);
  SerialCom->println("MECHENG706_Base_Code");
  delay(1000);
  SerialCom->println("Setup....");

  delay(500);  //settling time but no really needed
}



void loop() {
  // put your main code here, to run repeatedly:
  sensor_servo.write(90);
  delay(1000);
  sonarsensor_cm = read_sonarsensor();
    
  while (sonarsensor_cm > 20) {
    Serial.print("Sonar: ");
    Serial.println(sonarsensor_cm);
    sonarsensor_cm = read_sonarsensor(); 
    move(-600, 0); // should move in negative x direction 
    delay(20);   
  }
}


#ifndef NO_READ_GYRO
float GYRO_reading() {
  float gyroZ;
  if (bno08x.wasReset()) {
    bno08x.enableReport(SH2_GYROSCOPE_UNCALIBRATED);
  }

  if (bno08x.getSensorEvent(&sensorValue)) {
    if (sensorValue.sensorId == SH2_GYROSCOPE_UNCALIBRATED) {
      gyroZ = sensorValue.un.gyroscope.z; // Current Measured Angular Velocity Around The Z Axis
      // SerialCom->print("Gyroscope I2C: ");
    }
  }
  return gyroZ;
}
#endif

void home(int dir) {
  speed_val = 150;
  if (dir == LEFT) {
    read_IR_sensors(LEFT);
    home_side = LEFT;
      // go toward left 
    while (left_sonarsensor_cm > 15 && backleftsensor_cm > 5) {
      //if (left_sonarsensor_cm < 25 && backleftsensor_cm < 7) {
      //break;
      //}
      //strafe_left();
      straight_x_homing(LEFT);
      // read_IR_sensors(LEFT);  ////???? -- need to add in as safety
      left_sonarsensor_cm = read_sonarsensor();
      delay(50);
    }
    stop();
    align(LEFT);
  } else {
    read_IR_sensors(RIGHT);
    home_side = RIGHT;
    while (right_sonarsensor_cm > 15 && backrightsensor_cm > 5) {
      //if (right_sonarsensor_cm <25 && backrightsensor_cm < 12) {
        // break;
      //}
      //strafe_right();
      straight_x_homing(RIGHT);
      read_IR_sensors(RIGHT);
      right_sonarsensor_cm = read_sonarsensor();
      delay(50);
    }
    stop();
    align(RIGHT);
  }
  speed_val = 100;
}

void align(int dir) {
  Serial.print("left sonar");
  Serial.println(left_sonarsensor_cm);
  read_IR_sensors(dir);
  float error = 0;
  float error_sum = 0;
  if (dir == LEFT) {
    error = frontleftsensor_cm - backleftsensor_cm;
  } else {
  error = frontrightsensor_cm - backrightsensor_cm;
  }
  speed_val = 60 + 25.0*abs(error);

  // control loop 
  while (abs(error) > 0.2) {
    if (abs(error) < 2) {
      error_sum += abs(error);
    }
    speed_val = 60 + 25*abs(error) + 0*abs(error_sum);

    // dir 1 = left, 2 = right
    if (dir == LEFT) {
      // left 
      if (error > 0) { 
        // front is further than back, turn cw
         Serial.println("ccw");
        ccw();
        delay(100);
        stop();
        delay(50);
      } else {
        // back is further than front, turn ccw
         Serial.println("cw");
        cw();
        delay(100);
        stop();
        delay(50);
      } 
    } else {
      // right
      if (error > 0) { 
        // front is further than back, turn ccw
        Serial.println("cw");
        cw();
        delay(100);
        stop();
        delay(50);
      } else {
        Serial.println("ccw");
        // back is further than front, turn cw
        ccw();
        delay(100);
        stop();
        delay(50);
      } 
    }
    
    read_IR_sensors(dir);
    delay(50);
    if (dir == LEFT) {
      error = frontleftsensor_cm - backleftsensor_cm;
    } else {
      error = frontrightsensor_cm - backrightsensor_cm;
    }
    Serial.print("Error: ");
    Serial.println(error);
    
    // make sure moves back if too close to wall
    if (dir == LEFT) {
      if (backleftsensor_cm < 7) {
        reverse();
        delay(100);
        stop();
      }
    } else {
      if (backrightsensor_cm < 7) {
        reverse();
        delay(100);
        stop();
      }
    }
    // Serial.print("right front: ");
    // Serial.println(frontrightsensor_cm);    
    // Serial.print("right back: ");
    // Serial.println(backrightsensor_cm);
    delay(50);
  
  }
  stop();
  speed_val = 100; // reset speed value after alignment
  
}


void read_IR_sensors(int dir){
  // dir = 1
  long sumfl = 0, sumbl =0, sumfr = 0, sumbr = 0;
  if (dir == LEFT) { 
    // left
  for (int i = 0; i < 4; i++) {
    sumfl += analogRead(frontleftsensor);
    sumbl += analogRead(backleftsensor);
    delay(5);
  }
  signalADC0 = sumfl/4;
  signalADC1 = sumbl/4;
  frontleftsensor_cm = 17948*pow(signalADC0,-1.22);
  backleftsensor_cm = 17948*pow(signalADC1,-1.22);
  backleftsensor_cm = backleftsensor_cm / (0.024 * frontleftsensor + 0.86857);
  SerialCom->print("frontleftsensor_cm = ");
  SerialCom->println(frontleftsensor_cm);
  SerialCom->print("backleftsensor_cm = ");
  SerialCom->println(backleftsensor_cm);
  Serial.print("left front: ");
  Serial.println(frontleftsensor_cm);    
  Serial.print("left back: ");
  Serial.println(backleftsensor_cm);
  } else {
    // right
    for (int i = 0; i < 4; i++) {
    sumfr += analogRead(frontrightsensor);
    sumbr += analogRead(backrightsensor);
    delay(5);
    }
    signalADC2 = sumfr/4;
    signalADC3 = sumbr/4;
    frontrightsensor_cm = 17948*pow(signalADC2,-1.22);
    backrightsensor_cm = 0.5*17948*pow(signalADC3,-1.22);
    backrightsensor_cm = backrightsensor_cm * (-0.0167* frontrightsensor_cm + 1.0076);
    SerialCom->print("frontrightsensor_cm = ");
    SerialCom->println(frontrightsensor_cm);
    SerialCom->print("backrightsensor_cm = ");
    SerialCom->println(backrightsensor_cm);
    Serial.print("right front: ");
    Serial.println(frontrightsensor_cm);    
    Serial.print("right back: ");
    Serial.println(backrightsensor_cm);
  }

}
  

float read_sonarsensor() {
  unsigned long t1;
  unsigned long t2;
  unsigned long pulse_width;
  float cm;

  // Hold the trigger pin high for at least 10 us
  digitalWrite(TRIG_PIN, HIGH);
  delayMicroseconds(10);
  digitalWrite(TRIG_PIN, LOW);

   // Wait for pulse on echo pin
  t1 = micros();
  while (digitalRead(ECHO_PIN) == 0) {
    t2 = micros();
    pulse_width = t2 - t1;
    if (pulse_width > (MAX_DIST + 1000)) {
      SerialCom->println("HC-SR04: NOT found");
      return;
    }
  }

  // Measure how long the echo pin was held high (pulse width)
  // Note: the micros() counter will overflow after ~70 min

  t1 = micros();
  while (digitalRead(ECHO_PIN) == 1) {
    t2 = micros();
    pulse_width = t2 - t1;
    if (pulse_width > (MAX_DIST + 1000)) {
      SerialCom->println("HC-SR04: Out of range");
      return;
    }
  }

  t2 = micros();
  pulse_width = t2 - t1;

  // Calculate distance in centimeters and inches. The constants
  // are found in the datasheet, and calculated from the assumed speed
  //of sound in air at sea level (~340 m/s).
  cm = pulse_width / 58.0;

  // Print out results
  if (pulse_width > MAX_DIST) {
    SerialCom->println("HC-SR04: Out of range");
  } else {
    SerialCom->print("HC-SR04:");
    SerialCom->print(cm);
    SerialCom->println("cm");
  }
  return cm; 
}

void HC_SR04_range() {
  unsigned long t1;
  unsigned long t2;
  unsigned long pulse_width;
  float cm;
  float inches;

  // Hold the trigger pin high for at least 10 us
  digitalWrite(TRIG_PIN, HIGH);
  delayMicroseconds(10);
  digitalWrite(TRIG_PIN, LOW);

  // Wait for pulse on echo pin
  t1 = micros();
  while (digitalRead(ECHO_PIN) == 0) {
    t2 = micros();
    pulse_width = t2 - t1;
    if (pulse_width > (MAX_DIST + 1000)) {
      // SerialCom->println("HC-SR04: NOT found");
      return;
    }
  }

  // Measure how long the echo pin was held high (pulse width)
  // Note: the micros() counter will overflow after ~70 min

  t1 = micros();
  while (digitalRead(ECHO_PIN) == 1) {
    t2 = micros();
    pulse_width = t2 - t1;
    if (pulse_width > (MAX_DIST + 1000)) {
      // SerialCom->println("HC-SR04: Out of range");
      return;
    }
  }

  t2 = micros();
  pulse_width = t2 - t1;

  // Calculate distance in centimeters and inches. The constants
  // are found in the datasheet, and calculated from the assumed speed
  //of sound in air at sea level (~340 m/s).
  cm = pulse_width / 58.0;
  inches = pulse_width / 148.0;

  // Print out results
  if (pulse_width > MAX_DIST) {
    // SerialCom->println("HC-SR04: Out of range");
  } else {
    // SerialCom->print("HC-SR04:");
    // SerialCom->print(cm);
    // SerialCom->println("cm");
  }
}

//----------------------STRAIGHT SPEED CONTROL------------------------//
void straight_y_homing(int dir) {
  past_error_y = 0;
  float error = GYRO_reading();
  float right_side_error = 0;
  float left_side_error = 0;
  float dif = error + past_error_y;

  // float left_g = 200;
  // float right_g = 140;
  float gain = 125;
  float kp_side = 25;

  if (home_side == LEFT) {
    read_IR_sensors(LEFT);
    left_side_error = kp_side * (wall_sensor_left - frontleftsensor_cm);
    right_side_error = 0;
    // if error is positive, need to turn cw, if error is negative, need to turn ccw
  } else {
    read_IR_sensors(RIGHT);
    right_side_error = kp_side * (wall_sensor_right - frontrightsensor_cm);
    left_side_error = 0;
    // if error is positive, need to turn ccw, if error is negative, need to turn cw
  }

  //cw is add sped val to all motors, 
  // ccw is subtract speed val to all motors
  // left side error is pos need to turn cw so add left_side_error to all motors

  // float left_correction = abs(dif) * left_g;
  // float right_correction = abs(dif) * right_g;
  float correction = abs(dif) * gain;

  if (dir == 1) { //forward 
    if (dif > 0) {
      // SerialCom->print("positive error");
      left_front_motor.writeMicroseconds(1500 + speed_val + correction + left_side_error - right_side_error);
      left_rear_motor.writeMicroseconds(1500 + speed_val + correction + left_side_error - right_side_error);
      right_rear_motor.writeMicroseconds(1500 - speed_val + left_side_error - right_side_error); 
      right_front_motor.writeMicroseconds(1500 - speed_val + left_side_error - right_side_error);
    } else if (dif < 0) {
      // SerialCom->print("neg error");
      left_front_motor.writeMicroseconds(1500 + speed_val + left_side_error - right_side_error);
      left_rear_motor.writeMicroseconds(1500 + speed_val + left_side_error - right_side_error);
      right_rear_motor.writeMicroseconds(1500 - speed_val - correction + left_side_error - right_side_error);
      right_front_motor.writeMicroseconds(1500 - speed_val - correction + left_side_error - right_side_error); 
    } else {
      left_front_motor.writeMicroseconds(1500 + speed_val + left_side_error - right_side_error);
      left_rear_motor.writeMicroseconds(1500 + speed_val + left_side_error - right_side_error);
      right_rear_motor.writeMicroseconds(1500 - speed_val + left_side_error - right_side_error);
      right_front_motor.writeMicroseconds(1500 - speed_val + left_side_error - right_side_error); 
    }
  } else if (dir == -1) { //reverse
    if (dif > 0) {
      left_front_motor.writeMicroseconds(1500 - speed_val + left_side_error - right_side_error); 
      left_rear_motor.writeMicroseconds(1500 - speed_val + left_side_error - right_side_error);
      right_rear_motor.writeMicroseconds(1500 + speed_val + correction + left_side_error - right_side_error);
      right_front_motor.writeMicroseconds(1500 + speed_val + correction + left_side_error - right_side_error);
    } else if (dif < 0) {
      left_front_motor.writeMicroseconds(1500 - speed_val - correction + left_side_error - right_side_error);
      left_rear_motor.writeMicroseconds(1500 - speed_val - correction + left_side_error - right_side_error); 
      right_rear_motor.writeMicroseconds(1500 + speed_val - correction + left_side_error - right_side_error);
      right_front_motor.writeMicroseconds(1500 + speed_val - correction + left_side_error - right_side_error);
} else {
      left_front_motor.writeMicroseconds(1500 - speed_val + left_side_error - right_side_error);
      left_rear_motor.writeMicroseconds(1500 - speed_val + left_side_error - right_side_error); 
      right_rear_motor.writeMicroseconds(1500 + speed_val - left_side_error + right_side_error);
      right_front_motor.writeMicroseconds(1500 + speed_val - left_side_error + right_side_error);
}
}

past_error_y = error;

}

void straight_x_homing(int dir) {
  past_error_x = 0;
    float error = GYRO_reading();
    float dif = error + past_error_x;

    float gain = 250;

    float correction = abs(dif) * gain;
    // SerialCom->print("Correction:");
    // SerialCom->println(correction);

  if (dir == RIGHT) { //strafe right
    if (dif > 0) {
      left_front_motor.writeMicroseconds(1500 + ((speed_val)) + correction);
      left_rear_motor.writeMicroseconds(1500 - speed_val);
      right_rear_motor.writeMicroseconds(1500 - (speed_val*1.8)); //
      right_front_motor.writeMicroseconds(1500 + speed_val + correction);
    } else if (dif < 0) {
      left_front_motor.writeMicroseconds(1500 + (speed_val));
      left_rear_motor.writeMicroseconds(1500 - speed_val - correction); 
      right_rear_motor.writeMicroseconds(1500 - ((speed_val)*1.8) - correction);
      right_front_motor.writeMicroseconds(1500 + speed_val);
    } else {
       left_front_motor.writeMicroseconds(1500 + (speed_val));
      left_rear_motor.writeMicroseconds(1500 - (speed_val));
      right_rear_motor.writeMicroseconds(1500 - (speed_val * 1.8));
      right_front_motor.writeMicroseconds(1500 + speed_val);
    }
  } else if (dir == LEFT) { //strafe left
    if (dif > 0) {
      left_front_motor.writeMicroseconds(1500 - (speed_val)); 
      left_rear_motor.writeMicroseconds(1500 + speed_val + correction); 
      right_rear_motor.writeMicroseconds(1500 + ((speed_val) * 1.5) + correction); 
      right_front_motor.writeMicroseconds(1500 - speed_val);
    } else if (dif < 0) {
      left_front_motor.writeMicroseconds(1500 - ((speed_val)) - correction); 
      left_rear_motor.writeMicroseconds(1500 + speed_val);
      right_rear_motor.writeMicroseconds(1500 + (speed_val * 1.5));
      right_front_motor.writeMicroseconds(1500 - speed_val - correction); 
    } else {
      left_front_motor.writeMicroseconds(1500 - (speed_val));
      left_rear_motor.writeMicroseconds(1500 + speed_val);
      right_rear_motor.writeMicroseconds(1500 + (speed_val * 1.5));
      right_front_motor.writeMicroseconds(1500 - speed_val);
    }
  } 
  past_error_x = error;
}

//----------------------Motor moments------------------------
//The Vex Motor Controller 29 use Servo Control signals to determine speed and direction, with 0 degrees meaning neutral https://en.wikipedia.org/wiki/Servo_control

void disable_motors() {
  left_front_motor.detach();   // detach the servo on pin left_front to turn Vex Motor Controller 29 Off
  left_rear_motor.detach();   // detach the servo on pin left_rear to turn Vex Motor Controller 29 Off
  right_rear_motor.detach();  // detach the servo on pin right_rear to turn Vex Motor Controller 29 Off
  right_front_motor.detach();  // detach the servo on pin right_front to turn Vex Motor Controller 29 Off

  pinMode(left_front, INPUT);
  pinMode(left_rear, INPUT);
  pinMode(right_rear, INPUT);
  pinMode(right_front, INPUT);
}

void enable_motors() {
  left_front_motor.attach(left_front);    // attaches the servo on pin left_front to turn Vex Motor Controller 29 On
  left_rear_motor.attach(left_rear);     // attaches the servo on pin left_rear to turn Vex Motor Controller 29 On
  right_rear_motor.attach(right_rear);   // attaches the servo on pin right_rear to turn Vex Motor Controller 29 On
  right_front_motor.attach(right_front);  // attaches the servo on pin right_front to turn Vex Motor Controller 29 On
}
void stop()  //Stop
{
  left_front_motor.writeMicroseconds(1500);
  left_rear_motor.writeMicroseconds(1500);
  right_rear_motor.writeMicroseconds(1500);
  right_front_motor.writeMicroseconds(1500);
}

void forward() {
  left_front_motor.writeMicroseconds(1500 + speed_val);
  left_rear_motor.writeMicroseconds(1500 + speed_val);
  right_rear_motor.writeMicroseconds(1500 - speed_val);
  right_front_motor.writeMicroseconds(1500 - speed_val);
}

void reverse() {
  left_front_motor.writeMicroseconds(1500 - speed_val);
  left_rear_motor.writeMicroseconds(1500 - speed_val);
  right_rear_motor.writeMicroseconds(1500 + speed_val);
  right_front_motor.writeMicroseconds(1500 + speed_val);
}

void ccw() {
  left_front_motor.writeMicroseconds(1500 - speed_val);
  left_rear_motor.writeMicroseconds(1500 - speed_val);
  right_rear_motor.writeMicroseconds(1500 - speed_val);
  right_front_motor.writeMicroseconds(1500 - speed_val);
}

void cw() {
  left_front_motor.writeMicroseconds(1500 + speed_val);
  left_rear_motor.writeMicroseconds(1500 + speed_val);
  right_rear_motor.writeMicroseconds(1500 + speed_val);
  right_front_motor.writeMicroseconds(1500 + speed_val);
}

void strafe_left() {
  left_front_motor.writeMicroseconds(1500 - speed_val);
  left_rear_motor.writeMicroseconds(1500 + speed_val);
  right_rear_motor.writeMicroseconds(1500 + speed_val);
  right_front_motor.writeMicroseconds(1500 - speed_val);
}

void strafe_right() {
  left_front_motor.writeMicroseconds(1500 + speed_val);
  left_rear_motor.writeMicroseconds(1500 - speed_val);
  right_rear_motor.writeMicroseconds(1500 - speed_val);
  right_front_motor.writeMicroseconds(1500 + speed_val);
}

// ------------------------------------------------------------------- new functions

void compute_wheel_speeds(float vx, float vy, float wz) { 
  float k = (L + D); 
  theta_dot[0] = (1.0 / Rw) * ( vx - vy - k * wz ); // front left
  theta_dot[1] = (1.0 / Rw) * ( vx + vy + k * wz ); // front right
  theta_dot[2] = (1.0 / Rw) * ( vx - vy + k * wz ); // back left
  theta_dot[3] = (1.0 / Rw) * ( vx + vy - k * wz ); // back right 
} 

void move(float vx, float vy) {
  float wz = GYRO_reading();
  float error = wz;
  float dif = error + past_error_wz;
  Serial.print("error: ");
  Serial.print(error);
  float correction = dif * gain;
  Serial.print("correction: ");
  Serial.print(correction);

  compute_wheel_speeds(vx, vy, correction);
  left_front_motor.writeMicroseconds(1500 + theta_dot[0]);
  left_rear_motor.writeMicroseconds(1500 + theta_dot[3]); 
  right_rear_motor.writeMicroseconds(1500 + theta_dot[2]);
  right_front_motor.writeMicroseconds(1500 + theta_dot[1]);
  Serial.print("| left_front_ ");
  Serial.print(theta_dot[0]);
  Serial.print("| left_rear ");
  Serial.println(theta_dot[3]);
  Serial.print("| right_front ");
  Serial.print(theta_dot[1]);
  Serial.print("| right_rear ");
  Serial.println(theta_dot[2]);

  past_error_y = dif; 

}