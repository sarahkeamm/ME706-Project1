#include <Servo.h>  //Need for Servo pulse output

//#include <Adafruit_BNO08x.h>  //Need for Gyroscope

//Gyroscope initialisation
//Adafruit_BNO08x bno08x(-1);
//sh2_SensorValue_t sensorValue;
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

Servo left_font_motor;   // create servo object to control Vex Motor Controller 29
Servo left_rear_motor;   // create servo object to control Vex Motor Controller 29
Servo right_rear_motor;  // create servo object to control Vex Motor Controller 29
Servo right_font_motor;  // create servo object to control Vex Motor Controller 29
Servo turret_motor;

//Servo Setup
Servo sensor_servo;  

int speed_val = 100;
int speed_change;

// variables for IR and sonar sensors

int frontleftsensor = A7; //frontleftsensor is attached on pinA0
int backleftsensor = A6; //frontleftsensor is attached on pinA1
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

int pos = 0;
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

  // Use USB Serial for debug output and reserve Serial1 for command input only.
  SerialCom = &Serial1;
  SerialCom->begin(115200);
  SerialCom->println("MECHENG706_Base_Code");
  delay(1000);
  SerialCom->println("Setup....");

  delay(1000);  //settling time but no really needed
}


void loop() {
  // not sure what this bit is for 
  if (Serial.available()) // Check for input from terminal
  {
  serialRead = Serial.read(); // Read input
  if (serialRead==49) // Check for flag to execute, 49 is ascii for 1, stop serial printing
  {
  Serial.end(); // end the serial communication to get the sensor data
  }
  }

  // turn servo forward
  sensor_servo.write(90);
  delay(1000);
  sonarsensor_cm = read_sonarsensor();

  // turn servo 90deg to left and read sonar
  sensor_servo.write(180);
  delay(1000);
  left_sonarsensor_cm = read_sonarsensor();
  Serial.print("left sonar");
  Serial.println(left_sonarsensor_cm );
  delay(2000); 

  // turn servo 90deg to right and read sonar
  sensor_servo.write(0);
  delay(1000);
  right_sonarsensor_cm = read_sonarsensor();
  Serial.print("right sonar");
  Serial.println( right_sonarsensor_cm);
  delay(2000); 

  

  if (right_sonarsensor_cm >left_sonarsensor_cm) {
    // --------------------- If right is further than left, align to left wall first
    // safety check 
    if (sonarsensor_cm < 20 && left_sonarsensor_cm > 15) {
      reverse();
      delay(1000);
      stop();
    } 

    // turn servo 90 deg left
    sensor_servo.write(180);
    delay(1000);
    left_sonarsensor_cm = read_sonarsensor();
    while (left_sonarsensor_cm > 15) {
      strafe_left();
      left_sonarsensor_cm = read_sonarsensor();
    }
    stop();
    // align to left wall
    read_IR_sensors();
    delay(50);
    align(1);
    delay(50);
    // ------------------------------  check if aligned to correct wall
    // rotate servo 90 deg to right and read sonar
    sensor_servo.write(0);
    delay(1000);
    right_sonarsensor_cm = read_sonarsensor();
    if (right_sonarsensor_cm > 120) {
    Serial.print("Aligned to short wall");
     // if true aligned on short wall
    // rotate robot 90 deg with gyro ----------- could also change this so that it turns towards the closest wall
    cw();
    delay(2500);
    stop();
    if (right_sonarsensor_cm < 50) {
      // align to the right 
      while (right_sonarsensor_cm > 15) {
        strafe_right();
        right_sonarsensor_cm = read_sonarsensor();
        delay(50);
      }
      stop();
      align(2); 
    } else {
      // turn servo 90 deg left    
      sensor_servo.write(180);
      left_sonarsensor_cm = read_sonarsensor();
      while (left_sonarsensor_cm > 15) {
        strafe_left();
        left_sonarsensor_cm = read_sonarsensor();
        delay(50);
      }
      stop();
      // align to left wall
      align(1); 
    }
     
    }
  } else {
    // --------------------- If left is further than right, align to right wall first
    // safety check 
    if (sonarsensor_cm < 20 && right_sonarsensor_cm > 15) {
      reverse();
      delay(1000);
      stop();
    } 
    // turn servo 90 deg right
    sensor_servo.write(0);
    delay(1000);
    right_sonarsensor_cm = read_sonarsensor();
    while (right_sonarsensor_cm > 15) {
      strafe_right();
      right_sonarsensor_cm = read_sonarsensor();
      delay(50);
    }
    stop();
    // align to right wall
    align(2);
    delay(50);

    // ------------------------------  check if aligned to correct wall
    // rotate servo 90 deg to left and read sonar
    sensor_servo.write(180);
    delay(1000);
    left_sonarsensor_cm = read_sonarsensor();
    if (left_sonarsensor_cm > 120) {
     // if true aligned on short wall
    Serial.print("Aligned to short wall");
    // rotate robot 90 deg with gyro ----------- could also change this so that it turns towards the closest wall
    cw();
    delay(2500);
    stop();
    // check which way is closer wall 
    left_sonarsensor_cm = read_sonarsensor();
    if (left_sonarsensor_cm < 50) {
      // go toward left 
      while (left_sonarsensor_cm > 15) {
      strafe_left();
      left_sonarsensor_cm = read_sonarsensor();
      delay(50);
      }
      stop();
      align(1);
    } else {
      // turn servo 90 deg right
      sensor_servo.write(0);
      delay(500);
      right_sonarsensor_cm = read_sonarsensor();
      while (right_sonarsensor_cm > 15) {
      strafe_right();
      right_sonarsensor_cm = read_sonarsensor();
      delay(50);
    }
    stop();
    align(2);
    }
    
  }
}

  // align front 
  // turn servo forward
  sensor_servo.write(90);
  delay(1000);
  sonarsensor_cm = read_sonarsensor();
  float initial = sonarsensor_cm;
  if (sonarsensor_cm > 130) {
    while (sonarsensor_cm < 150) {
      reverse();
      delay(50);
    }
  } else {
    while (sonarsensor_cm > 5) {
    forward();
    sonarsensor_cm = read_sonarsensor(); 
    //speed_val = (initial- sonarsensor_cm)*25;
    delay(50);   
    }
  } 

  speed_val = 100;
  stop();
  // should be aligned to corner and homing complete
}

void align(int dir) {
  Serial.print("left sonar");
  Serial.println(left_sonarsensor_cm);
  read_IR_sensors();
  float error = 0;
  float error_sum = 0;
  if (dir == 1) {
      error = frontleftsensor_cm - backleftsensor_cm;
  } else {
      error = frontrightsensor_cm - backrightsensor_cm;
  }
  Serial.print("Error: ");
  Serial.println(error);
  speed_val = 25.0*abs(error);
  while (abs(error) > 1) {
    if (abs(error) < 2) {
      error_sum += abs(error);
    }
    speed_val = 25.0*abs(error) + 7.5*abs(error_sum);

    // dir 1 = left, 2 = right
    if (dir == 1) {
      // left 
      if (error > 0) { 
        // front is further than back, turn cw
        ccw();
        delay(50);
        stop();
        delay(50);
      } else {
        // back is further than front, turn ccw
        cw();
        delay(50);
        stop();
        delay(500);
      } 
    } else {
      // right
      if (error > 0) { 
        // front is further than back, turn ccw
        Serial.println("cw");
        cw();
        delay(100);
        stop();
        delay(500);
      } else {
        Serial.println("ccw");
        // back is further than front, turn cw
        ccw();
        delay(100);
        stop();
        delay(500);
      } 
    }
    
    read_IR_sensors();
    delay(50);
    if (dir == 1) {
      error = frontleftsensor_cm - backleftsensor_cm;
    } else {
      error = frontrightsensor_cm - backrightsensor_cm;
    }
    Serial.print("Error: ");
    Serial.println(error);
    Serial.print("Left front: ");
    Serial.println(frontleftsensor_cm);    
    Serial.print("Left back: ");
    Serial.println(backleftsensor_cm);
    delay(50);
  
  }
  stop();
  speed_val = 100; // reset speed value after alignment
  
}

void read_IR_sensors(){
  signalADC0 = analogRead(frontleftsensor); // read the signal from the front left sensor
  signalADC1 = analogRead(backleftsensor); // read the signal from the back left sensor
  signalADC2 = analogRead(frontrightsensor); // read the signal from the front right sensor
  signalADC3 = analogRead(backrightsensor); // read the signal from the back right
  frontleftsensor_cm = 0.5*17948*pow(signalADC0,-1.22);
  backleftsensor_cm = 17948*pow(signalADC1,-1.22);
  frontrightsensor_cm = 17948*pow(signalADC2,-1.22);
  backrightsensor_cm = 0.5*17948*pow(signalADC3,-1.22);
  SerialCom->print("frontleftsensor_cm = ");
  SerialCom->println(frontleftsensor_cm);
  SerialCom->print("backleftsensor_cm = ");
  SerialCom->println(backleftsensor_cm);
  SerialCom->print("frontrightsensor_cm = ");
  SerialCom->println(frontrightsensor_cm);
  SerialCom->print("backrightsensor_cm = ");
  SerialCom->println(backrightsensor_cm);

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


//----------------------Motor moments------------------------
//The Vex Motor Controller 29 use Servo Control signals to determine speed and direction, with 0 degrees meaning neutral https://en.wikipedia.org/wiki/Servo_control

void disable_motors() {
  left_font_motor.detach();   // detach the servo on pin left_front to turn Vex Motor Controller 29 Off
  left_rear_motor.detach();   // detach the servo on pin left_rear to turn Vex Motor Controller 29 Off
  right_rear_motor.detach();  // detach the servo on pin right_rear to turn Vex Motor Controller 29 Off
  right_font_motor.detach();  // detach the servo on pin right_front to turn Vex Motor Controller 29 Off

  pinMode(left_front, INPUT);
  pinMode(left_rear, INPUT);
  pinMode(right_rear, INPUT);
  pinMode(right_front, INPUT);
}

void enable_motors() {
  left_font_motor.attach(left_front);    // attaches the servo on pin left_front to turn Vex Motor Controller 29 On
  left_rear_motor.attach(left_rear);     // attaches the servo on pin left_rear to turn Vex Motor Controller 29 On
  right_rear_motor.attach(right_rear);   // attaches the servo on pin right_rear to turn Vex Motor Controller 29 On
  right_font_motor.attach(right_front);  // attaches the servo on pin right_front to turn Vex Motor Controller 29 On
}
void stop()  //Stop
{
  left_font_motor.writeMicroseconds(1500);
  left_rear_motor.writeMicroseconds(1500);
  right_rear_motor.writeMicroseconds(1500);
  right_font_motor.writeMicroseconds(1500);
}

void forward() {
  left_font_motor.writeMicroseconds(1500 + speed_val);
  left_rear_motor.writeMicroseconds(1500 + speed_val);
  right_rear_motor.writeMicroseconds(1500 - speed_val);
  right_font_motor.writeMicroseconds(1500 - speed_val);
}

void reverse() {
  left_font_motor.writeMicroseconds(1500 - speed_val);
  left_rear_motor.writeMicroseconds(1500 - speed_val);
  right_rear_motor.writeMicroseconds(1500 + speed_val);
  right_font_motor.writeMicroseconds(1500 + speed_val);
}

void ccw() {
  left_font_motor.writeMicroseconds(1500 - speed_val);
  left_rear_motor.writeMicroseconds(1500 - speed_val);
  right_rear_motor.writeMicroseconds(1500 - speed_val);
  right_font_motor.writeMicroseconds(1500 - speed_val);
}

void cw() {
  left_font_motor.writeMicroseconds(1500 + speed_val);
  left_rear_motor.writeMicroseconds(1500 + speed_val);
  right_rear_motor.writeMicroseconds(1500 + speed_val);
  right_font_motor.writeMicroseconds(1500 + speed_val);
}

void strafe_left() {
  left_font_motor.writeMicroseconds(1500 - speed_val);
  left_rear_motor.writeMicroseconds(1500 + speed_val);
  right_rear_motor.writeMicroseconds(1500 + speed_val);
  right_font_motor.writeMicroseconds(1500 - speed_val);
}

void strafe_right() {
  left_font_motor.writeMicroseconds(1500 + speed_val);
  left_rear_motor.writeMicroseconds(1500 - speed_val);
  right_rear_motor.writeMicroseconds(1500 - speed_val);
  right_font_motor.writeMicroseconds(1500 + speed_val);
}
